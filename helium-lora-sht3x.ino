#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <arduino_lmic_hal_boards.h>
#include <Preferences.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */

#define LED_BUILTIN 2

Adafruit_SHT31 sht31 = Adafruit_SHT31();

static const u1_t PROGMEM APPEUI[8] = { ... };
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

static const u1_t PROGMEM DEVEUI[8] = { ... };
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

static const u1_t PROGMEM APPKEY[16] = { ... };
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

// Lora Counter
static RTC_DATA_ATTR uint32_t count = 0;
RTC_DATA_ATTR uint32_t bootCount = 0;

struct __attribute__((packed)) SENSOR_DATA {
  uint8_t version;          //  1 B
  uint32_t bootcount;       //  4 B
  uint16_t battRaw;         //  2 B
  int16_t temperature;      //  2 B
  uint16_t humidity;        //  2 B
} sensorData;               // 12 B

static osjob_t sendjob;

// Pin mapping
// LoRa Probulator pinout
// DIO 0 - 27
// DIO 1 - 26
// CS    - 25

// SPI
// MISO - 19
// MOSI - 23
// SCLK - 18

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 26,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LMIC_UNUSED_PIN,
  .dio = {27, 25, LMIC_UNUSED_PIN},
};
// D0 27
// D1 25
// D2 26

void printHex2(unsigned v) {
  v &= 0xff;
  if (v < 16)
    Serial.print('0');
  Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        Serial.print("netid: ");
        Serial.println(netid, DEC);
        Serial.print("devaddr: ");
        Serial.println(devaddr, HEX);
        Serial.print("AppSKey: ");
        for (size_t i = 0; i < sizeof(artKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(artKey[i]);
        }
        Serial.println("");
        Serial.print("NwkSKey: ");
        for (size_t i = 0; i < sizeof(nwkKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(nwkKey[i]);
        }

        Preferences p;
        if (p.begin("lora", false)) {
          p.putUInt("netId", netid);
          p.putUInt("devAddr", devaddr);
          p.putBytes("nwkKey", nwkKey, sizeof(nwkKey));
          p.putBytes("artKey", artKey, sizeof(artKey));
          p.end();
        }

        count = 0;
        set_cnt(true);

        Serial.println();
      }

      LMIC_setLinkCheckMode(0);
      break;

    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      LMIC_shutdown();
      esp_deep_sleep_start();
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_SCAN_FOUND:
      ||    Serial.println(F("EV_SCAN_FOUND"));
      ||    break;
    */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      esp_deep_sleep_start();
      break;

    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
      break;
  }
}

void do_send(osjob_t* j) {
  set_cnt(false);

  Serial.print("Send counter: ");
  Serial.println(count);

  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {

    sensorData.bootcount = bootCount;

    uint8_t bs[sizeof(sensorData)];
    memcpy(bs, &sensorData, sizeof(sensorData));
    LMIC_setTxData2(1, bs, sizeof(sensorData), 0);

    Serial.println(F("Packet queued"));
  }

  count++;
}


// If the value for LORA packet counts is unknown, restore from flash
static void initCount() {
  if (count == 0) {
    Preferences p;
    if (p.begin("lora", true)) {
      count = p.getUInt("count", 0);
      p.end();
    }
  }
}

static void set_cnt(bool force) {
  LMIC_setSeqnoUp(count);

  static uint32_t lastWriteMsec = UINT32_MAX; // Ensure we write at least once
  uint32_t now = millis();
  if (now < lastWriteMsec || (now - lastWriteMsec) > 5 * 60 * 1000L || force) { // write if we roll over (50 days) or 5 mins
    lastWriteMsec = now;
    Serial.print("Saving counter: ");
    Serial.println(count);

    Preferences p;
    if (p.begin("lora", false)) {
      p.putUInt("count", count);
      p.end();
    }
  }
}

uint16_t sampledAdcRead(int8_t pin) {
  uint32_t ad = 0;
  uint16_t count = 10; // samples

  for (uint16_t i = 0; i < count; i++) {
    ad += analogRead(pin);
    delay(1);
  }

  return ad / count;
}


void setup() {
  while (! Serial);
  Serial.begin(115200);
  Serial.println(F("Starting"));

  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  Serial.println(F("Wire begin... "));
  Wire.begin(21, 22);

  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    ESP.restart();
  }

  sensorData.temperature = (int16_t)(sht31.readTemperature() * 100);
  sensorData.humidity = (uint16_t)(sht31.readHumidity() * 100);
  analogSetAttenuation(ADC_11db);
  sensorData.battRaw = sampledAdcRead(39);

  Serial.print("Temp *C = "); Serial.print(sensorData.temperature / 100.0); Serial.print("\t\t");
  Serial.print("Hum. % = "); Serial.println(sensorData.humidity / 100.0);
  Serial.print("Batt RAW = "); Serial.println(sensorData.battRaw);

  initCount();

  os_init_ex(&lmic_pins);

  LMIC_reset();

  LMIC_setClockError(1 * MAX_CLOCK_ERROR / 40);
  LMIC_setLinkCheckMode(0);
  // LMIC_startJoining();

  Preferences p;
  p.begin("lora", true);
  uint32_t netId = p.getUInt("netId", UINT32_MAX);
  uint32_t devAddr = p.getUInt("devAddr", UINT32_MAX);
  uint8_t nwkKey[16], artKey[16];
  bool keysgood = p.getBytes("nwkKey", nwkKey, sizeof(nwkKey)) == sizeof(nwkKey) &&
                  p.getBytes("artKey", artKey, sizeof(artKey)) == sizeof(artKey);
  p.end();

  if (!keysgood) {
    Serial.println("No session saved, joining from scratch");
    //LMIC_startJoining();
  }
  else {
    Serial.println("Rejoining saved session");
    Serial.print("netid: ");
    Serial.println(netId, DEC);
    Serial.print("devaddr: ");
    Serial.println(devAddr, HEX);
    Serial.print("AppSKey: ");
    for (size_t i = 0; i < sizeof(artKey); ++i) {
      if (i != 0)
        Serial.print("-");
      printHex2(artKey[i]);
    }
    Serial.println("");
    Serial.print("NwkSKey: ");
    for (size_t i = 0; i < sizeof(nwkKey); ++i) {
      if (i != 0)
        Serial.print("-");
      printHex2(nwkKey[i]);
    }
    Serial.println();
    LMIC_setSession(netId, devAddr, nwkKey, artKey);
  }

  LMIC_setDrTxpow(DR_SF9,14);

  do_send(&sendjob);

  Serial.flush();

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}

void loop() {
  os_runloop_once();
  delay(1);
}

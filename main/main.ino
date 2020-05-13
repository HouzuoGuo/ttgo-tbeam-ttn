#include <vector>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>

// static const u1_t PROGMEM NWKSKEY[16], APPSKEY[16], u4_t DEVADDR.
#include "keys.h"

/*
Payload decoder (from https://github.com/kizniche/ttgo-tbeam-ttn-tracker):
function Decoder(bytes, port) {
    var decoded = {};

    decoded.latitude = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
    decoded.latitude = (decoded.latitude / 16777215.0 * 180) - 90;

    decoded.longitude = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
    decoded.longitude = (decoded.longitude / 16777215.0 * 360) - 180;

    var altValue = ((bytes[6]<<8)>>>0) + bytes[7];
    var sign = bytes[6] & (1 << 7);
    if(sign) decoded.altitude = 0xFFFF0000 | altValue;
    else decoded.altitude = altValue;

    decoded.hdop = bytes[8] / 10.0;
    decoded.sats = bytes[9];

    return decoded;
}
*/

#define EV_QUEUED 100
#define EV_PENDING 101
#define EV_ACK 102
#define EV_RESPONSE 103
#define SCK_GPIO 5
#define MISO_GPIO 19
#define MOSI_GPIO 27
#define NSS_GPIO 18
#define RESET_GPIO 23
#define DIO0_GPIO 26
#define DIO1_GPIO 33
#define DIO2_GPIO 32

const lmic_pinmap lmic_pins = {
    .nss = NSS_GPIO,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = RESET_GPIO,
    .dio = {DIO0_GPIO, DIO1_GPIO, DIO2_GPIO},
};

void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

std::vector<void (*)(uint8_t message)> _lmic_callbacks;

void _ttn_callback(uint8_t message)
{
  for (uint8_t i = 0; i < _lmic_callbacks.size(); i++)
  {
    (_lmic_callbacks[i])(message);
  }
}

void onEvent(ev_t event)
{
  switch (event)
  {
  case EV_JOINED:
    Serial.println("Joined");
    break;
  case EV_TXCOMPLETE:
    Serial.println(F("EV_TXCOMPLETE (inc. RX win. wait)"));
    if (LMIC.txrxFlags & TXRX_ACK)
    {
      Serial.println(F("Received ack"));
      _ttn_callback(EV_ACK);
    }
    if (LMIC.dataLen)
    {
      Serial.print(F("Data Received: "));
      Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
      Serial.println();
      _ttn_callback(EV_RESPONSE);
    }
    break;
  default:
    Serial.printf("Other event: %d\n", event);
    break;
  }
  _ttn_callback(event);
}

void ttn_response(uint8_t *buffer, size_t len)
{
  for (uint8_t i = 0; i < LMIC.dataLen; i++)
  {
    buffer[i] = LMIC.frame[LMIC.dataBeg + i];
  }
}

void callback(uint8_t message)
{
  if (EV_JOINING == message)
    Serial.print("Joining TTN...\n");
  if (EV_JOINED == message)
    Serial.print("TTN joined!\n");
  if (EV_JOIN_FAILED == message)
    Serial.print("TTN join failed\n");
  if (EV_REJOIN_FAILED == message)
    Serial.print("TTN rejoin failed\n");
  if (EV_RESET == message)
    Serial.print("Reset TTN connection\n");
  if (EV_LINK_DEAD == message)
    Serial.print("TTN link dead\n");
  if (EV_ACK == message)
    Serial.print("ACK received\n");
  if (EV_PENDING == message)
    Serial.print("Message discarded\n");
  if (EV_QUEUED == message)
    Serial.print("Message queued\n");

  if (EV_TXCOMPLETE == message)
  {
    Serial.print("Message sent\n");
  }

  if (EV_RESPONSE == message)
  {
    Serial.print("[TTN] Response: ");

    size_t len = LMIC.dataLen;
    uint8_t data[len];
    ttn_response(data, len);

    char buffer[6];
    for (uint8_t i = 0; i < len; i++)
    {
      snprintf(buffer, sizeof(buffer), "%02X", data[i]);
      Serial.print(buffer);
    }
    Serial.print("\n");
  }
}

void setup()
{
  while (!Serial)
    ;
  Serial.begin(115200);
  SPI.begin(SCK_GPIO, MISO_GPIO, MOSI_GPIO, NSS_GPIO);
  os_init();
  LMIC_reset();

  _lmic_callbacks.push_back(callback);
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);

  LMIC_setAdrMode(0);
  LMIC_setLinkCheckMode(0);
  LMIC.dn2Dr = SF7;
  LMIC_setDrTxpow(DR_SF7, 20);
  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);

  _ttn_callback(EV_JOINED);
}

static uint32_t count = 0;
void loop()
{
  os_runloop_once();
  static uint32_t last = 0;
  if (0 == last || millis() - last > 10000)
  {
    last = millis();
    uint32_t LatitudeBinary;
    uint32_t LongitudeBinary;
    uint16_t altitudeGps;
    uint8_t hdopGps;
    uint8_t sats;
    LatitudeBinary = ((60.15 + 90) / 180.0) * 16777215;
    LongitudeBinary = ((24.91 + 180) / 360.0) * 16777215;
    altitudeGps = 30.0;
    hdopGps = 5.0;
    sats = 1;

    uint8_t txBuffer[10];
    txBuffer[0] = (LatitudeBinary >> 16) & 0xFF;
    txBuffer[1] = (LatitudeBinary >> 8) & 0xFF;
    txBuffer[2] = LatitudeBinary & 0xFF;
    txBuffer[3] = (LongitudeBinary >> 16) & 0xFF;
    txBuffer[4] = (LongitudeBinary >> 8) & 0xFF;
    txBuffer[5] = LongitudeBinary & 0xFF;
    txBuffer[6] = (altitudeGps >> 8) & 0xFF;
    txBuffer[7] = altitudeGps & 0xFF;
    txBuffer[8] = hdopGps & 0xFF;
    txBuffer[9] = sats & 0xFF;
    /*
    char *msg = "xxxxxxxxxxxx";
    for (int i = 10; i< 10+strlen(msg); ++i) {
      txBuffer[i] = msg[i-10];
    }
    */

    LMIC_setSeqnoUp(count);
    LMIC_setTxData2(10, txBuffer, sizeof(txBuffer), false);
    if (LMIC.opmode & OP_TXRXPEND)
    {
      _ttn_callback(EV_PENDING);
      return;
    }
    _ttn_callback(EV_QUEUED);
    count++;
  }
}
#include <Arduino.h>
#include <bluefruit.h>

static const char *DEVICE_NAME = "R40_TEST";

void setup() {
  // Deliberately no GPIO, I2C, sensor, or custom GATT initialization.
  // This image only proves that the nRF52832 + S132 can advertise.
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName(DEVICE_NAME);

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addName();
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(160, 160);
  Bluefruit.Advertising.setFastTimeout(0);
  Bluefruit.Advertising.start(0);
}

void loop() {
  delay(1000);
}

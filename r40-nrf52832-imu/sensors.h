#pragma once
#include <Arduino.h>
#include <Wire.h>

struct ImuReading {
  float ax, ay, az; // g
  float gx, gy, gz; // dps
  float mx, my, mz; // uT
  bool imuOk;
  bool magOk;
};

struct EnvReading {
  float tempC;
  float pressurePa;
  float lux;
  float humidityPct; // NaN if BME280 humidity absent
  uint16_t proximity;
  bool bmpOk;
  bool alsOk;
  bool magOk;
  bool humOk;
};

void sensorsBeginImuBus();
bool sensorsInitAll();
bool sensorsHasMag();
void sensorsReadImu(ImuReading *out);
void sensorsReadEnv(EnvReading *out);
void sensorsGetScanDiag(uint8_t *nImu, uint8_t *aImu, uint8_t *nEnv, uint8_t *aEnv);

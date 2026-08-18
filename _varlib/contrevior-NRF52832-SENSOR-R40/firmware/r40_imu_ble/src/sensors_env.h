#pragma once
#include <Arduino.h>
#include "sensors.h"

// Shared Wire bus (implemented in sensors.cpp)
void sensorsBusBegin(uint8_t sda, uint8_t scl);
bool sensorsWr(uint8_t addr, uint8_t reg, uint8_t val);
bool sensorsRd(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len);
int16_t sensorsLe16s(const uint8_t *p);
uint16_t sensorsLe16u(const uint8_t *p);

void sensorsEnvReset();
bool sensorsEnvBmpInitAt(uint8_t addr);
bool sensorsEnvProbeAls(uint8_t *sdaOut, uint8_t *sclOut);
bool sensorsEnvProbeBmeBusA();
bool sensorsEnvBmpOk();
bool sensorsEnvAlsOk();
bool sensorsEnvHumOk();
void sensorsEnvSetPins(uint8_t sdaBmp, uint8_t sclBmp);
void sensorsEnvRead(EnvReading *out, bool magOk);

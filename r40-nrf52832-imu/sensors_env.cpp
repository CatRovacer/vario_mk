#include "sensors_env.h"
#include <math.h>

static uint8_t g_bmp = 0x76;
static uint8_t g_sda_bmp = 11, g_scl_bmp = 12;
static uint8_t g_sda_als = 19, g_scl_als = 21;
static bool g_bmp_ok = false, g_als = false, g_bme_hum = false;

static uint16_t dig_T1;
static int16_t dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static uint8_t dig_H1, dig_H3;
static int16_t dig_H2, dig_H4, dig_H5;
static int8_t dig_H6;
static int32_t t_fine;

void sensorsEnvReset() {
  g_bmp_ok = g_als = g_bme_hum = false;
  g_bmp = 0x76;
  g_sda_bmp = 11;
  g_scl_bmp = 12;
  g_sda_als = 19;
  g_scl_als = 21;
}

bool sensorsEnvBmpOk() { return g_bmp_ok; }
bool sensorsEnvAlsOk() { return g_als; }
bool sensorsEnvHumOk() { return g_bme_hum; }

void sensorsEnvSetPins(uint8_t sdaBmp, uint8_t sclBmp) {
  g_sda_bmp = sdaBmp;
  g_scl_bmp = sclBmp;
}

static bool bmpReadCal() {
  uint8_t c[24];
  if (!sensorsRd(g_bmp, 0x88, c, 24)) return false;
  dig_T1 = sensorsLe16u(&c[0]);
  dig_T2 = sensorsLe16s(&c[2]);
  dig_T3 = sensorsLe16s(&c[4]);
  dig_P1 = sensorsLe16u(&c[6]);
  dig_P2 = sensorsLe16s(&c[8]);
  dig_P3 = sensorsLe16s(&c[10]);
  dig_P4 = sensorsLe16s(&c[12]);
  dig_P5 = sensorsLe16s(&c[14]);
  dig_P6 = sensorsLe16s(&c[16]);
  dig_P7 = sensorsLe16s(&c[18]);
  dig_P8 = sensorsLe16s(&c[20]);
  dig_P9 = sensorsLe16s(&c[22]);
  return dig_T1 != 0 && dig_T1 != 0xFFFF;
}

static bool bmeReadHumCal() {
  uint8_t h1 = 0, h[7];
  if (!sensorsRd(g_bmp, 0xA1, &h1, 1)) return false;
  if (!sensorsRd(g_bmp, 0xE1, h, 7)) return false;
  dig_H1 = h1;
  dig_H2 = sensorsLe16s(&h[0]);
  dig_H3 = h[2];
  dig_H4 = (int16_t)(((int8_t)h[3] * 16) | (h[4] & 0x0F));
  dig_H5 = (int16_t)(((int8_t)h[5] * 16) | (h[4] >> 4));
  dig_H6 = (int8_t)h[6];
  return true;
}

static float bmeHumidityPct(int32_t adc_H) {
  int32_t v = t_fine - (int32_t)76800;
  v = (((((adc_H << 14) - ((int32_t)dig_H4 << 20) - ((int32_t)dig_H5 * v)) + 16384) >> 15) *
       (((((((v * (int32_t)dig_H6) >> 10) * (((v * (int32_t)dig_H3) >> 11) + 32768)) >> 10) +
          2097152) *
             (int32_t)dig_H2 +
         8192) >>
        14));
  v = v - (((((v >> 15) * (v >> 15)) >> 7) * (int32_t)dig_H1) >> 4);
  if (v < 0) v = 0;
  if (v > 419430400) v = 419430400;
  return (v >> 12) / 1024.0f;
}

bool sensorsEnvBmpInitAt(uint8_t addr) {
  uint8_t id = 0;
  if (!sensorsRd(addr, 0xD0, &id, 1)) return false;
  if (id != 0x58 && id != 0x60) return false;
  g_bmp = addr;
  g_bme_hum = (id == 0x60);
  sensorsWr(addr, 0xE0, 0xB6);
  delay(20);
  if (!bmpReadCal()) return false;
  if (g_bme_hum && !bmeReadHumCal()) g_bme_hum = false;
  if (g_bme_hum) sensorsWr(addr, 0xF2, 0x01);
  sensorsWr(addr, 0xF4, 0x27); // T/P path same as BMP280
  sensorsWr(addr, 0xF5, 0x00);
  g_bmp_ok = true;
  return true;
}

static bool ap3216Init() {
  if (!sensorsWr(0x1E, 0x00, 0x04)) return false;
  delay(50);
  if (!sensorsWr(0x1E, 0x00, 0x03)) return false;
  delay(20);
  uint8_t mode = 0;
  return sensorsRd(0x1E, 0x00, &mode, 1) && (mode & 0x03) == 0x03;
}

bool sensorsEnvProbeAls(uint8_t *sdaOut, uint8_t *sclOut) {
  // Never probe P0.25/27 — those conflict with MX25 MOSI (25) and RGB (27).
  const uint8_t pairs[][2] = {{19, 21}, {21, 19}};
  for (uint8_t i = 0; i < 2; i++) {
    sensorsBusBegin(pairs[i][0], pairs[i][1]);
    if (ap3216Init()) {
      g_als = true;
      g_sda_als = pairs[i][0];
      g_scl_als = pairs[i][1];
      if (sdaOut) *sdaOut = g_sda_als;
      if (sclOut) *sclOut = g_scl_als;
      if (!g_bmp_ok && (sensorsEnvBmpInitAt(0x76) || sensorsEnvBmpInitAt(0x77))) {
        g_sda_bmp = g_sda_als;
        g_scl_bmp = g_scl_als;
      }
      return true;
    }
  }
  return false;
}

bool sensorsEnvProbeBmeBusA() {
  // BME280 (id 0x60) on Bus A — may coexist with BMP280 on Bus B.
  const uint8_t pairs[][2] = {{19, 21}, {21, 19}};
  const uint8_t addrs[] = {0x76, 0x77};
  for (uint8_t p = 0; p < 2; p++) {
    sensorsBusBegin(pairs[p][0], pairs[p][1]);
    for (uint8_t i = 0; i < 2; i++) {
      uint8_t id = 0;
      if (!sensorsRd(addrs[i], 0xD0, &id, 1) || id != 0x60) continue;
      // Prefer BME for T/P/H when present (replaces BMP path).
      if (sensorsEnvBmpInitAt(addrs[i])) {
        g_sda_bmp = pairs[p][0];
        g_scl_bmp = pairs[p][1];
        return true;
      }
    }
  }
  return false;
}

static float bmpTempC(int32_t adc_T) {
  int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
                  ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  return ((t_fine * 5 + 128) >> 8) / 100.0f;
}

static float bmpPressurePa(int32_t adc_P) {
  int64_t var1 = ((int64_t)t_fine) - 128000;
  int64_t var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
  if (var1 == 0) return 0;
  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
  return (float)p / 256.0f;
}

void sensorsEnvRead(EnvReading *out, bool magOk) {
  *out = {};
  out->humidityPct = NAN;
  out->bmpOk = g_bmp_ok;
  out->alsOk = g_als;
  out->magOk = magOk;
  out->humOk = false;
  if (!g_bmp_ok && !g_als) return;

  if (g_bmp_ok) {
    sensorsBusBegin(g_sda_bmp, g_scl_bmp);
    uint8_t d[8];
    uint8_t n = g_bme_hum ? 8 : 6;
    if (sensorsRd(g_bmp, 0xF7, d, n)) {
      int32_t adc_P = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | ((int32_t)d[2] >> 4);
      int32_t adc_T = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | ((int32_t)d[5] >> 4);
      out->tempC = bmpTempC(adc_T);
      out->pressurePa = bmpPressurePa(adc_P);
      if (g_bme_hum) {
        int32_t adc_H = ((int32_t)d[6] << 8) | (int32_t)d[7];
        out->humidityPct = bmeHumidityPct(adc_H);
        out->humOk = true;
      }
    }
  }
  if (g_als) {
    sensorsBusBegin(g_sda_als, g_scl_als);
    uint8_t a[2], p[2];
    if (sensorsRd(0x1E, 0x0C, a, 2)) {
      uint16_t als = (uint16_t)a[0] | ((uint16_t)a[1] << 8);
      out->lux = als * 0.35f;
    }
    if (sensorsRd(0x1E, 0x0E, p, 2)) {
      out->proximity = (uint16_t)(((p[1] & 0x3F) << 4) | (p[0] & 0x0F));
    }
  }
  sensorsBeginImuBus();
}

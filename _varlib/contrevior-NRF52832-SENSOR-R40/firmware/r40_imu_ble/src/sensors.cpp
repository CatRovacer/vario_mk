#include "sensors.h"
#include "sensors_env.h"

// R40 VER:A (hardware analysis):
//   Bus B P0.11/P0.12 (+pullups): BMI160 + MAG3110 + BMP280
//   Bus A P0.19/P0.21:            BME280 + AP3216C
// Fallback: some units answered BMI on 13/14 previously.
static const uint32_t I2C_HZ = 100000;

static uint8_t g_bmi = 0x69;
static uint8_t g_sda_imu = 11, g_scl_imu = 12;
static uint8_t g_sda_mag = 11, g_scl_mag = 12;
static bool g_imu = false, g_mag = false;
static bool g_mag_aux = false; // MAG via BMI160 secondary I2C
static bool g_mag_sep = false; // MAG on different Wire pins than BMI
static uint8_t g_cur_sda = 0xFF, g_cur_scl = 0xFF;
static uint8_t g_scan_imu = 0, g_scan_env = 0, g_scan_imu0 = 0, g_scan_env0 = 0;

bool sensorsWr(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool sensorsRd(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)len) != (int)len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static bool wr(uint8_t addr, uint8_t reg, uint8_t val) { return sensorsWr(addr, reg, val); }
static bool rd(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
  return sensorsRd(addr, reg, buf, len);
}

int16_t sensorsLe16s(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static int16_t le16s(const uint8_t *p) { return sensorsLe16s(p); }
static int16_t be16s(const uint8_t *p) {
  return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}
uint16_t sensorsLe16u(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

void sensorsBusBegin(uint8_t sda, uint8_t scl) {
  if (g_cur_sda == sda && g_cur_scl == scl) return;
  Wire.end();
  Wire.setPins(sda, scl);
  Wire.begin();
  Wire.setClock(I2C_HZ);
  g_cur_sda = sda;
  g_cur_scl = scl;
}

static void busBegin(uint8_t sda, uint8_t scl) { sensorsBusBegin(sda, scl); }

void sensorsBeginImuBus() { busBegin(g_sda_imu, g_scl_imu); }

static void cfgSensorIntInputs() {
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(20, INPUT);
}

static bool magInit() {
  uint8_t id = 0;
  for (uint8_t t = 0; t < 8; t++) {
    if (rd(0x0E, 0x07, &id, 1) && id == 0xC4) break;
    delay(15);
    id = 0;
  }
  if (id != 0xC4) return false;
  wr(0x0E, 0x10, 0x00);
  delay(10);
  wr(0x0E, 0x11, 0x80);
  delay(15);
  wr(0x0E, 0x10, 0x01); // continuous ~80 Hz
  delay(20);
  return true;
}

// MAG3110 behind BMI160 auxiliary (ASDA/ASCL) interface
static bool magAuxWait() {
  for (uint8_t i = 0; i < 50; i++) {
    uint8_t st = 0;
    if (!rd(g_bmi, 0x03, &st, 1)) return false;
    // STATUS bit mag_man_op is in register 0x03? Actually STATUS 0x1B bit 2
    uint8_t s = 0;
    if (rd(g_bmi, 0x1B, &s, 1) && (s & 0x04) == 0) return true;
    delay(1);
  }
  return false;
}

static bool magAuxWrite(uint8_t reg, uint8_t val) {
  if (!wr(g_bmi, 0x4F, val)) return false;  // MAG_IF[4] write data
  if (!wr(g_bmi, 0x4E, reg)) return false; // MAG_IF[3] write addr (triggers)
  return magAuxWait();
}

static bool magAuxRead(uint8_t reg, uint8_t *buf, uint8_t len) {
  if (len == 0 || len > 8) return false;
  if (!wr(g_bmi, 0x4D, reg)) return false; // MAG_IF[2] read addr
  if (!magAuxWait()) return false;
  // Burst data lands in DATA regs 0x04..
  return rd(g_bmi, 0x04, buf, len);
}

static bool magInitAux() {
  // Enable secondary mag interface + mag normal PMU
  wr(g_bmi, 0x6B, 0x20);
  wr(g_bmi, 0x7E, 0x19);
  delay(10);
  uint8_t pmu = 0;
  rd(g_bmi, 0x03, &pmu, 1);
  // Mag PMU bits[1:0] should be 1 (normal) when aux powered
  wr(g_bmi, 0x4B, (uint8_t)(0x0E << 1));
  wr(g_bmi, 0x4C, 0x80); // manual en
  delay(5);

  uint8_t id = 0;
  for (uint8_t t = 0; t < 5; t++) {
    if (magAuxRead(0x07, &id, 1) && id == 0xC4) break;
    id = 0;
    delay(10);
  }
  if (id != 0xC4) {
    // Leave aux disabled so primary bus stays clean
    wr(g_bmi, 0x4C, 0x00);
    wr(g_bmi, 0x7E, 0x18); // mag suspend
    wr(g_bmi, 0x6B, 0x00);
    return false;
  }

  magAuxWrite(0x10, 0x00);
  delay(10);
  magAuxWrite(0x11, 0x80);
  delay(15);
  magAuxWrite(0x10, 0x01);
  delay(20);

  wr(g_bmi, 0x4D, 0x01);
  wr(g_bmi, 0x44, 0x08);
  wr(g_bmi, 0x4C, 0x02);
  delay(2);
  g_mag_aux = true;
  (void)pmu;
  return true;
}

static bool probeMagOnly() {
  const uint8_t pairs[][2] = {
      {11, 12}, {12, 11}, {13, 14}, {14, 13}, {19, 21}, {21, 19}};
  for (uint8_t i = 0; i < sizeof(pairs) / sizeof(pairs[0]); i++) {
    busBegin(pairs[i][0], pairs[i][1]);
    if (magInit()) {
      g_mag = true;
      g_sda_mag = pairs[i][0];
      g_scl_mag = pairs[i][1];
      g_mag_sep = !(g_sda_mag == g_sda_imu && g_scl_mag == g_scl_imu);
      return true;
    }
  }
  return false;
}

static bool bmiChipId(uint8_t addr) {
  uint8_t id = 0;
  return rd(addr, 0x00, &id, 1) && id == 0xD1;
}

static bool bmiPowerNormal(uint8_t addr) {
  // ODR 100 Hz (BMI has no 80); BLE notifies at 80 Hz to match MAG3110
  wr(addr, 0x40, 0x28);
  wr(addr, 0x41, 0x03);
  wr(addr, 0x42, 0x28);
  wr(addr, 0x43, 0x00);
  delay(1);
  wr(addr, 0x7E, 0x11);
  delay(10);
  wr(addr, 0x7E, 0x15);
  delay(85);
  wr(addr, 0x40, 0x28);
  wr(addr, 0x41, 0x03);
  wr(addr, 0x42, 0x28);
  wr(addr, 0x43, 0x00);
  uint8_t pmu = 0;
  if (!rd(addr, 0x03, &pmu, 1)) return false;
  return (pmu & 0x3C) == 0x14;
}

static bool bmiInitAt(uint8_t addr) {
  if (!bmiChipId(addr)) return false;
  if (bmiPowerNormal(addr)) return true;
  wr(addr, 0x7E, 0xB6);
  delay(150);
  if (!bmiChipId(addr)) return false;
  for (uint8_t i = 0; i < 3; i++) {
    if (bmiPowerNormal(addr)) return true;
    delay(20);
  }
  return bmiChipId(addr);
}

static uint8_t i2cScan(uint8_t *found, uint8_t maxN) {
  uint8_t n = 0;
  for (uint8_t a = 0x03; a <= 0x77; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      if (n < maxN) found[n++] = a;
    }
  }
  return n;
}

static bool tryImuPair(uint8_t sda, uint8_t scl) {
  g_sda_imu = sda;
  g_scl_imu = scl;
  sensorsBeginImuBus();

  bool mag = magInit();
  bool imu = false;
  const uint8_t addrs[] = {0x69, 0x68};
  for (uint8_t i = 0; i < 2; i++) {
    if (bmiInitAt(addrs[i])) {
      g_bmi = addrs[i];
      imu = true;
      break;
    }
  }
  if (!imu && !mag) return false;

  g_imu = imu;
  if (mag) {
    g_mag = true;
    g_sda_mag = sda;
    g_scl_mag = scl;
    g_mag_sep = false;
  } else {
    g_mag = false;
  }

  if (!sensorsEnvBmpOk() && (sensorsEnvBmpInitAt(0x76) || sensorsEnvBmpInitAt(0x77))) {
    sensorsEnvSetPins(sda, scl);
  }
  return g_imu || g_mag;
}

static bool probeImuBus() {
  const uint8_t pairs[][2] = {{11, 12}, {12, 11}, {13, 14}, {14, 13}};
  for (uint8_t p = 0; p < 4; p++) {
    g_imu = false;
    g_mag = false;
    if (tryImuPair(pairs[p][0], pairs[p][1]) && g_imu) return true;
  }
  for (uint8_t p = 0; p < 4; p++) {
    if (tryImuPair(pairs[p][0], pairs[p][1])) return true;
  }
  return false;
}

bool sensorsInitAll() {
  g_imu = g_mag = false;
  g_mag_aux = g_mag_sep = false;
  g_cur_sda = g_cur_scl = 0xFF;
  sensorsEnvReset();
  cfgSensorIntInputs();

  probeImuBus();

  // Scan Bus B BEFORE aux experiments (aux can glitch the bus)
  uint8_t found[8];
  sensorsBeginImuBus();
  g_scan_imu = i2cScan(found, 8);
  g_scan_imu0 = g_scan_imu ? found[0] : 0;
  for (uint8_t i = 0; i < g_scan_imu; i++) {
    if (found[i] == 0x0E) {
      g_scan_imu0 = 0x0E;
      break;
    }
  }

  if (!g_mag) probeMagOnly();
  if (!g_mag && g_imu) {
    sensorsBeginImuBus();
    if (magInitAux()) g_mag = true;
  }
  sensorsEnvProbeAls(nullptr, nullptr);
  if (!sensorsEnvHumOk()) sensorsEnvProbeBmeBusA();

  sensorsBeginImuBus();
  if (g_imu) bmiPowerNormal(g_bmi);
  if (g_mag && !g_mag_aux) {
    if (g_mag_sep) busBegin(g_sda_mag, g_scl_mag);
    else sensorsBeginImuBus();
    magInit();
  }

  busBegin(19, 21);
  g_scan_env = i2cScan(found, 8);
  g_scan_env0 = g_scan_env ? found[0] : 0;

  sensorsBeginImuBus();
  return g_imu || g_mag || sensorsEnvBmpOk() || sensorsEnvAlsOk();
}

bool sensorsHasMag() { return g_mag; }

void sensorsGetScanDiag(uint8_t *nImu, uint8_t *aImu, uint8_t *nEnv, uint8_t *aEnv) {
  *nImu = g_scan_imu;
  *aImu = g_scan_imu0;
  *nEnv = g_scan_env;
  *aEnv = g_scan_env0;
}

void sensorsReadImu(ImuReading *out) {
  *out = {};
  out->imuOk = g_imu;
  out->magOk = g_mag;
  if (!g_imu && !g_mag) return;
  sensorsBeginImuBus();

  if (g_imu) {
    uint8_t raw[12];
    if (rd(g_bmi, 0x0C, raw, 12)) {
      out->gx = le16s(&raw[0]) / 16.4f;
      out->gy = le16s(&raw[2]) / 16.4f;
      out->gz = le16s(&raw[4]) / 16.4f;
      out->ax = le16s(&raw[6]) / 16384.0f;
      out->ay = le16s(&raw[8]) / 16384.0f;
      out->az = le16s(&raw[10]) / 16384.0f;
    }
  }
  if (g_mag) {
    uint8_t m[6];
    bool ok = false;
    if (g_mag_aux) {
      ok = rd(g_bmi, 0x04, m, 6);
    } else {
      if (g_mag_sep) busBegin(g_sda_mag, g_scl_mag);
      ok = rd(0x0E, 0x01, m, 6);
      if (g_mag_sep) sensorsBeginImuBus();
    }
    if (ok) {
      out->mx = be16s(&m[0]) * 0.1f;
      out->my = be16s(&m[2]) * 0.1f;
      out->mz = be16s(&m[4]) * 0.1f;
    }
  }
}

void sensorsReadEnv(EnvReading *out) { sensorsEnvRead(out, g_mag); }

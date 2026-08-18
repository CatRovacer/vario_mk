#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <bluefruit.h>
#include "sensors.h"
#include "flash_log.h"
#include "buttons.h"
#include "nfc_tag.h"
#include "nrf.h"
#include "nrf_gpio.h"

// Live-verified map: R=27 G=28 B=29 (common-anode). P0.30 = flash CS — not LED.
static const uint8_t LED_R = 27, LED_G = 28, LED_B = 29;
static const uint8_t FLASH_CS = 30, FLASH_MOSI = 25, FLASH_SCK = 26, FLASH_MISO = 31;
static const char *DEVICE_NAME = "R40_IMU";

static uint16_t g_sample_rate_hz = 100;
static uint32_t g_sample_period_ms = 10;

BLEService imuService("00001523-1212-EFDE-1523-785FEABCD123");
BLECharacteristic imuDataChar("00001524-1212-EFDE-1523-785FEABCD123");
BLECharacteristic imuCtrlChar("00001525-1212-EFDE-1523-785FEABCD123");
BLECharacteristic envDataChar("00001526-1212-EFDE-1523-785FEABCD123");
BLECharacteristic rgbChar("00001527-1212-EFDE-1523-785FEABCD123");
// Flash log cmd / notify (P0 logger)
BLECharacteristic flashCmdChar("00001528-1212-EFDE-1523-785FEABCD123");
BLECharacteristic flashNotifyChar("00001529-1212-EFDE-1523-785FEABCD123");
BLEBas blebas;

volatile bool streamEnabled = true;
uint32_t lastSampleMs = 0, sampleIndex = 0, lastBattMs = 0;
volatile uint8_t g_rgb_r = 0, g_rgb_g = 0, g_rgb_b = 0;
volatile bool g_rgb_dirty = false;

static void putI16(uint8_t *o, int16_t v) {
  o[0] = (uint8_t)(v & 0xFF);
  o[1] = (uint8_t)((v >> 8) & 0xFF);
}
static void putU16(uint8_t *o, uint16_t v) {
  o[0] = (uint8_t)(v & 0xFF);
  o[1] = (uint8_t)((v >> 8) & 0xFF);
}
static void putU32(uint8_t *o, uint32_t v) {
  o[0] = (uint8_t)(v & 0xFF);
  o[1] = (uint8_t)((v >> 8) & 0xFF);
  o[2] = (uint8_t)((v >> 16) & 0xFF);
  o[3] = (uint8_t)((v >> 24) & 0xFF);
}
static int16_t clampI16(long v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

static void rgbCfgDrive(uint8_t pin) {
  nrf_gpio_cfg(pin, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
               NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
}

static void setRgb(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t m = r;
  if (g > m) m = g;
  if (b > m) m = b;
  bool onR = (m > 0) && (r >= g) && (r >= b);
  bool onG = (m > 0) && !onR && (g >= b);
  bool onB = (m > 0) && !onR && !onG;
  rgbCfgDrive(LED_R);
  rgbCfgDrive(LED_G);
  rgbCfgDrive(LED_B);
  nrf_gpio_pin_write(LED_R, onR ? 0 : 1);
  nrf_gpio_pin_write(LED_G, onG ? 0 : 1);
  nrf_gpio_pin_write(LED_B, onB ? 0 : 1);
}

static void rgbInit() {
  pinMode(FLASH_CS, OUTPUT);
  digitalWrite(FLASH_CS, HIGH);
  setRgb(0, 0, 0);
}

static uint16_t readVddMv() {
  volatile int16_t result = 0;
  NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit << SAADC_RESOLUTION_VAL_Pos;
  NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos;
  NRF_SAADC->CH[0].CONFIG =
      (SAADC_CH_CONFIG_GAIN_Gain1_6 << SAADC_CH_CONFIG_GAIN_Pos) |
      (SAADC_CH_CONFIG_MODE_SE << SAADC_CH_CONFIG_MODE_Pos) |
      (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
      (SAADC_CH_CONFIG_TACQ_10us << SAADC_CH_CONFIG_TACQ_Pos) |
      (SAADC_CH_CONFIG_BURST_Disabled << SAADC_CH_CONFIG_BURST_Pos);
  NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_VDD << SAADC_CH_PSELP_PSELP_Pos;
  NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC << SAADC_CH_PSELN_PSELN_Pos;
  NRF_SAADC->RESULT.PTR = (uint32_t)&result;
  NRF_SAADC->RESULT.MAXCNT = 1;
  NRF_SAADC->EVENTS_STARTED = 0;
  NRF_SAADC->EVENTS_END = 0;
  NRF_SAADC->EVENTS_STOPPED = 0;
  NRF_SAADC->TASKS_START = 1;
  while (!NRF_SAADC->EVENTS_STARTED) {
  }
  NRF_SAADC->EVENTS_STARTED = 0;
  NRF_SAADC->TASKS_SAMPLE = 1;
  while (!NRF_SAADC->EVENTS_END) {
  }
  NRF_SAADC->EVENTS_END = 0;
  NRF_SAADC->TASKS_STOP = 1;
  while (!NRF_SAADC->EVENTS_STOPPED) {
  }
  NRF_SAADC->EVENTS_STOPPED = 0;
  NRF_SAADC->ENABLE = 0;
  if (result < 0) result = 0;
  return (uint16_t)((result * 3600L) / 4095);
}

static uint8_t cr2032Percent(uint16_t mv) {
  if (mv >= 3000) return 100;
  if (mv <= 2000) return 0;
  return (uint8_t)(((mv - 2000) * 100) / 1000);
}

static void packImu(uint8_t out[20], const ImuReading &r) {
  putI16(&out[0], clampI16(lroundf(r.ax * 1000.0f)));
  putI16(&out[2], clampI16(lroundf(r.ay * 1000.0f)));
  putI16(&out[4], clampI16(lroundf(r.az * 1000.0f)));
  putI16(&out[6], clampI16(lroundf(r.gx * 100.0f)));
  putI16(&out[8], clampI16(lroundf(r.gy * 100.0f)));
  putI16(&out[10], clampI16(lroundf(r.gz * 100.0f)));
  putI16(&out[12], clampI16(lroundf(r.mx * 10.0f)));
  putI16(&out[14], clampI16(lroundf(r.my * 10.0f)));
  putI16(&out[16], clampI16(lroundf(r.mz * 10.0f)));
  putU16(&out[18], r.imuOk ? g_sample_rate_hz : 0);
}

// ENV 16 B: keep first 12, + humidity_x100 + events/status
static void packEnv(uint8_t out[16], const EnvReading &e) {
  putI16(&out[0], clampI16(lroundf(e.tempC * 100.0f)));
  putU32(&out[2], (uint32_t)lroundf(e.pressurePa));
  putU16(&out[6], (uint16_t)constrain(lroundf(e.lux), 0, 65535));
  putU16(&out[8], e.proximity);
  uint16_t flags = 0;
  if (e.bmpOk) flags |= 0x01;
  if (e.alsOk) flags |= 0x02;
  if (e.magOk) flags |= 0x04;
  if (flashLogOk()) flags |= 0x08;
  if (e.humOk) flags |= 0x10; // bit4 = BME humidity OK
  if (nfcTagOk()) flags |= 0x20; // bit5 = NFC Type2 active
  // Scan diag in bits 8..14 only — never clobber capability bits 0..5
  // (old code ORed first I2C addr into low byte → wiped RH/flash flags).
  if (!e.magOk) {
    uint8_t nImu = 0, aImu = 0, nEnv = 0, aEnv = 0;
    sensorsGetScanDiag(&nImu, &aImu, &nEnv, &aEnv);
    (void)aImu;
    (void)nEnv;
    (void)aEnv;
    flags = (uint16_t)((flags & 0x003F) | 0x8000 | ((nImu & 0x7F) << 8));
  }
  putU16(&out[10], flags);
  if (e.humOk && !isnan(e.humidityPct))
    putI16(&out[12], clampI16(lroundf(e.humidityPct * 100.0f)));
  else
    putI16(&out[12], (int16_t)0x7FFF); // absent sentinel
  uint16_t ev = buttonsTakeSticky();
  if (streamEnabled) ev |= 0x04;
  if (flashLogIsLogging()) ev |= 0x08;
  if (flashLogOk()) ev |= 0x10;
  if (nfcTagOk()) ev |= 0x20;
  else ev |= (uint16_t)((nfcTagFail() & 0x07) << 8); // fail stage in bits 8..10
  putU16(&out[14], ev);
}

static void onControlWrite(uint16_t, BLECharacteristic *, uint8_t *data, uint16_t len) {
  if (len >= 1) streamEnabled = (data[0] == 0x01);
}

static void onRgbWrite(uint16_t, BLECharacteristic *, uint8_t *data, uint16_t len) {
  if (len < 3) return;
  g_rgb_r = data[0];
  g_rgb_g = data[1];
  g_rgb_b = data[2];
  g_rgb_dirty = true;
}

static void onFlashCmdWrite(uint16_t, BLECharacteristic *, uint8_t *data, uint16_t len) {
  if (len < 4) return;
  flashLogQueueCmd(data); // SPI in loop — avoid mid-I2C
}

static void onConnect(uint16_t conn_hdl) {
  BLEConnection *conn = Bluefruit.Connection(conn_hdl);
  if (conn) conn->requestConnectionParameter(6, 12);
  uint8_t pct = cr2032Percent(readVddMv());
  blebas.write(pct);
  blebas.notify(pct);
  setRgb(0, 0, 0);
}

// SoftDevice is already up when setup() runs, so NVMC UICR writes often no-op.
// Prefer post-flash: nrfjprog --memwr 0x1000120C --val 0xFFFFFFFE
static void ensureNfcPinsOrReset() {
  if ((NRF_UICR->NFCPINS & UICR_NFCPINS_PROTECT_Msk) ==
      (UICR_NFCPINS_PROTECT_NFC << UICR_NFCPINS_PROTECT_Pos))
    return;
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
  }
  NRF_UICR->NFCPINS = ~UICR_NFCPINS_PROTECT_Msk; // clear protect → NFC mode
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
  }
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
  }
  // Do not SystemReset here — SoftDevice may ignore UICR write; use nrfjprog.
}

void setup() {
  ensureNfcPinsOrReset(); // before SoftDevice; one-shot UICR fix + reboot
  rgbInit();
  flashLogBegin(FLASH_CS, FLASH_MOSI, FLASH_SCK, FLASH_MISO);
  flashLogProbe(); // before I2C — MOSI shares P0.25 with ALS alt probe
  buttonsBegin();

  Bluefruit.configPrphBandwidth(BANDWIDTH_HIGH);
  Bluefruit.begin();
  nfcTagBegin(); // after SoftDevice; Type2 NDEF text "R40_IMU" on P0.09/10
  Bluefruit.setTxPower(4);
  Bluefruit.setName(DEVICE_NAME);
  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setConnInterval(6, 12);

  blebas.begin();
  blebas.write(cr2032Percent(readVddMv()));

  imuService.begin();

  imuDataChar.setProperties(CHR_PROPS_NOTIFY);
  imuDataChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  imuDataChar.setFixedLen(20);
  imuDataChar.begin();

  imuCtrlChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  imuCtrlChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  imuCtrlChar.setFixedLen(1);
  imuCtrlChar.setWriteCallback(onControlWrite);
  imuCtrlChar.begin();

  envDataChar.setProperties(CHR_PROPS_NOTIFY);
  envDataChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  envDataChar.setFixedLen(16);
  envDataChar.begin();

  rgbChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  rgbChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  rgbChar.setFixedLen(3);
  rgbChar.setWriteCallback(onRgbWrite);
  rgbChar.begin();

  flashCmdChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  flashCmdChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  flashCmdChar.setFixedLen(4);
  flashCmdChar.setWriteCallback(onFlashCmdWrite);
  flashCmdChar.begin();

  flashNotifyChar.setProperties(CHR_PROPS_NOTIFY);
  flashNotifyChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  flashNotifyChar.setMaxLen(20);
  flashNotifyChar.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(imuService);
  Bluefruit.Advertising.addService(blebas);
  Bluefruit.Advertising.addName();
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(160, 160);
  Bluefruit.Advertising.setFastTimeout(0);
  Bluefruit.Advertising.start(0);

  bool ok = sensorsInitAll();
  // Re-claim SPI after I2C probes; JEDEC may have failed if pins were contested.
  flashLogProbe();
  if (!nfcTagOk()) nfcTagBegin();
  if (sensorsHasMag()) {
    g_sample_rate_hz = 80;
    g_sample_period_ms = 12;
  } else {
    g_sample_rate_hz = 100;
    g_sample_period_ms = 10;
  }
  setRgb(220, 0, 0);
  delay(250);
  setRgb(0, 220, 0);
  delay(250);
  setRgb(0, 0, 220);
  delay(250);
  setRgb(0, 0, 0);
  if (ok) setRgb(0, 40, 0);
  if (flashLogOk()) {
    delay(120);
    setRgb(0, 80, 120);
    delay(180);
    setRgb(ok ? 0 : 0, ok ? 40 : 0, 0);
  }
  streamEnabled = true;
}

void loop() {
  // Keep the Type 2 tag in SENSE after a phone removes its RF field.
  nfcTagPoll();
  buttonsPoll();
  ButtonEvents be = buttonsLast();
  if (be.key0Edge) streamEnabled = !streamEnabled;
  // KEY1 mark already in sticky bits → ENV status word

  uint8_t br, bg, bb;
  if (buttonsBlinkActive(&br, &bg, &bb)) {
    setRgb(br, bg, bb);
  } else if (g_rgb_dirty) {
    g_rgb_dirty = false;
    setRgb(g_rgb_r, g_rgb_g, g_rgb_b);
  }

  uint8_t flashReply[20];
  uint8_t flashLen = 0;
  if (flashLogPoll(flashReply, &flashLen) && flashNotifyChar.notifyEnabled()) {
    flashNotifyChar.notify(flashReply, flashLen);
  }

  if (!Bluefruit.connected()) {
    delay(5);
    return;
  }

  const uint32_t now = millis();
  if (now - lastBattMs >= 5000) {
    lastBattMs = now;
    uint8_t pct = cr2032Percent(readVddMv());
    blebas.write(pct);
    blebas.notify(pct);
  }

  if (!streamEnabled) {
    delay(5);
    return;
  }
  const bool imuNotify = imuDataChar.notifyEnabled();
  const bool envNotify = envDataChar.notifyEnabled();
  if (!imuNotify && !envNotify && !flashLogIsLogging()) {
    delay(5);
    return;
  }
  if (now - lastSampleMs < g_sample_period_ms) return;
  lastSampleMs = now;
  sampleIndex++;

  uint8_t imuPkt[20];
  bool haveImu = false;
  if (imuNotify || flashLogIsLogging()) {
    ImuReading imu;
    sensorsReadImu(&imu);
    packImu(imuPkt, imu);
    haveImu = true;
    if (imuNotify) imuDataChar.notify(imuPkt, sizeof(imuPkt));
  }

  EnvReading env = {};
  bool haveEnv = false;
  if ((envNotify && (sampleIndex % 10) == 0) || flashLogIsLogging()) {
    sensorsReadEnv(&env);
    haveEnv = true;
    if (envNotify && (sampleIndex % 10) == 0) {
      uint8_t pkt[16];
      packEnv(pkt, env);
      envDataChar.notify(pkt, sizeof(pkt));
    }
  }

  if (haveImu && flashLogIsLogging()) {
    uint8_t rec[32];
    memcpy(rec, imuPkt, 20);
    // ENV compact 12 B: temp, press, lux, prox, humidity (no flags/events)
    if (haveEnv) {
      putI16(&rec[20], clampI16(lroundf(env.tempC * 100.0f)));
      putU32(&rec[22], (uint32_t)lroundf(env.pressurePa));
      putU16(&rec[26], (uint16_t)constrain(lroundf(env.lux), 0, 65535));
      putU16(&rec[28], env.proximity);
      if (env.humOk && !isnan(env.humidityPct))
        putI16(&rec[30], clampI16(lroundf(env.humidityPct * 100.0f)));
      else
        putI16(&rec[30], (int16_t)0x7FFF);
    } else {
      memset(&rec[20], 0, 12);
      putI16(&rec[30], (int16_t)0x7FFF);
    }
    flashLogMaybeAppend(rec);
  }
}

// BLE OTA DFU: NOT implemented (SoftDevice S132 + dual-bank bootloader is a
// large separate deliverable). Field updates still use SWD / nrfjprog for now.
// Future: Adafruit nRF52 bootloader OTA or Nordic secure DFU service.

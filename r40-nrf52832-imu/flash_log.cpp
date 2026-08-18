#include "flash_log.h"
#include <SPI.h>
#include <string.h>

static const uint32_t MAGIC = 0x5234304Cul; // 'R40L'
static const uint32_t HDR_ADDR = 0;
static const uint32_t DATA_START = 0x1000;    // after header sector
static const uint32_t DATA_END = 0x100000;    // full MX25R80 1 MB
static const uint16_t PAGE = 256;
static const uint16_t SECTOR = 4096;
static const uint16_t REC = 32; // IMU20 + ENV12 (temp/press/lux/prox/hum) — divides PAGE
static const uint32_t MAX_SAMPLES = (DATA_END - DATA_START) / REC; // ~32640

static uint8_t g_cs = 30, g_mosi = 25, g_sck = 26, g_miso = 31;
static bool g_ok = false;
static uint32_t g_jedec = 0;
static bool g_logging = false;
static uint32_t g_writePtr = DATA_START;
static uint32_t g_count = 0;
static uint8_t g_skip = 0;
static uint8_t g_downsample = 1; // 1 → 100 Hz log (max)
static uint8_t g_logRateHz = 100;

static bool g_cmdPend = false;
static uint8_t g_cmd[4];
static bool g_replyPend = false;
static uint8_t g_reply[20];
static uint8_t g_replyLen = 0;

#pragma pack(push, 1)
struct LogHeader {
  uint32_t magic;
  uint32_t writePtr;
  uint32_t count;
  uint16_t flags; // bit0=active
  uint16_t logRateHz;
};
#pragma pack(pop)

static void applyLogRateHz(uint8_t hz) {
  // Discrete rates that divide 100 Hz loop.
  uint8_t rate = 100;
  uint8_t ds = 1;
  if (hz == 0) return; // keep current
  if (hz >= 100) {
    rate = 100;
    ds = 1;
  } else if (hz >= 50) {
    rate = 50;
    ds = 2;
  } else if (hz >= 25) {
    rate = 25;
    ds = 4;
  } else if (hz >= 20) {
    rate = 20;
    ds = 5;
  } else {
    rate = 10;
    ds = 10;
  }
  g_logRateHz = rate;
  g_downsample = ds;
}

static void csLow() { digitalWrite(g_cs, LOW); }
static void csHigh() { digitalWrite(g_cs, HIGH); }

static void spiClaimPins() {
  // Wire may have stolen P0.25 — reclaim before every SPI txn.
  // Adafruit nRF52: setPins(MISO, SCK, MOSI) — not Arduino AVR order.
  SPI.setPins(g_miso, g_sck, g_mosi);
  pinMode(g_cs, OUTPUT);
  digitalWrite(g_cs, HIGH);
}

static void spiBeginTxn() {
  spiClaimPins();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
}

static void spiEndTxn() {
  SPI.endTransaction();
  csHigh();
}

static uint8_t readStatus() {
  spiBeginTxn();
  csLow();
  SPI.transfer(0x05);
  uint8_t s = SPI.transfer(0);
  csHigh();
  spiEndTxn();
  return s;
}

static void waitReady() {
  for (uint16_t i = 0; i < 5000; i++) {
    if ((readStatus() & 0x01) == 0) return;
    delay(1);
  }
}

static void writeEnable() {
  spiBeginTxn();
  csLow();
  SPI.transfer(0x06);
  csHigh();
  spiEndTxn();
}

bool flashLogBegin(uint8_t cs, uint8_t mosi, uint8_t sck, uint8_t miso) {
  g_cs = cs;
  g_mosi = mosi;
  g_sck = sck;
  g_miso = miso;
  pinMode(g_cs, OUTPUT);
  digitalWrite(g_cs, HIGH);
  SPI.setPins(g_miso, g_sck, g_mosi); // Adafruit: MISO, SCK, MOSI
  SPI.begin();
  return true;
}

bool flashLogProbe() {
  spiClaimPins();
  // Wake / release deep power-down (MX25R): RDID often needs a dummy clock first.
  spiBeginTxn();
  csLow();
  SPI.transfer(0xAB); // Release from DPD
  csHigh();
  spiEndTxn();
  delay(1);

  spiBeginTxn();
  csLow();
  SPI.transfer(0x9F);
  uint8_t mfr = SPI.transfer(0);
  uint8_t typ = SPI.transfer(0);
  uint8_t cap = SPI.transfer(0);
  csHigh();
  spiEndTxn();
  g_jedec = ((uint32_t)mfr << 16) | ((uint32_t)typ << 8) | cap;
  // Macronix MX25R80 JEDEC ≈ C2 28 14 (or similar); reject float lines
  g_ok = (mfr != 0x00 && mfr != 0xFF);
  if (g_ok) {
    LogHeader h = {};
    flashLogRead(HDR_ADDR, (uint8_t *)&h, sizeof(h));
    if (h.magic == MAGIC) {
      g_writePtr = h.writePtr;
      g_count = h.count;
      g_logging = (h.flags & 0x01) != 0;
      if (h.logRateHz >= 10 && h.logRateHz <= 100) applyLogRateHz((uint8_t)h.logRateHz);
      if (g_writePtr < DATA_START || g_writePtr >= DATA_END) g_writePtr = DATA_START;
    }
  }
  return g_ok;
}

bool flashLogOk() { return g_ok; }
uint32_t flashLogJedec() { return g_jedec; }
bool flashLogIsLogging() { return g_logging; }
uint8_t flashLogRecordSize() { return (uint8_t)REC; }

void flashLogEraseSector(uint32_t addr) {
  if (!g_ok) return;
  addr &= ~(uint32_t)(SECTOR - 1);
  waitReady();
  writeEnable();
  spiBeginTxn();
  csLow();
  SPI.transfer(0x20);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
  csHigh();
  spiEndTxn();
  waitReady();
}

bool flashLogWritePage(uint32_t addr, const uint8_t *data, uint16_t len) {
  if (!g_ok || len == 0 || len > PAGE) return false;
  if ((addr % PAGE) + len > PAGE) return false;
  waitReady();
  writeEnable();
  spiBeginTxn();
  csLow();
  SPI.transfer(0x02);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
  for (uint16_t i = 0; i < len; i++) SPI.transfer(data[i]);
  csHigh();
  spiEndTxn();
  waitReady();
  return true;
}

void flashLogRead(uint32_t addr, uint8_t *data, uint16_t len) {
  if (!g_ok || !data || len == 0) return;
  spiBeginTxn();
  csLow();
  SPI.transfer(0x03);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
  for (uint16_t i = 0; i < len; i++) data[i] = SPI.transfer(0);
  csHigh();
  spiEndTxn();
}

static void saveHeader() {
  LogHeader h = {};
  h.magic = MAGIC;
  h.writePtr = g_writePtr;
  h.count = g_count;
  h.flags = g_logging ? 0x01 : 0;
  h.logRateHz = g_logRateHz;
  flashLogEraseSector(HDR_ADDR);
  flashLogWritePage(HDR_ADDR, (const uint8_t *)&h, sizeof(h));
}

static void fillStatus(uint8_t *out, uint8_t *len) {
  out[0] = 0x03;
  out[1] = (uint8_t)((g_ok ? 0x01 : 0) | (g_logging ? 0x02 : 0));
  out[2] = (uint8_t)(g_writePtr & 0xFF);
  out[3] = (uint8_t)((g_writePtr >> 8) & 0xFF);
  out[4] = (uint8_t)((g_writePtr >> 16) & 0xFF);
  out[5] = (uint8_t)((g_writePtr >> 24) & 0xFF);
  out[6] = (uint8_t)(g_count & 0xFF);
  out[7] = (uint8_t)((g_count >> 8) & 0xFF);
  out[8] = (uint8_t)((g_count >> 16) & 0xFF);
  out[9] = (uint8_t)((g_count >> 24) & 0xFF);
  out[10] = (uint8_t)((g_jedec >> 16) & 0xFF);
  out[11] = (uint8_t)((g_jedec >> 8) & 0xFF);
  out[12] = (uint8_t)(g_jedec & 0xFF);
  out[13] = (uint8_t)REC; // record size for app JSON decode
  out[14] = g_logRateHz;   // flash log rate (10/20/25/50/100)
  *len = 15;
}

static uint8_t g_page[PAGE];
static uint16_t g_pageFill = 0;
static uint32_t g_pageAddr = DATA_START;

static void flushPage() {
  if (!g_ok || g_pageFill == 0) return;
  if ((g_pageAddr % SECTOR) == 0) flashLogEraseSector(g_pageAddr);
  // One Page-Program per page address (MX25: further PP to same page corrupts).
  flashLogWritePage(g_pageAddr, g_page, g_pageFill);
  g_pageAddr += PAGE;
  g_pageFill = 0;
  g_writePtr = g_pageAddr;
}

static void doEraseAll() {
  g_logging = false;
  // Full chip data region (~1 MB). Slow; used only by explicit erase op.
  for (uint32_t a = 0; a < DATA_END; a += SECTOR) flashLogEraseSector(a);
  g_writePtr = DATA_START;
  g_pageAddr = DATA_START;
  g_pageFill = 0;
  g_count = 0;
  saveHeader();
}

static void doReadChunk(uint32_t offset, uint8_t n) {
  if (n > 16) n = 16;
  uint32_t addr = DATA_START + offset;
  if (addr >= DATA_END) {
    n = 0;
  } else if (addr + n > DATA_END) {
    n = (uint8_t)(DATA_END - addr);
  }
  g_reply[0] = 0x05;
  g_reply[1] = (uint8_t)(offset & 0xFF);
  g_reply[2] = (uint8_t)((offset >> 8) & 0xFF);
  g_reply[3] = n;
  if (n > 0) flashLogRead(addr, &g_reply[4], n);
  g_replyLen = (uint8_t)(4 + n);
  g_replyPend = true;
}

void flashLogQueueCmd(const uint8_t cmd[4]) {
  memcpy(g_cmd, cmd, 4);
  g_cmdPend = true;
}

bool flashLogPoll(uint8_t reply[20], uint8_t *replyLen) {
  if (g_cmdPend) {
    g_cmdPend = false;
    if (!g_ok) {
      fillStatus(g_reply, &g_replyLen);
      g_reply[1] = 0;
      g_replyPend = true;
    } else {
      switch (g_cmd[0]) {
        case 0x01:
          // a = log rate Hz (0 = keep; else 10/20/25/50/100). Max = 100.
          applyLogRateHz(g_cmd[1]);
          g_logging = true;
          g_writePtr = DATA_START;
          g_pageAddr = DATA_START;
          g_pageFill = 0;
          g_count = 0;
          g_skip = 0;
          // Sector erase happens in flushPage (full 1 MB wipe on Record is too slow).
          saveHeader();
          fillStatus(g_reply, &g_replyLen);
          g_replyPend = true;
          break;
        case 0x02:
          g_logging = false;
          flushPage(); // one PP for partial last page
          saveHeader();
          fillStatus(g_reply, &g_replyLen);
          g_replyPend = true;
          break;
        case 0x03:
          fillStatus(g_reply, &g_replyLen);
          g_replyPend = true;
          break;
        case 0x04:
          doEraseAll();
          fillStatus(g_reply, &g_replyLen);
          g_replyPend = true;
          break;
        case 0x05: {
          // 24-bit byte offset into data region; chunk length fixed at 16.
          uint32_t off = (uint32_t)g_cmd[1] | ((uint32_t)g_cmd[2] << 8) |
                         ((uint32_t)g_cmd[3] << 16);
          doReadChunk(off, 16);
          break;
        }
        default:
          break;
      }
    }
  }
  if (!g_replyPend) return false;
  g_replyPend = false;
  memcpy(reply, g_reply, g_replyLen);
  *replyLen = g_replyLen;
  return true;
}

void flashLogMaybeAppend(const uint8_t rec[32]) {
  if (!g_ok || !g_logging || !rec) return;
  if (++g_skip < g_downsample) return;
  g_skip = 0;

  if (g_count >= MAX_SAMPLES) {
    g_logging = false;
    flushPage();
    saveHeader();
    return;
  }
  if (g_pageAddr >= DATA_END) {
    g_logging = false;
    flushPage();
    saveHeader();
    return;
  }

  if (g_pageFill + REC > PAGE) flushPage();
  if (g_pageAddr >= DATA_END) {
    g_logging = false;
    saveHeader();
    return;
  }
  memcpy(&g_page[g_pageFill], rec, REC);
  g_pageFill = (uint16_t)(g_pageFill + REC);
  g_count++;
  g_writePtr = g_pageAddr + g_pageFill;
  if (g_pageFill >= PAGE) flushPage();
  if ((g_count & 0x1F) == 0) saveHeader();
}

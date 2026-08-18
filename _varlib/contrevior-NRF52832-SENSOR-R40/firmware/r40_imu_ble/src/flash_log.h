#pragma once
#include <Arduino.h>

// Minimal MX25R80 ring logger (SPI). Talk SPI only outside I2C sample window.

bool flashLogBegin(uint8_t cs, uint8_t mosi, uint8_t sck, uint8_t miso);
bool flashLogProbe(); // JEDEC 0x9F; sets ok flag
bool flashLogOk();
uint32_t flashLogJedec();

void flashLogEraseSector(uint32_t addr);
bool flashLogWritePage(uint32_t addr, const uint8_t *data, uint16_t len);
void flashLogRead(uint32_t addr, uint8_t *data, uint16_t len);

// Queue BLE op (op,a,b,c); processed in flashLogPoll()
void flashLogQueueCmd(const uint8_t cmd[4]);
// Returns true if a notify payload is ready (status/data chunk, ≤20 B)
bool flashLogPoll(uint8_t reply[20], uint8_t *replyLen);

// Append 32-byte record (IMU20 + temp/press/lux/prox/hum) every Nth call when logging.
// Capacity: full MX25R80 (~1 MB). Log rate set by Record op byte a (10–100 Hz).
void flashLogMaybeAppend(const uint8_t rec[32]);
bool flashLogIsLogging();
uint8_t flashLogRecordSize(); // 32
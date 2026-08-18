#include "nfc_tag.h"
#include <string.h>
#include <nrf.h>
#include <nrf_nfct.h>
#include <nrf_soc.h>
#include <nrf_nvic.h>

// Read-only Type 2 Tag: UID + CC + NDEF Text "R40_IMU" (Android tap notify).

static const uint8_t kPageSize = 4;
static const uint8_t kPageCount = 16; // 64 B emulated memory
static const uint8_t kFirstData = 4;
static const uint8_t kNak = 0x00;
static const uint8_t kFrameMax = 64;
static const uint32_t kFieldLostSettleUs = 300;
static const uint32_t kFrameDelayMaxDefault = 0x00001000UL;

static uint8_t g_mem[kPageCount * kPageSize] __attribute__((aligned(4)));
static uint8_t g_frame[kFrameMax] __attribute__((aligned(4)));
static volatile bool g_ok = false;
static volatile bool g_sleep = false;
static volatile bool g_fieldActive = false;
static volatile bool g_rearmPending = false;
static volatile uint32_t g_rearmAtUs = 0;
static uint8_t g_fail = 0; // 1=pins 2=hfclk 3=mem 4=irq

bool nfcTagOk() { return g_ok; }
uint8_t nfcTagFail() { return g_fail; }

static bool pinsAreNfc() {
  // Absolute UICR NFCPINS @ 0x1000120C — bit0=0 means NFC mode.
  volatile uint32_t *nfcpins = (volatile uint32_t *)0x1000120CUL;
  return (*nfcpins & 1u) == 0u;
}

static bool hfclkRequest(uint32_t timeoutMs) {
  uint32_t running = 0;
  (void)sd_clock_hfclk_is_running(&running);
  if (running) return true;
  // With S132 enabled, direct CLOCK access is not permitted. Keep the
  // SoftDevice's reference until reset so NFCT always has the HFXO it needs.
  if (sd_clock_hfclk_request() != NRF_SUCCESS) return false;
  uint32_t t0 = millis();
  do {
    (void)sd_clock_hfclk_is_running(&running);
    if (running) return true;
  } while (millis() - t0 < timeoutMs);
  return false;
}

static void getId(uint8_t id[7]) {
  uint32_t h0 = NRF_FICR->NFC.TAGHEADER0;
  uint32_t h1 = NRF_FICR->NFC.TAGHEADER1;
  id[0] = (uint8_t)(h0 >> 0);
  id[1] = (uint8_t)(h0 >> 8);
  id[2] = (uint8_t)(h0 >> 16);
  id[3] = (uint8_t)(h1 >> 0);
  id[4] = (uint8_t)(h1 >> 8);
  id[5] = (uint8_t)(h1 >> 16);
  id[6] = (uint8_t)(h1 >> 24);
  // NFCID1 byte 3 cannot be the cascade tag (0x88).
  if (id[3] == 0x88) id[3] |= 0x11;
}

static void buildMemory() {
  // NDEF Text UTF-8 "en" + "R40_IMU" (short well-known record).
  static const uint8_t ndef[] = {
      0xD1, 0x01, 0x0A, 0x54, 0x02, 0x65, 0x6E,
      'R',  '4',  '0',  '_',  'I',  'M',  'U'};
  uint8_t id[7];
  getId(id);
  memset(g_mem, 0, sizeof(g_mem));
  g_mem[0] = id[0];
  g_mem[1] = id[1];
  g_mem[2] = id[2];
  g_mem[3] = (uint8_t)(id[0] ^ id[1] ^ id[2]); // Type2 BCC0 (no cascade 0x88)
  g_mem[4] = id[3];
  g_mem[5] = id[4];
  g_mem[6] = id[5];
  g_mem[7] = id[6];
  // page 2: lock/internal — not NTAG AUTH0 (0x48 broke Android NDEF path)
  g_mem[8] = 0x00;
  g_mem[9] = 0x00;
  g_mem[10] = 0x00;
  g_mem[11] = 0x00;
  g_mem[12] = 0xE1;
  g_mem[13] = 0x10;
  g_mem[14] = (uint8_t)(((kPageCount - kFirstData) * kPageSize) / 8);
  g_mem[15] = 0x0F; // read-only CC
  uint16_t o = kFirstData * kPageSize;
  g_mem[o++] = 0x03;
  g_mem[o++] = sizeof(ndef);
  memcpy(&g_mem[o], ndef, sizeof(ndef));
  o += sizeof(ndef);
  g_mem[o] = 0xFE;
}

static void startRx() {
  nrf_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_RXFRAMEEND);
  nrf_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_RXERROR);
  nrf_nfct_rx_frame_status_clear(NRF_NFCT, 0xFFFFFFFF);
  nrf_nfct_rxtx_buffer_set(NRF_NFCT, g_frame, sizeof(g_frame));
  nrf_nfct_rx_frame_config_set(
      NRF_NFCT, NRF_NFCT_RX_FRAME_CONFIG_PARITY | NRF_NFCT_RX_FRAME_CONFIG_SOF |
                    NRF_NFCT_RX_FRAME_CONFIG_CRC16);
  nrf_nfct_int_enable(NRF_NFCT, NRF_NFCT_INT_RXFRAMEEND_MASK | NRF_NFCT_INT_RXERROR_MASK);
  nrf_nfct_task_trigger(NRF_NFCT, NRF_NFCT_TASK_ENABLERXDATA);
}

static void scheduleSense() {
  g_rearmPending = true;
  g_rearmAtUs = micros() + kFieldLostSettleUs;
}

static void enterSense() {
  nrf_nfct_int_disable(NRF_NFCT, NRF_NFCT_INT_RXFRAMEEND_MASK | NRF_NFCT_INT_RXERROR_MASK |
                                     NRF_NFCT_INT_TXFRAMEEND_MASK);
  nrf_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_RXFRAMEEND);
  nrf_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_RXERROR);
  nrf_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_TXFRAMEEND);
  // Required after SLP_REQ on nRF52832 anomaly 218.
  nrf_nfct_frame_delay_max_set(NRF_NFCT, kFrameDelayMaxDefault);
  nrf_nfct_task_trigger(NRF_NFCT, NRF_NFCT_TASK_SENSE);
  g_fieldActive = false;
  g_sleep = false;
  g_rearmPending = false;
}

static void sendFrame(const uint8_t *data, uint16_t bits, bool crc) {
  uint16_t bytes = (uint16_t)((bits + 7) / 8);
  if (data != g_frame) memcpy(g_frame, data, bytes);
  uint8_t cfg = NRF_NFCT_TX_FRAME_CONFIG_SOF;
  if ((bits % 8) == 0)
    cfg |= NRF_NFCT_TX_FRAME_CONFIG_PARITY | NRF_NFCT_TX_FRAME_CONFIG_DISCARD_START;
  if (crc) cfg |= NRF_NFCT_TX_FRAME_CONFIG_CRC16;
  nrf_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_TXFRAMEEND);
  nrf_nfct_rxtx_buffer_set(NRF_NFCT, g_frame, sizeof(g_frame));
  nrf_nfct_tx_frame_config_set(NRF_NFCT, cfg);
  nrf_nfct_tx_bits_set(NRF_NFCT, bits);
  nrf_nfct_frame_delay_mode_set(NRF_NFCT, NRF_NFCT_FRAME_DELAY_MODE_WINDOWGRID);
  nrf_nfct_int_enable(NRF_NFCT, NRF_NFCT_INT_TXFRAMEEND_MASK);
  nrf_nfct_task_trigger(NRF_NFCT, NRF_NFCT_TASK_STARTTX);
}

static void sendRead(uint8_t page) {
  uint8_t rsp[16];
  uint16_t off = (uint16_t)((page % kPageCount) * kPageSize);
  for (uint8_t i = 0; i < 16; i++) rsp[i] = g_mem[(off + i) % sizeof(g_mem)];
  sendFrame(rsp, 128, true);
}

static void handleCmd(uint8_t *d, uint16_t len) {
  if (len == 0) {
    startRx();
    return;
  }
  switch (d[0]) {
  case 0x30: // READ
    if (len >= 2) sendRead(d[1]);
    else startRx();
    break;
  case 0x3A: // FAST_READ
    if (len >= 3 && d[2] >= d[1] && d[2] < kPageCount) {
      uint16_t n = (uint16_t)(d[2] - d[1] + 1) * kPageSize;
      if (n <= sizeof(g_frame)) {
        memcpy(g_frame, &g_mem[d[1] * kPageSize], n);
        sendFrame(g_frame, (uint16_t)(n * 8), true);
        break;
      }
    }
    sendFrame(&kNak, 4, false);
    break;
  case 0x60: { // GET_VERSION (NTAG-ish)
    static const uint8_t ver[8] = {0x00, 0x04, 0x04, 0x02, 0x01, 0x00, 0x0F, 0x03};
    sendFrame(ver, 64, true);
    break;
  }
  case 0x50: // HLTA
    g_sleep = true;
    nrf_nfct_int_disable(NRF_NFCT, NRF_NFCT_INT_RXFRAMEEND_MASK | NRF_NFCT_INT_RXERROR_MASK |
                                       NRF_NFCT_INT_TXFRAMEEND_MASK);
    nrf_nfct_task_trigger(NRF_NFCT, NRF_NFCT_TASK_GOSLEEP);
    break;
  default:
    sendFrame(&kNak, 4, false);
    break;
  }
}

static bool evt(nrf_nfct_event_t e, uint32_t mask) {
  return nrf_nfct_event_check(NRF_NFCT, e) && nrf_nfct_int_enable_check(NRF_NFCT, mask);
}

extern "C" void NFCT_IRQHandler(void) {
  if (evt(NRF_NFCT_EVENT_FIELDDETECTED, NRF_NFCT_INT_FIELDDETECTED_MASK)) {
    nrf_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_FIELDDETECTED);
    g_fieldActive = true;
  }
  if (evt(NRF_NFCT_EVENT_SELECTED, NRF_NFCT_INT_SELECTED_MASK)) {
    nrf_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_SELECTED);
    nrf_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_RXFRAMEEND);
    nrf_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_RXERROR);
    nrf_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_TXFRAMEEND);
    nrf_nfct_rx_frame_status_clear(NRF_NFCT, 0xFFFFFFFF);
    nrf_nfct_error_status_clear(NRF_NFCT, 0xFFFFFFFF);
    g_fieldActive = true;
    g_sleep = false;
    startRx();
  }
  if (evt(NRF_NFCT_EVENT_RXFRAMEEND, NRF_NFCT_INT_RXFRAMEEND_MASK)) {
    nrf_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_RXFRAMEEND);
    handleCmd(g_frame, (uint16_t)(nrf_nfct_rx_bits_get(NRF_NFCT, true) / 8));
  }
  if (evt(NRF_NFCT_EVENT_TXFRAMEEND, NRF_NFCT_INT_TXFRAMEEND_MASK)) {
    nrf_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_TXFRAMEEND);
    startRx();
  }
  if (evt(NRF_NFCT_EVENT_RXERROR, NRF_NFCT_INT_RXERROR_MASK)) {
    nrf_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_RXERROR);
    nrf_nfct_rx_frame_status_clear(NRF_NFCT, 0xFFFFFFFF);
    startRx();
  }
  if (evt(NRF_NFCT_EVENT_ERROR, NRF_NFCT_INT_ERROR_MASK)) {
    uint32_t st = NRF_NFCT->ERRORSTATUS;
    nrf_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_ERROR);
    nrf_nfct_error_status_clear(NRF_NFCT, 0xFFFFFFFF);
    if (st == NFCT_ERRORSTATUS_FRAMEDELAYTIMEOUT_Msk && g_sleep) g_sleep = false;
  }
  if (evt(NRF_NFCT_EVENT_FIELDLOST, NRF_NFCT_INT_FIELDLOST_MASK)) {
    nrf_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_FIELDLOST);
    // Do not use FIELDLOST->SENSE short: it can create a false FIELDDETECTED
    // on nRF52832. nfcTagPoll() rearms SENSE after the required settle time.
    scheduleSense();
  }
}

void nfcTagPoll() {
  if (!g_ok) return;
  // Polling covers revisions where FIELDLOST is absent or unreliable.
  if (g_fieldActive && (NRF_NFCT->FIELDPRESENT & NFCT_FIELDPRESENT_FIELDPRESENT_Msk) == 0)
    scheduleSense();
  if (g_rearmPending && (int32_t)(micros() - g_rearmAtUs) >= 0) enterSense();
}

bool nfcTagBegin() {
  g_ok = false;
  g_fail = 0;
  if (!pinsAreNfc()) {
    g_fail = 1;
    return false;
  }
  if (!hfclkRequest(500)) {
    g_fail = 2;
    return false;
  }

  buildMemory();
  // Structural self-check: TLV + terminator present.
  if (g_mem[16] != 0x03 || g_mem[17] != 14 || g_mem[18 + 14] != 0xFE) {
    g_fail = 3;
    return false;
  }

  uint8_t id[7];
  getId(id);
  nrf_nfct_int_disable(NRF_NFCT, NRF_NFCT_DISABLE_ALL_INT);
  nrf_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_FIELDDETECTED);
  nrf_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_FIELDLOST);
  nrf_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_SELECTED);
  nrf_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_ERROR);
  nrf_nfct_error_status_clear(NRF_NFCT, 0xFFFFFFFF);
  nrf_nfct_nfcid1_set(NRF_NFCT, id, NRF_NFCT_SENSRES_NFCID1_SIZE_DOUBLE);
  nrf_nfct_sensres_bit_frame_sdd_set(NRF_NFCT, NRF_NFCT_SENSRES_BIT_FRAME_SDD_00100);
  nrf_nfct_sensres_platform_config_set(NRF_NFCT, NRF_NFCT_SENSRES_PLATFORM_CONFIG_OTHER); // not T1T; SEL_RES=T2T
  nrf_nfct_selres_protocol_set(NRF_NFCT, NRF_NFCT_SELRES_PROTOCOL_T2T);
  nrf_nfct_frame_delay_min_set(NRF_NFCT, 0x00000480UL);
  nrf_nfct_frame_delay_max_set(NRF_NFCT, kFrameDelayMaxDefault);
  nrf_nfct_frame_delay_mode_set(NRF_NFCT, NRF_NFCT_FRAME_DELAY_MODE_WINDOWGRID);
  nrf_nfct_shorts_set(NRF_NFCT, NRF_NFCT_SHORT_FIELDDETECTED_ACTIVATE_MASK);
  nrf_nfct_int_enable(NRF_NFCT, NRF_NFCT_INT_FIELDDETECTED_MASK | NRF_NFCT_INT_SELECTED_MASK |
                                    NRF_NFCT_INT_FIELDLOST_MASK | NRF_NFCT_INT_ERROR_MASK);

  if (sd_nvic_SetPriority(NFCT_IRQn, 3) != NRF_SUCCESS ||
      sd_nvic_ClearPendingIRQ(NFCT_IRQn) != NRF_SUCCESS ||
      sd_nvic_EnableIRQ(NFCT_IRQn) != NRF_SUCCESS) {
    g_fail = 4;
    return false;
  }
  nrf_nfct_task_trigger(NRF_NFCT, NRF_NFCT_TASK_SENSE);
  g_fieldActive = false;
  g_rearmPending = false;
  g_ok = true;
  g_fail = 0;
  return true;
}

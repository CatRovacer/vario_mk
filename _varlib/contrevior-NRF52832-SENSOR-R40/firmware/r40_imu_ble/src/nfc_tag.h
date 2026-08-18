#pragma once
#include <Arduino.h>

// Minimal NFC Forum Type 2 Tag (NDEF text) for R40 nRF52832.
//
// Pins: NFC1=P0.09, NFC2=P0.10 (antenna connector). Requires UICR NFCPINS
// PROTECT=NFC (default after chip erase / virgin UICR = 0xFFFFFFFF).
//
// Adafruit PlatformIO core has NO Nordic nfc_t2t_lib / NDEF helpers — only
// nrfx NFCT HAL headers + an nfc_to_gpio example. This module drives NFCT
// directly (read-only Type 2 + static NDEF text "R40_IMU").
//
// SoftDevice: call nfcTagBegin() AFTER Bluefruit.begin(). HFCLK is requested
// via sd_clock_hfclk_request(); NFCT IRQ uses sd_nvic_* (prio 3). Call
// nfcTagPoll() regularly so a lost field reliably returns NFCT to SENSE.
// Limitation: not Nordic's certified T2T binary — phones that only speak
// proprietary NTAG extras beyond READ/FAST_READ/GET_VERSION may fail.
// If UICR was switched to GPIO (nfc_to_gpio), restore NFC mode with SWD
// before SoftDevice runs; this API will not rewrite UICR while SD is on.
//
// Success: nfcTagOk()==true and Android NFC notification shows "R40_IMU".

bool nfcTagBegin(); // start Type2 sense; false if pins/HFCLK/NFCT fail
void nfcTagPoll();  // service nRF52832 lost-field/SENSE workarounds
bool nfcTagOk();    // true after successful nfcTagBegin()
uint8_t nfcTagFail(); // 0=ok/not tried, 1=pins, 2=hfclk, 3=mem, 4=irq

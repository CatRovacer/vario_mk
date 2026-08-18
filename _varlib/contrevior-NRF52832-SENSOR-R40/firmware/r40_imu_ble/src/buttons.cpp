#include "buttons.h"

static const uint8_t PIN_KEY0 = 18;
static const uint8_t PIN_KEY1 = 3;
static const uint32_t DEBOUNCE_MS = 20;
static const uint32_t BLINK_MS = 80;

static bool g_raw0 = false, g_raw1 = false;
static bool g_stable0 = false, g_stable1 = false;
static uint32_t g_chg0 = 0, g_chg1 = 0;
static ButtonEvents g_ev = {};
static uint16_t g_sticky = 0;
static uint32_t g_blinkUntil = 0;
static uint8_t g_blinkR = 0, g_blinkG = 0, g_blinkB = 0;

void buttonsBegin() {
  // External 100k pulldown on schematic; INPUT_PULLDOWN also fine on nRF52
  pinMode(PIN_KEY0, INPUT_PULLDOWN);
  pinMode(PIN_KEY1, INPUT_PULLDOWN);
}

static void armBlink(uint8_t r, uint8_t g, uint8_t b) {
  g_blinkR = r;
  g_blinkG = g;
  g_blinkB = b;
  g_blinkUntil = millis() + BLINK_MS;
}

void buttonsPoll() {
  const uint32_t now = millis();
  bool r0 = digitalRead(PIN_KEY0) == HIGH;
  bool r1 = digitalRead(PIN_KEY1) == HIGH;

  g_ev.key0Edge = false;
  g_ev.key1Edge = false;

  if (r0 != g_raw0) {
    g_raw0 = r0;
    g_chg0 = now;
  }
  if (r1 != g_raw1) {
    g_raw1 = r1;
    g_chg1 = now;
  }

  if ((now - g_chg0) >= DEBOUNCE_MS && r0 != g_stable0) {
    g_stable0 = r0;
    if (g_stable0) {
      g_ev.key0Edge = true;
      g_sticky |= 0x01;
      armBlink(0, 180, 0); // green = KEY0
    }
  }
  if ((now - g_chg1) >= DEBOUNCE_MS && r1 != g_stable1) {
    g_stable1 = r1;
    if (g_stable1) {
      g_ev.key1Edge = true;
      g_sticky |= 0x02;
      armBlink(180, 80, 0); // amber = KEY1 mark
    }
  }
}

ButtonEvents buttonsLast() { return g_ev; }

uint16_t buttonsTakeSticky() {
  uint16_t s = g_sticky;
  g_sticky = 0;
  return s;
}

bool buttonsBlinkActive(uint8_t *r, uint8_t *g, uint8_t *b) {
  if (millis() >= g_blinkUntil) return false;
  *r = g_blinkR;
  *g = g_blinkG;
  *b = g_blinkB;
  return true;
}

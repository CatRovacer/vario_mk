#pragma once
#include <Arduino.h>

// KEY0=P0.18 KEY1=P0.03 active-HIGH (schematic 100k PD → use INPUT)

struct ButtonEvents {
  bool key0Edge;
  bool key1Edge;
  uint16_t sticky; // bit0=KEY0 bit1=KEY1; cleared by buttonsTakeSticky()
};

void buttonsBegin();
void buttonsPoll(); // ~20 ms debounce; call from loop()
ButtonEvents buttonsLast();
uint16_t buttonsTakeSticky(); // read-and-clear sticky edges
bool buttonsBlinkActive(uint8_t *r, uint8_t *g, uint8_t *b); // brief RGB pulse

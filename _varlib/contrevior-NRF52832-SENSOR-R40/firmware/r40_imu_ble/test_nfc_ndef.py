#!/usr/bin/env python3
"""Structural check for R40 NFC Type2 NDEF text layout (mirrors nfc_tag.cpp)."""
ndef = bytes([0xD1, 0x01, 0x0A, 0x54, 0x02, 0x65, 0x6E]) + b"R40_IMU"
assert len(ndef) == 14
mem = bytearray(64)
mem[12], mem[13], mem[14], mem[15] = 0xE1, 0x10, 6, 0x0F
o = 16
mem[o] = 0x03
mem[o + 1] = len(ndef)
mem[o + 2 : o + 2 + len(ndef)] = ndef
mem[o + 2 + len(ndef)] = 0xFE
assert mem[16] == 0x03 and mem[17] == 14 and mem[18 + 14] == 0xFE
assert mem[18:32] == ndef
print("nfc_ndef_layout_ok")

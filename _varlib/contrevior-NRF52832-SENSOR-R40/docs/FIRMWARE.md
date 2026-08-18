# Reference firmware protocol

Advertising name: **`R40_IMU`**. SoftDevice **S132 v6.1.1**, app @ **`0x26000`**.

Also exposes standard **Battery Service** (`180F` / `2A19`) — percent from VDD SAADC / CR2032 curve.

## Custom GATT

Base UUID pattern: `0000152x-1212-EFDE-1523-785FEABCD123`

| UUID | Role | Dir |
|------|------|-----|
| `...1523` | Service | — |
| `...1524` | IMU data | Notify, **20 B** |
| `...1525` | IMU stream control | Write, 1 B (`0x01` on / `0x00` off) |
| `...1526` | ENV data | Notify, **16 B** |
| `...1527` | RGB | Write, 3 B `R,G,B` (0–255; common-anode drive) |
| `...1528` | Flash command | Write, **4 B** `op,a,b,c` |
| `...1529` | Flash reply | Notify, ≤20 B |

Default IMU notify rate: **100 Hz** (10 ms). ENV notify ~10 Hz when streaming.

## IMU notify (20 B, little-endian)

| Off | Type | Field |
|-----|------|--------|
| 0 | int16 ×3 | Acc ×1000 (g) |
| 6 | int16 ×3 | Gyro ×100 (dps) |
| 12 | int16 ×3 | Mag ×10 (µT) — usually 0 if MAG not fitted |
| 18 | uint16 | Sample rate Hz (0 if IMU fail) |

## ENV notify (16 B, little-endian)

| Off | Type | Field |
|-----|------|--------|
| 0 | int16 | Temp ×100 (°C) |
| 2 | uint32 | Pressure (Pa) |
| 6 | uint16 | Lux |
| 8 | uint16 | Proximity |
| 10 | uint16 | Flags |
| 12 | int16 | Humidity ×100 (%RH); `0x7FFF` = absent |
| 14 | uint16 | Events / status |

### Flags (byte 10–11)

| Bit | Meaning |
|-----|---------|
| 0 | BMP/BME pressure path OK |
| 1 | AP3216 OK |
| 2 | MAG OK |
| 3 | MX25 JEDEC OK |
| 4 | BME humidity OK |
| 5 | NFC Type2 init OK |

### Events (byte 14–15)

| Bit | Meaning |
|-----|---------|
| 0 | KEY0 edge (sticky) |
| 1 | KEY1 edge (sticky) |
| 2 | Stream enabled |
| 3 | Flash logging active |
| 4 | Flash OK |
| 5 | NFC OK |

## Flash logger (`1528` / `1529`)

Record size: **32 B** = IMU20 + ENV12 (temp, press, lux, prox, humidity).  
Capacity: MX25 data region ~1 MB (~32k samples). Log rate: **10 / 20 / 25 / 50 / 100 Hz**.

| Op | Name | Args |
|----|------|------|
| `0x01` | Start record | `a` = log rate Hz (0 = keep); resets pointer |
| `0x02` | Stop | flush last page |
| `0x03` | Status | — |
| `0x04` | Erase | full data region (slow) |
| `0x05` | Read chunk | 24-bit byte offset in `a\|b\|c`; returns 16 B payload |

### Status notify (`op=0x03`, 15 B)

| Off | Field |
|-----|--------|
| 0 | `0x03` |
| 1 | flags: bit0=flash OK, bit1=logging |
| 2–5 | writePtr |
| 6–9 | sample count |
| 10–12 | JEDEC bytes |
| 13 | record size (`32`) |
| 14 | log rate Hz |

### Read notify (`op=0x05`)

`[0]=0x05, [1..2]=offset lo, [3]=n, [4..]=n bytes` (n≤16).

Sectors erase on write; Record does not wipe the whole chip first.

## NFC

Type 2 NDEF text **`R40_IMU`** on P0.09/10 when UICR NFC pins enabled and antenna present.

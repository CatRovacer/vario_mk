# R40 pinout (confirmed)

**Board:** `NRF52832_SENSOR_R40` / `nRF52_Sensor_Tag_R40` VER:A  
**Full analysis:** [`R40_Board_Developer_Hardware_Analysis.md`](R40_Board_Developer_Hardware_Analysis.md)

## SWD (right edge)

| Silk | Signal |
|------|--------|
| SWCLK | SWDCLK |
| SWDIO | SWDIO |
| VCC | VCC_NRF (~3 V) |
| GND | GND |

## Expansion pads

| Silk | GPIO | Notes |
|------|------|--------|
| **04** | P0.04 | J1 / AIN2 |
| **05** | P0.05 | J1 / AIN3 |
| **06** | P0.06 | J1 GPIO |
| **17** | P0.17 | Unconfirmed |
| **18** | P0.18 | KEY0 |
| **19** | P0.19 | Bus A SDA (SCL = P0.21, no silk twin) |

Silkscreen **04 / 05 / 06** = **P0.04 / P0.05 / P0.06**.

## I²C Bus B — BMI + BMP (+ MAG if fitted)

| Net | GPIO |
|-----|------|
| SDA | **P0.11** |
| SCL | **P0.12** |

| Device | Addr | Notes |
|--------|------|--------|
| BMI160 | `0x69` | INT1 P0.08, INT2 P0.07 |
| BMP280 | `0x76` | Temp / pressure |
| MAG3110 | `0x0E` | Often not populated |

## I²C Bus A — ALS (+ BME if fitted)

| Net | GPIO |
|-----|------|
| SDA | **P0.19** |
| SCL | **P0.21** |

| Device | Addr | Notes |
|--------|------|--------|
| AP3216C | `0x1E` | INT P0.20; lux + proximity |
| BME280 | `0x76` | Humidity if chip ID `0x60` |

Do not configure P0.21 as nRESET.

## RGB (common-anode, active-low)

| Color | GPIO |
|-------|------|
| Red | **P0.27** |
| Green | **P0.28** |
| Blue | **P0.29** |

P0.30 = flash CS.

## SPI flash MX25R80

| Signal | GPIO |
|--------|------|
| MOSI (SI) | P0.25 |
| SCK | P0.26 |
| CS | P0.30 |
| MISO (SO) | P0.31 |

## Buttons

| Key | GPIO | Sense |
|-----|------|--------|
| KEY0 | P0.18 | Active-high |
| KEY1 | P0.03 | Active-high |

## NFC

| Net | GPIO |
|-----|------|
| NFC1 | P0.09 |
| NFC2 | P0.10 |

UICR `NFCPINS` bit0 = 0 (`0xFFFFFFFE`). Antenna may be unpopulated.

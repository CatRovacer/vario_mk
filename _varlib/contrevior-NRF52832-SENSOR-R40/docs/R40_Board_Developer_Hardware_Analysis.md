# R40 Board — Developer Hardware Analysis

**Board names:** `NRF52832_SENSOR_R40`, `nRF52_Sensor_Tag_R40`  
**Schematic:** ShenZhen Infor Link **VER:A**, 2017-04-10 ([`../schematics/`](../schematics/))  
**MCU:** Nordic nRF52832 (third-party tag — not a Nordic reference design)  
**Retail:** sample from **DUOWEISI** (AliExpress *DUOWEISI Module Store*); identical model string sold by other vendors — see README search tags  
**Companion docs:** [`R40_PINOUT.md`](R40_PINOUT.md) · [`FLASHING.md`](FLASHING.md) · [`FIRMWARE.md`](FIRMWARE.md)

**Confidence tags**

| Tag | Meaning |
|-----|---------|
| Confirmed | Measured / exercised on hardware (SWD + BLE) |
| Schematic | From VER:A / placement; not all SKUs populate every part |
| Unconfirmed | Net unclear — verify with continuity |

---

## 1. Executive summary

Compact CR2032 / externally powered BLE sensor tag:

| Block | Mapping | Confidence |
|-------|---------|------------|
| nRF52832 + BLE | SoftDevice-capable | Confirmed |
| BMI160 ACC+GYR | Bus B P0.11/12, `0x69` | Confirmed |
| BMP280 temp/pressure | Bus B `0x76` | Confirmed |
| BME280 humidity | Bus A `0x76`, ID `0x60` when fitted | Confirmed when present |
| AP3216C lux + proximity | Bus A P0.19/21, `0x1E` | Confirmed |
| MX25R80 SPI NOR | P0.25/26/30/31, JEDEC e.g. `C2 28 14` | Confirmed |
| RGB LED | R=P0.27 G=P0.28 B=P0.29, common-anode, active-low | Confirmed |
| KEY0 / KEY1 | P0.18 / P0.03, active-high | Confirmed |
| SWD | SWCLK, SWDIO, VCC, GND | Confirmed |
| MAG3110 | Bus B `0x0E`, INT P0.13 | Schematic; often not fitted |
| NFC | P0.09/10; UICR `NFCPINS` | Pins confirmed; antenna SKU-dependent |
| J1 | P0.04/05/06 silk 04/05/06 | Confirmed (signals only) |

Bring-up constraints:

1. Two separate I²C buses.  
2. **P0.21** is Bus A SCL — must not be UICR nRESET.  
3. **P0.30** is MX25 CS — not an LED channel.  
4. Programming the nRF HEX does not program the MX25.  
5. `--chiperase` clears UICR; restore `NFCPINS` for NFC pin mode.

---

## 2. Architecture

```text
VCC_NRF (battery switch K1 / SWD VCC)
    |
    +-- nRF52832
    |     BLE 2.4 GHz --> chip antenna
    |     NFC P0.09/10 --> connector / coil (if fitted)
    |     SWD --> SWCLK SWDIO VCC GND
    |     I2C Bus B P0.11/12 --> BMI160 + BMP280 [+ MAG3110]
    |     I2C Bus A P0.19/21 --> AP3216C [+ BME280]
    |     SPI P0.25/26/30/31 --> MX25R80
    |     GPIO P0.27/28/29 --> RGB (common-anode)
    |     GPIO P0.18 / P0.03 --> KEY0 / KEY1
    |     GPIO P0.04/05/06 --> J1 expansion (signals only)
```

---

## 3. Programming (SWD)

| Silk | Signal |
|------|--------|
| SWCLK | SWDCLK |
| SWDIO | SWDIO |
| VCC | VCC_NRF |
| GND | GND |

Pads **04/05/06** are GPIOs, not SWD. Procedure: [`FLASHING.md`](FLASHING.md).

Reference layout: SoftDevice **S132 6.1.1**, application base **`0x26000`**.

---

## 4. I²C buses

### Bus B — P0.11 (SDA) / P0.12 (SCL)

Schematic shows discrete pull-ups. Devices:

| Device | Addr | Role | Notes |
|--------|------|------|--------|
| BMI160 | `0x69` | Acc + gyro | INT1 P0.08, INT2 P0.07 |
| BMP280 | `0x76` | Temp + pressure | Bus B |
| MAG3110 | `0x0E` | Mag | Optional; INT P0.13 |

Confirmed path: BMI + BMP. Typical ACC/GYR ODR 100 Hz.

### Bus A — P0.19 (SDA) / P0.21 (SCL)

| Device | Addr | Role | Notes |
|--------|------|------|--------|
| AP3216C | `0x1E` | Lux + proximity | INT P0.20 |
| BME280 | `0x76` | + humidity | Present when chip ID `0x60` |

Keep **P0.21** as GPIO SCL (board define / UICR: not nRESET).

BMP280 and BME280 share address `0x76` but sit on **different** buses.

---

## 5. BMI160

- I²C address `0x69` (SDO high on this design).  
- Enable ACC and GYR; check PMU status if data stuck.  
- INT lines available; polling is enough for bring-up.

---

## 6. Environmental sensors

- **BMP280** (Bus B): temperature + pressure.  
- **BME280** (Bus A, if fitted): humidity (ID `0x60`).  
- **AP3216C** (Bus A): lux + proximity.

---

## 7. MX25R80 SPI flash

| Signal | GPIO |
|--------|------|
| SI (MOSI) | P0.25 |
| SCK | P0.26 |
| CS | P0.30 |
| SO (MISO) | P0.31 |

- JEDEC `0x9F` → e.g. `C2 28 14`.  
- Adafruit nRF52 Arduino: `SPI.setPins(MISO, SCK, MOSI)`.  
- Hold CS high when idle.  
- Reference firmware can log to this chip over a custom BLE service.

---

## 8. RGB LED

Common-anode, **active-low** (LOW = on):

| Color | GPIO |
|-------|------|
| Red | P0.27 |
| Green | P0.28 |
| Blue | P0.29 |

Idle = drive pins HIGH. P0.30 is flash CS only.

---

## 9. Buttons

| Key | GPIO | Polarity |
|-----|------|----------|
| KEY0 | P0.18 | Active-high (external pulldown) |
| KEY1 | P0.03 | Active-high (external pulldown) |

Debounce in software (~10–30 ms).

---

## 10. Expansion J1 (silk 04 / 05 / 06)

| Silk | GPIO | Analog |
|------|------|--------|
| 04 | P0.04 | AIN2 |
| 05 | P0.05 | AIN3 |
| 06 | P0.06 | Digital |

Signals only — no 3V3/GND on these pads. Use **SWD VCC/GND** for accessory power.

Adjacent silk: **18** = KEY0, **19** = Bus A SDA, **17** unconfirmed.

---

## 11. NFC

| Net | GPIO |
|-----|------|
| NFC1 | P0.09 |
| NFC2 | P0.10 |

After chip erase:

```text
nrfjprog --memwr 0x1000120C --val 0xFFFFFFFE
```

Tuning footprints may be unpopulated. Phone detection requires a working antenna, not only UICR + firmware init.

---

## 12. Clocks and RF

- HF crystal per schematic (~32 MHz class).  
- LF 32.768 kHz — use LFXO (`USE_LFXO`).  
- 2.4 GHz chip antenna + matching network.

---

## 13. Power

- Single `VCC_NRF` rail (~3.0–3.3 V).  
- Mechanical switch K1 on battery path.  
- No dedicated battery divider on confirmed builds; VDD SAADC is only a rough battery estimate.

---

## 14. GPIO map

| GPIO | Function | Confidence |
|------|----------|------------|
| P0.02 | Tied / monitor? | Unconfirmed |
| P0.03 | KEY1 | Confirmed |
| P0.04–06 | J1 | Confirmed |
| P0.07–08 | BMI INT2 / INT1 | Schematic |
| P0.09–10 | NFC | Confirmed (UICR) |
| P0.11–12 | I²C Bus B | Confirmed |
| P0.13 | MAG INT | Schematic / if MAG fitted |
| P0.14–17, 22–24 | Unknown / NC | Unconfirmed |
| P0.18 | KEY0 | Confirmed |
| P0.19 | I²C Bus A SDA | Confirmed |
| P0.20 | AP3216 INT | Schematic |
| P0.21 | I²C Bus A SCL | Confirmed |
| P0.25–26, 30–31 | SPI flash | Confirmed |
| P0.27–29 | RGB R / G / B | Confirmed |

---

## 15. Firmware bring-up checklist

1. SWD at VTref ≈ 2.8–3.3 V.  
2. Flash SoftDevice + app ([`FLASHING.md`](FLASHING.md)).  
3. Restore `NFCPINS` if the chip was erased.  
4. Confirm LFXO; P0.21 not reset.  
5. I²C Bus B: BMI `0x69`, BMP `0x76`.  
6. I²C Bus A: AP3216 `0x1E` (± BME).  
7. SPI JEDEC on MX25.  
8. RGB R → G → B smoke test.  
9. KEY0 / KEY1 edges.  
10. BLE advertise + sensor notify.

Reference firmware: [`../firmware/r40_imu_ble/`](../firmware/r40_imu_ble/) (advertising name `R40_IMU`).  
BLE UUIDs and payloads: [`FIRMWARE.md`](FIRMWARE.md).

---

## 16. Sources

| Source | Role |
|--------|------|
| Schematic VER:A + placement PDF | Net names, part population |
| Lab bring-up (SWD, I²C/SPI probes, BLE) | Confirmed pin map and behavior |
| [`R40_PINOUT.md`](R40_PINOUT.md) | Short pin reference |
| [`FIRMWARE.md`](FIRMWARE.md) | GATT / flash logger protocol |

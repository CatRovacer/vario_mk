# NRF52832 SENSOR R40 — hardware & firmware package

![NRF52832 SENSOR R40](r40.jpg)

Board hardware, pinout, SWD flashing, and reference firmware for the **`NRF52832_SENSOR_R40`** tag.  
Not included: mobile apps or third-party accessories.

This package was developed against a unit bought from **DUOWEISI**. 

## Docs

| Doc | Purpose |
|-----|---------|
| **[`docs/R40_Board_Developer_Hardware_Analysis.md`](docs/R40_Board_Developer_Hardware_Analysis.md)** | Main hardware reference |
| [`docs/R40_PINOUT.md`](docs/R40_PINOUT.md) | Confirmed pin map |
| [`docs/FLASHING.md`](docs/FLASHING.md) | SoftDevice + app with `nrfjprog` |
| [`docs/FIRMWARE.md`](docs/FIRMWARE.md) | BLE GATT UUIDs, packet formats, flash logger |
| [`schematics/`](schematics/) | Schematic + placement (PDF) |
| [`firmware/r40_imu_ble/`](firmware/r40_imu_ble/) | PlatformIO source + flashable `dist/*.hex` |

## Confirmed on hardware

- BMI160 ACC+GYR @ **P0.11/P0.12**, `0x69`
- BMP280 / BME280 + AP3216C on **P0.19/P0.21**
- RGB common-anode **R=27 G=28 B=29** (active-low)
- MX25R80 **MOSI25 SCK26 CS30 MISO31**
- KEY0 **P0.18**, KEY1 **P0.03**
- SWD **SWCLK / SWDIO / VCC / GND**
- MAG3110 often not fitted
- NFC needs UICR `NFCPINS=0xFFFFFFFE`; antenna SKU-dependent

## Quick flash

```powershell
cd firmware\r40_imu_ble
nrfjprog -f NRF52 --program dist\r40_imu_ble_direct.hex --sectorerase --verify --reset
# after chiperase only:
nrfjprog -f NRF52 --memwr 0x1000120C --val 0xFFFFFFFE
nrfjprog -f NRF52 --reset
```

Advertising name **`R40_IMU`**. SoftDevice **S132 6.1.1**, app @ **`0x26000`**.  
Use `dist/r40_imu_ble_direct.hex` (SoftDevice + app). See [`docs/FLASHING.md`](docs/FLASHING.md).

## Repo layout

```text
R40/
  README.md
  r40.jpg
  docs/           hardware + flash + BLE protocol
  schematics/     VER:A PDF
  firmware/r40_imu_ble/
    src/          PlatformIO / Arduino (Adafruit nRF52)
    dist/         prebuilt hex
```

## License

[MIT](LICENSE)

## Provenance

Hardware is third-party (not Nordic). This sample was bought from **DUOWEISI Module Store** on AliExpress. Schematic VER:A (2017-04-10, Infor Link / `nRF52_Sensor_Tag_R40`). The same `NRF52832_SENSOR_R40` listing also appears under other sellers.

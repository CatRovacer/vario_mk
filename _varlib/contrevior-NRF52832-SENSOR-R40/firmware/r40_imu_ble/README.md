# R40 IMU BLE firmware

Streams R40 sensors over custom GATT. For board docs see [`../../docs/`](../../docs/).

## Hex

| File | Use |
|------|-----|
| `dist/r40_imu_ble_direct.hex` | **Flash this** — S132 6.1.1 + app @ `0x26000` |
| `dist/r40_imu_ble.hex` | App only |

See [`../../docs/FLASHING.md`](../../docs/FLASHING.md) and [`../../docs/FIRMWARE.md`](../../docs/FIRMWARE.md).

## Build

```powershell
python -m platformio run -e r40_imu
# merge SoftDevice crop + app → dist\r40_imu_ble_direct.hex (see FLASHING.md)
```

Requires: PlatformIO, Adafruit nRF52 core (Feather nRF52832 board), `srec_cat` for merge.

## Features (high level)

- BMI160 ACC/GYR @ 100 Hz notify  
- ENV: temp, pressure, lux, prox, humidity (if BME)  
- RGB GATT write  
- MX25 flash logger (rate selectable, up to 100 Hz log)  
- KEY0/KEY1  
- NFC Type2 text tag (needs UICR + antenna)  
- Battery Service (VDD estimate)

## Pin defines

Match [`../../docs/R40_PINOUT.md`](../../docs/R40_PINOUT.md): Bus B 11/12, Bus A 19/21, RGB 27/28/29, flash 25/26/30/31.

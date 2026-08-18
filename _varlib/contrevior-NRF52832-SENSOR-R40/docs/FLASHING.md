# Flashing the R40 (SWD)

## Hardware

1. Connect J-Link (or compatible) to **SWCLK, SWDIO, GND, VCC**.
2. Power the board (USB/external on VCC, or CR2032 with switch ON).
3. Do **not** use pads **04/05/06** as SWD — those are GPIOs.

## Tooling

- [nRF Command Line Tools](https://www.nordicsemi.com/Products/Development-tools/nRF-Command-Line-Tools) (`nrfjprog`)
- Optional: PlatformIO + `srec_cat` to rebuild the merged HEX

## Flash the prebuilt image

```powershell
cd firmware\r40_imu_ble
nrfjprog -f NRF52 --ids
nrfjprog -f NRF52 --snr <SNR> --program dist\r40_imu_ble_direct.hex --sectorerase --verify --reset
```

`r40_imu_ble_direct.hex` = **SoftDevice S132 v6.1.1** (cropped to `0x26000`) + application.

Prefer **`--sectorerase`** over **`--chiperase`** so UICR NFC pins survive.

### After `--chiperase` (NFC pins)

```powershell
nrfjprog -f NRF52 --memwr 0x1000120C --val 0xFFFFFFFE
nrfjprog -f NRF52 --reset
```

Confirm:

```powershell
nrfjprog -f NRF52 --memrd 0x1000120C --n 4
# expect FFFFFFFE
```

## Rebuild from source

```powershell
cd firmware\r40_imu_ble
python -m platformio run -e r40_imu

$srec = "$env:USERPROFILE\.platformio\packages\tool-sreccat\srec_cat.exe"
$base = "$env:USERPROFILE\.platformio\packages\framework-arduinoadafruitnrf52\bootloader\feather_nrf52832\feather_nrf52832_bootloader-0.9.1_s132_6.1.1.hex"
& $srec $base -Intel -crop 0 0x26000 .pio\build\r40_imu\firmware.hex -Intel -o dist\r40_imu_ble_direct.hex -Intel
Copy-Item .pio\build\r40_imu\firmware.hex dist\r40_imu_ble.hex -Force
```

## Sanity check

- Phone BLE scan: advertising name **`R40_IMU`**
- RGB boot blink on power-up (firmware)
- Optional: nRF Connect → custom GATT (see [`FIRMWARE.md`](FIRMWARE.md))

## Notes

- External **MX25** is **not** programmed by the nRF HEX — only on-chip flash.
- App start address **`0x26000`** (S132 6.1.1).
- SoftDevice blocks in-app UICR writes; use `nrfjprog --memwr` for NFCPINS.
- Prebuilt `dist/r40_imu_ble_direct.hex` is ready to program; rebuild only if you change source.

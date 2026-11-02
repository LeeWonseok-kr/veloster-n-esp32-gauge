# No-sleep startup ceremony firmware

Release: `v2026.11.02-nosleep` / STATUS build `20261102_r9_nosleep`.
Target: existing ESP32-S3 N16R8 development board and ILI9488 display. **Not the custom PCB.**

This version removes the automatic sleep/wake lifecycle. At power-on it shows a 2.2-second display-only minimum–maximum–minimum sweep, then live readings (or missing-data placeholders). CAN starts after the existing 3-second startup delay. GPIO13 controls the backlight and stays on while powered, including when the engine stops. A brief error or STOP still halts CAN requests; there is no automatic retry. Existing reset lockout protections remain.

**Disconnect gauge power when parking. This is not a low-power parking firmware.** An always-live OBD supply can drain the vehicle battery. GPIO13 remains connected to the TFT LED control input; all other wiring is unchanged.

## Build from source

Install Python and PlatformIO Core, then run from this `firmware` directory:

```sh
python -m platformio run -d experiments/obd_gauge_no_sleep -e esp32s3_obd_gauge_no_sleep
```

The configuration pins espressif32 6.13.0 and TFT_eSPI 2.5.43. Dependencies are downloaded by PlatformIO; generated caches are not included. Other environments inherited from the shared configuration are not release targets.

## Flash the release binary

Park, disconnect OBD and BEC power, and connect the board to USB only. Use the actual USB serial port (COM3 is only an example):

```sh
python -m pip install esptool==4.11.0
python -m esptool --chip esp32s3 --port COM3 --baud 115200 write_flash --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0 firmware-merged.bin
```

Download `firmware-merged.bin` and verify it against `SHA256SUMS.txt` from the GitHub release. The merged image includes bootloader, partition table, boot_app0, and application. Padding between regions also overwrites existing data such as NVS within that address range. The application-only file belongs at `0x10000`, not `0x0`. This layout replaces the firmware and partition table; back up any custom board data first.

After flashing without OBD, the first CAN request may fault as expected under the unchanged protection policy. Restore the existing safe vehicle-power wiring and fully power-cycle for a vehicle test. Upload verification does not prove vehicle operation or power performance.

## Verification and limitations

Host checks cover no sleep-controller calls, startup sweep endpoints, return to actual/missing readings, GPIO13 remaining on for a 60-second zero-RPM display simulation, and the existing ten-minute paired-query tests with rate/error/STOP protection. A clean source-package build passed before publication. The final release notes describe actual build/upload results. New-version road testing is not claimed.

## Dependencies and licensing

Third-party dependencies retain their own licenses. TFT_eSPI includes FreeBSD/MIT and other component notices; Arduino-ESP32 and ESP-IDF also contain separately licensed components. Included notices are in `THIRD_PARTY_NOTICES/`. See the pinned dependency distributions for their complete notices. No additional blanket open-source license is granted for the project-specific code by this publication.

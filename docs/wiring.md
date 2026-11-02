# Current development-board wiring

| Source | Destination |
|---|---|
| Vehicle CAN H / CAN L | CAN module H / L |
| Vehicle 12 V / GND | Existing BEC input |
| BEC 5 V output | ESP32-S3 5 V and TFT VCC |
| ESP32-S3 3V3 | 3.3 V CAN module VCC |
| GPIO4 | CAN TXD/D; retain the existing 10 kΩ pull-up to 3.3 V |
| GPIO5 | CAN RXD/R |
| GPIO6 | CAN RS/STB, only if exposed on the existing module |
| GPIO8 / 9 / 10 | TFT RST / DC / CS |
| GPIO11 / 12 | TFT MOSI / SCK |
| GPIO13 | TFT LED control input; remove the previous LED-to-3.3 V connection |
| GND | Common reference for the BEC, ESP32-S3, CAN module, TFT, and vehicle |

TFT VCC uses 5 V; logic signals use 3.3 V. GPIO13 is a backlight control signal, not a power supply for the backlight. The actual TFT module revision and LED input circuit still need verification. MISO, touch, and SD connections are currently unused.

Standard OBD pin 16 is battery positive, pin 6 is CAN H, and pin 14 is CAN L. Retain the existing verified cable ground connection. Do not apply a standard USB pinout to the custom vehicle cable based on connector shape. Do not add another 120 Ω termination resistor in parallel.

Flash firmware only while parked, with OBD and BEC disconnected and the board powered by USB alone. When using vehicle power, retain the existing USB VBUS isolation arrangement so the laptop's 5 V supply and the BEC's 5 V output are not tied together.

Sources: the project's `board_pins.hpp`, the current `auto_sleep` configuration, and the owner's confirmation of the GPIO13 LED connection. This overview does not independently verify the physical RS/STB connection, BEC ratings, or internal cable continuity.

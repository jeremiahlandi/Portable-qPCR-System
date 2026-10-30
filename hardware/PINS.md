# Pin mapping

Single source of truth for every signal (rule: `.cursor/rules/pin-mapping.mdc`). The firmware reads
its pins from `firmware/targets/<board>/pins.h`, which must match this file.

## Board: Waveshare ESP32-P4 module dev kit (`firmware/targets/esp32-p4`)

Decision LI-DL-004 (issue #2). Header map reference: [Waveshare P4 dev-kit header map](https://europe1.discourse-cdn.com/arduino/original/4X/3/2/d/32dc878cdeb932b9bf9e8611bebec2d9a789d12a.jpeg).

### Signals

| Signal | GPIO | Dir | Board pin / rail | Wire | Owner | Verified |
| --- | --- | --- | --- | --- | --- | --- |
| Display VCC | 5V | power | 5V rail | red | #3 | 2026-10 (commit 002ae04) |
| Display GND | GND | power | GND | black | #3 | 2026-10 (commit 002ae04) |
| Display CS | 20 | out | header | purple | #3 | 2026-10 (commit 002ae04) |
| Display RESET | 21 | out | header | grey | #3 | 2026-10 (commit 002ae04) |
| Display DC | 22 | out | header | blue | #3 | 2026-10 (commit 002ae04) |
| Display SDI (MOSI) | 23 | out | header, SPI3 | green | #3 | 2026-10 (commit 002ae04) |
| Display SCK | 24 | out | header, SPI3 | yellow | #3 | 2026-10 (commit 002ae04) |
| Display LED (backlight) | 3V3 | power | 3V3, always on | red | #3 | 2026-10 (commit 002ae04) |
| MAX6675 VCC (bring-up sensor only) | 3V3 | power | 3V3 | red | #6 | 2026-10 (PINS.md, pre-rules) |
| MAX6675 CLK | 6 | out | header | yellow | #6 | 2026-10 (PINS.md, pre-rules) |
| MAX6675 CS | 5 | out | header | purple | #6 | 2026-10 (PINS.md, pre-rules) |
| MAX6675 SO | 4 | in | header | green | #6 | 2026-10 (PINS.md, pre-rules) |
| Touch CS | tbd | out | same SPI as display | | #11 | planned (#11) |
| Touch IRQ | tbd | in | header | | #11 | planned (#11) |
| MAX31865 CS (block PT1000) | 5 (proposed, reuses bring-up CS) | out | header, SPI | | #6 | planned (#6) |
| H-bridge PWM | tbd (LEDC-capable) | out | header | | #7 | planned (#7) |
| H-bridge direction | tbd | out | header | | #7 | planned (#7) |
| Heatsink fan PWM | tbd | out | header | | #7 | planned (#7) |
| Lid heater PWM | tbd (LEDC-capable) | out | header | | #8 | planned (#8) |
| Lid NTC | tbd (ADC-capable) | in | header | | #8 | planned (#8) |
| Status LEDs red / yellow / blue | tbd ×3 | out | header | | #13 | planned (#13) |
| Buzzer | tbd | out | header | | #13 | planned (#13) |
| SD card | onboard slot | | SDMMC | | #14 | planned (#14) |
| RTC (I2C) | onboard I2C | | | | #14 | planned (#14) |
| AS7341 (I2C) | onboard I2C | | | | #18 | planned (#18) |
| Excitation LED | tbd | out | header | | #18 | planned (#18) |
| Wi-Fi / BLE | onboard C6 via SDIO | | reserved by board | | #15 | board feature |

### Reserved by the board

To be filled from the Waveshare wiki before #16 wires anything (the wiki was unreachable from the
authoring session). Expected entries: ESP32-C6 SDIO link (CLK, CMD, D0 to D3, reset), Ethernet
PHY RMII and MDIO/MDC, SD slot (SDMMC), audio codec I2S and I2C, USB, strapping and boot pins.

| Function | GPIO(s) | Source |
| --- | --- | --- |
| tbd | tbd | Waveshare wiki page for the ESP32-P4 module dev kit |

## Retired boards

- Arduino Mega 2560 + classic ESP32 (Kalix legacy, `Firmware/Legacy_Archive`): pin assignments are in the sketch headers; not maintained.

# Pin mapping

Single source of truth for every signal (rule: `.cursor/rules/pin-mapping.mdc`). The firmware reads
its pins from `firmware/targets/<board>/pins.h`, which must match this file.

## Board: Waveshare ESP32-P4-Module-DEV-KIT (`firmware/targets/esp32-p4`)

Decision LI-DL-004 (issue #2). Module ESP32-P4NRW32X (32 MB PSRAM in package) with onboard
16 MB NOR flash and an ESP32-C6 for Wi-Fi 6 / BLE over SDIO. Waveshare SKU 30560.
Pin definition: Waveshare product page, "Pin Definition" figure (copied below).

### 40-pin header (the only user GPIOs; 28 programmable)

Physical order, top to bottom, as printed on the board.

| Left | Right |
| --- | --- |
| 3V3 | 5V |
| SDA (GPIO7) | 5V |
| SCL (GPIO8) | GND |
| GPIO23 | TXD (GPIO37) |
| GND | RXD (GPIO38) |
| GPIO21 | GPIO22 |
| GPIO20 | GND |
| GPIO6 | GPIO5 |
| 3V3 | GPIO4 |
| GPIO3 | GND |
| GPIO2 | GPIO1 |
| GPIO0 | GPIO36 |
| GND | GPIO32 |
| GPIO24 | GPIO25 |
| GPIO33 | GND |
| GPIO26 | GPIO54 |
| GPIO48 | GND |
| GPIO53 | GPIO46 |
| GPIO47 | GPIO27 |
| GND | GPIO45 |

Pin notes (from the ESP32-P4 datasheet; confirm before relying on them):

- GPIO7 / GPIO8 are the board's I2C bus (SDA / SCL), shared with the onboard I2C header and
  any onboard I2C devices. Check addresses before adding the RTC and the AS7341.
- GPIO37 / GPIO38 are UART0 to the Type-C UART (programming and serial console). Using them
  as signals costs the console.
- GPIO36, GPIO37, GPIO38 are strapping pins on the ESP32-P4. Do not put a signal on them that
  is driven or pulled at reset.
- ADC: ADC1 is on GPIO16 to GPIO23 (header: 20, 21, 22, 23); ADC2 is on GPIO49 to GPIO54
  (header: 53, 54). The display SPI occupies the ADC1 header pins, so analog inputs use ADC2,
  which is free on the P4 because the radio lives on the C6.
- LEDC PWM can be routed to any GPIO.

### Signals

| Signal | GPIO | Dir | Board pin / rail | Wire | Owner | Verified |
| --- | --- | --- | --- | --- | --- | --- |
| Display VCC | 5V | power | header 5V | red | #3 | 2026-10 (commit 002ae04) |
| Display GND | GND | power | header GND | black | #3 | 2026-10 (commit 002ae04) |
| Display CS | 20 | out | header | purple | #3 | 2026-10 (commit 002ae04) |
| Display RESET | 21 | out | header | grey | #3 | 2026-10 (commit 002ae04) |
| Display DC | 22 | out | header | blue | #3 | 2026-10 (commit 002ae04) |
| Display SDI (MOSI) | 23 | out | header, SPI | green | #3 | 2026-10 (commit 002ae04) |
| Display SCK | 24 | out | header, SPI | yellow | #3 | 2026-10 (commit 002ae04) |
| Display LED (backlight) | 3V3 | power | header 3V3, always on | red | #3 | 2026-10 (commit 002ae04) |
| SPI MISO (display SDO, touch, MAX31865) | 25 | in | header | white | #11, #6 | planned (#11) |
| Touch CS | 26 | out | header | | #11 | planned (#11) |
| Touch IRQ | 27 | in | header | | #11 | planned (#11) |
| MAX31865 CS (block PT1000) | 5 | out | header, shared SPI | purple | #6 | planned (#6) |
| MAX6675 CLK / CS / SO (bring-up only) | 6 / 5 / 4 | | header | yellow / purple / green | #6 | retired when #6 lands; pins reassigned below |
| H-bridge PWM A (heat) | 6 | out | header, LEDC | | #7 | planned (#7) |
| H-bridge PWM B (cool) | 4 | out | header, LEDC | | #7 | planned (#7) |
| H-bridge enable | 3 | out | header | | #7 | planned (#7) |
| Heatsink fan PWM | 1 | out | header, LEDC | | #7 | planned (#7) |
| Lid heater PWM | 2 | out | header, LEDC | | #8 | planned (#8) |
| Lid NTC | 53 | in (ADC2_CH4) | header | | #8 | planned (#8) |
| Spare analog | 54 | in (ADC2_CH5) | header | | | unassigned |
| Status LED red (heating) | 32 | out | header | | #13 | planned (#13) |
| Status LED yellow (hold) | 33 | out | header | | #13 | planned (#13) |
| Status LED blue (cooling) | 46 | out | header | | #13 | planned (#13) |
| Buzzer | 45 | out | header | | #13 | planned (#13) |
| Excitation LED | 47 | out | header | | #18 | planned (#18) |
| Spare digital | 0, 48 | | header | | | unassigned |
| Strapping / console pins, keep free | 36, 37, 38 | | header | | | reserved |
| I2C SDA / SCL (RTC, AS7341) | 7 / 8 | bus | header, onboard I2C bus | | #14, #18 | planned |
| SD card | onboard slot | | SDIO | | #14 | board feature |
| Wi-Fi / BLE | onboard C6 via SDIO | | not on header | | #15 | board feature |
| Serial console | 37 / 38 | | Type-C UART | | | board feature |

### Reserved by the board

Every GPIO not listed on the 40-pin header above is used by the board and is not available:
ESP32-C6 SDIO link, RJ45 Ethernet PHY, TF card slot (SDIO 3.0), MIPI-DSI and MIPI-CSI
connectors, USB 2.0 OTG, onboard microphone and speaker codec, RTC battery, PoE header.
Source: Waveshare product page ("28 remaining programmable GPIOs" on the 2×20 header).

## Retired boards

- Arduino Mega 2560 + classic ESP32 (Kalix legacy, `Firmware/Legacy_Archive`): pin assignments are in the sketch headers; not maintained.

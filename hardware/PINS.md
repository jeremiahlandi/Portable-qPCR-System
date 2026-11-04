# Pin mapping

Single source of truth for every signal (rule: `.cursor/rules/pin-mapping.mdc`). The firmware reads
its pins from `firmware/targets/<board>/pins.h`, which must match this file.

## Board: Waveshare ESP32-P4-WIFI6 3.5inch Touch LCD development board (`firmware/targets/esp32-p4`)

Decision LI-DL-004, revised 2026-10-30 (issue #2): the 3.5-inch 320 × 480 SPI touch display ships
with this board and connects to it over the board's own ribbon (FPC) connector, so the eight
hand-wired display lines of the previous board go away. Same ESP32-P4 + ESP32-C6 (Wi-Fi 6, BLE 5)
combination, plus an OV5647 5 MP camera, microphone and speaker.

### Fixed by the board (display, touch, camera, radio, storage, audio)

Source: Waveshare's Arduino board library for this board,
`examples/arduino/libraries/Waveshare_LCD35/src/lcd35_board.h` in
[waveshareteam/ESP32-P4-WIFI6-Touch-LCD-3.5](https://github.com/waveshareteam/ESP32-P4-WIFI6-Touch-LCD-3.5)
(schematic PDF in the same repo under `schematic/`). Display bring-up verified on hardware
2026-11-01 with `firmware/targets/esp32-p4` (ST7796 over SPI, LovyanGFX `Panel_ST7796`,
`offset_rotation = 0`, `invert = true`, 40 MHz; vendor uses 80 MHz).

| Function | GPIO(s) | Notes |
| --- | --- | --- |
| Display controller | ST7796, 320 × 480 IPS, SPI | vendor Arduino_GFX config: IPS, rotation 4 (X mirrored) |
| Display SPI MOSI | 20 | |
| Display SPI SCK | 21 | |
| Display CS | 23 | |
| Display DC | 26 | |
| Display RST | 27 | |
| Display backlight | 28 | LEDC PWM 5 kHz, 10-bit (vendor) |
| Display SPI MISO | 22 | routed on the schematic (LCD_MISO); firmware leaves it unused, panel is not read |
| Touch controller | FT6336 (FT5x06 family), I2C address 0x38 | Polled by LVGL over the board I2C bus (#11) |
| Touch RST | 29 | schematic J14 pin 2, 10 k pull-down (R32): the chip is in reset until firmware drives GPIO29 high; verified 2026-11-03 (0x38 absent from the bus until released) |
| Touch INT | 50 | schematic J14 pin 3, 4.7 k pull-up (R58); not used, touch is polled |
| I2C SDA / SCL | 7 / 8 | board bus: touch, AXP2101 PMIC, ES8311 codec; also on the header |
| TF card (SDIO 4-bit) CLK / CMD | 43 / 44 | #14 |
| TF card D0 / D1 / D2 / D3 | 39 / 40 / 41 / 42 | #14 |
| Audio I2S MCLK / BCLK / LRCK | 13 / 12 / 10 | ES8311 codec |
| Audio I2S DOUT (P4 → codec) / DIN (codec → P4) | 9 / 11 | ES8311 codec |
| Audio amplifier enable (NS4150B PA) | 53 | active high |
| Camera (MIPI-CSI 2-lane), microphone, speaker | board internal | Waveshare wiki |
| ESP32-C6 SDIO link | board internal | not published by the vendor |
| USB-to-UART (CH343, `/dev/ttyACM0`) | UART0 GPIO37 / 38 | programming and `Serial` console (`ARDUINO_USB_CDC_ON_BOOT=0`) |
| USB OTG 2.0 HS | native USB | second Type-C port |

### J6 "Interface" header (2 × 16, 2.54 mm)

Read from the Waveshare schematic (`schematic/ESP32-P4-WIFI6-Touch-LCD-3.5-schematic.pdf`,
connector J6) on 2026-11-01. Pin 1 is 3V3, pin 2 is USB 5V; odd pins run down one row, even
pins down the other.

| Odd pin | Net | Even pin | Net |
| --- | --- | --- | --- |
| 1 | 3V3 | 2 | USB0_5V |
| 3 | GND | 4 | GND |
| 5 | GPIO7 (I2C SDA, board bus) | 6 | USB1P1_N (GPIO24, USB OTG) |
| 7 | GPIO8 (I2C SCL, board bus) | 8 | USB1P1_P (GPIO25, USB OTG) |
| 9 | GPIO2 | 10 | GND |
| 11 | GPIO3 | 12 | VBAT |
| 13 | GPIO4 | 14 | GPIO34 (strapping) |
| 15 | GPIO5 | 16 | GPIO35 (strapping, BOOT button, UART auto-reset) |
| 17 | GPIO30 | 18 | GPIO36 (strapping) |
| 19 | GPIO31 | 20 | GPIO51 (ADC2) |
| 21 | GPIO32 | 22 | GPIO52 (ADC2) |
| 23 | GPIO33 | 24 | GND |
| 25 | C6_U0RXD (ESP32-C6) | 26 | ESP_LDO_VO4 |
| 27 | C6_U0TXD (ESP32-C6) | 28 | GPIO48 |
| 29 | C6_IO9 (ESP32-C6) | 30 | GPIO47 |
| 31 | GND | 32 | GPIO46 |

Usable for instrument signals: GPIO2, 3, 4, 5, 30, 31, 32, 33, 46, 47, 48 (digital / LEDC PWM) and
GPIO51, 52 (the only ADC-capable header pins, ADC2). Slow signals live on the I2C expanders
(#50, table below); direct GPIO is reserved for the H-bridge PWM and enable, the sensor SPI bus,
the lid PWM and the fan PWM. Free after the assignments below: GPIO48, 51, 52. Do not use GPIO7/8 (board I2C), GPIO24/25
(USB OTG), GPIO34/35/36 (strapping; 35 is also BOOT and the CH343 auto-reset line) or the C6 pins.

### Signals on the free header

Every instrument signal below moves to whichever GPIOs this board leaves free; the numbers
assigned for the previous board (section "Retired boards") no longer apply. Fill the GPIO column
from the J6 table above, keeping the same owners.

| Signal | GPIO | Dir | Board pin / rail | Wire | Owner | Verified |
| --- | --- | --- | --- | --- | --- | --- |
| Sensor SPI SCK / MOSI / MISO (both MAX31865 boards) | 30 / 31 / 32 | bus | J6 pins 17 / 19 / 21 | | #6, #8 | assigned 2026-11-04 (#8); software SPI or a second SPI host, the display owns SPI2 |
| MAX31865 CS, block PT1000 (Adafruit 3648 no. 1) | 33 | out | J6 pin 23 | | #6 | assigned 2026-11-04 (#8) |
| MAX31865 CS, lid PT1000 (Adafruit 3648 no. 2) | 46 | out | J6 pin 32 | | #8 | assigned 2026-11-04 (#8) |
| H-bridge RPWM (heat), IBT_2 / BTS7960 | 4 (LEDC) | out | J6 pin 13 | | #7 | 2026-11-03 bench probe: +25 % of a 15 V supply on M+/M- at 25 % duty, 20 kHz |
| H-bridge LPWM (cool), IBT_2 / BTS7960 | 5 (LEDC) | out | J6 pin 15 | | #7 | 2026-11-03 bench probe: −25 % of a 15 V supply on M+/M- at 25 % duty, 20 kHz |
| H-bridge enable (R_EN and L_EN tied together) | 3 | out | J6 pin 11 | | #7 | 2026-11-03 bench probe: output follows enable, ~0 V with enable low |
| H-bridge R_IS (heat current sense) | ADS1115 A0 (I2C 0x48) | in | carrier / breakout | | #7, #50 | moved 2026-11-04 from GPIO51; GPIO51 freed |
| H-bridge L_IS (cool current sense) | ADS1115 A1 (I2C 0x48) | in | carrier / breakout | | #7, #50 | moved 2026-11-04 from GPIO52; GPIO52 freed |
| H-bridge logic VCC / GND | 3V3 / GND | power | J6 pin 1 / pin 3 | | #7 | 2026-11-03 bench probe: VCC at 3V3 drives the module's 74HC244 cleanly; B+/B− from the 15 V supply, Peltier disconnected |
| Heatsink fan PWM (25 kHz, 4-wire fan) | 2 (LEDC) | out | J6 pin 9 | | #7 | assigned 2026-11-04 (#50); tach input on MCP23017 if wanted |
| Lid heater PWM, logic-level MOSFET gate (IRLZ44N) | 47 (LEDC) | out | J6 pin 30 | | #8 | assigned 2026-11-04 (#8); 130 °C bimetal cutoff in series with the heater |
| Lid NTC (legacy ring thermistor) | none | | | | #8 | retired 2026-11-04: lid temperature moves to the second MAX31865 on the sensor SPI bus, leaving GPIO51/52 to the H-bridge current sense |
| Status LED red / yellow / blue | PCA9685 ch0 / ch1 / ch2 (I2C 0x40) | out | carrier / breakout | | #13, #50 | assigned 2026-11-04 (#50) |
| Buzzer / alarm tone | ES8311 codec + speaker (board), fallback PCA9685 ch3 | out | board | | #13, #50 | decision on #13 |
| Excitation LED channel 1 / channel 2 (enable, PWM) | PCA9685 ch4 / ch5 (I2C 0x40) | out | carrier / breakout | | #18, #50 | assigned 2026-11-04 (#50); current driver per #18 |
| I2C SDA / SCL (RTC, AS7341) | 7 / 8 (board I2C bus, shared with touch and PMIC) | bus | J6 pins 5 / 7 | | #14, #18 | planned |
| SD card | onboard TF slot (SDIO, GPIO39–44) | | | | #14 | board feature |
| Wi-Fi / BLE | onboard C6 | | | | #15 | board feature |

### I2C expansion channels (bus GPIO7 / GPIO8, 400 kHz)

Decision LI-DL-007 (proposed, #50): everything that is not timing-critical goes on the I2C bus
the board brings out on J6 pins 5 / 7. Each channel is a signal row like a GPIO.

| Device | Address | Channel | Signal | Owner | Verified |
| --- | --- | --- | --- | --- | --- |
| FT6336 touch | 0x38 | | board | #11 | board feature |
| AXP2101 PMIC | 0x34 | | board (battery gauge later) | #37 | board feature |
| ES8311 codec | 0x18 | | board (alarm tone) | #13 | board feature |
| AS7341 spectral sensor | 0x39 | | optics | #18 | planned |
| PCA9685 PWM | 0x40 | ch0 / ch1 / ch2 | status LED red / yellow / blue | #13 | planned (#50) |
| PCA9685 PWM | 0x40 | ch3 | passive buzzer (only if the codec tone is rejected) | #13 | planned (#50) |
| PCA9685 PWM | 0x40 | ch4 / ch5 | excitation LED channel 1 / 2 enable | #18 | planned (#50) |
| PCA9685 PWM | 0x40 | ch6–ch15 | spare | | |
| ADS1115 ADC | 0x48 | A0 / A1 | H-bridge R_IS / L_IS | #7 | planned (#50) |
| ADS1115 ADC | 0x48 | A2 / A3 | spare (rail monitor, cutoff sense) | | |
| MCP23017 I/O | 0x20 | GPA0 | lid-open switch (future interlock) | #8 | planned (#50) |
| MCP23017 I/O | 0x20 | GPA1 | thermal cutoff sense | #5 | planned (#50) |
| MCP23017 I/O | 0x20 | GPA2 | fan tach (optional) | #7 | planned (#50) |
| MCP23017 I/O | 0x20 | others | spare | | |

Pin notes from the ESP32-P4 datasheet and the ESP-IDF GPIO reference: GPIO34 to GPIO38 are
strapping pins; ADC1 is GPIO16 to 23 (none on J6), ADC2 is GPIO49 to 54 (J6 has 51 and 52);
LEDC PWM routes to any GPIO. The IBT_2 power side (B+/B−, M+/M−) is fed from the Peltier
supply, never from the board's 5 V; its logic GND ties the supply ground to the P4 ground.

## Retired boards

### Waveshare ESP32-P4-Module-DEV-KIT (previous board, retired 2026-10-30)

Superseded by the Touch LCD board above; kept because its display wiring was verified on hardware. Module ESP32-P4NRW32X (32 MB PSRAM in package) with onboard
16 MB NOR flash and an ESP32-C6 for Wi-Fi 6 / BLE over SDIO. Waveshare SKU 30560.
Pin definition: Waveshare product page, "Pin Definition" figure (copied below).

#### 40-pin header (the only user GPIOs; 28 programmable)

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

#### Signals

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

#### Reserved by the board

Every GPIO not listed on the 40-pin header above is used by the board and is not available:
ESP32-C6 SDIO link, RJ45 Ethernet PHY, TF card slot (SDIO 3.0), MIPI-DSI and MIPI-CSI
connectors, USB 2.0 OTG, onboard microphone and speaker codec, RTC battery, PoE header.
Source: Waveshare product page ("28 remaining programmable GPIOs" on the 2×20 header).


- Arduino Mega 2560 + classic ESP32 (Kalix legacy, `Firmware/Legacy_Archive`): pin assignments are in the sketch headers; not maintained.

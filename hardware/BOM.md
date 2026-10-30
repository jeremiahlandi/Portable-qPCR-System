# Running bill of materials

Engineering record of every part in the instrument (rule: `.cursor/rules/running-bom.mdc`).
Quantities are per unit. The purchasing spreadsheet ("Base Unit BOM", Google Sheets) is the
ordering record and is updated from this file. Lead times over four weeks are marked **(long)**.

Status: `in use` = in the current design, `planned` = chosen by an issue but not yet built,
`alternative` = qualified substitute, `retired` = removed, reason given.

## Controller, display, storage

| Part | Qty | Purpose (why) | Vendor | Part number | Link | Unit cost | Lead | Status | Owner | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Waveshare ESP32-P4-WIFI6 3.5inch Touch LCD development board (P4 + ESP32-C6, 3.5" 320×480 SPI touch display on ribbon, OV5647 camera, mic, speaker) | 1 | Single controller with the operator touchscreen attached by the board's own ribbon connector: no hand-wired display lines, touch included; C6 gives Wi-Fi 6 and BLE 5 | Waveshare (also Amazon) | ESP32-P4-WIFI6-Touch-LCD-3.5 (confirm exact model) | (Base Unit BOM sheet) | | | in use | #2 | LI-DL-004 revised 2026-10-30; free GPIO count and pin map to confirm from the wiki |
| Waveshare ESP32-P4-Module-DEV-KIT (P4NRW32X + ESP32-C6, 16 MB flash, 32 MB PSRAM) | 1 | Previous controller; 28 free GPIOs on a 40-pin header, display hand-wired over SPI | Waveshare (also Amazon) | SKU 30560, ESP32-P4-Module-DEV-KIT (basic kit) | https://www.amazon.com/dp/B0F2MP483W | 19.99 | in stock | retired | #2 | Replaced by the Touch LCD board to drop the hand-wired display; keep on hand as a bench spare |
| 3.5" TFT ILI9488 480×320 SPI shield | 1 | Operator touchscreen on the previous board; verified at 20 MHz over SPI | Amazon | | (Base Unit BOM sheet) | | | retired | #3, #11 | Display now ships with the Touch LCD board |
| Nextion 3.2" HMI NX4024T032 | 1 | Legacy display; needs the Kalix HMI project file to be usable | Amazon | NX4024T032 | (Base Unit BOM sheet) | 34.49 | | retired | #2 | Replaced by the ILI9488 on the P4; no HMI file |
| Micro SD card module | 1 | Run records and logs (if the onboard slot is not used) | Amazon | | (Base Unit BOM sheet) | 2.99 | | alternative | #14 | P4 kit has an onboard slot |
| SanDisk 32 GB microSDHC | 1 | Run record storage | Amazon | | (Base Unit BOM sheet) | 6.33 | | in use | #14 | |
| Arduino Nano ESP32 | 1 | Earlier controller candidate | Amazon | ABX00083 | (Base Unit BOM sheet) | 23.00 | | retired | #2 | Fewer pins; no advantage over the P4 kit |

## Thermal

| Part | Qty | Purpose (why) | Vendor | Part number | Link | Unit cost | Lead | Status | Owner | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Peltier module 30×30×3.85 mm, 3 A | 1 | Heats and cools the block | Digi-Key | 102-4045-ND | | 26.79 | 13 wk **(long)** | in use | #7, #16 | Verify Vmax against the supply rail; substitute an in-stock 12 V TEC for November |
| Heat sink, half brick 58×61×22.9 mm | 1 | Hot-side sink for the Peltier | Digi-Key | ATS1506-ND | | 17.74 | 8 wk **(long)** | in use | #16 | Any 40 mm CPU cooler is the in-stock alternative |
| Fan 40×10 mm 12 V dual ball bearing | 1 | Heat sink airflow | Amazon | YDM4010B12 | (Base Unit BOM sheet) | 6.99 | | in use | #7 | Set the fan voltage to 12 V or buy 24 V fans |
| Fan guard 40 mm | 1 | Finger guard | Amazon | | (Base Unit BOM sheet) | 1.50 | | in use | #16 | |
| Thermal paste Corsair XTM70 | as needed | Peltier to sink and block interface | Amazon | | (Base Unit BOM sheet) | | | in use | #16 | |
| Polyimide film heater 25 mm, 20 W | 1 | Heated lid | eBay | | (Base Unit BOM sheet) | 6.65 | | in use | #8 | 12 V part; buy the 24 V version if the rail is 24 V |
| NTC ring thermistor 10 kΩ, wire leaded | 1 | Lid temperature | Newark | Molex 213860-1637 | | 5.36 | | in use | #8 | |
| MAX6675 module + K-type thermocouple | 1 | Block temperature for bring-up only (1 °C integer, ±2 °C) | SainSmart | 101-60-176 | | 9.99 | | alternative | #6 | Replaced by PT1000 + MAX31865 for the product |
| PT1000 RTD + MAX31865 breakout | 1 | Block temperature, 0.1 °C | | | | | | planned | #6 | |
| H-bridge driver, BTS7960 class | 1 | Bidirectional Peltier drive so the PID can cool | | | | | | planned | #7 | Replaces the relay polarity switch |
| LC filter for the bridge output (≈47 µH, 470 µF) | 1 | Low-ripple TEC current | | | | | | planned | #7 | |
| Heat block, aluminium, drilled for 0.2 ml tubes | 1 | Sample block | custom | | | | | planned | #16 | Well format decision pending |

## Power

| Part | Qty | Purpose (why) | Vendor | Part number | Link | Unit cost | Lead | Status | Owner | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Power supply 15 V 5 A | 1 | Main supply (current BOM) | Amazon | | (Base Unit BOM sheet) | 23.60 | | alternative | #16 | Marginal at full TEC + lid load |
| Power supply 24 V 6 A, e.g. Mean Well LRS-150-24 | 1 | Main supply (planned), UL listed | | LRS-150-24 | | | | planned | #16 | |
| Buck converter 24 V to 12 V, 10 A | 1 | TEC rail from a 24 V supply | | | | | | planned | #16 | |
| Linear regulator 5 V 1.5 A TO-220 | 1 | Logic rail | Digi-Key | 497-1443-5-ND | | 0.35 | 12 wk **(long)** | retired | #16 | 15 W dissipation at 15 V in; use a buck module |
| Linear regulator 12 V 1.5 A TO-220 | 1 | 12 V rail | Digi-Key | 497-1452-5-ND | | 0.43 | 12 wk **(long)** | retired | #16 | Use a buck module |
| MOSFET N-ch 60 V 50 A TO-220 | 1 | Lid heater switch | Digi-Key | 497-2777-5-ND | | 1.05 | 26 wk **(long)** | in use | #8 | Alternatives below |
| MOSFET N-ch 142 A 40 V (alternative) | 1 | Lid heater switch | Rochester Electronics | | (Base Unit BOM sheet) | 0.98 | | alternative | #8 | |
| MOSFET N-ch 40 V 100 A TO-220 (alternative) | 1 | Lid heater switch | Digi-Key | | (Base Unit BOM sheet) | 1.10 | | alternative | #8 | |
| MOSFET NexFET 40 V (alternative) | 1 | Lid heater switch | Mouser | | https://mou.sr/3lbss0x | 1.10 | | alternative | #8 | |
| DC jack 5.5×2.1 mm | 1 | Power input | Amazon | | (Base Unit BOM sheet) | 0.50 | | in use | #16 | |
| Rocker switch | 1 | Power on/off | Amazon | | (Base Unit BOM sheet) | 0.90 | | in use | #16 | |
| Relay module 5 V 2-channel | 1 | Legacy Peltier polarity switch | Amazon | | (Base Unit BOM sheet) | 3.17 | | retired | #7 | Replaced by the H-bridge |

## Optics

| Part | Qty | Purpose (why) | Vendor | Part number | Link | Unit cost | Lead | Status | Owner | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| AS7341 10-channel spectral sensor breakout | 1 | Fluorescence detector | Digi-Key | | (Base Unit BOM sheet) | 15.95 | | in use | #18 | |
| Longpass filter 515 nm, 12.7 mm | 1 | Emission filter (blocks excitation below 515 nm) | Newport | 5CGA-515 | https://www.newport.com/p/5CGA-515 | 38.00 | 2 wk | in use | #18 | Only works with a blue excitation LED, not white |
| White LED 5 mm (super bright) | 1 | Excitation (current BOM) | Adafruit | | (Base Unit BOM sheet) | 0.40 | | retired | #18 | White light passes the 515 nm longpass; use blue |
| Blue LED 470 to 490 nm + excitation filter | 1 | FAM / SYBR excitation | | | | | | planned | #18 | |

## Indicators, controls, wiring, enclosure

| Part | Qty | Purpose (why) | Vendor | Part number | Link | Unit cost | Lead | Status | Owner | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| RGB LEDs 5 mm (pack) | 3 | Status: heating, cooling, hold, fault | Amazon | CHANZON | (Base Unit BOM sheet) | 0.09 | | in use | #13 | |
| Active buzzer 5 V | 1 | End-of-run and fault alarm | Amazon | | (Base Unit BOM sheet) | 0.75 | | in use | #13 | |
| Tactile push switch 12 mm | 2 | Start / stop (if not on touch) | eBay | | (Base Unit BOM sheet) | 0.13 | | alternative | #12 | Touchscreen may make these redundant |
| Wire 18 AWG solid, tinned | as needed | Power wiring | Amazon | | (Base Unit BOM sheet) | | | in use | #16 | |
| Wago 222-413 3-wire connectors | as needed | Power splices | Amazon | 222-413 | (Base Unit BOM sheet) | 0.26 | | in use | #16 | |
| PLA filament light grey 1.75 mm | as needed | Enclosure prints | Amazon | Overture | (Base Unit BOM sheet) | | | in use | #16 | |
| PLA filament space grey 1.75 mm | as needed | Enclosure prints | Amazon | Overture | (Base Unit BOM sheet) | | | in use | #16 | |
| Fuse, reverse-polarity protection | 1 | Supply protection | | | | | | planned | #16 | |
| RTC module DS3231 class | 1 | Timestamps without network | | | | | | planned | #14 | |
| Lid clamp hardware | 1 | Press the heated lid onto the tube cap | custom | | | | | planned | #16 | |

## Consumables

| Part | Qty | Purpose (why) | Vendor | Part number | Link | Unit cost | Lead | Status | Owner | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0.2 ml PCR tubes | per run | Sample vessel | | | | | | planned | #16 | Well format decision pending |
| Serial sticker | 1 | Device serial for run records | | | | | | planned | #14 | |

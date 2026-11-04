# Running bill of materials

Engineering record of every part in the instrument (rule: `.cursor/rules/running-bom.mdc`).
Quantities are per unit. The purchasing spreadsheet ("Base Unit BOM", Google Sheets) is the
ordering record and is updated from this file. Lead times over four weeks are marked **(long)**.

Status: `in use` = in the current design, `planned` = chosen by an issue but not yet built,
`ordered` = purchase placed (date and order in Notes), `alternative` = qualified substitute,
`retired` = removed, reason given.

Rev: which hardware revision a row belongs to, so one revision's parts list can be read on its
own (filter on Rev and Status). `v1` = the November 2026 build (bench unit and the ten units);
`v1-dev` = bench-only bring-up part, never in a shipped unit; `v1-alt` = qualified substitute
for a v1 part; `v2-candidate` = evaluated for the next revision, not in v1; `legacy` = Kalix
design part retired from v1.

## Controller, display, storage

| Part | Qty | Purpose (why) | Vendor | Part number | Link | Unit cost | Lead | Status | Rev | Owner | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Waveshare ESP32-P4-WIFI6 3.5inch Touch LCD development board (P4 + ESP32-C6, 3.5" 320×480 SPI touch display on ribbon, OV5647 camera, mic, speaker) | 1 | Single controller with the operator touchscreen attached by the board's own ribbon connector: no hand-wired display lines, touch included; C6 gives Wi-Fi 6 and BLE 5 | Waveshare (also Amazon) | ESP32-P4-WIFI6-Touch-LCD-3.5 (confirm exact model) | (Base Unit BOM sheet) |  |  | in use | v1 | #2 | LI-DL-004 revised 2026-10-30; free GPIO count and pin map to confirm from the wiki |
| Waveshare ESP32-P4-Module-DEV-KIT (P4NRW32X + ESP32-C6, 16 MB flash, 32 MB PSRAM) | 1 | Previous controller; 28 free GPIOs on a 40-pin header, display hand-wired over SPI | Waveshare (also Amazon) | SKU 30560, ESP32-P4-Module-DEV-KIT (basic kit) | https://www.amazon.com/dp/B0F2MP483W | 19.99 | in stock | retired | legacy | #2 | Replaced by the Touch LCD board to drop the hand-wired display; keep on hand as a bench spare |
| 3.5" TFT ILI9488 480×320 SPI shield | 1 | Operator touchscreen on the previous board; verified at 20 MHz over SPI | Amazon |  | (Base Unit BOM sheet) |  |  | retired | legacy | #3, #11 | Display now ships with the Touch LCD board |
| Nextion 3.2" HMI NX4024T032 | 1 | Legacy display; needs the Kalix HMI project file to be usable | Amazon | NX4024T032 | (Base Unit BOM sheet) | 34.49 |  | retired | legacy | #2 | Replaced by the ILI9488 on the P4; no HMI file |
| Micro SD card module | 1 | Run records and logs (if the onboard slot is not used) | Amazon |  | (Base Unit BOM sheet) | 2.99 |  | alternative | v1-alt | #14 | P4 kit has an onboard slot |
| SanDisk 32 GB microSDHC | 1 | Run record storage | Amazon |  | (Base Unit BOM sheet) | 6.33 |  | in use | v1 | #14 |  |
| Arduino Nano ESP32 | 1 | Earlier controller candidate | Amazon | ABX00083 | (Base Unit BOM sheet) | 23.00 |  | retired | legacy | #2 | Fewer pins; no advantage over the P4 kit |

| PCA9685 16-channel 12-bit PWM driver breakout | 1 | I2C PWM for the status LEDs, the excitation LED enables and a passive buzzer; frees J6 GPIOs (slow signals only, 1.6 kHz max) | Adafruit | 815 | https://www.adafruit.com/product/815 | 14.95 | | planned | v1 | #50 | Address 0x40; V+ from J6 pin 2 (5 V) for the LEDs; same chip goes SMD on the carrier |
| ADS1115 4-channel 16-bit ADC breakout | 1 | I2C ADC for the H-bridge current sense (R_IS, L_IS) and two spare inputs; frees GPIO51/52 and beats the P4's own ADC | Adafruit | 1085 | https://www.adafruit.com/product/1085 | 14.95 | | planned | v1 | #50, #7 | Address 0x48 |
| MCP23017 16-channel I/O expander breakout, STEMMA QT | 1 | I2C inputs and slow outputs: lid-open switch, cutoff sense, future enables | Adafruit | 5346 | https://www.adafruit.com/product/5346 | 5.95 | | planned | v1 | #50 | Address 0x20; optional on the bench unit, fitted on the carrier |
| STEMMA QT / Qwiic cable 100 mm | 4 | Daisy-chain the I2C breakouts from one J6 header | Adafruit | 4210 | https://www.adafruit.com/product/4210 | 0.95 | | planned | v1 | #50 | Bench unit only; the carrier routes I2C on copper |

## Thermal

| Part | Qty | Purpose (why) | Vendor | Part number | Link | Unit cost | Lead | Status | Rev | Owner | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Peltier module CUI CP303385H, 30×30×3.85 mm, 3 A | 1 | Heats and cools the block | Digi-Key | CP303385H (Digi-Key 102-4045-ND) | https://www.digikey.com/en/products/detail/cui-devices/CP303385H/6217103 | 26.79 | 13 wk **(long)** | in use | v1 | #7, #16 | Verify Vmax against the supply rail; substitute an in-stock 12 V TEC for November |
| Heat sink ATS-1146-C1-R0, half brick 58×61×22.9 mm | 1 | Hot-side sink for the Peltier | Digi-Key | ATS-1146-C1-R0 (Digi-Key ATS1506-ND) | https://www.digikey.com/en/products/detail/advanced-thermal-solutions-inc/ATS-1146-C1-R0/4146488 | 17.74 | 8 wk **(long)** | in use | v1 | #16 | Any 40 mm CPU cooler is the in-stock alternative |
| Fan 40×10 mm 12 V dual ball bearing | 1 | Heat sink airflow | Amazon | YDM4010B12 | (Base Unit BOM sheet) | 6.99 |  | in use | v1 | #7 | Set the fan voltage to 12 V or buy 24 V fans |
| Fan guard 40 mm | 1 | Finger guard | Amazon |  | (Base Unit BOM sheet) | 1.50 |  | in use | v1 | #16 |  |
| Thermal paste Corsair XTM70 | as needed | Peltier to sink and block interface | Amazon |  | (Base Unit BOM sheet) |  |  | in use | v1 | #16 |  |
| Polyimide film heater 25 mm, 20 W | 1 | Heated lid | eBay |  | (Base Unit BOM sheet) | 6.65 |  | in use | v1 | #8 | 12 V part on hand; order the 24 V, 20 W variant if the TEC rail is 24 V (#33) |
| NTC ring thermistor 10 kΩ, wire leaded | 1 | Lid temperature (legacy, corner lug) | Newark | Molex 213860-1637 | https://www.newark.com/molex/213860-1637/ntc-ring-thermistor-10k-wire-leaded/dp/54AH2536 | 5.36 |  | retired | legacy | #8 | Replaced by a PT1000 cartridge on a second MAX31865 (2026-11-04): no free ADC pin on the P4 header, and one sensor stack for block and lid |
| MAX6675 module + K-type thermocouple | 1 | Block temperature for bring-up only (1 °C integer, ±2 °C) | SainSmart | 101-60-176 |  | 9.99 |  | alternative | v1-dev | #6 | Replaced by PT1000 + MAX31865 for the product |
| MAX31865 RTD amplifier, Adafruit 3648 (PT1000, 4.3 kΩ 0.1 % ref) | 2 | RTD to digital over SPI: one for the block (#6), one for the lid (#8); open/short faults in hardware | Adafruit | 3648 | https://www.adafruit.com/product/3648 | 14.95 |  | ordered 2026-11-04 (order 3776961-2867127889, USPS Priority, 2 pcs, 42.45 total) | v1 | #6, #8 | 2-wire mode: bridge F+ to RTD+ and F− to RTD− at the terminal block. Reference 4300 Ω goes into the #24 driver |
| PT1000 cartridge 3 × 15 mm stainless, 2-wire, Slice Engineering RTD Pt1000 | 2 | Block sensor (blind hole to the block centre, #33) and lid sensor (hole near the centre of the lid plate) | Bondtech / Slice | RTD Pt1000 | https://www.bondtech.se/product/pt1000-rtd-temperature-sensor/ | 29.99 |  | planned | v1 | #6, #8 | IEC 60751 class B; the two-point calibration in #6 removes R0 and slope error. 2 m leads |
| PT1000 cartridge 3 × 15 mm, TriangleLab 450 °C (alternative) | 2 | Same holes, lower cost | West3D | PT1000 cartridge | https://west3d.com/products/pt1000-thermistor-cartridge-450c | 12.00 |  | alternative | v1-alt | #6, #8 | Tolerance class not stated |
| PT1000 thin-film element class A, Heraeus M222 (alternative, block) | 1 | Tightest absolute accuracy for the block; potted in a 2 mm hole | RS | M222 PT1000 class A (RS 4538105) | https://uk.rs-online.com/web/p/rtd-sensors/4538105 | 3.00 |  | alternative | v1-alt | #6 | Pack of 5; ±0.35 °C at 100 °C before calibration |
| Thermal cutoff, bimetal 130 °C normally closed, in series with the lid heater | 1 | Hardware backstop if the lid FET fails shorted (software limit is 120 °C) | Digi-Key | Sensata Airpax 67F130 (or equivalent 130 °C NC bimetal) | https://www.digikey.com/en/products/filter/thermostats-mechanical/1035 | 3.50 |  | planned | v1 | #8, #5 | Part number to confirm against the heater current and rail voltage |
| H-bridge driver IBT_2 (HW-039), dual BTS7960, 43 A | 1 | Bidirectional Peltier drive so the PID can cool | Amazon | HW-039 / IBT_2 |  |  | on hand | in use | v1 | #7 | Replaces the relay polarity switch. Logic VCC on 3V3. Bench-verified 2026-11-03 on GPIO3/4/5 (PINS.md) |
| LC filter for the bridge output (≈47 µH, 470 µF) | 1 | Low-ripple TEC current |  |  |  |  |  | planned | v1 | #7 |  |
| Pololu Simple Motor Controller G2 18v25 | 1 | I2C/serial bidirectional Peltier driver with built-in current limit and fault reporting; evaluated as a replacement for the IBT_2 if direct PWM proves hard to tune or certification wants a qualified driver | Pololu | 1366 | https://www.pololu.com/product/1366 | 59.95 | | alternative | v2-candidate | #7 | Not in v1: IBT_2 on GPIO3/4/5 is bench-verified and free |
| Meerstetter TEC-1092 precision TEC controller | 1 | UART-controlled TEC controller with PID, autotune, limits and safety; escalation path if the #7 loop cannot meet ramp and overshoot targets | Meerstetter | TEC-1092 | https://www.meerstetter.ch/products/tec-controllers/tec-1092 | 480.00 | 4 wk | alternative | v2-candidate | #7 | Would replace the Klipper-port loop and the IBT_2; cost rules it out for v1 |
| Heat block, aluminium, drilled for 0.2 ml tubes | 1 | Sample block | custom |  |  |  |  | planned | v1 | #16 | Well format decision pending |

## Power

| Part | Qty | Purpose (why) | Vendor | Part number | Link | Unit cost | Lead | Status | Rev | Owner | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Power supply 15 V 5 A | 1 | Main supply (current BOM) | Amazon |  | (Base Unit BOM sheet) | 23.60 |  | alternative | v1-alt | #16 | Marginal at full TEC + lid load |
| Power supply 24 V 6 A, e.g. Mean Well LRS-150-24 | 1 | Main supply (planned), UL listed |  | LRS-150-24 |  |  |  | planned | v1 | #16 |  |
| Buck converter 24 V to 12 V, 10 A | 1 | TEC rail from a 24 V supply |  |  |  |  |  | planned | v1 | #16 |  |
| Linear regulator 5 V 1.5 A TO-220 | 1 | Logic rail | Digi-Key | 497-1443-5-ND |  | 0.35 | 12 wk **(long)** | retired | legacy | #16 | 15 W dissipation at 15 V in; use a buck module |
| Linear regulator 12 V 1.5 A TO-220 | 1 | 12 V rail | Digi-Key | 497-1452-5-ND |  | 0.43 | 12 wk **(long)** | retired | legacy | #16 | Use a buck module |
| MOSFET N-ch logic-level IRLZ44N TO-220, 55 V 47 A | 1 | Lid heater low-side switch driven straight from a 3.3 V GPIO (LEDC PWM) | Digi-Key | IRLZ44NPBF (Digi-Key IRLZ44NPBF-ND) | https://www.digikey.com/en/products/detail/infineon-technologies/IRLZ44NPBF/811808 | 1.30 |  | planned | v1 | #8 | Replaces STP55NF06, which needs ~10 V on the gate; lid current 1.7 A at 12 V, under 1 A at 24 V |
| MOSFET N-ch 60 V 50 A TO-220, STP55NF06 | 1 | Lid heater switch (legacy) | Digi-Key | 497-2777-5-ND | https://www.digikey.com/en/products/detail/stmicroelectronics/STP55NF06/603802 | 1.05 | 26 wk **(long)** | retired | legacy | #8 | Standard-level gate (full enhancement at 10 V); half-on from 3.3 V or 5 V logic. Usable only behind a gate driver |
| MOSFET N-ch 142 A 40 V (alternative) | 1 | Lid heater switch | Rochester Electronics |  | (Base Unit BOM sheet) | 0.98 |  | alternative | v1-alt | #8 |  |
| MOSFET N-ch 40 V 100 A TO-220 (alternative) | 1 | Lid heater switch | Digi-Key |  | (Base Unit BOM sheet) | 1.10 |  | alternative | v1-alt | #8 |  |
| MOSFET NexFET 40 V (alternative) | 1 | Lid heater switch | Mouser |  | https://mou.sr/3lbss0x | 1.10 |  | alternative | v1-alt | #8 |  |
| DC jack 5.5×2.1 mm | 1 | Power input | Amazon |  | (Base Unit BOM sheet) | 0.50 |  | in use | v1 | #16 |  |
| Rocker switch | 1 | Power on/off | Amazon |  | (Base Unit BOM sheet) | 0.90 |  | in use | v1 | #16 |  |
| Relay module 5 V 2-channel | 1 | Legacy Peltier polarity switch | Amazon |  | (Base Unit BOM sheet) | 3.17 |  | retired | legacy | #7 | Replaced by the H-bridge |

## Optics

| Part | Qty | Purpose (why) | Vendor | Part number | Link | Unit cost | Lead | Status | Rev | Owner | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| AS7341 10-channel spectral sensor breakout | 1 | Fluorescence detector | Digi-Key |  | (Base Unit BOM sheet) | 15.95 |  | in use | v1 | #18 |  |
| Longpass filter 515 nm, 12.7 mm | 1 | Emission filter (blocks excitation below 515 nm) | Newport | 5CGA-515 | https://www.newport.com/p/5CGA-515 | 38.00 | 2 wk | in use | v1 | #18 | Only works with a blue excitation LED, not white |
| White LED 5 mm (super bright) | 1 | Excitation (current BOM) | Adafruit |  | (Base Unit BOM sheet) | 0.40 |  | retired | legacy | #18 | White light passes the 515 nm longpass; use blue |
| Blue LED 470 to 490 nm + excitation filter | 1 | FAM / SYBR excitation |  |  |  |  |  | planned | v1 | #18 |  |

## Indicators, controls, wiring, enclosure

| Part | Qty | Purpose (why) | Vendor | Part number | Link | Unit cost | Lead | Status | Rev | Owner | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| RGB LEDs 5 mm (pack) | 3 | Status: heating, cooling, hold, fault | Amazon | CHANZON | (Base Unit BOM sheet) | 0.09 |  | in use | v1 | #13 |  |
| Active buzzer 5 V | 1 | End-of-run and fault alarm (legacy) | Amazon | | (Base Unit BOM sheet) | 0.75 | | alternative | v1-alt | #13, #50 | Preferred: tone from the board's ES8311 codec and speaker, no part and no pin; if a buzzer is kept, a passive one on a PCA9685 channel |
| Tactile push switch 12 mm | 2 | Start / stop (if not on touch) | eBay |  | (Base Unit BOM sheet) | 0.13 |  | alternative | v1-alt | #12 | Touchscreen may make these redundant |
| Wire 18 AWG solid, tinned | as needed | Power wiring | Amazon |  | (Base Unit BOM sheet) |  |  | in use | v1 | #16 |  |
| Wago 222-413 3-wire connectors | as needed | Power splices | Amazon | 222-413 | (Base Unit BOM sheet) | 0.26 |  | in use | v1 | #16 |  |
| PLA filament light grey 1.75 mm | as needed | Enclosure prints | Amazon | Overture | (Base Unit BOM sheet) |  |  | in use | v1 | #16 |  |
| PLA filament space grey 1.75 mm | as needed | Enclosure prints | Amazon | Overture | (Base Unit BOM sheet) |  |  | in use | v1 | #16 |  |
| Fuse, reverse-polarity protection | 1 | Supply protection |  |  |  |  |  | planned | v1 | #16 |  |
| RTC module DS3231 class | 1 | Timestamps without network |  |  |  |  |  | planned | v1 | #14 |  |
| Lid clamp hardware | 1 | Press the heated lid onto the tube cap | custom |  |  |  |  | planned | v1 | #16 |  |

## Consumables

| Part | Qty | Purpose (why) | Vendor | Part number | Link | Unit cost | Lead | Status | Rev | Owner | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0.2 ml PCR tubes | per run | Sample vessel |  |  |  |  |  | planned | v1 | #16 | Well format decision pending |
| Serial sticker | 1 | Device serial for run records |  |  |  |  |  | planned | v1 | #14 |  |

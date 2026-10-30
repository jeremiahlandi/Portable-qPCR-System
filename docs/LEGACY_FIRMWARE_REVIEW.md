# Review of the Kalix Labs legacy firmware

Files reviewed (byte-identical to the uploads `Kal_ATMega_New_7_1.ino` and `Kal_ESP_6_No_ID_1.ino`):

- `Firmware/Legacy_Archive/Kal_ATMega_New_7.ino` (2613 lines, ATmega2560)
- `Firmware/Legacy_Archive/Kal_ESP_6_No_ID.ino` (2240 lines, classic ESP32)

Line numbers below refer to the archived copies.

## 1. What the delivered system is

Two microcontrollers, loosely coupled over a 9600-baud UART plus four GPIO handshake lines.

**ATmega2560 = thermal cycler.** It owns every actuator and sensor that matters:

| Function | Hardware | Pins |
| --- | --- | --- |
| Well temperature | MAX6675 K-type thermocouple, bit-banged | CLK 52, CS 29, DO 50 |
| Lid temperature | NTC thermistor on a 5.49 kΩ divider, log fit | A0 |
| Peltier drive | PWM on a MOSFET, polarity via two relay channels (H-bridge) | PWM 6, relays 39/41 |
| Lid heater | PWM on a MOSFET | 4 |
| HMI | Nextion touchscreen | Serial1 (115200) |
| Clock | DS1307 RTC over I2C | A4/A5 |
| Protocol presets | SD card, tab-separated `kalds.txt` | CS 40 (hardware SPI) |
| Link to ESP32 | Serial0 (shared with USB) + GPIO 48, 11, 12, 13, 25 | |

It implements four run types: 1 = reverse transcription (up to 3 holds), 2 = PCR (2-step or 3-step, optional activation and initial hold, final hold), 3 = extraction (single hold), 4 = PCR loaded from the SD preset table by disease code. Control is a one-second Timer1 interrupt plus a big `if/else` state machine in `loop()`. Each stage ramps at full power until within a few degrees, then runs a PID_v1 loop during the hold.

**ESP32 = phone bridge + (intended) optics.** It bridges Bluetooth Classic SPP to the Mega, re-encodes the same byte protocol, pushes a status frame to the phone once a second, logs run completion to `kaldata.txt` on its own SD card, and initializes an AS7341 spectral sensor. WiFi, web server, and TensorFlow Lite classification are all present as commented-out stubs.

## 2. Verdict

The thermal cycler half works as a prototype but is not production firmware. The optics half is not implemented. The code is a single file of ~300 globals with copy-pasted serial blocks, no framing or checksums, blocking loops with no timeouts, and several real control and safety defects. I recommend treating it as a **behavioral specification** (what stages exist, what the phone app expects on the wire, what the Nextion pages are called) and rewriting rather than porting.

## 3. Findings, ATmega2560 (thermal control)

Ordered by severity.

### Safety

1. **Overtemperature hard-stop is effectively disabled during a run.** Line 429 only enters the `emergency` state when `flag_lcd_init == 1`. That flag is set by Nextion command 109 and cleared in `idle` immediately, so during a run it is 0. The only remaining protection is `temp < emer_temp` in the hold `while` loops, and when that trips the code advances to the next stage instead of stopping. The ramp branches (full power) have no check at all. The 150 °C lid limit is never enforced.

2. **An open thermocouple drives the Peltier at full power with no cutoff.** `Temp_Read()` (line 2349) does `temp = thermocouple.readCelsius() + 1;`. The MAX6675 library returns NaN when the thermocouple is open. NaN cast to `int` on AVR gives 0 or INT_MIN. Either value is below every setpoint, so every stage heats at 255 forever, and the `temp < emer_temp` guard stays true.

3. **A shorted lid thermistor freezes the lid reading.** `Lid_Read()` (line 2353) skips the update when the ADC reads 0, so `temp_lid` holds its last value and `Lid_Pid()` keeps heating against stale data. An open thermistor does produce a huge value and would trip the (disabled, see 1) limit.

4. **Emergency state requires a power cycle** (`while(1)` at line 2148). Acceptable for a prototype, but there is no logging, no message to the phone, and the Nextion is not told.

### Control correctness

5. **Initial hold is bang-bang by accident.** In the `start` stage the PID is computed (line 1467) but `denatInput` is never assigned from `temp` (compare line 1535 in `denat`). The PID runs on a stale input, saturates at 255 until `temp > init_temp`, then the code forces 0. Expect overshoot on the first denaturation.

6. **Temperature is an integer.** `temp` is `int`, so the PID sees 1 °C steps and the thermocouple's 0.25 °C resolution is thrown away. The MAX6675 is also ±2 °C class. Annealing specificity in PCR is sensitive to about ±0.5 °C. This is the most important hardware-level limitation for the rebuild.

7. **Each temperature read blocks 250 ms** (line 2350). The whole control loop runs at roughly 3 Hz with the PID's default 100 ms sample time, so it computes on every pass but with uneven timing.

8. **No active cooling during holds.** Cooling is only used to ramp down (relays reversed, PWM 255). Once in a hold the relays are set to heat and the PID can only heat or coast. Overshoot below an anneal setpoint cannot be corrected.

9. **Reverse-transcription holds below 55 °C flip the relays and PID direction on a ±0.5 °C band** (line 1871 onward). That is relay chatter around the setpoint, and PID_v1's integral is not reset when direction flips.

10. **Relay polarity is switched without zeroing PWM in the ramp branches** (e.g. line 1403). Transitions from cooling to heating switch the relay contacts under full Peltier current.

11. **Lid PID gains (75, 25, 25) are effectively bang-bang** and the lid setpoint is fixed at 60 °C (line 92). Commercial heated lids run 100 to 105 °C to stop condensation. Worth confirming what the lid actually is.

12. **Pause is not implemented.** Nextion 107/108 set `flag_pause` (line 2572) but the timer checks `pause` (line 2326), which is never written, and nothing stops the heater. The `qPCR` flag, the three physical buttons, the `light` pin, and the `select`, `fin`, `fluo_meas` states are all dead.

### Robustness

13. **Serial work inside the interrupt.** The 2 Hz ISR (line 406) calls `Flag_Check()`, which writes several hundred bytes to the Nextion, reads commands, and calls `Read_data()` (line 2600) which contains `delay(50)`. `delay()` inside an ISR on AVR only works by accident of how `micros()` handles a pending overflow. The ISR can take tens of milliseconds, and `Serial1` is read from both the ISR and `loop()` with a non-atomic, non-volatile `flag_read` as the only guard.

14. **No shared variable is `volatile`.** `flag`, `flag_btn`, `pcr_step`, `count_*` and the time counters are written in the ISR and read in `loop()`. It works today only because every polling loop happens to call a function, which forces the compiler to reload globals.

15. **Blocking loops with no timeout.** Lines 616, 621, 653, 716 (`while (digitalRead(ESP_sense) == 1)`, `while (char_temp != 127)`), the `set`, `rna_set`, `extract_set` and `pcr` loops, and the SD line-skip loop at line 1977 (infinite if the file has fewer lines than the disease code). A dropped byte or a cable fault hangs the controller.

16. **Wire protocol is raw bytes with magic terminators (127, 254/0xFF) and no checksum.** Any data value equal to a terminator corrupts the frame. Temperatures, hours, minutes and seconds are each one byte; the inter-MCU exchange adds `delay(60)` per byte (107 occurrences), so a PCR configuration takes about 2 seconds to transfer.

17. **Mega Serial0 is shared between the ESP32 link and USB.** No debugging output is possible while the ESP is connected, and the 5 V Mega UART into the 3.3 V ESP32 needs a level shifter (hardware note; the ESP32-P4 is also not 5 V tolerant).

18. **Hardware SPI is shared with the bit-banged thermocouple.** MAX6675 uses pins 50/52 (MISO/SCK). The code works around it with `SPI.end()` after SD init (line 387) and `SPI.begin()`/`SPI.end()` around preset reads (lines 1952, 2124). Fragile but functional.

19. **Type 4 (preset by disease code) over Bluetooth is not wired through.** The ESP sends command 4; the Mega only handles 1, 2, 3 in the `ESP_sense` path. Presets only work from the Nextion (command 102) reading the Mega's own SD card.

20. **Minor.** `Rem_Time()` has a parenthesis bug (`abs((lid_temp - temp_lid / heat_rate))`) and a missing `/heat_rate` in the type 1 branch. `percentage_elap` divides by `estim_sec`, which can be 0. The denaturation hold silently adds 2 seconds (`hold_denat * 2 + 4`). `Serial.flush()` at line 804 flushes TX, not the RX buffer the author intended to clear.

## 4. Findings, ESP32 (bridge and optics)

1. **No fluorescence measurement happens.** `Fluo_Read()` and `Read_Chan()` (lines 1830 to 1880) are never called. The "qPCR" button at line 1747 only blinks two excitation LEDs for a second each. There is no per-cycle read, no excitation/read synchronization with the cycler, no baseline, no Ct calculation, and no data path to the phone or SD. The 38 KB `fluo_data[8][16][75]` array (line 218) is never used. The product is named qPCR but the delivered firmware is a conventional PCR cycler.

2. **AS7341 integration time is set by mistake.** `setATIME(5500)` (line 427) targets a uint8 register, so it truncates to 124. The resulting integration is about 1.7 s at 32× gain. Any optical validation Kalix did was with this accidental setting.

3. **Out-of-bounds read on a zero-length array.** `animal_IDs[]` (line 228) is empty but indexed up to `num_samples` in both type 4 status senders (lines 1686, 1961). If a type 4 run ever reached status, this reads `String` objects off the end of the array and would likely crash.

4. **Blocking receive loops with no timeout** at lines 571, 704, 925 (phone packet) and 987, 1041, 1137 (Mega packet). A phone disconnect mid-packet hangs the bridge until reset.

5. **A debug `Serial.println("out")` (line 951) goes onto the Mega link.** Harmless today only because the Mega ignores unknown bytes in idle, which confirms the type 4 path was never tested end to end.

6. **Vestigial half-features on both sides.** The Mega waits up to 2 s for animal/disease strings the ESP no longer sends (commented out at lines 581 to 590 of the ESP file). `animal_IDs_received`, the eight `m1..m8` arrays, WiFi AP, web server and TFLite are dead.

7. **Not portable to the ESP32-P4 as written.** `BluetoothSerial` (Bluetooth Classic SPP) does not exist on the P4; the P4 has no radio and the Waveshare kit's ESP32-C6 companion only offers WiFi and BLE through ESP-Hosted. The `timerBegin(id, prescaler, edge)` call is the Arduino-ESP32 2.x API and does not compile on the 3.x core the P4 needs. The phone app's SPP protocol therefore has to change to BLE or WiFi regardless of anything else.

## 5. What is worth keeping

- The **stage model**: lid pre-heat, optional activation, optional initial hold, cycles of denature/anneal/extend (2-step or 3-step), final hold; plus RT (1 to 3 holds) and extraction (1 hold). This is the behavior the Kalix app and users already understand.
- The **phone protocol as documentation**: command bytes 122 to 127 and the field order per run type are fully recoverable from the two files, so a new BLE or WiFi protocol can offer the same fields.
- The **Nextion page and object names** (`page1`, `page33`, `page36`, `status`, `lid`, `well`, `cycle`, `totcyc`, `h/m/s`, `j0`) if the touchscreen is reused, though the P4 build already drives an ILI9488 directly.
- The **relay H-bridge plus PWM** approach is cheap and works, but should be replaced by a proper H-bridge driver (see parts list) so cooling can be used during holds.

## 6. Implications for the ESP32-P4 rebuild

Current repo state: `src/main.cpp` is a verified display hello-world on the P4 with LovyanGFX; the PIO config targets the pioarduino fork; MAX6675 is listed as a dependency and `PINS.md` wires it on GPIO 4/5/6.

Decisions the rebuild has to make, in rough priority:

1. **Single MCU.** The P4 can run display, thermal control, optics, logging and the C6 radio link itself. The two-MCU split and its byte handshake should not be recreated.
2. **Safety first.** Independent watchdog, sensor-fault detection (NaN, stuck, out-of-range on both sensors), a hard overtemp cutoff that cannot be disabled by UI state, and fail-safe actuator defaults on any fault or reset.
3. **Temperature sensing.** The MAX6675 is already on the bench and fine for bring-up, but a real qPCR block wants a PT100/PT1000 or a precision thermistor with a proper ADC (MAX31865 or similar), 0.1 °C resolution, and a calibration routine. Decide now whether the thermocouple is a stepping stone or the product sensor.
4. **Actuation.** A real H-bridge (BTS7960 or a TEC driver) with signed PWM so the PID can cool as well as heat, with the lid as a second loop.
5. **Control loop.** A fixed-rate task (10 to 20 Hz) with a float PID, ramp-rate limiting, per-stage feed-forward, and anti-windup. Nothing blocking, no serial in interrupts.
6. **Optics, the actual qPCR part.** Decide the measurement: AS7341 per well versus a photodiode/filter set, how many wells, excitation LED control, timing (read at end of each extension), background subtraction, and Ct calling. None of this exists yet.
7. **Connectivity.** BLE or WiFi via the C6, with a framed protocol (length, type, CRC) and versioned messages. Keep the field semantics from the legacy protocol so the app port is mechanical.
8. **Architecture.** Split into modules (sensors, actuators, thermal control, protocol runner, UI, comms, storage) with a protocol/run description as data, not as 28-way `if` chains.

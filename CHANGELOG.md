# Changelog

## [1.1.0](https://github.com/jeremiahlandi/Portable-qPCR-System/compare/v1.0.0...v1.1.0) (2026-11-04)


### Features

* **ui:** LVGL home screen with status bar, launcher tiles and touch ([#11](https://github.com/jeremiahlandi/Portable-qPCR-System/issues/11)) ([#36](https://github.com/jeremiahlandi/Portable-qPCR-System/issues/36)) ([f90fabf](https://github.com/jeremiahlandi/Portable-qPCR-System/commit/f90fabfbdcfe43f13d6092914a7b2dafbbc26dd9))

## 1.0.0 (2026-11-02)


### Features

* **platform:** board-agnostic control core skeleton with native tests and CI ([#3](https://github.com/jeremiahlandi/Portable-qPCR-System/issues/3)) ([8106552](https://github.com/jeremiahlandi/Portable-qPCR-System/commit/8106552334c5942c8d5b9121cfd178654e072746))
* **platform:** bring up Waveshare ESP32-P4 Touch-LCD-3.5 on rev 3.x silicon ([#30](https://github.com/jeremiahlandi/Portable-qPCR-System/issues/30)) ([#31](https://github.com/jeremiahlandi/Portable-qPCR-System/issues/31)) ([ce90252](https://github.com/jeremiahlandi/Portable-qPCR-System/commit/ce902525a8d07228aab3069d2d9ce10d397aa001))
* **thermal:** port Klipper PID, heater check, RTD, thermistor and autotune with replay tests ([#4](https://github.com/jeremiahlandi/Portable-qPCR-System/issues/4)) ([#24](https://github.com/jeremiahlandi/Portable-qPCR-System/issues/24)) ([a3020a7](https://github.com/jeremiahlandi/Portable-qPCR-System/commit/a3020a73d8c9cc6c04ed808690cb818f1ed28c86))
* **ui:** show the firmware version bottom-right on the display ([#28](https://github.com/jeremiahlandi/Portable-qPCR-System/issues/28)) ([#29](https://github.com/jeremiahlandi/Portable-qPCR-System/issues/29)) ([50bca64](https://github.com/jeremiahlandi/Portable-qPCR-System/commit/50bca6458a7535173bc16dc7be91ba554f959c60))


### Bug Fixes

* Rename/Update main.cpp to resolve undefined reference to setup() and loop() ([f04feeb](https://github.com/jeremiahlandi/Portable-qPCR-System/commit/f04feeb393904cffd68503b59226ca8ed415532d))

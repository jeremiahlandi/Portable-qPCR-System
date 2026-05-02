# Waveshare ESP32-P4 qPCR Pinout & Wiring Colors

## 🗺 Reference Diagrams
1. [Waveshare P4 Dev-Kit Header Map](https://europe1.discourse-cdn.com/arduino/original/4X/3/2/d/32dc878cdeb932b9bf9e8611bebec2d9a789d12a.jpeg)
2. [TFT Display Module Pinout Reference](https://community.volumio.com/uploads/default/original/3X/6/8/6845be5563db2efd1b8c28e772407490ffddbc8b.jpeg)

---

## 📺 Display: 2.8" TFT SPI
| Screen Pin | P4 GPIO | Wire Color | Notes |
| :--- | :--- | :--- | :--- |
| **VCC** | **5V** | 🔴 **Red** | Connect to 5V rail |
| **GND** | **GND** | ⚫ **Black** | Connect to any GND |
| **CS** | **GPIO 20**| 🟣 **Purple** | Chip Select |
| **RESET**| **GPIO 21**| 🔘 **Grey** | Hardware Reset |
| **DC** | **GPIO 22**| 🔵 **Blue** | Data/Command |
| **SDI(MOSI)**| **GPIO 23**| 🟢 **Green** | SPI Master Out |
| **SCK** | **GPIO 24**| 🟡 **Yellow**| SPI Clock |
| **LED** | **3V3** | 🔴 **Red** | Backlight - Always On |

## 🌡 Sensor: MAX6675 Thermocouple
| Function | P4 GPIO | Wire Color | Notes |
| :--- | :--- | :--- | :--- |
| **VCC** | **3V3** | 🔴 **Red** | 3.3V Only |
| **GND** | **GND** | ⚫ **Black** | |
| **CLK** | **GPIO 6** | 🟡 **Yellow**| Match Display SCK style |
| **CS** | **GPIO 5** | 🟣 **Purple**| Match Display CS style |
| **SO** | **GPIO 4** | 🟢 **Green** | Match Display SDI style |
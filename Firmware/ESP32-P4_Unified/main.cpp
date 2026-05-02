#include <Arduino.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <max6675.h>

// Pins for MAX6675
int thermoCLK = 6;
int thermoCS = 7;
int thermoSO = 8;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoSO);

// Setup LovyanGFX for a generic SPI screen
class LGFX_P4 : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel_instance; // Assuming ST7789, adjust if your Hosyond is different
  lgfx::Bus_SPI       _bus_instance;
public:
  LGFX_P4() {
    auto cfg = _bus_instance.config();
    cfg.spi_host = SPI2_HOST; // P4 uses SPI2/SPI3
    cfg.pin_sclk = 13;        // Adjust these to your actual screen pins
    cfg.pin_mosi = 12;
    cfg.pin_miso = -1;
    cfg.pin_dc   = 14;
    _bus_instance.config(cfg);
    _panel_instance.setBus(&_bus_instance);
    
    auto p_cfg = _panel_instance.config();
    p_cfg.pin_cs = 15;
    p_cfg.pin_rst = 16;
    _panel_instance.config(p_cfg);
    setPanel(&_panel_instance);
  }
};

LGFX_P4 lcd;

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.setRotation(1);
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextColor(TFT_WHITE);
  lcd.setTextSize(3);
  lcd.drawString("qPCR Online", 10, 10);
}

void loop() {
  double temp = thermocouple.readCelsius();
  lcd.setCursor(10, 60);
  lcd.setTextSize(5);
  lcd.printf("%.2f C   ", temp);
  Serial.printf("Temp: %.2f\n", temp);
  delay(1000);
}
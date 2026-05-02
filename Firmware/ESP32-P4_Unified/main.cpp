#include <Arduino.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <max6675.h>

// Pins for MAX6675
int thermoCLK = 6;
int thermoCS = 7;
int thermoSO = 8;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoSO);

// Generic Display Setup
class LGFX_P4 : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel_instance; 
  lgfx::Bus_SPI       _bus_instance;
public:
  LGFX_P4() {
    auto cfg = _bus_instance.config();
    cfg.spi_host = SPI2_HOST;
    cfg.pin_sclk = 13;        
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
  lcd.drawString("qPCR Online", 10, 10);
}

void loop() {
  double temp = thermocouple.readCelsius();
  lcd.setCursor(10, 60);
  lcd.printf("Temp: %.2f C", temp);
  Serial.printf("Temp: %.2f\n", temp);
  delay(1000);
}
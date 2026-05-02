#include <Arduino.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

class LGFX_P4 : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9488 _panel_instance;
  lgfx::Bus_SPI       _bus_instance;
public:
  LGFX_P4() {
    auto cfg = _bus_instance.config();
    cfg.spi_host = SPI3_HOST;
    cfg.spi_mode = 0;
    cfg.freq_write = 20000000; // Original speed
    cfg.pin_sclk = 24; 
    cfg.pin_mosi = 23; 
    cfg.pin_dc   = 22; 
    _bus_instance.config(cfg);
    _panel_instance.setBus(&_bus_instance);

    auto p_cfg = _panel_instance.config();
    p_cfg.pin_cs = 20; 
    p_cfg.pin_rst = 21;
    p_cfg.panel_width = 320;
    p_cfg.panel_height = 480;
    _panel_instance.config(p_cfg);
    setPanel(&_panel_instance);
  }
};

LGFX_P4 lcd;

void setup() {
  Serial.begin(115200);
  delay(1000); 

  if (lcd.init()) {
    lcd.setRotation(1); 
    lcd.fillScreen(TFT_BLACK);
    
    // The exact text from last night
    lcd.setTextColor(TFT_WHITE);
    lcd.setTextSize(3);
    lcd.setCursor(20, 20);
    lcd.print("SYSTEM VERIFIED");
    
    lcd.setTextColor(TFT_GREEN);
    lcd.setCursor(20, 80);
    lcd.print("Brain: ONLINE");
    
    lcd.setTextColor(TFT_BLUE);
    lcd.setCursor(20, 140);
    lcd.print("Screen: ILI9488");
    
    Serial.println("Reverted to Verified Code State.");
  }
}

void loop() {
  delay(1000);
}
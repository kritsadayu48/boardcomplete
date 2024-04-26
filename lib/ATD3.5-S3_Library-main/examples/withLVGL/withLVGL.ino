#include <Arduino.h>
#include <lvgl.h>
#include <ATD3.5-S3.h>
#include <lv_demos.h>

void setup() {
  Serial.begin(115200);

  Display.begin(0); // rotation number 0
  Touch.begin();
  Sound.begin();

  Display.useLVGL(); // Map display to LVGL
  Touch.useLVGL(); // Map touch screen to LVGL
  Sound.useLVGL(); // Map speaker to LVGL
  
  // lv_demo_music();
  lv_demo_widgets();
}

void loop() {
  Display.loop();
}

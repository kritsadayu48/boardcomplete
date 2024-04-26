#include <Arduino.h>
#include <lvgl.h>
#include <ATD3.5-S3.h>

void setup() {
  Serial.begin(115200);

  Display.begin(0); // rotation number 0
  Touch.begin();
  Sound.begin();
  Card.begin();

  Display.useLVGL(); // Map display to LVGL
  Touch.useLVGL(); // Map touch screen to LVGL
  Sound.useLVGL(); // Map speaker to LVGL
  
  lv_obj_t * img1 = lv_img_create(lv_scr_act());
  lv_img_set_src_from_sd_card(img1, "/logo.png"); // load logo.png to img1
  lv_obj_align(img1, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_size(img1, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
}

void loop() {
  Display.loop();
}

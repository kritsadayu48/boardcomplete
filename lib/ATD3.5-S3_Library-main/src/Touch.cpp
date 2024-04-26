#include <Arduino.h>
#include <Wire.h>
#include "LCD.h"
#include "Touch.h"

static const char * TAG = "Touch";

#define FT6336_ADDR 0x38

#define TOUCH_SDA_PIN (15)
#define TOUCH_SCL_PIN (16)

TwoWire TouchWire(1);

FT6336::FT6336() { }

void FT6336::begin() {
    TouchWire.begin(TOUCH_SDA_PIN, TOUCH_SCL_PIN, (uint32_t) 400E3);
}

uint8_t FT6336::read(uint16_t *cx, uint16_t *cy) {
    TouchWire.beginTransmission(FT6336_ADDR);
    TouchWire.write(0x02); // Set point to TD_STATUS 
    if (TouchWire.endTransmission(false) != 0) {
        ESP_LOGE(TAG, "Write error !");
        return 0;
    }

    uint8_t count = TouchWire.requestFrom(FT6336_ADDR, 5);
    if (count != 5) {
        ESP_LOGE(TAG, "Read error !");
        return 0;
    }

    // Process Data
    uint8_t TD_STATUS = TouchWire.read();
    uint8_t TOUCH1_XH = TouchWire.read();
    uint8_t TOUCH1_XL = TouchWire.read();
    uint8_t TOUCH1_YH = TouchWire.read();
    uint8_t TOUCH1_YL = TouchWire.read();
    /*
    *cx = (((uint16_t)TOUCH1_XH&0x0F)<<8)|TOUCH1_XL;
    *cy = (((uint16_t)TOUCH1_YH&0x0F)<<8)|TOUCH1_YL;
    */
    
    uint16_t x = (((uint16_t)TOUCH1_XH&0x0F)<<8)|TOUCH1_XL;
    uint16_t y = (((uint16_t)TOUCH1_YH&0x0F)<<8)|TOUCH1_YL;
    uint8_t m = Display.getRotation();
    if (m == 0) {
        *cx = y;
        *cy = 320 - x;
    } else if (m == 1) {
        *cx = 480 - y;
        *cy = x;
    } else if (m == 2) {
        *cx = 320 - x;
        *cy = 480 - y;
    } else if (m == 3) {
        *cx = x;
        *cy = y;
    }

    uint8_t touchPoint = TD_STATUS&0x0F;
    if (touchPoint > 5) {
        touchPoint = 0;
    }

    return touchPoint;
}

#ifdef USE_LVGL
static void touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t * data) {
  FT6336 * touch = (FT6336 *) indev_driver->user_data;

  uint8_t touchPoint = touch->read((uint16_t*)(&data->point.x), (uint16_t*)(&data->point.y));
  if (touchPoint > 0) {
    ESP_LOGV(TAG, "X: %d, Y: %d", data->point.x, data->point.y);
  }
  data->state = touchPoint > 0 ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
}

static void touch_feedback(lv_indev_drv_t *indev_driver, uint8_t event) {
  extern void beep_inp_feedback(lv_indev_drv_t *indev_driver, uint8_t event) ;
  beep_inp_feedback(indev_driver, event) ;

  extern void display_inp_feedback(lv_indev_drv_t *indev_driver, uint8_t event) ;
  display_inp_feedback(indev_driver, event) ;
}

lv_indev_t * lvgl_indev = NULL;

void FT6336::useLVGL() {
  static lv_indev_drv_t lvgl_indev_drv;
  lv_indev_drv_init(&lvgl_indev_drv);
  lvgl_indev_drv.type = LV_INDEV_TYPE_POINTER;
  lvgl_indev_drv.read_cb = touchpad_read;
  lvgl_indev_drv.user_data = this;
  lvgl_indev_drv.feedback_cb = touch_feedback;
  lvgl_indev = lv_indev_drv_register(&lvgl_indev_drv);
}
#endif

FT6336 Touch;

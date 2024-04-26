#include "Common.h"
#include "Sound.h"
#include <Arduino.h>
#include <driver/i2s.h>

// Sound
#define I2S_DOUT (47)
#define I2S_BCLK (48)
#define I2S_LRC  (45)
#define I2S_NUM  I2S_NUM_0

#ifdef USE_LVGL
uint16_t beep_sound_wave[800]; // 16 kHz sample rate
#endif

ATD_Sound::ATD_Sound() { }

void ATD_Sound::begin() {
#ifdef USE_LVGL
  // Gen sound beep
  for (uint16_t i=0;i<800;i++) {
    beep_sound_wave[i] = (i % 16) > 8 ? 0x3FFF : 0x0000;
    // Serial.println(beep_sound_wave[i]);
  }
#endif

  // Setup peripherals
  static const i2s_pin_config_t pin_config = {
    .mck_io_num = I2S_PIN_NO_CHANGE,
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRC,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  static i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = 16000,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,  // Interrupt level 1, default 0
      .dma_buf_count = 8,
      .dma_buf_len = 64,
      .use_apll = false,
      .tx_desc_auto_clear = true
  };

  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, &pin_config);
}

void ATD_Sound::play(const void * data, size_t len) {
  size_t out_bytes = 0;
  i2s_write(I2S_NUM, data, len, &out_bytes, 100);
}

#ifdef USE_LVGL
static bool enable_useLVGL = false;

void beep_inp_feedback(lv_indev_drv_t *indev_driver, uint8_t event) {
  if (!enable_useLVGL) {
    return;
  }

  if((event == LV_EVENT_CLICKED) || (event == LV_EVENT_KEY)) {
    Sound.play(beep_sound_wave, sizeof(beep_sound_wave));
  }
}

void ATD_Sound::useLVGL() {
  enable_useLVGL = true;
}
#endif

ATD_Sound Sound;

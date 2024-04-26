#if defined(__has_include)
#if __has_include(<lvgl.h>)
#include <lvgl.h>
#define USE_LVGL
#endif
#else
#error "__has_include not work"
#endif

#define SPI_SCK_PIN  (12)
#define SPI_MOSI_PIN (11)
#define SPI_MISO_PIN (13)

#define LCD_CS_PIN   (10)
#define LCD_DC_PIN   (21)
#define LCD_RST_PIN  (14)
#define LCD_BL_PIN   (3)

#define SD_CS_PIN    (18)
#define SD_CD_PIN    (17)

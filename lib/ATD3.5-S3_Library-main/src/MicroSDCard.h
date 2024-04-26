#ifndef __MICRO_SD_CARD_H__
#define __MICRO_SD_CARD_H__

#include "Common.h"
#include "Arduino.h"
#include <SD.h>

class MicroSDCard : public SDFS {
    public:
        MicroSDCard(FSImplPtr impl) ;

#ifdef USE_LVGL
        lv_fs_drv_t drv;
#endif

        bool begin() ;
#ifdef USE_LVGL
        void useLVGL() ;
#endif

}
;

extern MicroSDCard Card;

// LVGL extra
extern void lv_img_set_src_from_sd_card(lv_obj_t * obj, const char * path) ;
#endif

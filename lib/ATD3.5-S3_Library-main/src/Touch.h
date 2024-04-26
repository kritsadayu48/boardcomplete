#ifndef __TOUCH_H__
#define __TOUCH_H__

#include <stdint.h>
#include "Common.h"

class FT6336 {
    public: 
        FT6336() ;

        void begin() ;
        uint8_t read(uint16_t *, uint16_t *) ;

#ifdef USE_LVGL
        void useLVGL() ;
#endif

};

extern FT6336 Touch;
#endif

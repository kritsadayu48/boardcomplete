#ifndef __SOUND_H__
#define __SOUND_H__

#include "Common.h"
#include <stddef.h>
#include <stdint.h>

class ATD_Sound {
    public:
        ATD_Sound() ;
        void begin() ;
        void play(const void * data, size_t len) ;

#ifdef USE_LVGL
        void useLVGL() ;
#endif

};

extern ATD_Sound Sound;
#endif

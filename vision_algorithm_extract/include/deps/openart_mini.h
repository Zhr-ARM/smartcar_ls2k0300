#ifndef VISION_EXTRACT_OPENART_MINI_H
#define VISION_EXTRACT_OPENART_MINI_H

#include "headfile.h"

typedef struct openart_param_t {
    int8_t rx_array[5];
    int8_t openart_result;
    int32_t receiver_time;

    float fruit_delta;

    enum {
        TAG_NONE, TAG_SEARCH, TAG_STOP, TAG_SHOOTING
    } tag_type;

    int8_t fa_num[2];
    int32_t animaltime;
    int32_t fruittime;
    int32_t aprilencoder;
    int32_t aprilwaitencoder;

    enum {
        NONE, ANIMAL, FRUIT,
    } fa_type;

    enum {
        OFF_MODE, TAG_MODE, NUM_MODE, OBJ_MODE, FA_MODE,
    } openart_mode;

} openart_param_t;

extern openart_param_t openart;

#endif

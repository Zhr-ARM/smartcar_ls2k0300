#ifndef VISION_EXTRACT_MOTOR_H
#define VISION_EXTRACT_MOTOR_H

#include "headfile.h"

#define ENCODER_PER_METER (5800)

typedef struct motor_param_t {
    float total_encoder;
    float target_encoder;
} motor_param_t;

extern motor_param_t motor_l;
extern motor_param_t motor_r;

int64_t get_total_encoder(void);

#endif

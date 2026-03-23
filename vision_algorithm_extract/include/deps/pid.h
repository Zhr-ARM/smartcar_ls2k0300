#ifndef VISION_EXTRACT_PID_H
#define VISION_EXTRACT_PID_H

typedef struct pid_param_t {
    float kp;
    float ki;
    float kd;
    float low_pass;
    float p_max;
    float i_max;
    float d_max;
} pid_param_t;

#endif

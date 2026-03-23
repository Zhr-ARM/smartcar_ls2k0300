#ifndef VISION_EXTRACT_ATTITUDE_SOLUTION_H
#define VISION_EXTRACT_ATTITUDE_SOLUTION_H

typedef struct {
    float pitch;
    float roll;
    float yaw;
} euler_param_t;

extern euler_param_t eulerAngle;

#endif

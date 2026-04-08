#ifndef _zf_driver_uvc_h
#define _zf_driver_uvc_h


#include "zf_common_typedef.h"

#define UVC_FPS     120

int8 uvc_camera_init(const char *path, int width, int height);
int8 wait_image_refresh();


extern uint8_t *rgay_image;
extern uint8_t *rgb565_image;
extern uint8_t *bgr_image;


#endif

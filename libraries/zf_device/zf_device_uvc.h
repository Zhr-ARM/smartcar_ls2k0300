#ifndef _zf_driver_uvc_h
#define _zf_driver_uvc_h


#include "zf_common_typedef.h"

// UVC采图分辨率配置（只需改这一项）：
// 0 -> 160x120
// 1 -> 320x240
#define UVC_RES_PRESET_160X120 0
#define UVC_RES_PRESET_320X240 1

#ifndef UVC_RES_PRESET
#define UVC_RES_PRESET UVC_RES_PRESET_320X240
#endif

#if (UVC_RES_PRESET == UVC_RES_PRESET_160X120)
#define UVC_WIDTH   160
#define UVC_HEIGHT  120
#elif (UVC_RES_PRESET == UVC_RES_PRESET_320X240)
#define UVC_WIDTH   320
#define UVC_HEIGHT  240
#else
#error "Invalid UVC_RES_PRESET. Use UVC_RES_PRESET_160X120 or UVC_RES_PRESET_320X240."
#endif

#define UVC_FPS     120

int8 uvc_camera_init(const char *path);
int8 wait_image_refresh();


extern uint8_t *rgay_image;
extern uint8_t *rgb565_image;
extern uint8_t *bgr_image;


#endif

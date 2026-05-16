#ifndef _zf_driver_uvc_h
#define _zf_driver_uvc_h


#include "zf_common_typedef.h"

// UVC采图分辨率配置（只需改这一项）：
// 0 -> 160x120
// 1 -> 320x240
#define UVC_RES_PRESET_160X120 0
#define UVC_RES_PRESET_320X240 1

#ifndef UVC_RES_PRESET
#define UVC_RES_PRESET UVC_RES_PRESET_160X120
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

// UVC输出格式配置（构建时由 UVC_FORMAT_PRESET 选择）：
// 0 -> YUY2
// 1 -> MJPG
#define UVC_FORMAT_PRESET_YUY2 0
#define UVC_FORMAT_PRESET_MJPG 1

#ifndef UVC_FORMAT_PRESET
#define UVC_FORMAT_PRESET UVC_FORMAT_PRESET_MJPG
#endif

#if (UVC_FORMAT_PRESET == UVC_FORMAT_PRESET_YUY2)
#define UVC_FOURCC_CHAR_0 'Y'
#define UVC_FOURCC_CHAR_1 'U'
#define UVC_FOURCC_CHAR_2 'Y'
#define UVC_FOURCC_CHAR_3 '2'
#define UVC_FORMAT_NAME "YUY2"
#elif (UVC_FORMAT_PRESET == UVC_FORMAT_PRESET_MJPG)
#define UVC_FOURCC_CHAR_0 'M'
#define UVC_FOURCC_CHAR_1 'J'
#define UVC_FOURCC_CHAR_2 'P'
#define UVC_FOURCC_CHAR_3 'G'
#define UVC_FORMAT_NAME "MJPG"
#else
#error "Invalid UVC_FORMAT_PRESET. Use 0=YUY2 or 1=MJPG."
#endif

#define UVC_FPS     120

int8 uvc_camera_init(const char *path);
int8 wait_image_refresh();


extern uint8_t *rgay_image;
extern uint8_t *rgb565_image;
extern uint8_t *bgr_image;


#endif

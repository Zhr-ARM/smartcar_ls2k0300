#ifndef _zf_common_headfile_h_
#define _zf_common_headfile_h_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;
typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;
typedef int64_t int64;

#define UVC_RES_PRESET_160X120 0
#define UVC_RES_PRESET_320X240 1

#ifndef UVC_RES_PRESET
#define UVC_RES_PRESET UVC_RES_PRESET_160X120
#endif

#if (UVC_RES_PRESET == UVC_RES_PRESET_160X120)
#define UVC_WIDTH 160
#define UVC_HEIGHT 120
#elif (UVC_RES_PRESET == UVC_RES_PRESET_320X240)
#define UVC_WIDTH 320
#define UVC_HEIGHT 240
#else
#error "Invalid UVC_RES_PRESET."
#endif

#endif

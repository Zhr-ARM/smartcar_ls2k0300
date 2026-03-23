#ifndef VISION_EXTRACT_HEADFILE_H
#define VISION_EXTRACT_HEADFILE_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;
typedef int64_t int64;
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;

typedef void* rt_mailbox_t;

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef PI
#define PI 3.14159265358979323846f
#endif

#ifndef MT9V03X_CSI_W
#define MT9V03X_CSI_W 376
#endif
#ifndef MT9V03X_CSI_H
#define MT9V03X_CSI_H 240
#endif

int32_t rt_tick_get_millisecond(void);
int rt_mb_send(rt_mailbox_t mailbox, uint32_t value);

#endif

#ifndef VISION_EXTRACT_COMMON_H
#define VISION_EXTRACT_COMMON_H

#include "headfile.h"
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef PI
#define PI 3.14159265358979323846f
#endif

#ifndef AT_ITCM_SECTION_INIT
#define AT_ITCM_SECTION_INIT(var) var
#endif
#ifndef AT_DTCM_SECTION_ALIGN_INIT
#define AT_DTCM_SECTION_ALIGN_INIT(var, alignbytes) var
#endif
#ifndef AT_DTCM_SECTION_INIT
#define AT_DTCM_SECTION_INIT(var) var
#endif
#ifndef AT_SDRAM_SECTION_ALIGN_INIT
#define AT_SDRAM_SECTION_ALIGN_INIT(var, alignbytes) var
#endif
#ifndef AT_SDRAM_SECTION_INIT
#define AT_SDRAM_SECTION_INIT(var) var
#endif

#endif

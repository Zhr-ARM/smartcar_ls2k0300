#ifndef VISION_IMAGE_PROCESSOR_H_
#define VISION_IMAGE_PROCESSOR_H_

#include "zf_common_headfile.h"
#include <stddef.h>

#define VISION_DOWNSAMPLED_WIDTH 160
#define VISION_DOWNSAMPLED_HEIGHT 120
#define VISION_IPM_WIDTH 280
#define VISION_IPM_HEIGHT 140

extern int line_error;
extern float line_sample_ratio;

typedef enum
{
    VISION_IPM_LINE_ERROR_FROM_LEFT_SHIFT = 0,
    VISION_IPM_LINE_ERROR_FROM_RIGHT_SHIFT = 1,
    VISION_IPM_LINE_ERROR_FROM_AUTO = 2
} vision_ipm_line_error_source_enum;

typedef enum
{
    VISION_IPM_LINE_ERROR_FIXED_INDEX = 0,
    VISION_IPM_LINE_ERROR_WEIGHTED_INDEX = 1,
    VISION_IPM_LINE_ERROR_SPEED_INDEX = 2
} vision_ipm_line_error_method_enum;

bool vision_image_processor_init(const char *camera_path);
void vision_image_processor_cleanup();
bool vision_image_processor_process_step();

vision_ipm_line_error_source_enum vision_image_processor_ipm_line_error_source();

void vision_image_processor_get_ipm_boundaries(uint16 **x1, uint16 **x2, uint16 **x3,
                                               uint16 **y1, uint16 **y2, uint16 **y3,
                                               uint16 *dot_num);
void vision_image_processor_get_boundaries(uint16 **x1, uint16 **x2, uint16 **x3,
                                           uint16 **y1, uint16 **y2, uint16 **y3,
                                           uint16 *dot_num);
void vision_image_processor_get_ipm_shifted_centerline_from_left(uint16 **x, uint16 **y, uint16 *dot_num);
void vision_image_processor_get_ipm_shifted_centerline_from_right(uint16 **x, uint16 **y, uint16 *dot_num);
void vision_image_processor_get_src_shifted_centerline_from_left(uint16 **x, uint16 **y, uint16 *dot_num);
void vision_image_processor_get_src_shifted_centerline_from_right(uint16 **x, uint16 **y, uint16 *dot_num);
void vision_image_processor_get_ipm_line_error_track_point(bool *valid, int *x, int *y);
int vision_image_processor_ipm_line_error_track_index();
uint8 vision_image_processor_get_last_otsu_threshold();

#endif

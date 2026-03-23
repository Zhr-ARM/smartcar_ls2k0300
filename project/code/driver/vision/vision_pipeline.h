#ifndef VISION_PIPELINE_H_
#define VISION_PIPELINE_H_

#include "zf_common_headfile.h"
#include "driver/vision/lq_ncnn.hpp"

typedef enum
{
    VISION_SEND_BINARY = 0,         // 1bit/像素打包（二值图）
    VISION_SEND_GRAY = 1,           // 8bit灰度（原始灰度）
    VISION_SEND_RGB565 = 2,         // 16bit彩色纯图（原始BGR转RGB565）
    VISION_SEND_IPM_RGB565 = 3,     // 16bit彩色逆透视图
    VISION_SEND_IPM_EDGE_GRAY = 4,  // 8bit逆透视边线图（黑底白线）
    VISION_SEND_RGB565_OVERLAY = 5  // 16bit彩色叠加调试图（中心线/边线）
} vision_send_mode_enum;

bool vision_pipeline_init(const char *camera_path, LQ_NCNN *ncnn, bool ncnn_enabled);
bool vision_pipeline_process_step();
void vision_pipeline_send_step();
bool vision_pipeline_step();
const uint8 *vision_pipeline_bgr_image();
void vision_pipeline_set_send_mode(vision_send_mode_enum mode);
vision_send_mode_enum vision_pipeline_get_send_mode();
void vision_pipeline_set_send_max_fps(uint32 max_fps);
uint32 vision_pipeline_get_send_max_fps();
void vision_pipeline_set_send_enabled(bool enabled);
bool vision_pipeline_is_send_enabled();

// 红色实心矩形坐标（透传 image_processor 全局结果）
void vision_pipeline_get_red_rect(bool *found, int *x, int *y, int *w, int *h, int *cx, int *cy);
int vision_pipeline_get_red_rect_area();

#endif

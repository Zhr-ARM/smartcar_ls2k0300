#ifndef VISION_PIPELINE_H_
#define VISION_PIPELINE_H_

#include "zf_common_headfile.h"
#include "driver/vision/lq_ncnn.hpp"

typedef enum
{
    VISION_SEND_BINARY = 0,         // 已合并到灰度显示模式（画边线/中线）
    VISION_SEND_GRAY = 1,           // 灰度图（画边线/中线）
    VISION_SEND_RGB565 = 2,         // 彩色纯图（不画线）
    VISION_SEND_IPM_RGB565 = 3,     // 已合并到彩色纯图模式
    VISION_SEND_IPM_EDGE_GRAY = 4,  // 已合并到灰度显示模式
    VISION_SEND_RGB565_OVERLAY = 5  // 已合并到灰度显示模式
} vision_send_mode_enum;

bool vision_pipeline_init(const char *camera_path, LQ_NCNN *ncnn, bool ncnn_enabled);
void vision_pipeline_cleanup();
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

// 采图模式：检测到红色矩形后，保存用于推理的 ROI 彩图（PNG）。
void vision_pipeline_set_roi_capture_mode(bool enabled);
bool vision_pipeline_roi_capture_mode_enabled();

// 红色实心矩形坐标（透传 image_processor 全局结果）
void vision_pipeline_get_red_rect(bool *found, int *x, int *y, int *w, int *h, int *cx, int *cy);
int vision_pipeline_get_red_rect_area();

#endif

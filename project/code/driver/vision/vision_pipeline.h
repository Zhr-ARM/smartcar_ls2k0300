#ifndef VISION_PIPELINE_H_
#define VISION_PIPELINE_H_

#include "zf_common_headfile.h"
#include "driver/vision/vision_infer_async.h"
#include "driver/vision/vision_transport.h"

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

// 推理总开关：关闭后跳过红色识别与ncnn推理。
void vision_pipeline_set_infer_enabled(bool enabled);
bool vision_pipeline_infer_enabled();

// 采图模式：检测到红色矩形后，保存用于推理的 ROI 彩图（PNG）。
void vision_pipeline_set_roi_capture_mode(bool enabled);
bool vision_pipeline_roi_capture_mode_enabled();

// 红色实心矩形坐标（透传 image_processor 全局结果）
void vision_pipeline_get_red_rect(bool *found, int *x, int *y, int *w, int *h, int *cx, int *cy);
int vision_pipeline_get_red_rect_area();

#endif

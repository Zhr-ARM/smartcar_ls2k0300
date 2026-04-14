#ifndef VISION_THREAD_H_
#define VISION_THREAD_H_

#include "zf_common_typedef.h"

class LQ_NCNN;

typedef enum
{
	VISION_THREAD_SEND_BINARY = 0,
	VISION_THREAD_SEND_GRAY = 1,
	VISION_THREAD_SEND_RGB565 = 2
} vision_thread_send_mode_enum;

bool vision_thread_init(const char *camera_path);
bool vision_thread_init(const char *camera_path, LQ_NCNN *ncnn, bool ncnn_ready);

void vision_thread_cleanup();
bool vision_thread_is_running();

void vision_thread_set_send_mode(vision_thread_send_mode_enum mode);
vision_thread_send_mode_enum vision_thread_get_send_mode();
// 设置图传最大发送帧率，0 表示不限速。
void vision_thread_set_send_max_fps(uint32 max_fps);
uint32 vision_thread_get_send_max_fps();

void vision_thread_set_client_sender_enabled(bool enabled);
bool vision_thread_client_sender_enabled();

void vision_thread_set_infer_enabled(bool enabled);
bool vision_thread_infer_enabled();
void vision_thread_set_ncnn_enabled(bool enabled);
bool vision_thread_ncnn_enabled();

// 采图模式：检测到红色矩形后，保存推理 ROI 的 PNG 彩图。
void vision_thread_set_roi_capture_mode(bool enabled);
bool vision_thread_roi_capture_mode_enabled();

#endif

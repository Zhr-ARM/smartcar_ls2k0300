#ifndef VISION_THREAD_H_
#define VISION_THREAD_H_

#include "zf_common_typedef.h"

#if defined(ENABLE_E99_VISION_STACK)
class LQ_NCNN;
#endif

typedef enum
{
	VISION_THREAD_SEND_BINARY = 0,         // 已合并到灰度显示模式（画边线/中线）
	VISION_THREAD_SEND_GRAY = 1,           // 灰度图（画边线/中线）
	VISION_THREAD_SEND_RGB565 = 2,         // 彩色纯图（不画线）
	VISION_THREAD_SEND_IPM_RGB565 = 3,     // 已合并到彩色纯图模式
	VISION_THREAD_SEND_IPM_EDGE_GRAY = 4,  // 已合并到灰度显示模式
	VISION_THREAD_SEND_RGB565_OVERLAY = 5  // 已合并到灰度显示模式
} vision_thread_send_mode_enum;

bool vision_thread_init(const char *camera_path);

#if defined(ENABLE_E99_VISION_STACK)
bool vision_thread_init(const char *camera_path, LQ_NCNN *ncnn, bool ncnn_ready);
#endif

void vision_thread_cleanup();
bool vision_thread_is_running();

void vision_thread_set_send_mode(vision_thread_send_mode_enum mode);
vision_thread_send_mode_enum vision_thread_get_send_mode();
// 设置图传最大发送帧率，0 表示不限速。
void vision_thread_set_send_max_fps(uint32 max_fps);
uint32 vision_thread_get_send_max_fps();

// 仅在定义 ENABLE_SEEKFREE_ASSISTANT_CAMERA_STREAM 时生效。
void vision_thread_set_assistant_camera_stream(bool enabled);
bool vision_thread_assistant_camera_stream_enabled();

// 采图模式：检测到红色矩形后，保存推理 ROI 的 PNG 彩图。
void vision_thread_set_roi_capture_mode(bool enabled);
bool vision_thread_roi_capture_mode_enabled();

#endif

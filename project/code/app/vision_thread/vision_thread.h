#ifndef VISION_THREAD_H_
#define VISION_THREAD_H_

#if defined(ENABLE_E99_VISION_STACK)
class LQ_NCNN;
#endif

typedef enum
{
	VISION_THREAD_SEND_BINARY = 0, // 1bit/像素打包
	VISION_THREAD_SEND_GRAY   = 1, // 8bit灰度
	VISION_THREAD_SEND_RGB565 = 2  // 16bit彩色
} vision_thread_send_mode_enum;

bool vision_thread_init(const char *camera_path);

#if defined(ENABLE_E99_VISION_STACK)
bool vision_thread_init(const char *camera_path, LQ_NCNN *ncnn, bool ncnn_ready);
#endif

void vision_thread_cleanup();
bool vision_thread_is_running();

void vision_thread_set_send_mode(vision_thread_send_mode_enum mode);
vision_thread_send_mode_enum vision_thread_get_send_mode();

// 仅在定义 ENABLE_SEEKFREE_ASSISTANT_CAMERA_STREAM 时生效。
void vision_thread_set_assistant_camera_stream(bool enabled);
bool vision_thread_assistant_camera_stream_enabled();

#endif

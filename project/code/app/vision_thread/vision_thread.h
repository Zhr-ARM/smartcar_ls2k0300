#ifndef VISION_THREAD_H_
#define VISION_THREAD_H_

#include "zf_common_typedef.h"

class LQ_NCNN;

bool vision_thread_init(const char *camera_path);
bool vision_thread_init(const char *camera_path, LQ_NCNN *ncnn, bool ncnn_ready);

void vision_thread_cleanup();
bool vision_thread_is_running();

void vision_thread_set_infer_enabled(bool enabled);
bool vision_thread_infer_enabled();
void vision_thread_set_ncnn_enabled(bool enabled);
bool vision_thread_ncnn_enabled();

// 读取视觉处理线程最近 1 秒窗口统计得到的处理帧率。
uint32 vision_thread_process_fps();

#endif

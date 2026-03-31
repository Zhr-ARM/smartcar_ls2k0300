#ifndef VISION_FRAME_CAPTURE_H_
#define VISION_FRAME_CAPTURE_H_

#include "zf_common_headfile.h"

#include <cstddef>

// 初始化采图模块并启动采图线程；camera_path 为空时默认 /dev/video0。
bool vision_frame_capture_init(const char *camera_path);

// 停止采图线程并释放采图模块资源。
void vision_frame_capture_cleanup();

// 阻塞等待新帧并拷贝到 out_bgr：
// - out_bgr_bytes 需至少为 UVC_WIDTH * UVC_HEIGHT * 3；
// - timeout_ms 为等待超时；
// - wait_us 返回本次等待耗时。
bool vision_frame_capture_wait_next_bgr(uint8 *out_bgr, size_t out_bgr_bytes, uint32 timeout_ms, uint32 *wait_us);

#endif

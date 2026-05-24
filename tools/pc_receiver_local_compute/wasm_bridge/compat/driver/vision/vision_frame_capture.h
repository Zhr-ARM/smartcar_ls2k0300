#ifndef VISION_FRAME_CAPTURE_H_
#define VISION_FRAME_CAPTURE_H_

#include "zf_common_headfile.h"

#include <stddef.h>

bool vision_frame_capture_init(const char *camera_path);
void vision_frame_capture_cleanup();
bool vision_frame_capture_wait_next_bgr(uint8 *out_bgr, size_t out_bgr_bytes, uint32 timeout_ms, uint32 *wait_us);

// WASM bridge 专用：由桥接层把浏览器帧写入 compat 采图缓冲。
void wasm_compat_vision_frame_capture_set_bgr_frame(const uint8 *bgr, size_t bytes);

#endif

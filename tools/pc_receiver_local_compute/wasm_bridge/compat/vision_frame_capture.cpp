#include "driver/vision/vision_frame_capture.h"

#include <algorithm>
#include <cstring>

namespace
{
const uint8 *g_bgr_frame = nullptr;
size_t g_bgr_frame_bytes = 0;
}

bool vision_frame_capture_init(const char *camera_path)
{
    (void)camera_path;
    return true;
}

void vision_frame_capture_cleanup()
{
    g_bgr_frame = nullptr;
    g_bgr_frame_bytes = 0;
}

bool vision_frame_capture_wait_next_bgr(uint8 *out_bgr, size_t out_bgr_bytes, uint32 timeout_ms, uint32 *wait_us)
{
    (void)timeout_ms;
    if (wait_us != nullptr)
    {
        *wait_us = 0;
    }
    if (out_bgr == nullptr || g_bgr_frame == nullptr || g_bgr_frame_bytes == 0 || out_bgr_bytes < g_bgr_frame_bytes)
    {
        return false;
    }

    std::memcpy(out_bgr, g_bgr_frame, std::min(out_bgr_bytes, g_bgr_frame_bytes));
    return true;
}

void wasm_compat_vision_frame_capture_set_bgr_frame(const uint8 *bgr, size_t bytes)
{
    g_bgr_frame = bgr;
    g_bgr_frame_bytes = bytes;
}

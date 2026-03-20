#include "vision_thread.h"

#if defined(ENABLE_E99_VISION_STACK)
#include "driver/vision/vision_pipeline.h"
#else
#include "driver/vision/vision_image_processor.h"
#endif

#include <atomic>
#include <thread>

#if !defined(ENABLE_E99_VISION_STACK)
#include <algorithm>
#include <cstring>
#include <vector>

#if defined(ENABLE_SEEKFREE_ASSISTANT_CAMERA_STREAM)
#include "seekfree_assistant.h"
#endif
#endif

namespace
{
std::thread g_vision_thread;
std::atomic<bool> g_vision_running(false);

#if !defined(ENABLE_E99_VISION_STACK)
#define VISION_THREAD_BINARY_PACKED_SIZE (UVC_HEIGHT * UVC_WIDTH / 8)

std::atomic<bool> g_assistant_camera_stream_enabled(false);
std::atomic<int> g_send_mode(VISION_THREAD_SEND_BINARY);

#if defined(ENABLE_SEEKFREE_ASSISTANT_CAMERA_STREAM)
uint8 g_image_binary_1bit[VISION_THREAD_BINARY_PACKED_SIZE];

void pack_binary_1bit(const uint8 *src, uint8 *dst, int width, int height)
{
    std::memset(dst, 0, static_cast<size_t>(width) * static_cast<size_t>(height) / 8);
    int out_idx = 0;
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; x += 8)
        {
            uint8 v = 0;
            for (int b = 0; b < 8; ++b)
            {
                if (src[y * width + x + b] > 0)
                {
                    v |= static_cast<uint8>(1u << (7 - b));
                }
            }
            dst[out_idx++] = v;
        }
    }
}

void vision_send_camera_by_seekfree_assistant()
{
    uint8 *x1 = nullptr;
    uint8 *x2 = nullptr;
    uint8 *x3 = nullptr;
    uint8 *y1 = nullptr;
    uint8 *y2 = nullptr;
    uint8 *y3 = nullptr;
    uint16 dot_num = 0;

    vision_image_processor_get_boundaries(&x1, &x2, &x3, &y1, &y2, &y3, &dot_num);

    constexpr int kSampleLineStepPx = 2;
    std::vector<uint8> overlay_x1;
    std::vector<uint8> overlay_x2;
    std::vector<uint8> overlay_x3;
    std::vector<uint8> overlay_y1;
    std::vector<uint8> overlay_y2;
    std::vector<uint8> overlay_y3;

    const int den = (line_sample_ratio_den <= 0) ? 1 : line_sample_ratio_den;
    const int num = (line_sample_ratio_num < 0) ? 0 : line_sample_ratio_num;
    const int sample_y = std::min(UVC_HEIGHT - 1, (UVC_HEIGHT * num) / den);
    const uint16 sample_line_points = static_cast<uint16>((UVC_WIDTH + kSampleLineStepPx - 1) / kSampleLineStepPx);

    overlay_x1.reserve(dot_num + sample_line_points);
    overlay_x2.reserve(dot_num + sample_line_points);
    overlay_x3.reserve(dot_num + sample_line_points);
    overlay_y1.reserve(dot_num + sample_line_points);
    overlay_y2.reserve(dot_num + sample_line_points);
    overlay_y3.reserve(dot_num + sample_line_points);

    for (uint16 i = 0; i < dot_num; ++i)
    {
        overlay_x1.push_back(x1[i]);
        overlay_x2.push_back(x2[i]);
        overlay_x3.push_back(x3[i]);
        overlay_y1.push_back(y1[i]);
        overlay_y2.push_back(y2[i]);
        overlay_y3.push_back(y3[i]);
    }

    for (int x = 0; x < UVC_WIDTH; x += kSampleLineStepPx)
    {
        const uint8 px = static_cast<uint8>(x);
        const uint8 py = static_cast<uint8>(sample_y);
        overlay_x1.push_back(px);
        overlay_x2.push_back(px);
        overlay_x3.push_back(px);
        overlay_y1.push_back(py);
        overlay_y2.push_back(py);
        overlay_y3.push_back(py);
    }

    const uint16 overlay_dot_num = static_cast<uint16>(overlay_x1.size());

    vision_thread_send_mode_enum mode = static_cast<vision_thread_send_mode_enum>(g_send_mode.load());
    switch (mode)
    {
        case VISION_THREAD_SEND_BINARY:
        {
            const uint8 *binary_u8 = vision_image_processor_binary_u8_image();
            if (binary_u8 == nullptr)
            {
                return;
            }
            pack_binary_1bit(binary_u8, g_image_binary_1bit, UVC_WIDTH, UVC_HEIGHT);
            seekfree_assistant_camera_information_config(
                SEEKFREE_ASSISTANT_BINARY,
                g_image_binary_1bit,
                UVC_WIDTH,
                UVC_HEIGHT);
            break;
        }
        case VISION_THREAD_SEND_RGB565:
        {
            const uint8 *rgb565 = vision_image_processor_rgb565_image();
            if (rgb565 == nullptr)
            {
                return;
            }
            seekfree_assistant_camera_information_config(
                SEEKFREE_ASSISTANT_RGB565,
                const_cast<uint8 *>(rgb565),
                UVC_WIDTH,
                UVC_HEIGHT);
            break;
        }
        case VISION_THREAD_SEND_GRAY:
        default:
        {
            const uint8 *gray = vision_image_processor_gray_image();
            if (gray == nullptr)
            {
                return;
            }
            seekfree_assistant_camera_information_config(
                SEEKFREE_ASSISTANT_GRAY,
                const_cast<uint8 *>(gray),
                UVC_WIDTH,
                UVC_HEIGHT);
            break;
        }
    }

    seekfree_assistant_camera_boundary_config(
        XY_BOUNDARY,
        overlay_dot_num,
        overlay_x1.data(),
        overlay_x2.data(),
        overlay_x3.data(),
        overlay_y1.data(),
        overlay_y2.data(),
        overlay_y3.data());

    seekfree_assistant_camera_send();
}
#endif
#endif

void vision_loop()
{
    while (g_vision_running.load())
    {
#if defined(ENABLE_E99_VISION_STACK)
        if (!vision_pipeline_process_step())
        {
            system_delay_ms(5);
            continue;
        }

        vision_pipeline_send_step();
#else
        if (!vision_image_processor_process_step())
        {
            system_delay_ms(5);
            continue;
        }

#if defined(ENABLE_SEEKFREE_ASSISTANT_CAMERA_STREAM)
        if (g_assistant_camera_stream_enabled.load())
        {
            vision_send_camera_by_seekfree_assistant();
        }
#endif
#endif
    }
}
} // namespace

#if defined(ENABLE_E99_VISION_STACK)
bool vision_thread_init(const char *camera_path, LQ_NCNN *ncnn, bool ncnn_ready)
{
    if (g_vision_running.load())
    {
        return true;
    }

    if (!vision_pipeline_init(camera_path, ncnn, ncnn_ready))
    {
        return false;
    }

    g_vision_running = true;
    g_vision_thread = std::thread(vision_loop);
    return true;
}
#endif

bool vision_thread_init(const char *camera_path)
{
    if (g_vision_running.load())
    {
        return true;
    }

#if defined(ENABLE_E99_VISION_STACK)
    return vision_thread_init(camera_path, nullptr, false);
#else
    if (!vision_image_processor_init(camera_path))
    {
        return false;
    }

    g_vision_running = true;
    g_vision_thread = std::thread(vision_loop);
    return true;
#endif
}

void vision_thread_cleanup()
{
    g_vision_running = false;

    if (g_vision_thread.joinable())
    {
        g_vision_thread.join();
    }
}

bool vision_thread_is_running()
{
    return g_vision_running.load();
}

void vision_thread_set_send_mode(vision_thread_send_mode_enum mode)
{
    int m = static_cast<int>(mode);
    if (m < static_cast<int>(VISION_THREAD_SEND_BINARY) || m > static_cast<int>(VISION_THREAD_SEND_RGB565))
    {
        return;
    }

#if defined(ENABLE_E99_VISION_STACK)
    vision_pipeline_set_send_mode(static_cast<vision_send_mode_enum>(m));
#else
    g_send_mode.store(m);
#endif
}

vision_thread_send_mode_enum vision_thread_get_send_mode()
{
#if defined(ENABLE_E99_VISION_STACK)
    return static_cast<vision_thread_send_mode_enum>(vision_pipeline_get_send_mode());
#else
    return static_cast<vision_thread_send_mode_enum>(g_send_mode.load());
#endif
}

void vision_thread_set_assistant_camera_stream(bool enabled)
{
#if defined(ENABLE_E99_VISION_STACK)
    vision_pipeline_set_send_enabled(enabled);
#elif defined(ENABLE_SEEKFREE_ASSISTANT_CAMERA_STREAM)
    g_assistant_camera_stream_enabled.store(enabled);
#else
    (void)enabled;
#endif
}

bool vision_thread_assistant_camera_stream_enabled()
{
#if defined(ENABLE_E99_VISION_STACK)
    return vision_pipeline_is_send_enabled();
#elif defined(ENABLE_SEEKFREE_ASSISTANT_CAMERA_STREAM)
    return g_assistant_camera_stream_enabled.load();
#else
    return false;
#endif
}

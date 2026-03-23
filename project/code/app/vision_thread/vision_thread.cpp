#include "vision_thread.h"

#if defined(ENABLE_E99_VISION_STACK)
#include "driver/vision/vision_pipeline.h"
#endif
#include "driver/vision/vision_image_processor.h"
#if defined(ENABLE_E99_VISION_STACK)
#include "driver/vision/vision_client_sender.h"
#endif

#include <atomic>
#include <chrono>
#include <cstdint>
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

constexpr int64_t VISION_PERF_LOG_INTERVAL_US = 1000 * 1000;
constexpr uint32_t VISION_THREAD_SEND_DEFAULT_MAX_FPS = 30;
constexpr uint32_t VISION_THREAD_SEND_MAX_FPS_UPPER = 240;

struct vision_perf_accum_t
{
    uint64_t frame_count = 0;
    uint64_t capture_wait_us = 0;
    uint64_t preprocess_us = 0;
    uint64_t otsu_us = 0;
    uint64_t maze_us = 0;
    uint64_t process_us = 0;
    uint64_t send_us = 0;
    uint64_t loop_us = 0;
};

static void vision_perf_print_and_reset(vision_perf_accum_t &acc, int64_t window_us)
{
    if (acc.frame_count == 0 || window_us <= 0)
    {
        acc = vision_perf_accum_t{};
        return;
    }

    const double inv_frames = 1.0 / static_cast<double>(acc.frame_count);
    const double fps = (static_cast<double>(acc.frame_count) * 1000000.0) / static_cast<double>(window_us);

    printf("[VISION PERF] fps=%.1f loop=%.2fms proc=%.2fms(wait=%.2f pre=%.2f otsu=%.2f maze=%.2f) send=%.2fms\r\n",
           fps,
           (static_cast<double>(acc.loop_us) * inv_frames) / 1000.0,
           (static_cast<double>(acc.process_us) * inv_frames) / 1000.0,
           (static_cast<double>(acc.capture_wait_us) * inv_frames) / 1000.0,
           (static_cast<double>(acc.preprocess_us) * inv_frames) / 1000.0,
           (static_cast<double>(acc.otsu_us) * inv_frames) / 1000.0,
           (static_cast<double>(acc.maze_us) * inv_frames) / 1000.0,
           (static_cast<double>(acc.send_us) * inv_frames) / 1000.0);

    acc = vision_perf_accum_t{};
}

#if !defined(ENABLE_E99_VISION_STACK)
#define VISION_THREAD_BINARY_PACKED_SIZE (VISION_DOWNSAMPLED_HEIGHT * VISION_DOWNSAMPLED_WIDTH / 8)

std::atomic<bool> g_assistant_camera_stream_enabled(false);
std::atomic<int> g_send_mode(VISION_THREAD_SEND_BINARY);
std::atomic<uint32_t> g_send_max_fps(VISION_THREAD_SEND_DEFAULT_MAX_FPS);
std::atomic<uint64_t> g_last_send_tick_us(0);

#if defined(ENABLE_SEEKFREE_ASSISTANT_CAMERA_STREAM)
uint8 g_image_binary_1bit[VISION_THREAD_BINARY_PACKED_SIZE];
uint8 g_image_rgb565_temp[VISION_DOWNSAMPLED_WIDTH * VISION_DOWNSAMPLED_HEIGHT * 2];

static void vision_thread_convert_bgr_to_rgb565_be(const uint8 *bgr, uint8 *rgb565, int width, int height)
{
    if (bgr == nullptr || rgb565 == nullptr || width <= 0 || height <= 0)
    {
        return;
    }

    const int pixel_num = width * height;
    for (int i = 0; i < pixel_num; ++i)
    {
        uint8 b = bgr[i * 3 + 0];
        uint8 g = bgr[i * 3 + 1];
        uint8 r = bgr[i * 3 + 2];
        uint16 v = static_cast<uint16>(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
        rgb565[i * 2 + 0] = static_cast<uint8>(v >> 8);
        rgb565[i * 2 + 1] = static_cast<uint8>(v & 0xFF);
    }
}

static bool vision_thread_try_acquire_send_slot()
{
    const uint32_t max_fps = g_send_max_fps.load();
    if (max_fps == 0)
    {
        return true;
    }

    const uint64_t now_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
    const uint64_t min_interval_us = 1000000ULL / static_cast<uint64_t>(max_fps);
    const uint64_t last_us = g_last_send_tick_us.load();
    if (last_us != 0 && (now_us - last_us) < min_interval_us)
    {
        return false;
    }

    g_last_send_tick_us.store(now_us);
    return true;
}

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

    vision_thread_send_mode_enum mode = static_cast<vision_thread_send_mode_enum>(g_send_mode.load());
    switch (mode)
    {
        case VISION_THREAD_SEND_BINARY:
        {
            const uint8 *binary_u8_down = vision_image_processor_binary_downsampled_u8_image();
            if (binary_u8_down == nullptr)
            {
                return;
            }
            pack_binary_1bit(binary_u8_down, g_image_binary_1bit, VISION_DOWNSAMPLED_WIDTH, VISION_DOWNSAMPLED_HEIGHT);
            seekfree_assistant_camera_information_config(
                SEEKFREE_ASSISTANT_BINARY,
                g_image_binary_1bit,
                VISION_DOWNSAMPLED_WIDTH,
                VISION_DOWNSAMPLED_HEIGHT);
            break;
        }
        case VISION_THREAD_SEND_RGB565:
        {
            const uint8 *rgb565 = vision_image_processor_rgb565_downsampled_image();
            if (rgb565 == nullptr)
            {
                return;
            }
            seekfree_assistant_camera_information_config(
                SEEKFREE_ASSISTANT_RGB565,
                const_cast<uint8 *>(rgb565),
                VISION_DOWNSAMPLED_WIDTH,
                VISION_DOWNSAMPLED_HEIGHT);
            break;
        }
        case VISION_THREAD_SEND_IPM_RGB565:
        {
            const uint8 *ipm_bgr = vision_image_processor_ipm_bgr_downsampled_image();
            if (ipm_bgr == nullptr)
            {
                return;
            }
            vision_thread_convert_bgr_to_rgb565_be(ipm_bgr,
                                                   g_image_rgb565_temp,
                                                   VISION_DOWNSAMPLED_WIDTH,
                                                   VISION_DOWNSAMPLED_HEIGHT);
            seekfree_assistant_camera_information_config(
                SEEKFREE_ASSISTANT_RGB565,
                g_image_rgb565_temp,
                VISION_DOWNSAMPLED_WIDTH,
                VISION_DOWNSAMPLED_HEIGHT);
            break;
        }
        case VISION_THREAD_SEND_IPM_EDGE_GRAY:
        {
            const uint8 *ipm_edge = vision_image_processor_ipm_edge_gray_downsampled_image();
            if (ipm_edge == nullptr)
            {
                return;
            }
            seekfree_assistant_camera_information_config(
                SEEKFREE_ASSISTANT_GRAY,
                const_cast<uint8 *>(ipm_edge),
                VISION_DOWNSAMPLED_WIDTH,
                VISION_DOWNSAMPLED_HEIGHT);
            break;
        }
        case VISION_THREAD_SEND_RGB565_OVERLAY:
        {
            // 非 E99 路径下暂复用纯彩色发送。
            const uint8 *rgb565 = vision_image_processor_rgb565_downsampled_image();
            if (rgb565 == nullptr)
            {
                return;
            }
            seekfree_assistant_camera_information_config(
                SEEKFREE_ASSISTANT_RGB565,
                const_cast<uint8 *>(rgb565),
                VISION_DOWNSAMPLED_WIDTH,
                VISION_DOWNSAMPLED_HEIGHT);
            break;
        }
        case VISION_THREAD_SEND_GRAY:
        default:
        {
            const uint8 *gray = vision_image_processor_gray_downsampled_image();
            if (gray == nullptr)
            {
                return;
            }
            seekfree_assistant_camera_information_config(
                SEEKFREE_ASSISTANT_GRAY,
                const_cast<uint8 *>(gray),
                VISION_DOWNSAMPLED_WIDTH,
                VISION_DOWNSAMPLED_HEIGHT);
            break;
        }
    }

    if (dot_num > 0 && x1 && x2 && x3 && y1 && y2 && y3)
    {
        seekfree_assistant_camera_boundary_config(
            XY_BOUNDARY,
            dot_num,
            x1,
            x2,
            x3,
            y1,
            y2,
            y3);
    }
    else
    {
        seekfree_assistant_camera_boundary_config(
            NO_BOUNDARY,
            0,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr);
    }

    seekfree_assistant_camera_send();
}
#endif
#endif

void vision_loop()
{
    vision_perf_accum_t perf_acc;
    auto perf_window_start = std::chrono::steady_clock::now();

    while (g_vision_running.load())
    {
        auto loop_start = std::chrono::steady_clock::now();
        uint32 send_us_this_frame = 0;

#if defined(ENABLE_E99_VISION_STACK)
        if (!vision_pipeline_process_step())
        {
            system_delay_ms(5);
            continue;
        }

        vision_pipeline_send_step();
        send_us_this_frame = vision_client_sender_get_last_send_time_us();
#else
        if (!vision_image_processor_process_step())
        {
            system_delay_ms(5);
            continue;
        }

#if defined(ENABLE_SEEKFREE_ASSISTANT_CAMERA_STREAM)
        if (g_assistant_camera_stream_enabled.load() && vision_thread_try_acquire_send_slot())
        {
            auto send_start = std::chrono::steady_clock::now();
            vision_send_camera_by_seekfree_assistant();
            auto send_end = std::chrono::steady_clock::now();
            send_us_this_frame = static_cast<uint32>(std::chrono::duration_cast<std::chrono::microseconds>(send_end - send_start).count());
        }
#endif
#endif

        uint32 capture_wait_us = 0;
        uint32 preprocess_us = 0;
        uint32 otsu_us = 0;
        uint32 maze_us = 0;
        uint32 process_us = 0;
        vision_image_processor_get_last_perf_us(&capture_wait_us, &preprocess_us, &otsu_us, &maze_us, &process_us);

        auto loop_end = std::chrono::steady_clock::now();
        uint32 loop_us_this_frame = static_cast<uint32>(std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start).count());

        perf_acc.frame_count++;
        perf_acc.capture_wait_us += capture_wait_us;
        perf_acc.preprocess_us += preprocess_us;
        perf_acc.otsu_us += otsu_us;
        perf_acc.maze_us += maze_us;
        perf_acc.process_us += process_us;
        perf_acc.send_us += send_us_this_frame;
        perf_acc.loop_us += loop_us_this_frame;

        int64_t window_us = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - perf_window_start).count();
        if (window_us >= VISION_PERF_LOG_INTERVAL_US)
        {
            vision_perf_print_and_reset(perf_acc, window_us);
            perf_window_start = loop_end;
        }
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

    vision_image_processor_cleanup();
}

bool vision_thread_is_running()
{
    return g_vision_running.load();
}

void vision_thread_set_send_mode(vision_thread_send_mode_enum mode)
{
    int m = static_cast<int>(mode);
    if (m < static_cast<int>(VISION_THREAD_SEND_BINARY) || m > static_cast<int>(VISION_THREAD_SEND_RGB565_OVERLAY))
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

void vision_thread_set_send_max_fps(uint32 max_fps)
{
    uint32 v = max_fps;
    if (v > VISION_THREAD_SEND_MAX_FPS_UPPER)
    {
        v = VISION_THREAD_SEND_MAX_FPS_UPPER;
    }

#if defined(ENABLE_E99_VISION_STACK)
    vision_pipeline_set_send_max_fps(v);
#else
    g_send_max_fps.store(v);
    g_last_send_tick_us.store(0);
#endif
}

uint32 vision_thread_get_send_max_fps()
{
#if defined(ENABLE_E99_VISION_STACK)
    return vision_pipeline_get_send_max_fps();
#else
    return g_send_max_fps.load();
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

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

static vision_thread_send_mode_enum vision_thread_sanitize_send_mode(vision_thread_send_mode_enum mode)
{
    // 仅保留两种显示模式：
    // 1) 彩色纯图不画线（RGB565）
    // 2) 灰度图画边线/中线（GRAY）
    if (mode == VISION_THREAD_SEND_RGB565 || mode == VISION_THREAD_SEND_IPM_RGB565)
    {
        return VISION_THREAD_SEND_RGB565;
    }
    return VISION_THREAD_SEND_GRAY;
}

struct vision_perf_accum_t
{
    uint64_t frame_count = 0;
    uint64_t capture_wait_us = 0;
    uint64_t preprocess_us = 0;
    uint64_t red_detect_us = 0;
    uint64_t otsu_us = 0;
    uint64_t maze_us = 0;
    uint64_t maze_setup_us = 0;
    uint64_t maze_start_us = 0;
    uint64_t maze_trace_left_us = 0;
    uint64_t maze_trace_right_us = 0;
    uint64_t maze_post_us = 0;
    uint64_t maze_left_points = 0;
    uint64_t maze_right_points = 0;
    uint64_t maze_left_ok_frames = 0;
    uint64_t maze_right_ok_frames = 0;
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

    printf("[VISION PERF] fps=%.1f loop=%.2fms proc=%.2fms(wait=%.2f pre=%.2f red=%.2f otsu=%.2f maze=%.2f) send=%.2fms\r\n",
           fps,
           (static_cast<double>(acc.loop_us) * inv_frames) / 1000.0,
           (static_cast<double>(acc.process_us) * inv_frames) / 1000.0,
           (static_cast<double>(acc.capture_wait_us) * inv_frames) / 1000.0,
           (static_cast<double>(acc.preprocess_us) * inv_frames) / 1000.0,
           (static_cast<double>(acc.red_detect_us) * inv_frames) / 1000.0,
           (static_cast<double>(acc.otsu_us) * inv_frames) / 1000.0,
           (static_cast<double>(acc.maze_us) * inv_frames) / 1000.0,
           (static_cast<double>(acc.send_us) * inv_frames) / 1000.0);

    printf("[VISION MAZE] setup=%.3fms start=%.3fms traceL=%.3fms traceR=%.3fms post=%.3fms pts(L/R)=%.1f/%.1f ok(L/R)=%.1f%%/%.1f%%\r\n",
           (static_cast<double>(acc.maze_setup_us) * inv_frames) / 1000.0,
           (static_cast<double>(acc.maze_start_us) * inv_frames) / 1000.0,
           (static_cast<double>(acc.maze_trace_left_us) * inv_frames) / 1000.0,
           (static_cast<double>(acc.maze_trace_right_us) * inv_frames) / 1000.0,
           (static_cast<double>(acc.maze_post_us) * inv_frames) / 1000.0,
           static_cast<double>(acc.maze_left_points) * inv_frames,
           static_cast<double>(acc.maze_right_points) * inv_frames,
           (static_cast<double>(acc.maze_left_ok_frames) * 100.0) * inv_frames,
           (static_cast<double>(acc.maze_right_ok_frames) * 100.0) * inv_frames);

    acc = vision_perf_accum_t{};
}

#if !defined(ENABLE_E99_VISION_STACK)
std::atomic<bool> g_assistant_camera_stream_enabled(false);
std::atomic<int> g_send_mode(VISION_THREAD_SEND_BINARY);
std::atomic<uint32_t> g_send_max_fps(VISION_THREAD_SEND_DEFAULT_MAX_FPS);
std::atomic<uint64_t> g_last_send_tick_us(0);

#if defined(ENABLE_SEEKFREE_ASSISTANT_CAMERA_STREAM)
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

void vision_send_camera_by_seekfree_assistant()
{
    uint16 *x1 = nullptr;
    uint16 *x2 = nullptr;
    uint16 *x3 = nullptr;
    uint16 *y1 = nullptr;
    uint16 *y2 = nullptr;
    uint16 *y3 = nullptr;
    uint16 dot_num = 0;
    vision_image_processor_get_boundaries(&x1, &x2, &x3, &y1, &y2, &y3, &dot_num);

    vision_thread_send_mode_enum mode = vision_thread_sanitize_send_mode(
        static_cast<vision_thread_send_mode_enum>(g_send_mode.load()));
    switch (mode)
    {
        case VISION_THREAD_SEND_RGB565:
        case VISION_THREAD_SEND_IPM_RGB565:
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
        case VISION_THREAD_SEND_BINARY:
        case VISION_THREAD_SEND_IPM_EDGE_GRAY:
        case VISION_THREAD_SEND_RGB565_OVERLAY:
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

    const bool draw_boundary = (mode == VISION_THREAD_SEND_GRAY);
    if (draw_boundary && dot_num > 0 && x1 && x2 && x3 && y1 && y2 && y3)
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
        uint32 red_detect_us = 0;
        uint32 otsu_us = 0;
        uint32 maze_us = 0;
        uint32 process_us = 0;
        vision_image_processor_get_last_perf_us(&capture_wait_us, &preprocess_us, &otsu_us, &maze_us, &process_us);
        vision_image_processor_get_last_red_detect_us(&red_detect_us);
        uint32 maze_setup_us = 0;
        uint32 maze_start_us = 0;
        uint32 maze_trace_left_us = 0;
        uint32 maze_trace_right_us = 0;
        uint32 maze_post_us = 0;
        uint16 maze_left_points = 0;
        uint16 maze_right_points = 0;
        bool maze_left_ok = false;
        bool maze_right_ok = false;
        vision_image_processor_get_last_maze_detail_us(&maze_setup_us,
                                                       &maze_start_us,
                                                       &maze_trace_left_us,
                                                       &maze_trace_right_us,
                                                       &maze_post_us,
                                                       &maze_left_points,
                                                       &maze_right_points,
                                                       &maze_left_ok,
                                                       &maze_right_ok);

        auto loop_end = std::chrono::steady_clock::now();
        uint32 loop_us_this_frame = static_cast<uint32>(std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start).count());

        perf_acc.frame_count++;
        perf_acc.capture_wait_us += capture_wait_us;
        perf_acc.preprocess_us += preprocess_us;
        perf_acc.red_detect_us += red_detect_us;
        perf_acc.otsu_us += otsu_us;
        perf_acc.maze_us += maze_us;
        perf_acc.maze_setup_us += maze_setup_us;
        perf_acc.maze_start_us += maze_start_us;
        perf_acc.maze_trace_left_us += maze_trace_left_us;
        perf_acc.maze_trace_right_us += maze_trace_right_us;
        perf_acc.maze_post_us += maze_post_us;
        perf_acc.maze_left_points += maze_left_points;
        perf_acc.maze_right_points += maze_right_points;
        perf_acc.maze_left_ok_frames += maze_left_ok ? 1U : 0U;
        perf_acc.maze_right_ok_frames += maze_right_ok ? 1U : 0U;
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

#if defined(ENABLE_E99_VISION_STACK)
    vision_pipeline_cleanup();
#else
    vision_image_processor_cleanup();
#endif
}

bool vision_thread_is_running()
{
    return g_vision_running.load();
}

void vision_thread_set_send_mode(vision_thread_send_mode_enum mode)
{
    const vision_thread_send_mode_enum mode_sanitized = vision_thread_sanitize_send_mode(mode);
    int m = static_cast<int>(mode_sanitized);
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

void vision_thread_set_roi_capture_mode(bool enabled)
{
#if defined(ENABLE_E99_VISION_STACK)
    vision_pipeline_set_roi_capture_mode(enabled);
#else
    (void)enabled;
#endif
}

bool vision_thread_roi_capture_mode_enabled()
{
#if defined(ENABLE_E99_VISION_STACK)
    return vision_pipeline_roi_capture_mode_enabled();
#else
    return false;
#endif
}

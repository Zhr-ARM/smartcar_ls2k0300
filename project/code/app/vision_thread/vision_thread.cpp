#include "vision_thread.h"

#include "driver/vision/vision_image_processor.h"
#include "driver/vision/vision_pipeline.h"
#include "driver/vision/vision_transport.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

namespace
{
std::thread g_vision_thread;
std::atomic<bool> g_vision_running(false);

constexpr int64_t VISION_PERF_LOG_INTERVAL_US = 1000 * 1000;

static vision_thread_send_mode_enum vision_thread_sanitize_send_mode(vision_thread_send_mode_enum mode)
{
    if (mode == VISION_THREAD_SEND_BINARY)
    {
        return VISION_THREAD_SEND_BINARY;
    }
    if (mode == VISION_THREAD_SEND_RGB565)
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

    (void)window_us;
    acc = vision_perf_accum_t{};
}

void vision_loop()
{
    vision_perf_accum_t perf_acc;
    auto perf_window_start = std::chrono::steady_clock::now();

    while (g_vision_running.load())
    {
        auto loop_start = std::chrono::steady_clock::now();
        uint32 send_us_this_frame = 0;

        if (!vision_pipeline_process_step())
        {
            system_delay_ms(5);
            continue;
        }

        vision_pipeline_send_step();
        send_us_this_frame = vision_transport_get_last_send_time_us();

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

bool vision_thread_init(const char *camera_path)
{
    return vision_thread_init(camera_path, nullptr, false);
}

void vision_thread_cleanup()
{
    g_vision_running = false;

    if (g_vision_thread.joinable())
    {
        g_vision_thread.join();
    }

    vision_pipeline_cleanup();
}

bool vision_thread_is_running()
{
    return g_vision_running.load();
}

void vision_thread_set_send_mode(vision_thread_send_mode_enum mode)
{
    const vision_thread_send_mode_enum mode_sanitized = vision_thread_sanitize_send_mode(mode);
    vision_pipeline_set_send_mode(static_cast<vision_send_mode_enum>(mode_sanitized));
}

vision_thread_send_mode_enum vision_thread_get_send_mode()
{
    return static_cast<vision_thread_send_mode_enum>(vision_pipeline_get_send_mode());
}

void vision_thread_set_send_max_fps(uint32 max_fps)
{
    vision_pipeline_set_send_max_fps(max_fps);
}

uint32 vision_thread_get_send_max_fps()
{
    return vision_pipeline_get_send_max_fps();
}

void vision_thread_set_client_sender_enabled(bool enabled)
{
    vision_pipeline_set_send_enabled(enabled);
}

bool vision_thread_client_sender_enabled()
{
    return vision_pipeline_is_send_enabled();
}

void vision_thread_set_infer_enabled(bool enabled)
{
    vision_pipeline_set_infer_enabled(enabled);
}

bool vision_thread_infer_enabled()
{
    return vision_pipeline_infer_enabled();
}

void vision_thread_set_ncnn_enabled(bool enabled)
{
    vision_pipeline_set_ncnn_enabled(enabled);
}

bool vision_thread_ncnn_enabled()
{
    return vision_pipeline_ncnn_enabled();
}

void vision_thread_set_roi_capture_mode(bool enabled)
{
    vision_pipeline_set_roi_capture_mode(enabled);
}

bool vision_thread_roi_capture_mode_enabled()
{
    return vision_pipeline_roi_capture_mode_enabled();
}

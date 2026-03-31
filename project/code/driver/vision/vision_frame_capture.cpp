#include "driver/vision/vision_frame_capture.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>
#include <condition_variable>

namespace
{
static std::thread g_capture_thread;
static std::atomic<bool> g_capture_running(false);
static std::mutex g_capture_mutex;
static std::condition_variable g_capture_cv;
static std::array<uint8, UVC_HEIGHT * UVC_WIDTH * 3> g_capture_frame{};
static uint32 g_capture_seq = 0;
static uint32 g_processed_seq = 0;

static void capture_loop()
{
    while (g_capture_running.load())
    {
        if (wait_image_refresh() < 0 || bgr_image == nullptr)
        {
            system_delay_ms(1);
            continue;
        }

        {
            std::lock_guard<std::mutex> lk(g_capture_mutex);
            std::memcpy(g_capture_frame.data(), bgr_image, g_capture_frame.size());
            ++g_capture_seq;
        }
        g_capture_cv.notify_one();
    }
}

} // namespace

bool vision_frame_capture_init(const char *camera_path)
{
    if (g_capture_running.load())
    {
        return true;
    }

    if (camera_path == nullptr || camera_path[0] == '\0')
    {
        camera_path = "/dev/video0";
    }

    if (uvc_camera_init(camera_path) != 0)
    {
        return false;
    }

    {
        std::lock_guard<std::mutex> lk(g_capture_mutex);
        g_capture_seq = 0;
        g_processed_seq = 0;
    }

    g_capture_running.store(true);
    g_capture_thread = std::thread(capture_loop);
    return true;
}

void vision_frame_capture_cleanup()
{
    if (!g_capture_running.load())
    {
        return;
    }

    g_capture_running.store(false);
    g_capture_cv.notify_all();

    if (g_capture_thread.joinable())
    {
        g_capture_thread.join();
    }
}

bool vision_frame_capture_wait_next_bgr(uint8 *out_bgr, size_t out_bgr_bytes, uint32 timeout_ms, uint32 *wait_us)
{
    auto t0 = std::chrono::steady_clock::now();
    if (out_bgr == nullptr || out_bgr_bytes < g_capture_frame.size())
    {
        if (wait_us != nullptr)
        {
            *wait_us = 0;
        }
        return false;
    }

    std::unique_lock<std::mutex> lk(g_capture_mutex);
    const bool ok = g_capture_cv.wait_for(
        lk,
        std::chrono::milliseconds(timeout_ms),
        []() { return (g_capture_seq != g_processed_seq) || !g_capture_running.load(); });

    if (!ok || g_capture_seq == g_processed_seq)
    {
        if (wait_us != nullptr)
        {
            *wait_us = static_cast<uint32>(
                std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count());
        }
        return false;
    }

    std::memcpy(out_bgr, g_capture_frame.data(), g_capture_frame.size());
    g_processed_seq = g_capture_seq;
    if (wait_us != nullptr)
    {
        *wait_us = static_cast<uint32>(
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count());
    }
    return true;
}

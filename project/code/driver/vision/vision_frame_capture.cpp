#include "driver/vision/vision_frame_capture.h"
#include "driver/vision/vision_image_processor.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>
#include <condition_variable>

namespace
{
static inline size_t current_full_frame_bytes()
{
    return static_cast<size_t>(vision_camera_width()) *
           static_cast<size_t>(vision_camera_height()) * 3U;
}

// 采图线程对象：由 init 创建，cleanup 回收。
static std::thread g_capture_thread;
// 采图线程运行标志：true=持续采图，false=停止。
static std::atomic<bool> g_capture_running(false);
// 采图共享缓冲互斥锁与条件变量。
static std::mutex g_capture_mutex;
static std::condition_variable g_capture_cv;
// 最新一帧 BGR 缓冲（full 分辨率）。
static std::array<uint8, VISION_MAX_CAMERA_HEIGHT * VISION_MAX_CAMERA_WIDTH * 3> g_capture_frame{};
// 生产序号（采图线程递增）与消费序号（处理线程递增）。
static uint32 g_capture_seq = 0;
static uint32 g_processed_seq = 0;

// 作用：采图线程主循环。
// 调用关系：仅由 vision_frame_capture_init 创建线程后调用。
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
            std::memcpy(g_capture_frame.data(), bgr_image, current_full_frame_bytes());
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

    // camera_path 为空时使用默认设备。
    if (camera_path == nullptr || camera_path[0] == '\0')
    {
        camera_path = "/dev/video0";
    }

    if (uvc_camera_init(camera_path, vision_camera_width(), vision_camera_height()) != 0)
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
    // 参数合法性检查：输出缓冲大小必须覆盖 full BGR 一帧。
    const size_t frame_bytes = current_full_frame_bytes();
    if (out_bgr == nullptr || out_bgr_bytes < frame_bytes)
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

    std::memcpy(out_bgr, g_capture_frame.data(), frame_bytes);
    g_processed_seq = g_capture_seq;
    if (wait_us != nullptr)
    {
        *wait_us = static_cast<uint32>(
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count());
    }
    return true;
}

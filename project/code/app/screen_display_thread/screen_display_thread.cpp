#include "screen_display_thread.h"

#include "battery.h"
#include "line_follow_thread.h"
#include "motor_thread.h"
#include "driver/vision/vision_image_processor.h"
#include "vision_thread.h"
#include "zf_common_headfile.h"

#include <algorithm>
#include <vector>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>

namespace
{
// 车载屏显刷新周期：20Hz，兼顾可读性与CPU占用。
constexpr int32 SCREEN_REFRESH_PERIOD_MS = 50;
constexpr int32 OVERLAY_LINE_THICKNESS_PX = 3;

std::thread g_screen_display_thread;
std::atomic<bool> g_screen_display_running(false);

void draw_thick_overlay_line(int32 x_start,
                             int32 y_start,
                             int32 x_end,
                             int32 y_end,
                             uint16 color,
                             int32 thickness)
{
    int32 max_x = vision_processing_width() - 1;
    int32 max_y = vision_processing_height() - 1;

    int32 sx = std::clamp(x_start, 0, max_x);
    int32 sy = std::clamp(y_start, 0, max_y);
    int32 ex = std::clamp(x_end, 0, max_x);
    int32 ey = std::clamp(y_end, 0, max_y);

    if (thickness <= 1)
    {
        ips200_draw_line(static_cast<uint16>(sx),
                         static_cast<uint16>(sy),
                         static_cast<uint16>(ex),
                         static_cast<uint16>(ey),
                         color);
        return;
    }

    const int32 half = thickness / 2;
    const int32 dx = std::abs(ex - sx);
    const int32 dy = std::abs(ey - sy);

    // 近似法线方向加偏移，低成本实现边线加粗效果。
    if (dx >= dy)
    {
        for (int32 offset = -half; offset <= half; ++offset)
        {
            int32 sy_off = std::clamp(sy + offset, 0, max_y);
            int32 ey_off = std::clamp(ey + offset, 0, max_y);
            ips200_draw_line(static_cast<uint16>(sx),
                             static_cast<uint16>(sy_off),
                             static_cast<uint16>(ex),
                             static_cast<uint16>(ey_off),
                             color);
        }
    }
    else
    {
        for (int32 offset = -half; offset <= half; ++offset)
        {
            int32 sx_off = std::clamp(sx + offset, 0, max_x);
            int32 ex_off = std::clamp(ex + offset, 0, max_x);
            ips200_draw_line(static_cast<uint16>(sx_off),
                             static_cast<uint16>(sy),
                             static_cast<uint16>(ex_off),
                             static_cast<uint16>(ey),
                             color);
        }
    }
}

/**
 * @brief 车载屏幕显示线程主循环
 *
 * 显示内容由两部分构成：
 * 1) 图像区：展示视觉线程输出的灰度图。
 * 2) 状态区：展示控制闭环关键量（误差、左右目标、位置环输出、电池电压）。
 *
 * 约定：
 * - 红线为图像中心线；
 * - 绿线为根据位置环误差反推的当前轨迹线位置；
 * - 文本单位与控制线程保持一致，避免调试歧义。
 */
void screen_display_loop()
{
    // 使用本地快照避免上游图像缓冲区在刷新时被并发改写导致撕裂。
    std::vector<uint8> img_snapshot(VISION_MAX_PROC_WIDTH * VISION_MAX_PROC_HEIGHT * 2, 0);

    while (g_screen_display_running.load())
    {
        vision_thread_send_mode_enum mode = vision_thread_get_send_mode();

        const int32 vision_proc_width = vision_processing_width();
        const int32 vision_proc_height = vision_processing_height();

        if (mode == VISION_THREAD_SEND_BINARY)
        {
            const uint8 *img = vision_image_processor_binary_downsampled_u8_image();
            if (img != nullptr)
            {
                std::memcpy(img_snapshot.data(), img, static_cast<size_t>(vision_proc_width) * vision_proc_height);
                // 二值图(0/255的数组)作为灰度图画出来就是黑白的
                ips200_show_gray_image(0, 0, img_snapshot.data(), vision_proc_width, vision_proc_height);
            }
        }
        else if (mode == VISION_THREAD_SEND_RGB565)
        {
            const uint8 *img = vision_image_processor_rgb565_downsampled_image();
            if (img != nullptr)
            {
                std::memcpy(img_snapshot.data(), img, static_cast<size_t>(vision_proc_width) * vision_proc_height * 2U);
                ips200_show_rgb565_image(0, 0, img_snapshot.data(), vision_proc_width, vision_proc_height);
            }
        }
        else // VISION_THREAD_SEND_GRAY
        {
            const uint8 *img = vision_image_processor_gray_downsampled_image();
            if (img != nullptr)
            {
                std::memcpy(img_snapshot.data(), img, static_cast<size_t>(vision_proc_width) * vision_proc_height);
                ips200_show_gray_image(0, 0, img_snapshot.data(), vision_proc_width, vision_proc_height);
            }
        }

        const float error_px = line_follow_thread_error();
        const int32 x_center = vision_proc_width / 2;
        const float ratio = std::clamp(line_sample_ratio, 0.0f, 1.0f);
        int32 sample_y = std::clamp<int32>(static_cast<int32>(vision_proc_height * ratio), 0, vision_proc_height - 1);

        // 获取并绘制视觉边界数据
        uint16 *x1 = nullptr, *x2 = nullptr, *x3 = nullptr;
        uint16 *y1 = nullptr, *y2 = nullptr, *y3 = nullptr;
        uint16 dot_num = 0;
        vision_image_processor_get_boundaries(&x1, &x2, &x3, &y1, &y2, &y3, &dot_num);

        // // 竖直中心线
        // ips200_draw_line(static_cast<uint16>(x_center),
        //                  0,
        //                  static_cast<uint16>(x_center),
        //                  static_cast<uint16>(VISION_PROC_HEIGHT - 1),
        //                  RGB565_GREEN);

        // 左右边界和左右均值中线（不画横向虚线）
        if (dot_num > 1 && x1 && y1 && x2 && y2 && x3 && y3)
        {
            for (uint16 i = 1; i < dot_num; ++i)
            {
                draw_thick_overlay_line(x1[i - 1], y1[i - 1], x1[i], y1[i], RGB565_RED, OVERLAY_LINE_THICKNESS_PX);
                draw_thick_overlay_line(x2[i - 1], y2[i - 1], x2[i], y2[i], RGB565_YELLOW, OVERLAY_LINE_THICKNESS_PX);
                draw_thick_overlay_line(x3[i - 1], y3[i - 1], x3[i], y3[i], RGB565_BLUE, OVERLAY_LINE_THICKNESS_PX);
            }
        }

        char buf[96];

        std::snprintf(buf, sizeof(buf), "Bat:%5.2fV", battery_monitor.voltage_v());
        ips200_show_string(0, vision_proc_height + 10, buf);

        std::snprintf(buf, sizeof(buf), "Err:%+6.1f", error_px);
        ips200_show_string(0, vision_proc_height + 30, buf);

        std::snprintf(buf, sizeof(buf), "TarL:%7.1f", motor_thread_left_target_count());
        ips200_show_string(0, vision_proc_height + 50, buf);

        std::snprintf(buf, sizeof(buf), "TarR:%7.1f", motor_thread_right_target_count());
        ips200_show_string(0, vision_proc_height + 70, buf);

        std::snprintf(buf, sizeof(buf), "Out:%+6.1f", line_follow_thread_turn_output());
        ips200_show_string(0, vision_proc_height + 90, buf);

        std::snprintf(buf, sizeof(buf), "SmpY:%3d(%.2f)", sample_y, ratio);
        ips200_show_string(0, vision_proc_height + 110, buf);

        system_delay_ms(SCREEN_REFRESH_PERIOD_MS);
    }
}
}

bool screen_display_thread_init()
{
    if (g_screen_display_running.load())
    {
        return true;
    }

    // 帧缓冲设备初始化仅在显示线程首次启动时执行一次。
    ips200_init("/dev/fb0");
    ips200_clear();

    g_screen_display_running = true;
    g_screen_display_thread = std::thread(screen_display_loop);
    return true;
}

void screen_display_thread_cleanup()
{
    if (!g_screen_display_running.load())
    {
        return;
    }

    // 先置停止标志，再等待线程退出，确保资源释放顺序确定。
    g_screen_display_running = false;

    if (g_screen_display_thread.joinable())
    {
        g_screen_display_thread.join();
    }
}

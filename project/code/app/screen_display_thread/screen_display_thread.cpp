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
#include <cstdio>
#include <cstring>
#include <thread>

namespace
{
// 车载屏显刷新周期：20Hz，兼顾可读性与CPU占用。
constexpr int32 SCREEN_REFRESH_PERIOD_MS = 50;
constexpr int32 VISION_PROC_WIDTH = VISION_DOWNSAMPLED_WIDTH;
constexpr int32 VISION_PROC_HEIGHT = VISION_DOWNSAMPLED_HEIGHT;

std::thread g_screen_display_thread;
std::atomic<bool> g_screen_display_running(false);

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
    std::vector<uint8> img_snapshot(VISION_PROC_WIDTH * VISION_PROC_HEIGHT * 2, 0);

    while (g_screen_display_running.load())
    {
        vision_thread_send_mode_enum mode = vision_thread_get_send_mode();

        if (mode == VISION_THREAD_SEND_BINARY)
        {
            const uint8 *img = vision_image_processor_binary_downsampled_u8_image();
            if (img != nullptr)
            {
                std::memcpy(img_snapshot.data(), img, VISION_PROC_WIDTH * VISION_PROC_HEIGHT);
                // 二值图(0/255的数组)作为灰度图画出来就是黑白的
                ips200_show_gray_image(0, 0, img_snapshot.data(), VISION_PROC_WIDTH, VISION_PROC_HEIGHT);
            }
        }
        else if (mode == VISION_THREAD_SEND_RGB565)
        {
            const uint8 *img = vision_image_processor_rgb565_downsampled_image();
            if (img != nullptr)
            {
                std::memcpy(img_snapshot.data(), img, VISION_PROC_WIDTH * VISION_PROC_HEIGHT * 2);
                ips200_show_rgb565_image(0, 0, img_snapshot.data(), VISION_PROC_WIDTH, VISION_PROC_HEIGHT);
            }
        }
        else // VISION_THREAD_SEND_GRAY
        {
            const uint8 *img = vision_image_processor_gray_downsampled_image();
            if (img != nullptr)
            {
                std::memcpy(img_snapshot.data(), img, VISION_PROC_WIDTH * VISION_PROC_HEIGHT);
                ips200_show_gray_image(0, 0, img_snapshot.data(), VISION_PROC_WIDTH, VISION_PROC_HEIGHT);
            }
        }

        const float error_px = line_follow_thread_error();
        const int32 x_center = VISION_PROC_WIDTH / 2;
        int32 den = (line_sample_ratio_den <= 0) ? 1 : line_sample_ratio_den;
        int32 num = std::max<int32>(0, line_sample_ratio_num);
        int32 sample_y = std::min<int32>(VISION_PROC_HEIGHT - 1, (VISION_PROC_HEIGHT * num) / den);

        // 获取并绘制视觉边界数据
        uint16 *x1 = nullptr, *x2 = nullptr, *x3 = nullptr;
        uint16 *y1 = nullptr, *y2 = nullptr, *y3 = nullptr;
        uint16 dot_num = 0;
        vision_image_processor_get_boundaries(&x1, &x2, &x3, &y1, &y2, &y3, &dot_num);

        // 竖直中心线
        ips200_draw_line(static_cast<uint16>(x_center),
                         0,
                         static_cast<uint16>(x_center),
                         static_cast<uint16>(VISION_PROC_HEIGHT - 1),
                         RGB565_GREEN);

        // 左右边界和左右均值中线（不画横向虚线）
        if (dot_num > 1 && x1 && y1 && x2 && y2 && x3 && y3)
        {
            for (uint16 i = 1; i < dot_num; ++i)
            {
                ips200_draw_line(x1[i - 1], y1[i - 1], x1[i], y1[i], RGB565_RED);
                ips200_draw_line(x2[i - 1], y2[i - 1], x2[i], y2[i], RGB565_YELLOW);
                ips200_draw_line(x3[i - 1], y3[i - 1], x3[i], y3[i], RGB565_BLUE);
            }
        }

        char buf[96];

        std::snprintf(buf, sizeof(buf), "Bat:%5.2fV", battery_monitor.voltage_v());
        ips200_show_string(0, VISION_PROC_HEIGHT + 10, buf);

        std::snprintf(buf, sizeof(buf), "Err:%+6.1f", error_px);
        ips200_show_string(0, VISION_PROC_HEIGHT + 30, buf);

        std::snprintf(buf, sizeof(buf), "TarL:%7.1f", motor_thread_left_target_count());
        ips200_show_string(0, VISION_PROC_HEIGHT + 50, buf);

        std::snprintf(buf, sizeof(buf), "TarR:%7.1f", motor_thread_right_target_count());
        ips200_show_string(0, VISION_PROC_HEIGHT + 70, buf);

        std::snprintf(buf, sizeof(buf), "Out:%+6.1f", line_follow_thread_turn_output());
        ips200_show_string(0, VISION_PROC_HEIGHT + 90, buf);

        std::snprintf(buf, sizeof(buf), "SmpY:%3d(%d/%d)", sample_y, num, den);
        ips200_show_string(0, VISION_PROC_HEIGHT + 110, buf);

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

#include "driver/vision/vision_client_sender.h"

#include "driver/vision/vision_image_processor.h"

#include <opencv2/opencv.hpp>

#include <atomic>
#include <cstring>
#include <vector>

// ============================== 参数区 ==============================
// 1bit打包二值图缓存大小
#define VISION_BINARY_PACKED_SIZE (UVC_HEIGHT * UVC_WIDTH / 8)

// ============================== 全局变量区 ==============================
static uint8 g_image_binary_1bit[VISION_BINARY_PACKED_SIZE];
static std::vector<uint8> g_image_rgb565_overlay(UVC_WIDTH * UVC_HEIGHT * 2);
// 保持与旧工程一致：默认图传和屏显都使用二值图。
static std::atomic<int> g_send_mode(VISION_SEND_BINARY);
static std::atomic<bool> g_send_enabled(true);
static std::atomic<int> g_last_send_mode(-1);

static void pack_binary_1bit(const uint8 *src, uint8 *dst, int width, int height)
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

static void config_camera_send_packet(vision_send_mode_enum mode)
{
    uint8 *x1 = nullptr, *x2 = nullptr, *x3 = nullptr;
    uint8 *y1 = nullptr, *y2 = nullptr, *y3 = nullptr;
    uint16 dot_num = 0;
    vision_image_processor_get_boundaries(&x1, &x2, &x3, &y1, &y2, &y3, &dot_num);

    switch (mode)
    {
        case VISION_SEND_BINARY:
            seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_BINARY, g_image_binary_1bit, UVC_WIDTH, UVC_HEIGHT);
            break;
        case VISION_SEND_GRAY:
            seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_GRAY, const_cast<uint8 *>(vision_image_processor_gray_image()), UVC_WIDTH, UVC_HEIGHT);
            break;
        case VISION_SEND_RGB565:
        default:
            seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_RGB565, g_image_rgb565_overlay.data(), UVC_WIDTH, UVC_HEIGHT);
            break;
    }

    seekfree_assistant_camera_boundary_config(XY_BOUNDARY, dot_num, x1, x2, x3, y1, y2, y3);
    g_last_send_mode.store(static_cast<int>(mode));
}

static void update_rgb565_overlay_frame()
{
    const uint8 *bgr_data = vision_image_processor_bgr_image();
    if (bgr_data == nullptr)
    {
        std::memset(g_image_rgb565_overlay.data(), 0, g_image_rgb565_overlay.size());
        return;
    }

    cv::Mat frame(UVC_HEIGHT, UVC_WIDTH, CV_8UC3, const_cast<uint8 *>(bgr_data));
    cv::Mat draw = frame.clone();

    // 在全局比例高度处绘制采样高度线，并叠加偏差线。
    int den = (line_sample_ratio_den <= 0) ? 1 : line_sample_ratio_den;
    int num = std::max(0, line_sample_ratio_num);
    int y = std::min(UVC_HEIGHT - 1, (UVC_HEIGHT * num) / den);
    int x_center = UVC_WIDTH / 2;
    int x_mid = x_center + line_error;
    x_mid = std::max(0, std::min(x_mid, UVC_WIDTH - 1));

    cv::line(draw, cv::Point(0, y), cv::Point(UVC_WIDTH - 1, y), cv::Scalar(0, 255, 0), 1);
    cv::line(draw, cv::Point(x_center, y), cv::Point(x_mid, y), cv::Scalar(0, 255, 255), 2);
    cv::circle(draw, cv::Point(x_center, y), 2, cv::Scalar(255, 0, 0), -1);
    cv::circle(draw, cv::Point(x_mid, y), 2, cv::Scalar(0, 0, 255), -1);

    char text[48];
    std::snprintf(text, sizeof(text), "sample_y=%d(%d/%d)", y, num, den);
    cv::putText(draw,
                text,
                cv::Point(2, (y > 12) ? (y - 4) : 12),
                cv::FONT_HERSHEY_SIMPLEX,
                0.35,
                cv::Scalar(0, 255, 0),
                1,
                cv::LINE_AA);

    for (int y = 0; y < UVC_HEIGHT; ++y)
    {
        const cv::Vec3b *row = draw.ptr<cv::Vec3b>(y);
        for (int x = 0; x < UVC_WIDTH; ++x)
        {
            uint8 b = row[x][0];
            uint8 g = row[x][1];
            uint8 r = row[x][2];
            uint16 v = static_cast<uint16>(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
            int i = (y * UVC_WIDTH + x) * 2;
            // RGB565 大端
            g_image_rgb565_overlay[i] = static_cast<uint8>(v >> 8);
            g_image_rgb565_overlay[i + 1] = static_cast<uint8>(v & 0xFF);
        }
    }
}

void vision_client_sender_init()
{
    // 延后到 send_step 首帧再做配置，避免初始化阶段图像指针尚未就绪导致黑屏
    g_last_send_mode.store(-1);
}

void vision_client_sender_send_step()
{
    if (!g_send_enabled.load())
    {
        return;
    }

    vision_send_mode_enum mode = static_cast<vision_send_mode_enum>(g_send_mode.load());
    if (g_last_send_mode.load() != static_cast<int>(mode))
    {
        config_camera_send_packet(mode);
    }

    if (mode == VISION_SEND_BINARY)
    {
        pack_binary_1bit(vision_image_processor_binary_u8_image(), g_image_binary_1bit, UVC_WIDTH, UVC_HEIGHT);
    }
    else if (mode == VISION_SEND_RGB565)
    {
        update_rgb565_overlay_frame();
    }

    seekfree_assistant_camera_send();
}

void vision_client_sender_set_mode(vision_send_mode_enum mode)
{
    int m = static_cast<int>(mode);
    if (m < static_cast<int>(VISION_SEND_BINARY) || m > static_cast<int>(VISION_SEND_RGB565))
    {
        return;
    }
    g_send_mode.store(m);
}

vision_send_mode_enum vision_client_sender_get_mode()
{
    return static_cast<vision_send_mode_enum>(g_send_mode.load());
}

void vision_client_sender_set_enabled(bool enabled)
{
    g_send_enabled.store(enabled);
}

bool vision_client_sender_is_enabled()
{
    return g_send_enabled.load();
}

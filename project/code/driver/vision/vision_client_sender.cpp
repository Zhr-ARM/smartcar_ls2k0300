#include "driver/vision/vision_client_sender.h"

#include "driver/vision/vision_image_processor.h"

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <cstring>
#include <vector>

// ============================== 参数区 ==============================
// 1bit打包二值图缓存大小（发送分辨率固定为降采样图）
#define VISION_BINARY_PACKED_SIZE (VISION_DOWNSAMPLED_HEIGHT * VISION_DOWNSAMPLED_WIDTH / 8)
// 图传发送限频，避免处理线程过快导致 TCP 发送队列持续堆积。
static constexpr uint32 VISION_SEND_DEFAULT_MAX_FPS = 30;
static constexpr uint32 VISION_SEND_MAX_FPS_UPPER = 240;

// ============================== 全局变量区 ==============================
static uint8 g_image_binary_1bit[VISION_BINARY_PACKED_SIZE];
static std::vector<uint8> g_image_rgb565_frame(VISION_DOWNSAMPLED_WIDTH * VISION_DOWNSAMPLED_HEIGHT * 2);
// 保持与旧工程一致：默认图传和屏显都使用二值图。
static std::atomic<int> g_send_mode(VISION_SEND_BINARY);
static std::atomic<bool> g_send_enabled(true);
static std::atomic<int> g_last_send_mode(-1);
static std::atomic<uint32> g_last_send_time_us(0);
static std::atomic<uint64> g_last_send_tick_us(0);
static std::atomic<uint32> g_send_max_fps(VISION_SEND_DEFAULT_MAX_FPS);

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

static void convert_bgr_to_rgb565_be(const uint8 *bgr_data, uint8 *rgb565_data, int width, int height)
{
    if (bgr_data == nullptr || rgb565_data == nullptr || width <= 0 || height <= 0)
    {
        return;
    }

    const int pixel_num = width * height;
    for (int i = 0; i < pixel_num; ++i)
    {
        uint8 b = bgr_data[i * 3 + 0];
        uint8 g = bgr_data[i * 3 + 1];
        uint8 r = bgr_data[i * 3 + 2];
        uint16 v = static_cast<uint16>(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
        rgb565_data[i * 2 + 0] = static_cast<uint8>(v >> 8);
        rgb565_data[i * 2 + 1] = static_cast<uint8>(v & 0xFF);
    }
}

static void update_rgb565_from_bgr_source(const uint8 *bgr_data)
{
    if (bgr_data == nullptr)
    {
        std::memset(g_image_rgb565_frame.data(), 0, g_image_rgb565_frame.size());
        return;
    }

    convert_bgr_to_rgb565_be(bgr_data,
                             g_image_rgb565_frame.data(),
                             VISION_DOWNSAMPLED_WIDTH,
                             VISION_DOWNSAMPLED_HEIGHT);
}

static bool mode_enable_boundary_packet(vision_send_mode_enum mode)
{
    // 纯彩图模式默认关闭边界包，保证画面“纯净”用于调试。
    if (mode == VISION_SEND_RGB565 || mode == VISION_SEND_IPM_RGB565)
    {
        return false;
    }
    return true;
}

static void config_camera_send_packet(vision_send_mode_enum mode)
{
    uint8 *x1 = nullptr;
    uint8 *x2 = nullptr;
    uint8 *x3 = nullptr;
    uint8 *y1 = nullptr;
    uint8 *y2 = nullptr;
    uint8 *y3 = nullptr;
    uint16 dot_num = 0;
    vision_image_processor_get_boundaries(&x1, &x2, &x3, &y1, &y2, &y3, &dot_num);

    switch (mode)
    {
        case VISION_SEND_BINARY:
            seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_BINARY,
                                                         g_image_binary_1bit,
                                                         VISION_DOWNSAMPLED_WIDTH,
                                                         VISION_DOWNSAMPLED_HEIGHT);
            break;
        case VISION_SEND_GRAY:
            seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_GRAY,
                                                         const_cast<uint8 *>(vision_image_processor_gray_downsampled_image()),
                                                         VISION_DOWNSAMPLED_WIDTH,
                                                         VISION_DOWNSAMPLED_HEIGHT);
            break;
        case VISION_SEND_RGB565:
        case VISION_SEND_IPM_RGB565:
        case VISION_SEND_RGB565_OVERLAY:
            seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_RGB565,
                                                         g_image_rgb565_frame.data(),
                                                         VISION_DOWNSAMPLED_WIDTH,
                                                         VISION_DOWNSAMPLED_HEIGHT);
            break;
        case VISION_SEND_IPM_EDGE_GRAY:
            seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_GRAY,
                                                         const_cast<uint8 *>(vision_image_processor_ipm_edge_gray_downsampled_image()),
                                                         VISION_DOWNSAMPLED_WIDTH,
                                                         VISION_DOWNSAMPLED_HEIGHT);
            break;
        default:
            seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_GRAY,
                                                         const_cast<uint8 *>(vision_image_processor_gray_downsampled_image()),
                                                         VISION_DOWNSAMPLED_WIDTH,
                                                         VISION_DOWNSAMPLED_HEIGHT);
            break;
    }

    if (mode_enable_boundary_packet(mode) && dot_num > 0 && x1 && x2 && x3 && y1 && y2 && y3)
    {
        seekfree_assistant_camera_boundary_config(XY_BOUNDARY, dot_num, x1, x2, x3, y1, y2, y3);
    }
    else
    {
        seekfree_assistant_camera_boundary_config(NO_BOUNDARY, 0, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    }
    g_last_send_mode.store(static_cast<int>(mode));
}

static void refresh_camera_boundary_packet(vision_send_mode_enum mode)
{
    if (!mode_enable_boundary_packet(mode))
    {
        seekfree_assistant_camera_boundary_config(NO_BOUNDARY, 0, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
        return;
    }

    uint8 *x1 = nullptr;
    uint8 *x2 = nullptr;
    uint8 *x3 = nullptr;
    uint8 *y1 = nullptr;
    uint8 *y2 = nullptr;
    uint8 *y3 = nullptr;
    uint16 dot_num = 0;
    vision_image_processor_get_boundaries(&x1, &x2, &x3, &y1, &y2, &y3, &dot_num);
    if (dot_num > 0 && x1 && x2 && x3 && y1 && y2 && y3)
    {
        seekfree_assistant_camera_boundary_config(XY_BOUNDARY, dot_num, x1, x2, x3, y1, y2, y3);
    }
    else
    {
        seekfree_assistant_camera_boundary_config(NO_BOUNDARY, 0, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    }
}

static void update_rgb565_overlay_frame()
{
    const uint8 *bgr_data = vision_image_processor_bgr_downsampled_image();
    if (bgr_data == nullptr)
    {
        std::memset(g_image_rgb565_frame.data(), 0, g_image_rgb565_frame.size());
        return;
    }

    cv::Mat frame(VISION_DOWNSAMPLED_HEIGHT, VISION_DOWNSAMPLED_WIDTH, CV_8UC3, const_cast<uint8 *>(bgr_data));
    cv::Mat draw = frame.clone();

    // 竖直中心线（去掉原先横向虚线）
    const int x_center = VISION_DOWNSAMPLED_WIDTH / 2;
    cv::line(draw,
             cv::Point(x_center, 0),
             cv::Point(x_center, VISION_DOWNSAMPLED_HEIGHT - 1),
             cv::Scalar(0, 255, 0),
             1);

    // 画“左右边界均值中线”（x2/y2）
    uint8 *x2 = nullptr;
    uint8 *y2 = nullptr;
    uint16 dot_num = 0;
    vision_image_processor_get_boundaries(nullptr, &x2, nullptr, nullptr, &y2, nullptr, &dot_num);
    if (dot_num > 1 && x2 != nullptr && y2 != nullptr)
    {
        for (uint16 i = 1; i < dot_num; ++i)
        {
            cv::Point p0(x2[i - 1], y2[i - 1]);
            cv::Point p1(x2[i], y2[i]);
            cv::line(draw, p0, p1, cv::Scalar(0, 255, 255), 1);
        }
    }

    for (int y = 0; y < VISION_DOWNSAMPLED_HEIGHT; ++y)
    {
        const cv::Vec3b *row = draw.ptr<cv::Vec3b>(y);
        for (int x = 0; x < VISION_DOWNSAMPLED_WIDTH; ++x)
        {
            uint8 b = row[x][0];
            uint8 g = row[x][1];
            uint8 r = row[x][2];
            uint16 v = static_cast<uint16>(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
            int i = (y * VISION_DOWNSAMPLED_WIDTH + x) * 2;
            // RGB565 大端
            g_image_rgb565_frame[i] = static_cast<uint8>(v >> 8);
            g_image_rgb565_frame[i + 1] = static_cast<uint8>(v & 0xFF);
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
    auto t0 = std::chrono::steady_clock::now();
    const uint32 max_fps = g_send_max_fps.load();
    if (max_fps > 0)
    {
        const uint64 now_us = static_cast<uint64>(
            std::chrono::duration_cast<std::chrono::microseconds>(t0.time_since_epoch()).count());
        const uint64 min_interval_us = 1000000ULL / static_cast<uint64>(max_fps);
        const uint64 last_us = g_last_send_tick_us.load();
        if (last_us != 0 && (now_us - last_us) < min_interval_us)
        {
            g_last_send_time_us.store(0);
            return;
        }
    }

    if (!g_send_enabled.load())
    {
        g_last_send_time_us.store(0);
        return;
    }

    vision_send_mode_enum mode = static_cast<vision_send_mode_enum>(g_send_mode.load());
    if (g_last_send_mode.load() != static_cast<int>(mode))
    {
        config_camera_send_packet(mode);
    }
    refresh_camera_boundary_packet(mode);

    if (mode == VISION_SEND_BINARY)
    {
        pack_binary_1bit(vision_image_processor_binary_downsampled_u8_image(),
                         g_image_binary_1bit,
                         VISION_DOWNSAMPLED_WIDTH,
                         VISION_DOWNSAMPLED_HEIGHT);
    }
    else if (mode == VISION_SEND_RGB565)
    {
        update_rgb565_from_bgr_source(vision_image_processor_bgr_downsampled_image());
    }
    else if (mode == VISION_SEND_IPM_RGB565)
    {
        update_rgb565_from_bgr_source(vision_image_processor_ipm_bgr_downsampled_image());
    }
    else if (mode == VISION_SEND_RGB565_OVERLAY)
    {
        update_rgb565_overlay_frame();
    }

    seekfree_assistant_camera_send();
    auto t1 = std::chrono::steady_clock::now();
    g_last_send_tick_us.store(static_cast<uint64>(
        std::chrono::duration_cast<std::chrono::microseconds>(t1.time_since_epoch()).count()));
    g_last_send_time_us.store(static_cast<uint32>(std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count()));
}

uint32 vision_client_sender_get_last_send_time_us()
{
    return g_last_send_time_us.load();
}

void vision_client_sender_set_mode(vision_send_mode_enum mode)
{
    int m = static_cast<int>(mode);
    if (m < static_cast<int>(VISION_SEND_BINARY) || m > static_cast<int>(VISION_SEND_RGB565_OVERLAY))
    {
        return;
    }
    g_send_mode.store(m);
}

vision_send_mode_enum vision_client_sender_get_mode()
{
    return static_cast<vision_send_mode_enum>(g_send_mode.load());
}

void vision_client_sender_set_max_fps(uint32 max_fps)
{
    uint32 v = max_fps;
    if (v > VISION_SEND_MAX_FPS_UPPER)
    {
        v = VISION_SEND_MAX_FPS_UPPER;
    }
    g_send_max_fps.store(v);
    g_last_send_tick_us.store(0);
}

uint32 vision_client_sender_get_max_fps()
{
    return g_send_max_fps.load();
}

void vision_client_sender_set_enabled(bool enabled)
{
    g_send_enabled.store(enabled);
}

bool vision_client_sender_is_enabled()
{
    return g_send_enabled.load();
}

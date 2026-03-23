#include "driver/vision/vision_client_sender.h"

#include "driver/vision/vision_image_processor.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <vector>

// ============================== 参数区 ==============================
// 图传发送限频，避免处理线程过快导致 TCP 发送队列持续堆积。
static constexpr uint32 VISION_SEND_DEFAULT_MAX_FPS = 30;
static constexpr uint32 VISION_SEND_MAX_FPS_UPPER = 240;

// ============================== 全局变量区 ==============================
static std::vector<uint8> g_image_rgb565_frame(VISION_DOWNSAMPLED_WIDTH * VISION_DOWNSAMPLED_HEIGHT * 2);
// 保持与旧工程一致：默认图传和屏显都使用二值图。
static std::atomic<int> g_send_mode(VISION_SEND_BINARY);
static std::atomic<bool> g_send_enabled(true);
static std::atomic<int> g_last_send_mode(-1);
static std::atomic<uint32> g_last_send_time_us(0);
static std::atomic<uint64> g_last_send_tick_us(0);
static std::atomic<uint32> g_send_max_fps(VISION_SEND_DEFAULT_MAX_FPS);

static vision_send_mode_enum vision_sender_sanitize_mode(vision_send_mode_enum mode)
{
    // 仅保留两种显示模式：
    // 1) 彩色纯图不画线（RGB565）
    // 2) 灰度图画边线/中线（GRAY）
    if (mode == VISION_SEND_RGB565 || mode == VISION_SEND_IPM_RGB565)
    {
        return VISION_SEND_RGB565;
    }
    return VISION_SEND_GRAY;
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
    return mode == VISION_SEND_GRAY;
}

static void config_camera_send_packet(vision_send_mode_enum mode)
{
    uint16 *x1 = nullptr;
    uint16 *x2 = nullptr;
    uint16 *x3 = nullptr;
    uint16 *y1 = nullptr;
    uint16 *y2 = nullptr;
    uint16 *y3 = nullptr;
    uint16 dot_num = 0;
    vision_image_processor_get_boundaries(&x1, &x2, &x3, &y1, &y2, &y3, &dot_num);

    switch (mode)
    {
        case VISION_SEND_RGB565:
        case VISION_SEND_IPM_RGB565:
            seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_RGB565,
                                                         g_image_rgb565_frame.data(),
                                                         VISION_DOWNSAMPLED_WIDTH,
                                                         VISION_DOWNSAMPLED_HEIGHT);
            break;
        case VISION_SEND_BINARY:
        case VISION_SEND_IPM_EDGE_GRAY:
        case VISION_SEND_RGB565_OVERLAY:
        case VISION_SEND_GRAY:
            seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_GRAY,
                                                         const_cast<uint8 *>(vision_image_processor_gray_downsampled_image()),
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

    uint16 *x1 = nullptr;
    uint16 *x2 = nullptr;
    uint16 *x3 = nullptr;
    uint16 *y1 = nullptr;
    uint16 *y2 = nullptr;
    uint16 *y3 = nullptr;
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

    vision_send_mode_enum mode = vision_sender_sanitize_mode(
        static_cast<vision_send_mode_enum>(g_send_mode.load()));
    if (g_last_send_mode.load() != static_cast<int>(mode))
    {
        config_camera_send_packet(mode);
    }
    refresh_camera_boundary_packet(mode);

    if (mode == VISION_SEND_RGB565 || mode == VISION_SEND_IPM_RGB565)
    {
        update_rgb565_from_bgr_source(vision_image_processor_bgr_downsampled_image());
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
    const vision_send_mode_enum mode_sanitized = vision_sender_sanitize_mode(mode);
    int m = static_cast<int>(mode_sanitized);
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

#include "driver/vision/vision_transport.h"

#include "driver/vision/vision_image_processor.h"

#include "zf_driver_tcp_client.h"
#include "zf_driver_udp.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include <arpa/inet.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

namespace
{
// ---------- client sender ----------
static constexpr uint32 kClientDefaultMaxFps = 30;
static constexpr uint32 kClientMaxFpsUpper = 240;
// RGB565 临时缓存：仅在 VISION_SEND_RGB565 模式使用。
static std::vector<uint8> g_image_rgb565_frame(VISION_DOWNSAMPLED_WIDTH * VISION_DOWNSAMPLED_HEIGHT * 2);
// 客户端发送模式与开关。
static std::atomic<int> g_send_mode(VISION_SEND_BINARY);
static std::atomic<bool> g_send_enabled(true);
// 最近一次已配置模式（避免重复配置底层发送结构）。
static std::atomic<int> g_last_send_mode(-1);
// 最近一次客户端发送耗时（us）。
static std::atomic<uint32> g_last_send_time_us(0);
// 最近一次客户端发送时间戳（用于限频）。
static std::atomic<uint64> g_last_send_tick_us(0);
// 客户端发送最大 FPS（0 表示不限）。
static std::atomic<uint32> g_send_max_fps(kClientDefaultMaxFps);

// ---------- udp/tcp sender ----------
constexpr uint32 kUdpDefaultMaxFps = 30;
constexpr uint32 kUdpMaxFpsUpper = 120;
constexpr uint32 kMaxUdpPayload = 1200;
constexpr uint32 kUdpHeaderSize = 20;
constexpr uint32 kMagic = 0x56535544;
constexpr uint32 kTcpStatusMinIntervalMs = 100;
constexpr int kIpmCanvasWidth = VISION_IPM_WIDTH;
constexpr int kIpmCanvasHeight = VISION_IPM_HEIGHT;

#pragma pack(push, 1)
struct udp_chunk_header_t
{
    uint32 magic;
    uint32 frame_id;
    uint16 chunk_idx;
    uint16 chunk_total;
    uint16 payload_len;
    uint16 width;
    uint16 height;
    uint8 mode;
    uint8 reserved;
};
#pragma pack(pop)

std::atomic<bool> g_udp_enabled(false);            // UDP 图像发送开关。
std::atomic<bool> g_tcp_enabled(true);             // TCP 状态发送开关。
std::atomic<uint32> g_udp_max_fps(kUdpDefaultMaxFps); // UDP 图像限频。
std::atomic<uint64> g_udp_last_send_tick_us(0);    // UDP 限频时间戳。
std::atomic<uint32> g_udp_frame_id(0);             // UDP 分片帧号。
std::atomic<int64_t> g_last_status_tick_ms(0);     // TCP 状态上报节流时间戳。

std::mutex g_init_mutex;      // UDP/TCP 初始化锁。
bool g_udp_ready = false;     // UDP 初始化是否成功。
bool g_tcp_ready = false;     // TCP 初始化是否成功。
char g_server_ip[64] = {0};   // 接收端 IP。
uint16 g_video_port = 0;      // UDP 视频端口。
uint16 g_meta_port = 0;       // TCP 状态端口。

static vision_send_mode_enum vision_sender_sanitize_mode(vision_send_mode_enum mode)
{
    if (mode == VISION_SEND_BINARY)
    {
        return VISION_SEND_BINARY;
    }
    if (mode == VISION_SEND_RGB565)
    {
        return VISION_SEND_RGB565;
    }
    return VISION_SEND_GRAY;
}

// 作用：BGR 转 RGB565（大端）用于助手发送。
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

// 作用：更新 RGB565 缓冲。
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

// 作用：首次配置助手图像发送与边线发送格式。
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
            seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_RGB565,
                                                         g_image_rgb565_frame.data(),
                                                         VISION_DOWNSAMPLED_WIDTH,
                                                         VISION_DOWNSAMPLED_HEIGHT);
            break;
        case VISION_SEND_BINARY:
            seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_GRAY,
                                                         const_cast<uint8 *>(vision_image_processor_binary_downsampled_u8_image()),
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

// 作用：刷新边线包（灰度模式下每帧刷新）。
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

static uint64 now_us()
{
    return static_cast<uint64>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

static int64_t now_ms()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// 作用：UDP 限频抢占。
static bool try_acquire_udp_send_slot()
{
    const uint32 max_fps = g_udp_max_fps.load();
    if (max_fps == 0)
    {
        return true;
    }

    const uint64 now = now_us();
    const uint64 min_interval_us = 1000000ULL / static_cast<uint64>(max_fps);
    const uint64 last = g_udp_last_send_tick_us.load();
    if (last != 0 && (now - last) < min_interval_us)
    {
        return false;
    }
    g_udp_last_send_tick_us.store(now);
    return true;
}

// 作用：将灰度/二值图编码为 JPEG（供 UDP 发送）。
static bool encode_jpeg_gray_like(const uint8 *gray_u8, int width, int height, std::vector<uint8> *jpeg_out)
{
    if (gray_u8 == nullptr || jpeg_out == nullptr || width <= 0 || height <= 0)
    {
        return false;
    }
    cv::Mat img(height, width, CV_8UC1, const_cast<uint8 *>(gray_u8));
    std::vector<int> enc_params = {cv::IMWRITE_JPEG_QUALITY, 80};
    return cv::imencode(".jpg", img, *jpeg_out, enc_params);
}

static bool build_gray_jpeg(std::vector<uint8> *jpeg_out, int *width, int *height, uint8 *mode_out)
{
    if (jpeg_out == nullptr || width == nullptr || height == nullptr || mode_out == nullptr)
    {
        return false;
    }
    const int w = VISION_DOWNSAMPLED_WIDTH;
    const int h = VISION_DOWNSAMPLED_HEIGHT;
    const uint8 *gray = vision_image_processor_gray_downsampled_image();
    if (!encode_jpeg_gray_like(gray, w, h, jpeg_out))
    {
        return false;
    }
    *width = w;
    *height = h;
    *mode_out = 1;
    return true;
}

static bool build_binary_jpeg(std::vector<uint8> *jpeg_out, int *width, int *height, uint8 *mode_out)
{
    if (jpeg_out == nullptr || width == nullptr || height == nullptr || mode_out == nullptr)
    {
        return false;
    }
    const int w = VISION_DOWNSAMPLED_WIDTH;
    const int h = VISION_DOWNSAMPLED_HEIGHT;
    const uint8 *binary = vision_image_processor_binary_downsampled_u8_image();
    if (!encode_jpeg_gray_like(binary, w, h, jpeg_out))
    {
        return false;
    }
    *width = w;
    *height = h;
    *mode_out = 0;
    return true;
}

// 作用：发送 UDP 分片帧。
static void send_udp_frame(const std::vector<uint8> &jpeg, int width, int height, uint8 mode)
{
    if (!g_udp_ready || jpeg.empty())
    {
        return;
    }

    const uint32 frame_id = g_udp_frame_id.fetch_add(1);
    const uint32 max_chunk_data = kMaxUdpPayload - kUdpHeaderSize;
    const uint32 chunk_total = (static_cast<uint32>(jpeg.size()) + max_chunk_data - 1U) / max_chunk_data;
    std::vector<uint8> packet;
    packet.resize(kMaxUdpPayload);

    for (uint32 i = 0; i < chunk_total; ++i)
    {
        const uint32 offset = i * max_chunk_data;
        const uint32 remain = static_cast<uint32>(jpeg.size()) - offset;
        const uint32 payload_len = (remain > max_chunk_data) ? max_chunk_data : remain;

        udp_chunk_header_t hdr{};
        hdr.magic = htonl(kMagic);
        hdr.frame_id = htonl(frame_id);
        hdr.chunk_idx = htons(static_cast<uint16>(i));
        hdr.chunk_total = htons(static_cast<uint16>(chunk_total));
        hdr.payload_len = htons(static_cast<uint16>(payload_len));
        hdr.width = htons(static_cast<uint16>(width));
        hdr.height = htons(static_cast<uint16>(height));
        hdr.mode = mode;

        std::memcpy(packet.data(), &hdr, sizeof(hdr));
        std::memcpy(packet.data() + sizeof(hdr), jpeg.data() + offset, payload_len);
        udp_send_data(packet.data(), payload_len + sizeof(hdr));
    }
}

// 作用：发送 TCP JSON 状态（性能+边线+红框+ROI）。
static void send_tcp_status()
{
    if (!g_tcp_enabled.load() || !g_tcp_ready)
    {
        return;
    }

    const int64_t now = now_ms();
    const int64_t last = g_last_status_tick_ms.load();
    if (last != 0 && (now - last) < static_cast<int64_t>(kTcpStatusMinIntervalMs))
    {
        return;
    }
    g_last_status_tick_ms.store(now);

    uint32 capture_wait_us = 0;
    uint32 preprocess_us = 0;
    uint32 otsu_us = 0;
    uint32 maze_us = 0;
    uint32 total_us = 0;
    vision_image_processor_get_last_perf_us(&capture_wait_us, &preprocess_us, &otsu_us, &maze_us, &total_us);

    bool red_found = false;
    int red_x = 0;
    int red_y = 0;
    int red_w = 0;
    int red_h = 0;
    int red_cx = 0;
    int red_cy = 0;
    vision_image_processor_get_red_rect(&red_found, &red_x, &red_y, &red_w, &red_h, &red_cx, &red_cy);

    bool roi_valid = false;
    int roi_x = 0;
    int roi_y = 0;
    int roi_w = 0;
    int roi_h = 0;
    vision_image_processor_get_ncnn_roi(&roi_valid, &roi_x, &roi_y, &roi_w, &roi_h);

    uint16 *x1 = nullptr;
    uint16 *x3 = nullptr;
    uint16 *y1 = nullptr;
    uint16 *y3 = nullptr;
    uint16 dot_num = 0;
    vision_image_processor_get_boundaries(&x1, nullptr, &x3, &y1, nullptr, &y3, &dot_num);

    uint16 *ipm_x1 = nullptr;
    uint16 *ipm_x3 = nullptr;
    uint16 *ipm_y1 = nullptr;
    uint16 *ipm_y3 = nullptr;
    uint16 ipm_dot_num = 0;
    vision_image_processor_get_ipm_boundaries(&ipm_x1, nullptr, &ipm_x3, &ipm_y1, nullptr, &ipm_y3, &ipm_dot_num);
    uint16 *ipm_raw_x1 = nullptr;
    uint16 *ipm_raw_x3 = nullptr;
    uint16 *ipm_raw_y1 = nullptr;
    uint16 *ipm_raw_y3 = nullptr;
    uint16 ipm_raw_dot_num = 0;
    vision_image_processor_get_ipm_boundaries_raw(&ipm_raw_x1, nullptr, &ipm_raw_x3, &ipm_raw_y1, nullptr, &ipm_raw_y3, &ipm_raw_dot_num);
    uint16 *ipm_center_left_x = nullptr;
    uint16 *ipm_center_left_y = nullptr;
    uint16 ipm_center_left_num = 0;
    vision_image_processor_get_ipm_shifted_centerline_from_left(&ipm_center_left_x, &ipm_center_left_y, &ipm_center_left_num);
    uint16 *ipm_center_right_x = nullptr;
    uint16 *ipm_center_right_y = nullptr;
    uint16 ipm_center_right_num = 0;
    vision_image_processor_get_ipm_shifted_centerline_from_right(&ipm_center_right_x, &ipm_center_right_y, &ipm_center_right_num);
    bool ipm_track_valid = false;
    int ipm_track_x = 0;
    int ipm_track_y = 0;
    vision_image_processor_get_ipm_line_error_track_point(&ipm_track_valid, &ipm_track_x, &ipm_track_y);

    std::string line;
    line.reserve(8192);
    line += "{\"ts_ms\":";
    line += std::to_string(static_cast<long long>(now));
    line += ",\"line_error\":";
    line += std::to_string(line_error);
    line += ",\"capture_wait_us\":";
    line += std::to_string(capture_wait_us);
    line += ",\"preprocess_us\":";
    line += std::to_string(preprocess_us);
    line += ",\"otsu_us\":";
    line += std::to_string(otsu_us);
    line += ",\"maze_us\":";
    line += std::to_string(maze_us);
    line += ",\"total_us\":";
    line += std::to_string(total_us);
    line += ",\"red_found\":";
    line += red_found ? "1" : "0";
    line += ",\"red\":[";
    line += std::to_string(red_x) + "," + std::to_string(red_y) + "," + std::to_string(red_w) + "," +
            std::to_string(red_h) + "," + std::to_string(red_cx) + "," + std::to_string(red_cy) + "]";
    line += ",\"roi_valid\":";
    line += roi_valid ? "1" : "0";
    line += ",\"roi\":[";
    line += std::to_string(roi_x) + "," + std::to_string(roi_y) + "," + std::to_string(roi_w) + "," +
            std::to_string(roi_h) + "]";
    line += ",\"ipm_track_valid\":";
    line += ipm_track_valid ? "1" : "0";
    line += ",\"ipm_track_point\":[";
    line += std::to_string(ipm_track_x) + "," + std::to_string(ipm_track_y) + "]";

    auto append_points = [&line](const char *name, uint16 *xs, uint16 *ys, uint16 n, int max_w, int max_h) {
        line += ",\"";
        line += name;
        line += "\":[";
        for (uint16 i = 0; i < n; ++i)
        {
            if (i > 0)
            {
                line += ",";
            }
            int px = static_cast<int>(xs ? xs[i] : 0);
            int py = static_cast<int>(ys ? ys[i] : 0);
            px = std::clamp(px, 0, std::max(0, max_w - 1));
            py = std::clamp(py, 0, std::max(0, max_h - 1));
            line += "[";
            line += std::to_string(px);
            line += ",";
            line += std::to_string(py);
            line += "]";
        }
        line += "]";
    };

    append_points("left_boundary", x1, y1, dot_num, VISION_DOWNSAMPLED_WIDTH, VISION_DOWNSAMPLED_HEIGHT);
    append_points("right_boundary", x3, y3, dot_num, VISION_DOWNSAMPLED_WIDTH, VISION_DOWNSAMPLED_HEIGHT);
    append_points("ipm_left_boundary", ipm_x1, ipm_y1, ipm_dot_num, kIpmCanvasWidth, kIpmCanvasHeight);
    append_points("ipm_right_boundary", ipm_x3, ipm_y3, ipm_dot_num, kIpmCanvasWidth, kIpmCanvasHeight);
    append_points("ipm_raw_left_boundary", ipm_raw_x1, ipm_raw_y1, ipm_raw_dot_num, kIpmCanvasWidth, kIpmCanvasHeight);
    append_points("ipm_raw_right_boundary", ipm_raw_x3, ipm_raw_y3, ipm_raw_dot_num, kIpmCanvasWidth, kIpmCanvasHeight);
    append_points("ipm_centerline_from_left_shift", ipm_center_left_x, ipm_center_left_y, ipm_center_left_num, kIpmCanvasWidth, kIpmCanvasHeight);
    append_points("ipm_centerline_from_right_shift", ipm_center_right_x, ipm_center_right_y, ipm_center_right_num, kIpmCanvasWidth, kIpmCanvasHeight);

    line += ",\"gray_size\":[";
    line += std::to_string(VISION_DOWNSAMPLED_WIDTH);
    line += ",";
    line += std::to_string(VISION_DOWNSAMPLED_HEIGHT);
    line += "],\"ipm_size\":[";
    line += std::to_string(kIpmCanvasWidth);
    line += ",";
    line += std::to_string(kIpmCanvasHeight);
    line += "]}";
    line += "\n";

    tcp_client_send_data(reinterpret_cast<const uint8 *>(line.data()), static_cast<uint32>(line.size()));
}

} // namespace

void vision_transport_init()
{
    // 首帧前不做底层图像包配置，避免空指针或黑屏。
    g_last_send_mode.store(-1);
}

void vision_transport_send_step()
{
    // 发送统一入口：先客户端发送，再 UDP/TCP 发送。
    auto t0 = std::chrono::steady_clock::now();
    bool allow_send = true;
    const uint32 max_fps = g_send_max_fps.load();
    if (max_fps > 0)
    {
        const uint64 now_us = static_cast<uint64>(
            std::chrono::duration_cast<std::chrono::microseconds>(t0.time_since_epoch()).count());
        const uint64 min_interval_us = 1000000ULL / static_cast<uint64>(max_fps);
        const uint64 last_us = g_last_send_tick_us.load();
        if (last_us != 0 && (now_us - last_us) < min_interval_us)
        {
            allow_send = false;
        }
    }

    if (!allow_send || !g_send_enabled.load())
    {
        g_last_send_time_us.store(0);
    }
    else
    {
        vision_send_mode_enum mode = vision_sender_sanitize_mode(
            static_cast<vision_send_mode_enum>(g_send_mode.load()));
        if (g_last_send_mode.load() != static_cast<int>(mode))
        {
            config_camera_send_packet(mode);
        }
        refresh_camera_boundary_packet(mode);
        if (mode == VISION_SEND_RGB565)
        {
            update_rgb565_from_bgr_source(vision_image_processor_bgr_downsampled_image());
        }
        seekfree_assistant_camera_send();
        auto t1 = std::chrono::steady_clock::now();
        g_last_send_tick_us.store(static_cast<uint64>(
            std::chrono::duration_cast<std::chrono::microseconds>(t1.time_since_epoch()).count()));
        g_last_send_time_us.store(static_cast<uint32>(
            std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count()));
    }

    if (g_udp_enabled.load() && try_acquire_udp_send_slot())
    {
        std::vector<uint8> gray_jpeg;
        std::vector<uint8> binary_jpeg;
        int width = 0;
        int height = 0;
        uint8 mode = 0;
        if (build_gray_jpeg(&gray_jpeg, &width, &height, &mode))
        {
            send_udp_frame(gray_jpeg, width, height, mode);
        }
        if (build_binary_jpeg(&binary_jpeg, &width, &height, &mode))
        {
            send_udp_frame(binary_jpeg, width, height, mode);
        }
        send_tcp_status();
    }
}

uint32 vision_transport_get_last_send_time_us()
{
    return g_last_send_time_us.load();
}

void vision_transport_set_send_mode(vision_send_mode_enum mode)
{
    g_send_mode.store(static_cast<int>(vision_sender_sanitize_mode(mode)));
}

vision_send_mode_enum vision_transport_get_send_mode()
{
    return static_cast<vision_send_mode_enum>(g_send_mode.load());
}

void vision_transport_set_send_max_fps(uint32 max_fps)
{
    uint32 v = max_fps;
    if (v > kClientMaxFpsUpper)
    {
        v = kClientMaxFpsUpper;
    }
    g_send_max_fps.store(v);
    g_last_send_tick_us.store(0);
}

uint32 vision_transport_get_send_max_fps()
{
    return g_send_max_fps.load();
}

void vision_transport_set_send_enabled(bool enabled)
{
    g_send_enabled.store(enabled);
}

bool vision_transport_is_send_enabled()
{
    return g_send_enabled.load();
}

bool vision_transport_udp_init(const char *server_ip, uint16 video_port, uint16 meta_port)
{
    // 参数意义：
    // - server_ip: PC 接收端 IP；
    // - video_port: UDP 视频端口；
    // - meta_port: TCP 状态端口（可为 0）。
    std::lock_guard<std::mutex> lk(g_init_mutex);
    if (server_ip == nullptr || server_ip[0] == '\0' || video_port == 0)
    {
        return false;
    }
    std::snprintf(g_server_ip, sizeof(g_server_ip), "%s", server_ip);
    g_video_port = video_port;
    g_meta_port = meta_port;

    g_udp_ready = (udp_init(g_server_ip, g_video_port) == 0);
    g_tcp_ready = false;
    if (g_meta_port > 0)
    {
        g_tcp_ready = (tcp_client_init(g_server_ip, g_meta_port) == 0);
        if (!g_tcp_ready)
        {
            g_tcp_ready = true;
        }
    }

    g_udp_last_send_tick_us.store(0);
    g_last_status_tick_ms.store(0);
    return g_udp_ready;
}

void vision_transport_udp_cleanup()
{
}

void vision_transport_udp_set_enabled(bool enabled)
{
    g_udp_enabled.store(enabled);
}

bool vision_transport_udp_is_enabled()
{
    return g_udp_enabled.load();
}

void vision_transport_udp_set_max_fps(uint32 max_fps)
{
    uint32 v = max_fps;
    if (v > kUdpMaxFpsUpper)
    {
        v = kUdpMaxFpsUpper;
    }
    g_udp_max_fps.store(v);
    g_udp_last_send_tick_us.store(0);
}

uint32 vision_transport_udp_get_max_fps()
{
    return g_udp_max_fps.load();
}

void vision_transport_udp_set_tcp_enabled(bool enabled)
{
    g_tcp_enabled.store(enabled);
}

bool vision_transport_udp_tcp_enabled()
{
    return g_tcp_enabled.load();
}

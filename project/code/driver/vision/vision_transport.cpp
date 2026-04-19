#include "driver/vision/vision_transport.h"

#include "motor_thread.h"
#include "line_follow_thread.h"
#include "driver/vision/vision_config.h"
#include "driver/vision/vision_infer_async.h"
#include "driver/vision/vision_image_processor.h"
#include "driver/vision/vision_frame_capture.h"
#include "vision_thread.h"

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
#include <limits>
#include <mutex>
#include <string>
#include <vector>

namespace
{
// ---------- client sender ----------
static constexpr uint32 kClientDefaultMaxFps = 30;
static constexpr uint32 kClientMaxFpsUpper = 240;
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
constexpr int kGrayJpegQuality = 100;
constexpr int kRgbJpegQuality = 80;
constexpr int kIpmCanvasWidth = VISION_IPM_WIDTH;
constexpr int kIpmCanvasHeight = VISION_IPM_HEIGHT;
constexpr int kRoi64Size = 64;
constexpr int kRoiJpegQuality = 90;

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
std::atomic<uint32> g_udp_tx_fps(0);               // UDP 发送帧率（mode 0/1/2 合计，不含 ROI64）。

std::mutex g_init_mutex;      // UDP/TCP 初始化锁。
bool g_udp_ready = false;     // UDP 初始化是否成功。
bool g_tcp_ready = false;     // TCP 初始化是否成功。
char g_server_ip[64] = {0};   // 接收端 IP。
uint16 g_video_port = 0;      // UDP 视频端口。
uint16 g_meta_port = 0;       // TCP 状态端口。

struct system_usage_cache_t
{
    uint64 last_refresh_us = 0;
    uint64 prev_total_ticks = 0;
    uint64 prev_idle_ticks = 0;
    bool has_prev_cpu_sample = false;
    float cpu_usage_percent = 0.0f;
    float mem_usage_percent = 0.0f;
};

system_usage_cache_t g_system_usage_cache;

static uint64 now_us();

static bool read_cpu_usage_sample(uint64 *total_ticks, uint64 *idle_ticks)
{
    if (total_ticks == nullptr || idle_ticks == nullptr)
    {
        return false;
    }

    FILE *fp = std::fopen("/proc/stat", "r");
    if (fp == nullptr)
    {
        return false;
    }

    char line[256] = {0};
    const char *result = std::fgets(line, sizeof(line), fp);
    std::fclose(fp);
    if (result == nullptr)
    {
        return false;
    }

    unsigned long long user = 0;
    unsigned long long nice = 0;
    unsigned long long system = 0;
    unsigned long long idle = 0;
    unsigned long long iowait = 0;
    unsigned long long irq = 0;
    unsigned long long softirq = 0;
    unsigned long long steal = 0;
    const int parsed = std::sscanf(line,
                                   "cpu %llu %llu %llu %llu %llu %llu %llu %llu",
                                   &user,
                                   &nice,
                                   &system,
                                   &idle,
                                   &iowait,
                                   &irq,
                                   &softirq,
                                   &steal);
    if (parsed < 4)
    {
        return false;
    }

    *idle_ticks = static_cast<uint64>(idle + iowait);
    *total_ticks = static_cast<uint64>(user + nice + system + idle + iowait + irq + softirq + steal);
    return true;
}

static bool read_mem_usage_percent(float *mem_usage_percent)
{
    if (mem_usage_percent == nullptr)
    {
        return false;
    }

    FILE *fp = std::fopen("/proc/meminfo", "r");
    if (fp == nullptr)
    {
        return false;
    }

    char key[64] = {0};
    unsigned long long value_kb = 0;
    char unit[32] = {0};
    uint64 mem_total_kb = 0;
    uint64 mem_available_kb = 0;
    uint64 mem_free_kb = 0;
    uint64 buffers_kb = 0;
    uint64 cached_kb = 0;

    while (std::fscanf(fp, "%63s %llu %31s", key, &value_kb, unit) == 3)
    {
        if (std::strcmp(key, "MemTotal:") == 0)
        {
            mem_total_kb = static_cast<uint64>(value_kb);
        }
        else if (std::strcmp(key, "MemAvailable:") == 0)
        {
            mem_available_kb = static_cast<uint64>(value_kb);
        }
        else if (std::strcmp(key, "MemFree:") == 0)
        {
            mem_free_kb = static_cast<uint64>(value_kb);
        }
        else if (std::strcmp(key, "Buffers:") == 0)
        {
            buffers_kb = static_cast<uint64>(value_kb);
        }
        else if (std::strcmp(key, "Cached:") == 0)
        {
            cached_kb = static_cast<uint64>(value_kb);
        }
    }
    std::fclose(fp);

    if (mem_total_kb == 0)
    {
        return false;
    }

    if (mem_available_kb == 0)
    {
        mem_available_kb = std::min(mem_total_kb, mem_free_kb + buffers_kb + cached_kb);
    }

    const double used_ratio = 1.0 - (static_cast<double>(mem_available_kb) / static_cast<double>(mem_total_kb));
    *mem_usage_percent = static_cast<float>(std::clamp(used_ratio * 100.0, 0.0, 100.0));
    return true;
}

static void refresh_system_usage_cache_if_needed()
{
    constexpr uint64 kRefreshIntervalUs = 1000000ULL;
    const uint64 now = now_us();
    if (g_system_usage_cache.last_refresh_us != 0 &&
        (now - g_system_usage_cache.last_refresh_us) < kRefreshIntervalUs)
    {
        return;
    }

    uint64 total_ticks = 0;
    uint64 idle_ticks = 0;
    if (read_cpu_usage_sample(&total_ticks, &idle_ticks))
    {
        if (g_system_usage_cache.has_prev_cpu_sample &&
            total_ticks >= g_system_usage_cache.prev_total_ticks &&
            idle_ticks >= g_system_usage_cache.prev_idle_ticks)
        {
            const uint64 total_delta = total_ticks - g_system_usage_cache.prev_total_ticks;
            const uint64 idle_delta = idle_ticks - g_system_usage_cache.prev_idle_ticks;
            if (total_delta > 0)
            {
                const double busy_ratio = 1.0 - (static_cast<double>(idle_delta) / static_cast<double>(total_delta));
                g_system_usage_cache.cpu_usage_percent =
                    static_cast<float>(std::clamp(busy_ratio * 100.0, 0.0, 100.0));
            }
        }

        g_system_usage_cache.prev_total_ticks = total_ticks;
        g_system_usage_cache.prev_idle_ticks = idle_ticks;
        g_system_usage_cache.has_prev_cpu_sample = true;
    }

    float mem_usage_percent = 0.0f;
    if (read_mem_usage_percent(&mem_usage_percent))
    {
        g_system_usage_cache.mem_usage_percent = mem_usage_percent;
    }

    g_system_usage_cache.last_refresh_us = now;
}

static vision_web_image_format_enum sanitize_web_image_format(int format)
{
    switch (format)
    {
        case VISION_WEB_IMAGE_FORMAT_PNG:
            return VISION_WEB_IMAGE_FORMAT_PNG;
        case VISION_WEB_IMAGE_FORMAT_BMP:
            return VISION_WEB_IMAGE_FORMAT_BMP;
        default:
            return VISION_WEB_IMAGE_FORMAT_JPEG;
    }
}

static const char *opencv_ext_for_web_image_format(vision_web_image_format_enum format)
{
    switch (format)
    {
        case VISION_WEB_IMAGE_FORMAT_PNG:
            return ".png";
        case VISION_WEB_IMAGE_FORMAT_BMP:
            return ".bmp";
        default:
            return ".jpg";
    }
}

static vision_send_mode_enum vision_sender_sanitize_mode(vision_send_mode_enum mode)
{
    if (mode == VISION_SEND_BINARY)
    {
        return VISION_SEND_BINARY;
    }
    return VISION_SEND_GRAY;
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

// 作用：将灰度/二值图编码为指定格式（供 UDP 发送）。
static bool encode_gray_like_for_web(const uint8 *gray_u8,
                                     int width,
                                     int height,
                                     vision_web_image_format_enum format,
                                     int jpeg_quality,
                                     std::vector<uint8> *image_out)
{
    if (gray_u8 == nullptr || image_out == nullptr || width <= 0 || height <= 0)
    {
        return false;
    }
    cv::Mat img(height, width, CV_8UC1, const_cast<uint8 *>(gray_u8));
    std::vector<int> enc_params;
    if (format == VISION_WEB_IMAGE_FORMAT_JPEG)
    {
        // 本地计算页会基于该灰度帧重新跑同源算法，因此默认 JPEG 质量尽量高。
        enc_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality};
    }
    return cv::imencode(opencv_ext_for_web_image_format(format), img, *image_out, enc_params);
}

static bool build_gray_image(std::vector<uint8> *image_out, int *width, int *height, uint8 *mode_out)
{
    if (image_out == nullptr || width == nullptr || height == nullptr || mode_out == nullptr)
    {
        return false;
    }
    const int w = VISION_DOWNSAMPLED_WIDTH;
    const int h = VISION_DOWNSAMPLED_HEIGHT;
    const uint8 *gray = vision_image_processor_gray_downsampled_image();
    const vision_web_image_format_enum format =
        sanitize_web_image_format(g_vision_runtime_config.udp_web_gray_image_format);
    if (!encode_gray_like_for_web(gray, w, h, format, kGrayJpegQuality, image_out))
    {
        return false;
    }
    *width = w;
    *height = h;
    *mode_out = 1;
    return true;
}

static bool build_binary_image(std::vector<uint8> *image_out, int *width, int *height, uint8 *mode_out)
{
    if (image_out == nullptr || width == nullptr || height == nullptr || mode_out == nullptr)
    {
        return false;
    }
    const int w = VISION_DOWNSAMPLED_WIDTH;
    const int h = VISION_DOWNSAMPLED_HEIGHT;
    const uint8 *binary = vision_image_processor_binary_downsampled_u8_image();
    const vision_web_image_format_enum format =
        sanitize_web_image_format(g_vision_runtime_config.udp_web_binary_image_format);
    if (!encode_gray_like_for_web(binary, w, h, format, kGrayJpegQuality, image_out))
    {
        return false;
    }
    *width = w;
    *height = h;
    *mode_out = 0;
    return true;
}

static bool build_rgb_image(std::vector<uint8> *image_out, int *width, int *height, uint8 *mode_out)
{
    if (image_out == nullptr || width == nullptr || height == nullptr || mode_out == nullptr)
    {
        return false;
    }
    const int w = VISION_DOWNSAMPLED_WIDTH;
    const int h = VISION_DOWNSAMPLED_HEIGHT;
    const uint8 *bgr = vision_image_processor_bgr_downsampled_image();
    if (bgr == nullptr)
    {
        return false;
    }
    cv::Mat img(h, w, CV_8UC3, const_cast<uint8 *>(bgr));
    const vision_web_image_format_enum format =
        sanitize_web_image_format(g_vision_runtime_config.udp_web_rgb_image_format);
    std::vector<int> enc_params;
    if (format == VISION_WEB_IMAGE_FORMAT_JPEG)
    {
        enc_params = {cv::IMWRITE_JPEG_QUALITY, kRgbJpegQuality};
    }
    if (!cv::imencode(opencv_ext_for_web_image_format(format), img, *image_out, enc_params))
    {
        return false;
    }
    *width = w;
    *height = h;
    *mode_out = 2;
    return true;
}

static bool build_roi64_image(std::vector<uint8> *image_out, int *width, int *height, uint8 *mode_out)
{
    if (image_out == nullptr || width == nullptr || height == nullptr || mode_out == nullptr)
    {
        return false;
    }

    bool roi_valid = false;
    int roi_x = 0;
    int roi_y = 0;
    int roi_w = 0;
    int roi_h = 0;
    vision_image_processor_get_ncnn_roi(&roi_valid, &roi_x, &roi_y, &roi_w, &roi_h);
    if (!roi_valid || roi_w <= 0 || roi_h <= 0)
    {
        return false;
    }

    const cv::Rect roi_rect(roi_x, roi_y, roi_w, roi_h);
    const vision_web_image_format_enum format =
        sanitize_web_image_format(g_vision_runtime_config.udp_web_rgb_image_format);
    std::vector<int> enc_params;
    if (format == VISION_WEB_IMAGE_FORMAT_JPEG)
    {
        enc_params = {cv::IMWRITE_JPEG_QUALITY, kRoiJpegQuality};
    }

    const uint8 *bgr = vision_image_processor_bgr_downsampled_image();
    if (bgr != nullptr)
    {
        cv::Mat full(VISION_DOWNSAMPLED_HEIGHT, VISION_DOWNSAMPLED_WIDTH, CV_8UC3, const_cast<uint8 *>(bgr));
        cv::Rect safe = roi_rect & cv::Rect(0, 0, full.cols, full.rows);
        if (safe.width <= 0 || safe.height <= 0)
        {
            return false;
        }
        cv::Mat roi = full(safe);
        cv::Mat roi64;
        cv::resize(roi, roi64, cv::Size(kRoi64Size, kRoi64Size), 0.0, 0.0, cv::INTER_LINEAR);
        if (!cv::imencode(opencv_ext_for_web_image_format(format), roi64, *image_out, enc_params))
        {
            return false;
        }
    }
    else
    {
        const uint8 *gray = vision_image_processor_gray_downsampled_image();
        if (gray == nullptr)
        {
            return false;
        }
        cv::Mat full(VISION_DOWNSAMPLED_HEIGHT, VISION_DOWNSAMPLED_WIDTH, CV_8UC1, const_cast<uint8 *>(gray));
        cv::Rect safe = roi_rect & cv::Rect(0, 0, full.cols, full.rows);
        if (safe.width <= 0 || safe.height <= 0)
        {
            return false;
        }
        cv::Mat roi = full(safe);
        cv::Mat roi64;
        cv::resize(roi, roi64, cv::Size(kRoi64Size, kRoi64Size), 0.0, 0.0, cv::INTER_LINEAR);
        if (!cv::imencode(opencv_ext_for_web_image_format(format), roi64, *image_out, enc_params))
        {
            return false;
        }
    }

    *width = kRoi64Size;
    *height = kRoi64Size;
    *mode_out = 3;
    return true;
}

// 作用：发送 UDP 分片帧。
static void send_udp_frame(const std::vector<uint8> &image_bytes,
                           int width,
                           int height,
                           uint8 mode,
                           vision_web_image_format_enum format)
{
    if (!g_udp_ready || image_bytes.empty())
    {
        return;
    }

    // UDP 图像发送 FPS（1 秒窗口）统计，不含 ROI64(mode=3)。
    static uint32 window_frames = 0;
    static uint64 window_start_us = 0;
    const uint64 now = now_us();
    if (window_start_us == 0)
    {
        window_start_us = now;
    }
    if (mode <= 2)
    {
        ++window_frames;
    }
    const uint64 elapsed_us = now - window_start_us;
    if (elapsed_us >= 1000000ULL)
    {
        const uint32 fps = static_cast<uint32>(
            (static_cast<uint64>(window_frames) * 1000000ULL + elapsed_us / 2ULL) / elapsed_us);
        g_udp_tx_fps.store(fps);
        window_frames = 0;
        window_start_us = now;
    }

    const uint32 frame_id = g_udp_frame_id.fetch_add(1);
    const uint32 max_chunk_data = kMaxUdpPayload - kUdpHeaderSize;
    const uint32 chunk_total = (static_cast<uint32>(image_bytes.size()) + max_chunk_data - 1U) / max_chunk_data;
    std::vector<uint8> packet;
    packet.resize(kMaxUdpPayload);

    for (uint32 i = 0; i < chunk_total; ++i)
    {
        const uint32 offset = i * max_chunk_data;
        const uint32 remain = static_cast<uint32>(image_bytes.size()) - offset;
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
        hdr.reserved = static_cast<uint8>(format);

        std::memcpy(packet.data(), &hdr, sizeof(hdr));
        std::memcpy(packet.data() + sizeof(hdr), image_bytes.data() + offset, payload_len);
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

    refresh_system_usage_cache_if_needed();

    uint32 capture_wait_us = 0;
    uint32 preprocess_us = 0;
    uint32 otsu_us = 0;
    uint32 maze_us = 0;
    uint32 total_us = 0;
    vision_image_processor_get_last_perf_us(&capture_wait_us, &preprocess_us, &otsu_us, &maze_us, &total_us);
    const uint8 otsu_threshold = vision_image_processor_get_last_otsu_threshold();
    const uint32 capture_thread_fps = vision_frame_capture_fps();
    const uint32 vision_process_fps = vision_thread_process_fps();
    const uint32 udp_tx_fps = g_udp_tx_fps.load();
    const int data_profile = g_vision_runtime_config.udp_web_data_profile;
    const bool send_full_debug = (data_profile == static_cast<int>(VISION_WEB_DATA_PROFILE_FULL));

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
    vision_infer_async_result_t infer_result{};
    const bool has_infer_result = vision_infer_async_fetch_latest(&infer_result);
    const bool infer_enabled = vision_infer_async_enabled();

    uint16 *x1 = nullptr;
    uint16 *x3 = nullptr;
    uint16 *y1 = nullptr;
    uint16 *y3 = nullptr;
    uint16 dot_num = 0;
    uint16 left_dot_num = 0;
    uint16 right_dot_num = 0;
    vision_image_processor_get_boundaries(&x1, nullptr, &x3, &y1, nullptr, &y3, &dot_num);
    vision_image_processor_get_boundary_side_counts(&left_dot_num, &right_dot_num);
    uint16 *eight_left_x = nullptr;
    uint16 *eight_left_y = nullptr;
    uint8 *eight_left_dir = nullptr;
    uint16 eight_left_num = 0;
    int eight_left_first_frame_touch_index = -1;
    uint16 *eight_right_x = nullptr;
    uint16 *eight_right_y = nullptr;
    uint8 *eight_right_dir = nullptr;
    uint16 eight_right_num = 0;
    int eight_right_first_frame_touch_index = -1;
    vision_image_processor_get_eight_neighbor_left_trace(&eight_left_x,
                                                         &eight_left_y,
                                                         &eight_left_dir,
                                                         &eight_left_num,
                                                         &eight_left_first_frame_touch_index);
    vision_image_processor_get_eight_neighbor_right_trace(&eight_right_x,
                                                          &eight_right_y,
                                                          &eight_right_dir,
                                                          &eight_right_num,
                                                          &eight_right_first_frame_touch_index);
    bool cross_lower_left_found = false;
    int cross_lower_left_index = -1;
    int cross_lower_left_x = 0;
    int cross_lower_left_y = 0;
    bool cross_lower_right_found = false;
    int cross_lower_right_index = -1;
    int cross_lower_right_x = 0;
    int cross_lower_right_y = 0;
    bool cross_lower_pair_valid = false;
    vision_image_processor_get_cross_lower_corner_state(&cross_lower_left_found,
                                                        &cross_lower_left_index,
                                                        &cross_lower_left_x,
                                                        &cross_lower_left_y,
                                                        &cross_lower_right_found,
                                                        &cross_lower_right_index,
                                                        &cross_lower_right_x,
                                                        &cross_lower_right_y,
                                                        &cross_lower_pair_valid);
    bool cross_left_aux_found = false;
    int cross_left_aux_transition_x = 0;
    int cross_left_aux_transition_y = 0;
    uint16 *cross_left_aux_trace_x = nullptr;
    uint16 *cross_left_aux_trace_y = nullptr;
    uint8 *cross_left_aux_trace_dir = nullptr;
    uint16 cross_left_aux_trace_num = 0;
    uint16 *cross_left_aux_regular_x = nullptr;
    uint16 *cross_left_aux_regular_y = nullptr;
    uint16 cross_left_aux_regular_num = 0;
    bool cross_right_aux_found = false;
    int cross_right_aux_transition_x = 0;
    int cross_right_aux_transition_y = 0;
    uint16 *cross_right_aux_trace_x = nullptr;
    uint16 *cross_right_aux_trace_y = nullptr;
    uint8 *cross_right_aux_trace_dir = nullptr;
    uint16 cross_right_aux_trace_num = 0;
    uint16 *cross_right_aux_regular_x = nullptr;
    uint16 *cross_right_aux_regular_y = nullptr;
    uint16 cross_right_aux_regular_num = 0;
    vision_image_processor_get_cross_aux_line_state(true,
                                                    &cross_left_aux_found,
                                                    &cross_left_aux_transition_x,
                                                    &cross_left_aux_transition_y,
                                                    &cross_left_aux_trace_x,
                                                    &cross_left_aux_trace_y,
                                                    &cross_left_aux_trace_dir,
                                                    &cross_left_aux_trace_num,
                                                    &cross_left_aux_regular_x,
                                                    &cross_left_aux_regular_y,
                                                    &cross_left_aux_regular_num);
    vision_image_processor_get_cross_aux_line_state(false,
                                                    &cross_right_aux_found,
                                                    &cross_right_aux_transition_x,
                                                    &cross_right_aux_transition_y,
                                                    &cross_right_aux_trace_x,
                                                    &cross_right_aux_trace_y,
                                                    &cross_right_aux_trace_dir,
                                                    &cross_right_aux_trace_num,
                                                    &cross_right_aux_regular_x,
                                                    &cross_right_aux_regular_y,
                                                    &cross_right_aux_regular_num);
    bool cross_left_upper_found = false;
    int cross_left_upper_index = -1;
    int cross_left_upper_x = 0;
    int cross_left_upper_y = 0;
    bool cross_right_upper_found = false;
    int cross_right_upper_index = -1;
    int cross_right_upper_x = 0;
    int cross_right_upper_y = 0;
    bool cross_stage2_frozen_left_found = false;
    int cross_stage2_frozen_left_x = 0;
    int cross_stage2_frozen_left_y = 0;
    bool cross_stage2_frozen_right_found = false;
    int cross_stage2_frozen_right_x = 0;
    int cross_stage2_frozen_right_y = 0;
    vision_image_processor_get_cross_upper_corner_state(&cross_left_upper_found,
                                                        &cross_left_upper_index,
                                                        &cross_left_upper_x,
                                                        &cross_left_upper_y,
                                                        &cross_right_upper_found,
                                                        &cross_right_upper_index,
                                                        &cross_right_upper_x,
                                                        &cross_right_upper_y);
    vision_image_processor_get_cross_stage2_frozen_lower_corner_state(&cross_stage2_frozen_left_found,
                                                                      &cross_stage2_frozen_left_x,
                                                                      &cross_stage2_frozen_left_y,
                                                                      &cross_stage2_frozen_right_found,
                                                                      &cross_stage2_frozen_right_x,
                                                                      &cross_stage2_frozen_right_y);
    auto count_dir_hits = [](const uint8 *dirs, uint16 count, uint8 target_dir) -> int {
        if (dirs == nullptr || count == 0)
        {
            return 0;
        }
        int hits = 0;
        const int safe_count = std::max(0, static_cast<int>(count));
        for (int i = 0; i < safe_count; ++i)
        {
            if (dirs[i] == target_dir)
            {
                hits += 1;
            }
        }
        return hits;
    };
    const int cross_left_aux_dir5_count = count_dir_hits(cross_left_aux_trace_dir, cross_left_aux_trace_num, 5);
    const int cross_right_aux_dir5_count = count_dir_hits(cross_right_aux_trace_dir, cross_right_aux_trace_num, 5);
    int cross_left_corner_post_frame_wall_rows = 0;
    int cross_right_corner_post_frame_wall_rows = 0;
    int cross_start_boundary_gap_x = 0;
    vision_image_processor_get_cross_route_debug_state(&cross_left_corner_post_frame_wall_rows,
                                                       &cross_right_corner_post_frame_wall_rows,
                                                       &cross_start_boundary_gap_x);
    bool src_left_trace_has_frame_wall = false;
    bool src_right_trace_has_frame_wall = false;
    bool src_left_boundary_straight = false;
    bool src_right_boundary_straight = false;
    int src_left_start_frame_wall_rows = 0;
    int src_right_start_frame_wall_rows = 0;
    uint16 *src_left_circle_guide_x = nullptr;
    uint16 *src_left_circle_guide_y = nullptr;
    uint16 src_left_circle_guide_num = 0;
    uint16 *src_right_circle_guide_x = nullptr;
    uint16 *src_right_circle_guide_y = nullptr;
    uint16 src_right_circle_guide_num = 0;
    vision_image_processor_get_src_trace_frame_wall_state(&src_left_trace_has_frame_wall,
                                                          &src_right_trace_has_frame_wall);
    vision_image_processor_get_src_boundary_straight_state(&src_left_boundary_straight,
                                                           &src_right_boundary_straight);
    vision_image_processor_get_src_start_frame_wall_rows(&src_left_start_frame_wall_rows,
                                                         &src_right_start_frame_wall_rows);
    vision_image_processor_get_src_circle_guide_lines(&src_left_circle_guide_x,
                                                      &src_left_circle_guide_y,
                                                      &src_left_circle_guide_num,
                                                      &src_right_circle_guide_x,
                                                      &src_right_circle_guide_y,
                                                      &src_right_circle_guide_num);
    uint16 *ipm_x1 = nullptr;
    uint16 *ipm_x3 = nullptr;
    uint16 *ipm_y1 = nullptr;
    uint16 *ipm_y3 = nullptr;
    uint16 ipm_dot_num = 0;
    uint16 ipm_left_dot_num = 0;
    uint16 ipm_right_dot_num = 0;
    vision_image_processor_get_ipm_boundaries(&ipm_x1, nullptr, &ipm_x3, &ipm_y1, nullptr, &ipm_y3, &ipm_dot_num);
    vision_image_processor_get_ipm_boundary_side_counts(&ipm_left_dot_num, &ipm_right_dot_num);
    uint16 *ipm_center_left_x = nullptr;
    uint16 *ipm_center_left_y = nullptr;
    uint16 ipm_center_left_num = 0;
    vision_image_processor_get_ipm_shifted_centerline_from_left(&ipm_center_left_x, &ipm_center_left_y, &ipm_center_left_num);
    uint16 *ipm_center_right_x = nullptr;
    uint16 *ipm_center_right_y = nullptr;
    uint16 ipm_center_right_num = 0;
    vision_image_processor_get_ipm_shifted_centerline_from_right(&ipm_center_right_x, &ipm_center_right_y, &ipm_center_right_num);
    uint16 *src_center_left_x = nullptr;
    uint16 *src_center_left_y = nullptr;
    uint16 src_center_left_num = 0;
    vision_image_processor_get_src_shifted_centerline_from_left(&src_center_left_x, &src_center_left_y, &src_center_left_num);
    uint16 *src_center_right_x = nullptr;
    uint16 *src_center_right_y = nullptr;
    uint16 src_center_right_num = 0;
    vision_image_processor_get_src_shifted_centerline_from_right(&src_center_right_x, &src_center_right_y, &src_center_right_num);
    bool ipm_track_valid = false;
    int ipm_track_index = -1;
    int ipm_track_x = 0;
    int ipm_track_y = 0;
    uint32 maze_setup_us = 0;
    uint32 maze_start_us = 0;
    uint32 maze_trace_left_us = 0;
    uint32 maze_trace_right_us = 0;
    uint32 maze_post_us = 0;
    uint16 maze_left_points_raw = 0;
    uint16 maze_right_points_raw = 0;
    bool maze_left_ok = false;
    bool maze_right_ok = false;
    ipm_track_index = vision_image_processor_ipm_line_error_track_index();
    vision_image_processor_get_ipm_line_error_track_point(&ipm_track_valid, &ipm_track_x, &ipm_track_y);
    vision_image_processor_get_last_maze_detail_us(&maze_setup_us,
                                                   &maze_start_us,
                                                   &maze_trace_left_us,
                                                   &maze_trace_right_us,
                                                   &maze_post_us,
                                                   &maze_left_points_raw,
                                                   &maze_right_points_raw,
                                                   &maze_left_ok,
                                                   &maze_right_ok);
    struct circle_entry_raw_gap_debug_t
    {
        bool ok = false;
        int min_observed_gap_px = -1;
        int checked_rows = 0;
        int failed_row_y = -1;
        bool missing_point = false;
    };
    auto find_trace_x_at_row = [](uint16 *xs, uint16 *ys, uint16 n, int target_y, int *x_out) -> bool {
        if (x_out == nullptr || xs == nullptr || ys == nullptr || n == 0)
        {
            return false;
        }
        const int safe_n = std::max(0, static_cast<int>(n));
        for (int i = 0; i < safe_n; ++i)
        {
            if (static_cast<int>(ys[i]) != target_y)
            {
                continue;
            }
            *x_out = static_cast<int>(xs[i]);
            return true;
        }
        return false;
    };
    auto calc_circle_entry_raw_gap_debug = [&](int corner_y) -> circle_entry_raw_gap_debug_t {
        circle_entry_raw_gap_debug_t debug{};
        const int row_offset = std::max(g_vision_runtime_config.route_circle_entry_corner_row_offset, 0);
        const int rows_to_check = std::max(g_vision_runtime_config.route_circle_entry_gap_check_rows, 1);
        const int min_gap_px = g_vision_runtime_config.route_circle_entry_min_raw_boundary_gap;
        const int start_y = corner_y - row_offset;

        for (int i = 0; i < rows_to_check; ++i)
        {
            const int target_y = start_y - i;
            if (target_y <= 0 || target_y >= (VISION_DOWNSAMPLED_HEIGHT - 1))
            {
                debug.failed_row_y = target_y;
                return debug;
            }

            int left_x = 0;
            int right_x = 0;
            const bool left_found = find_trace_x_at_row(eight_left_x, eight_left_y, eight_left_num, target_y, &left_x);
            const bool right_found = find_trace_x_at_row(eight_right_x, eight_right_y, eight_right_num, target_y, &right_x);
            if (!left_found || !right_found)
            {
                debug.failed_row_y = target_y;
                debug.missing_point = true;
                return debug;
            }

            const int gap_px = right_x - left_x;
            if (debug.min_observed_gap_px < 0 || gap_px < debug.min_observed_gap_px)
            {
                debug.min_observed_gap_px = gap_px;
            }
            debug.checked_rows += 1;
            if (gap_px <= min_gap_px)
            {
                debug.failed_row_y = target_y;
                return debug;
            }
        }

        debug.ok = true;
        return debug;
    };
    const circle_entry_raw_gap_debug_t left_circle_raw_gap_debug =
        calc_circle_entry_raw_gap_debug(cross_lower_left_y);
    const circle_entry_raw_gap_debug_t right_circle_raw_gap_debug =
        calc_circle_entry_raw_gap_debug(cross_lower_right_y);

    const int64_t ts_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::steady_clock::now().time_since_epoch())
                              .count();

    std::string line;
    line.reserve(16384);
    line += "{";
    bool first_field = true;

    auto append_key = [&line, &first_field](const char *name) {
        if (!first_field)
        {
            line += ",";
        }
        first_field = false;
        line += "\"";
        line += name;
        line += "\":";
    };

    auto append_int = [&append_key, &line](bool enabled, const char *name, long long value) {
        if (!enabled)
        {
            return;
        }
        append_key(name);
        line += std::to_string(value);
    };

    auto append_float = [&append_key, &line](bool enabled, const char *name, float value) {
        if (!enabled)
        {
            return;
        }
        append_key(name);
        line += std::to_string(value);
    };

    auto append_bool = [&append_key, &line](bool enabled, const char *name, bool value) {
        if (!enabled)
        {
            return;
        }
        append_key(name);
        line += value ? "1" : "0";
    };

    auto append_int_array = [&append_key, &line](bool enabled, const char *name, const std::initializer_list<long long> &values) {
        if (!enabled)
        {
            return;
        }
        append_key(name);
        line += "[";
        bool first = true;
        for (long long value : values)
        {
            if (!first)
            {
                line += ",";
            }
            first = false;
            line += std::to_string(value);
        }
        line += "]";
    };

    auto append_string = [&append_key, &line](bool enabled, const char *name, const std::string &value) {
        if (!enabled)
        {
            return;
        }
        append_key(name);
        line += "\"";
        for (char ch : value)
        {
            if (ch == '\\' || ch == '"')
            {
                line += "\\";
            }
            line += ch;
        }
        line += "\"";
    };

    auto append_prob_pairs = [&append_key, &line](bool enabled, const char *name, const vision_infer_async_result_t &result) {
        if (!enabled)
        {
            return;
        }
        append_key(name);
        line += "[";
        for (int i = 0; i < result.ncnn_class_count; ++i)
        {
            if (i > 0)
            {
                line += ",";
            }
            line += "[\"";
            const char *label = result.ncnn_labels[i];
            for (int j = 0; label[j] != '\0'; ++j)
            {
                if (label[j] == '\\' || label[j] == '"')
                {
                    line += "\\";
                }
                line += label[j];
            }
            line += "\",";
            line += std::to_string(result.ncnn_probs[i]);
            line += "]";
        }
        line += "]";
    };

    MotorPidDebugStatus motor_pid_debug{};
    LineFollowPidDebugStatus line_follow_pid_debug{};
    motor_thread_get_pid_debug_status(motor_pid_debug);
    line_follow_thread_get_pid_debug_status(line_follow_pid_debug);

    auto append_pid_debug = [&](bool enabled) {
        append_bool(enabled, "pid_common_vision_updated", line_follow_pid_debug.vision_updated);
        append_bool(enabled, "pid_common_imu_updated", line_follow_pid_debug.imu_updated);
        append_int(enabled, "pid_common_route_main_state", line_follow_pid_debug.route_main_state);
        append_int(enabled, "pid_common_route_sub_state", line_follow_pid_debug.route_sub_state);
        append_float(enabled, "pid_common_normal_speed_reference", line_follow_pid_debug.normal_speed_reference);
        append_float(enabled, "pid_common_profile_base_speed", line_follow_pid_debug.profile_base_speed);
        append_float(enabled, "pid_common_desired_base_speed", line_follow_pid_debug.desired_base_speed);
        append_float(enabled, "pid_common_applied_base_speed", line_follow_pid_debug.applied_base_speed);
        append_float(enabled, "pid_common_raw_error_px", line_follow_pid_debug.raw_error_px);
        append_float(enabled, "pid_common_filtered_error_px", line_follow_pid_debug.filtered_error_px);
        append_float(enabled, "pid_common_abs_filtered_error_px", line_follow_pid_debug.abs_filtered_error_px);
        append_float(enabled, "pid_common_control_error_px", line_follow_pid_debug.control_error_px);
        append_bool(enabled, "pid_common_track_point_valid", line_follow_pid_debug.track_point_valid);
        append_int_array(enabled && line_follow_pid_debug.track_point_valid,
                         "pid_common_track_point",
                         {line_follow_pid_debug.track_point_x, line_follow_pid_debug.track_point_y});
        append_float(enabled, "pid_common_current_track_point_angle_deg", line_follow_pid_debug.current_track_point_angle_deg);
        append_float(enabled, "pid_common_filtered_track_point_angle_deg", line_follow_pid_debug.filtered_track_point_angle_deg);
        append_float(enabled, "pid_common_measured_yaw_rate_dps", line_follow_pid_debug.measured_yaw_rate_dps);
        append_float(enabled, "pid_common_yaw_rate_ref_dps", line_follow_pid_debug.yaw_rate_ref_dps);
        append_float(enabled, "pid_common_yaw_rate_error_dps", line_follow_pid_debug.yaw_rate_error_dps);
        append_float(enabled, "pid_common_dynamic_position_kp", line_follow_pid_debug.dynamic_position_kp);
        append_float(enabled, "pid_common_dynamic_yaw_rate_kp", line_follow_pid_debug.dynamic_yaw_rate_kp);
        append_float(enabled, "pid_common_applied_yaw_rate_kp", line_follow_pid_debug.applied_yaw_rate_kp);
        append_float(enabled, "pid_common_position_pid_kp", line_follow_pid_debug.position_pid_kp);
        append_float(enabled, "pid_common_position_pid_ki", line_follow_pid_debug.position_pid_ki);
        append_float(enabled, "pid_common_position_pid_kd", line_follow_pid_debug.position_pid_kd);
        append_float(enabled, "pid_common_position_pid_target", line_follow_pid_debug.position_pid_target);
        append_float(enabled, "pid_common_position_pid_error", line_follow_pid_debug.position_pid_error);
        append_float(enabled, "pid_common_position_pid_integral", line_follow_pid_debug.position_pid_integral);
        append_float(enabled, "pid_common_position_pid_output", line_follow_pid_debug.position_pid_output);
        append_float(enabled, "pid_common_position_pid_max_integral", line_follow_pid_debug.position_pid_max_integral);
        append_float(enabled, "pid_common_position_pid_max_output", line_follow_pid_debug.position_pid_max_output);
        append_float(enabled, "pid_common_yaw_pid_kp", line_follow_pid_debug.yaw_pid_kp);
        append_float(enabled, "pid_common_yaw_pid_ki", line_follow_pid_debug.yaw_pid_ki);
        append_float(enabled, "pid_common_yaw_pid_kd", line_follow_pid_debug.yaw_pid_kd);
        append_float(enabled, "pid_common_yaw_pid_target", line_follow_pid_debug.yaw_pid_target);
        append_float(enabled, "pid_common_yaw_pid_error", line_follow_pid_debug.yaw_pid_error);
        append_float(enabled, "pid_common_yaw_pid_integral", line_follow_pid_debug.yaw_pid_integral);
        append_float(enabled, "pid_common_yaw_pid_output", line_follow_pid_debug.yaw_pid_output);
        append_float(enabled, "pid_common_yaw_pid_max_integral", line_follow_pid_debug.yaw_pid_max_integral);
        append_float(enabled, "pid_common_yaw_pid_max_output", line_follow_pid_debug.yaw_pid_max_output);
        append_float(enabled, "pid_common_route_yaw_rate_ref_gain", line_follow_pid_debug.route_yaw_rate_ref_gain);
        append_float(enabled, "pid_common_route_yaw_rate_ref_limit", line_follow_pid_debug.route_yaw_rate_ref_limit);
        append_float(enabled, "pid_common_route_steering_max_output", line_follow_pid_debug.route_steering_max_output);
        append_float(enabled, "pid_common_route_yaw_rate_kp_enable_error_threshold_px",
                     line_follow_pid_debug.route_yaw_rate_kp_enable_error_threshold_px);
        append_float(enabled, "pid_common_mean_abs_path_error", line_follow_pid_debug.mean_abs_path_error);
        append_float(enabled, "pid_common_speed_scheme_blended_abs_error_sum",
                     line_follow_pid_debug.speed_scheme_blended_abs_error_sum);
        append_float(enabled, "pid_common_speed_scheme_error_scale_raw",
                     line_follow_pid_debug.speed_scheme_error_scale_raw);
        append_float(enabled, "pid_common_speed_scheme_final_speed_scale",
                     line_follow_pid_debug.speed_scheme_final_speed_scale);
        append_float(enabled, "pid_common_speed_scheme_split_ratio", line_follow_pid_debug.speed_scheme_split_ratio);
        append_float(enabled, "pid_common_speed_scheme_front_weight", line_follow_pid_debug.speed_scheme_front_weight);
        append_float(enabled, "pid_common_speed_scheme_rear_weight", line_follow_pid_debug.speed_scheme_rear_weight);
        append_float(enabled, "pid_common_speed_scheme_slowdown_start",
                     line_follow_pid_debug.speed_scheme_slowdown_start);
        append_float(enabled, "pid_common_speed_scheme_slowdown_full",
                     line_follow_pid_debug.speed_scheme_slowdown_full);
        append_float(enabled, "pid_common_speed_scheme_min_speed_scale",
                     line_follow_pid_debug.speed_scheme_min_speed_scale);
        append_float(enabled, "pid_common_speed_scheme_max_speed_scale",
                     line_follow_pid_debug.speed_scheme_max_speed_scale);
        append_int(enabled, "pid_common_speed_scheme_point_count",
                   line_follow_pid_debug.speed_scheme_point_count);
        append_bool(enabled, "pid_common_speed_scheme_ready",
                    line_follow_pid_debug.speed_scheme_ready);
        append_bool(enabled, "pid_common_speed_scheme_triggered",
                    line_follow_pid_debug.speed_scheme_triggered);
        append_int(enabled, "pid_common_speed_scheme_winner_branch",
                   line_follow_pid_debug.speed_scheme_winner_branch);
        append_float(enabled, "pid_common_speed_scheme_max_drop_ratio_per_cycle",
                     line_follow_pid_debug.speed_scheme_max_drop_ratio_per_cycle);
        append_float(enabled, "pid_common_speed_scheme_max_rise_ratio_per_cycle",
                     line_follow_pid_debug.speed_scheme_max_rise_ratio_per_cycle);
        append_int(enabled, "pid_common_speed_scheme_centerline_count_full_n",
                   line_follow_pid_debug.speed_scheme_centerline_count_full_n);
        append_float(enabled, "pid_common_speed_scheme_centerline_ratio_floor",
                     line_follow_pid_debug.speed_scheme_centerline_ratio_floor);
        append_float(enabled, "pid_common_speed_scheme_centerline_ratio_weight",
                     line_follow_pid_debug.speed_scheme_centerline_ratio_weight);
        append_float(enabled, "pid_common_speed_scheme_centerline_ratio_current",
                     line_follow_pid_debug.speed_scheme_centerline_ratio_current);
        append_int(enabled, "pid_common_speed_scheme_force_full_min_centerline_count",
                   line_follow_pid_debug.speed_scheme_force_full_min_centerline_count);
        append_float(enabled, "pid_common_speed_scheme_force_full_abs_error_sum_per_point",
                     line_follow_pid_debug.speed_scheme_force_full_abs_error_sum_per_point);
        append_float(enabled, "pid_common_speed_scheme_force_full_abs_error_sum_current",
                     line_follow_pid_debug.speed_scheme_force_full_abs_error_sum_current);
        append_float(enabled, "pid_common_speed_scheme_force_full_abs_error_sum_threshold",
                     line_follow_pid_debug.speed_scheme_force_full_abs_error_sum_threshold);
        append_bool(enabled, "pid_common_speed_scheme_force_full_centerline_ready",
                    line_follow_pid_debug.speed_scheme_force_full_centerline_ready);
        append_bool(enabled, "pid_common_speed_scheme_force_full_error_ready",
                    line_follow_pid_debug.speed_scheme_force_full_error_ready);
        append_bool(enabled, "pid_common_force_full_speed", line_follow_pid_debug.force_full_speed);
        append_float(enabled, "pid_common_raw_steering_output", line_follow_pid_debug.raw_steering_output);
        append_float(enabled, "pid_common_clamped_steering_output", line_follow_pid_debug.clamped_steering_output);
        append_float(enabled, "pid_common_applied_steering_output", line_follow_pid_debug.applied_steering_output);
        append_float(enabled, "pid_common_left_target_count_from_line_follow", line_follow_pid_debug.left_target_count);
        append_float(enabled, "pid_common_right_target_count_from_line_follow", line_follow_pid_debug.right_target_count);
        append_float(enabled, "pid_common_vision_dt_ms", line_follow_pid_debug.vision_dt_ms);
        append_float(enabled, "pid_common_imu_dt_ms", line_follow_pid_debug.imu_dt_ms);
        append_float(enabled, "pid_common_motor_integral_limit", motor_pid_debug.integral_limit);
        append_float(enabled, "pid_common_motor_max_output_step", motor_pid_debug.max_output_step);
        append_float(enabled, "pid_common_motor_correction_limit", motor_pid_debug.correction_limit);
        append_float(enabled, "pid_common_motor_duty_limit", motor_pid_debug.duty_limit);
        append_float(enabled, "pid_common_motor_feedforward_bias_threshold", motor_pid_debug.feedforward_bias_threshold);
        append_float(enabled, "pid_common_motor_decel_error_threshold", motor_pid_debug.decel_error_threshold);
        append_float(enabled, "pid_common_motor_decel_duty_gain", motor_pid_debug.decel_duty_gain);
        append_float(enabled, "pid_common_motor_decel_duty_limit", motor_pid_debug.decel_duty_limit);
        append_float(enabled, "pid_left_motor_pid_kp", motor_pid_debug.params.left_kp);
        append_float(enabled, "pid_left_motor_pid_ki", motor_pid_debug.params.left_ki);
        append_float(enabled, "pid_left_motor_pid_kd", motor_pid_debug.params.left_kd);
        append_float(enabled, "pid_left_motor_feedforward_gain", motor_pid_debug.left_feedforward_gain);
        append_float(enabled, "pid_left_motor_feedforward_bias", motor_pid_debug.left_feedforward_bias);
        append_float(enabled, "pid_left_motor_pid_target", motor_pid_debug.left_pid_target);
        append_float(enabled, "pid_left_motor_pid_error", motor_pid_debug.left_pid_error);
        append_float(enabled, "pid_left_motor_pid_output", motor_pid_debug.left_pid_output);
        append_float(enabled, "pid_left_motor_pid_output_min", motor_pid_debug.left_pid_output_min);
        append_float(enabled, "pid_left_motor_pid_output_max", motor_pid_debug.left_pid_output_max);
        append_float(enabled, "pid_left_motor_pid_integral_limit", motor_pid_debug.left_pid_integral_limit);
        append_float(enabled, "pid_left_motor_pid_max_output_step", motor_pid_debug.left_pid_max_output_step);
        append_float(enabled, "pid_left_target_count", motor_pid_debug.runtime.left_target_count);
        append_float(enabled, "pid_left_current_count", motor_pid_debug.runtime.left_current_count);
        append_float(enabled, "pid_left_feedback", motor_pid_debug.runtime.left_feedback);
        append_float(enabled, "pid_left_error", motor_pid_debug.runtime.left_error);
        append_float(enabled, "pid_left_feedforward", motor_pid_debug.runtime.left_feedforward);
        append_float(enabled, "pid_left_correction", motor_pid_debug.runtime.left_correction);
        append_float(enabled, "pid_left_decel_assist", motor_pid_debug.runtime.left_decel_assist);
        append_float(enabled, "pid_left_duty", motor_pid_debug.runtime.left_duty);
        append_float(enabled, "pid_left_hardware_duty", motor_pid_debug.runtime.left_hardware_duty);
        append_int(enabled, "pid_left_dir_level", motor_pid_debug.runtime.left_dir_level);
        append_float(enabled, "pid_right_motor_pid_kp", motor_pid_debug.params.right_kp);
        append_float(enabled, "pid_right_motor_pid_ki", motor_pid_debug.params.right_ki);
        append_float(enabled, "pid_right_motor_pid_kd", motor_pid_debug.params.right_kd);
        append_float(enabled, "pid_right_motor_feedforward_gain", motor_pid_debug.right_feedforward_gain);
        append_float(enabled, "pid_right_motor_feedforward_bias", motor_pid_debug.right_feedforward_bias);
        append_float(enabled, "pid_right_motor_pid_target", motor_pid_debug.right_pid_target);
        append_float(enabled, "pid_right_motor_pid_error", motor_pid_debug.right_pid_error);
        append_float(enabled, "pid_right_motor_pid_output", motor_pid_debug.right_pid_output);
        append_float(enabled, "pid_right_motor_pid_output_min", motor_pid_debug.right_pid_output_min);
        append_float(enabled, "pid_right_motor_pid_output_max", motor_pid_debug.right_pid_output_max);
        append_float(enabled, "pid_right_motor_pid_integral_limit", motor_pid_debug.right_pid_integral_limit);
        append_float(enabled, "pid_right_motor_pid_max_output_step", motor_pid_debug.right_pid_max_output_step);
        append_float(enabled, "pid_right_target_count", motor_pid_debug.runtime.right_target_count);
        append_float(enabled, "pid_right_current_count", motor_pid_debug.runtime.right_current_count);
        append_float(enabled, "pid_right_feedback", motor_pid_debug.runtime.right_feedback);
        append_float(enabled, "pid_right_error", motor_pid_debug.runtime.right_error);
        append_float(enabled, "pid_right_feedforward", motor_pid_debug.runtime.right_feedforward);
        append_float(enabled, "pid_right_correction", motor_pid_debug.runtime.right_correction);
        append_float(enabled, "pid_right_decel_assist", motor_pid_debug.runtime.right_decel_assist);
        append_float(enabled, "pid_right_duty", motor_pid_debug.runtime.right_duty);
        append_float(enabled, "pid_right_hardware_duty", motor_pid_debug.runtime.right_hardware_duty);
        append_int(enabled, "pid_right_dir_level", motor_pid_debug.runtime.right_dir_level);
    };

    append_int(true, "web_data_profile", data_profile);
    append_int(true, "web_full_debug", send_full_debug ? 1 : 0);
    append_int(true, "udp_web_max_fps", g_vision_runtime_config.udp_web_max_fps);
    append_int(true, "capture_thread_fps", capture_thread_fps);
    append_int(true, "vision_process_fps", vision_process_fps);
    append_int(true, "udp_tx_fps", udp_tx_fps);
    append_bool(true, "udp_web_send_gray", g_vision_runtime_config.udp_web_send_gray_jpeg);
    append_bool(true, "udp_web_send_binary", g_vision_runtime_config.udp_web_send_binary_jpeg);
    append_bool(true, "udp_web_send_rgb", g_vision_runtime_config.udp_web_send_rgb_jpeg);
    append_int(true, "udp_web_gray_image_format",
               static_cast<int>(sanitize_web_image_format(g_vision_runtime_config.udp_web_gray_image_format)));
    append_int(true, "udp_web_binary_image_format",
               static_cast<int>(sanitize_web_image_format(g_vision_runtime_config.udp_web_binary_image_format)));
    append_int(true, "udp_web_rgb_image_format",
               static_cast<int>(sanitize_web_image_format(g_vision_runtime_config.udp_web_rgb_image_format)));

    if (!send_full_debug)
    {
        append_int(true, "ts_ms", ts_ms);
        append_float(true, "cpu_usage_percent", g_system_usage_cache.cpu_usage_percent);
        append_float(true, "mem_usage_percent", g_system_usage_cache.mem_usage_percent);
        append_float(true, "base_speed", line_follow_thread_normal_speed_reference());
        append_float(true, "adjusted_base_speed", line_follow_thread_applied_base_speed());
        append_float(true, "left_target_count", motor_thread_left_target_count());
        append_float(true, "right_target_count", motor_thread_right_target_count());
        append_float(true, "left_current_count", motor_thread_left_count());
        append_float(true, "right_current_count", motor_thread_right_count());
        append_float(true, "left_filtered_count", motor_thread_left_filtered_count());
        append_float(true, "right_filtered_count", motor_thread_right_filtered_count());
        append_int(true, "zebra_cross_count", vision_image_processor_zebra_cross_count());
        append_int(true, "otsu_threshold", otsu_threshold);
        append_bool(true, "red_found", red_found);
        append_int_array(true, "red", {red_x, red_y, red_w, red_h, red_cx, red_cy});
        append_bool(true, "roi_valid", roi_valid);
        append_int_array(true, "roi", {roi_x, roi_y, roi_w, roi_h});
        append_bool(true, "infer_enabled", infer_enabled);
        append_bool(true, "ncnn_enabled", has_infer_result ? infer_result.ncnn_enabled : vision_infer_async_ncnn_enabled());
        append_bool(true, "ncnn_has_result", has_infer_result);
        append_bool(true, "ncnn_infer_valid", has_infer_result && infer_result.ncnn_infer_valid);
        append_int(true, "ncnn_infer_us", has_infer_result ? infer_result.ncnn_infer_us : 0);
        append_int(true, "ncnn_top_class_id", has_infer_result ? infer_result.ncnn_top_class_id : -1);
        append_float(true, "ncnn_top_score", has_infer_result ? infer_result.ncnn_top_score : 0.0f);
        append_string(true, "ncnn_top_label",
                      (has_infer_result && infer_result.ncnn_infer_valid) ? infer_result.ncnn_top_label : "");
        append_prob_pairs(true, "ncnn_probs", infer_result);
        append_pid_debug(true);
        append_bool(true, "udp_web_send_roi64", roi_valid);
        append_int_array(true, "roi64_size", {kRoi64Size, kRoi64Size});
        append_int_array(true, "gray_size",
                         {VISION_DOWNSAMPLED_WIDTH, VISION_DOWNSAMPLED_HEIGHT});
        line += "}";
        line += "\n";
        tcp_client_send_data(reinterpret_cast<const uint8 *>(line.data()), static_cast<uint32>(line.size()));
        return;
    }

    append_int(g_vision_runtime_config.udp_web_tcp_send_ts_ms, "ts_ms", ts_ms);
    append_int(g_vision_runtime_config.udp_web_tcp_send_line_error, "line_error", line_error);
    append_int(true, "zebra_cross_count", vision_image_processor_zebra_cross_count());
    append_float(g_vision_runtime_config.udp_web_tcp_send_cpu_usage_percent,
                 "cpu_usage_percent",
                 g_system_usage_cache.cpu_usage_percent);
    append_float(g_vision_runtime_config.udp_web_tcp_send_mem_usage_percent,
                 "mem_usage_percent",
                 g_system_usage_cache.mem_usage_percent);
    append_float(g_vision_runtime_config.udp_web_tcp_send_base_speed, "base_speed", line_follow_thread_normal_speed_reference());
    append_float(g_vision_runtime_config.udp_web_tcp_send_base_speed, "adjusted_base_speed", line_follow_thread_applied_base_speed());
    append_float(g_vision_runtime_config.udp_web_tcp_send_left_target_count, "left_target_count", motor_thread_left_target_count());
    append_float(g_vision_runtime_config.udp_web_tcp_send_right_target_count, "right_target_count", motor_thread_right_target_count());
    append_float(g_vision_runtime_config.udp_web_tcp_send_left_current_count, "left_current_count", motor_thread_left_count());
    append_float(g_vision_runtime_config.udp_web_tcp_send_right_current_count, "right_current_count", motor_thread_right_count());
    append_float(g_vision_runtime_config.udp_web_tcp_send_left_filtered_count, "left_filtered_count", motor_thread_left_filtered_count());
    append_float(g_vision_runtime_config.udp_web_tcp_send_right_filtered_count, "right_filtered_count", motor_thread_right_filtered_count());
    append_float(g_vision_runtime_config.udp_web_tcp_send_left_error, "left_error", motor_thread_left_error());
    append_float(g_vision_runtime_config.udp_web_tcp_send_right_error, "right_error", motor_thread_right_error());
    append_float(g_vision_runtime_config.udp_web_tcp_send_left_feedforward, "left_feedforward", motor_thread_left_feedforward());
    append_float(g_vision_runtime_config.udp_web_tcp_send_right_feedforward, "right_feedforward", motor_thread_right_feedforward());
    append_float(g_vision_runtime_config.udp_web_tcp_send_left_correction, "left_correction", motor_thread_left_correction());
    append_float(g_vision_runtime_config.udp_web_tcp_send_right_correction, "right_correction", motor_thread_right_correction());
    append_float(g_vision_runtime_config.udp_web_tcp_send_left_decel_assist, "left_decel_assist", motor_thread_left_decel_assist());
    append_float(g_vision_runtime_config.udp_web_tcp_send_right_decel_assist, "right_decel_assist", motor_thread_right_decel_assist());
    append_float(g_vision_runtime_config.udp_web_tcp_send_left_duty, "left_duty", motor_thread_left_duty());
    append_float(g_vision_runtime_config.udp_web_tcp_send_right_duty, "right_duty", motor_thread_right_duty());
    append_float(g_vision_runtime_config.udp_web_tcp_send_left_hardware_duty, "left_hardware_duty", motor_thread_left_hardware_duty());
    append_float(g_vision_runtime_config.udp_web_tcp_send_right_hardware_duty, "right_hardware_duty", motor_thread_right_hardware_duty());
    append_int(g_vision_runtime_config.udp_web_tcp_send_left_dir_level, "left_dir_level", motor_thread_left_dir_level());
    append_int(g_vision_runtime_config.udp_web_tcp_send_right_dir_level, "right_dir_level", motor_thread_right_dir_level());
    append_int(g_vision_runtime_config.udp_web_tcp_send_otsu_threshold, "otsu_threshold", otsu_threshold);
    append_int(g_vision_runtime_config.udp_web_tcp_send_perf_capture_wait_us, "capture_wait_us", capture_wait_us);
    append_int(g_vision_runtime_config.udp_web_tcp_send_perf_preprocess_us, "preprocess_us", preprocess_us);
    append_int(g_vision_runtime_config.udp_web_tcp_send_perf_otsu_us, "otsu_us", otsu_us);
    append_int(g_vision_runtime_config.udp_web_tcp_send_perf_maze_us, "maze_us", maze_us);
    append_int(g_vision_runtime_config.udp_web_tcp_send_perf_total_us, "total_us", total_us);
    append_int(g_vision_runtime_config.udp_web_tcp_send_maze_left_points_raw, "maze_left_points_raw", maze_left_points_raw);
    append_int(g_vision_runtime_config.udp_web_tcp_send_maze_right_points_raw, "maze_right_points_raw", maze_right_points_raw);
    append_bool(g_vision_runtime_config.udp_web_tcp_send_red_found, "red_found", red_found);
    append_int_array(g_vision_runtime_config.udp_web_tcp_send_red_rect, "red",
                     {red_x, red_y, red_w, red_h, red_cx, red_cy});
    append_bool(g_vision_runtime_config.udp_web_tcp_send_roi_valid, "roi_valid", roi_valid);
    append_int_array(g_vision_runtime_config.udp_web_tcp_send_roi_rect, "roi",
                     {roi_x, roi_y, roi_w, roi_h});
    append_bool(true, "infer_enabled", infer_enabled);
    append_bool(true, "ncnn_enabled", has_infer_result ? infer_result.ncnn_enabled : vision_infer_async_ncnn_enabled());
    append_bool(true, "ncnn_has_result", has_infer_result);
    append_bool(true, "ncnn_infer_valid", has_infer_result && infer_result.ncnn_infer_valid);
    append_int(true, "ncnn_infer_us", has_infer_result ? infer_result.ncnn_infer_us : 0);
    append_int(true, "ncnn_top_class_id", has_infer_result ? infer_result.ncnn_top_class_id : -1);
    append_float(true, "ncnn_top_score", has_infer_result ? infer_result.ncnn_top_score : 0.0f);
    append_string(true, "ncnn_top_label",
                  (has_infer_result && infer_result.ncnn_infer_valid) ? infer_result.ncnn_top_label : "");
    append_prob_pairs(true, "ncnn_probs", infer_result);
    append_pid_debug(true);
    append_bool(true, "udp_web_send_roi64", roi_valid);
    append_int_array(true, "roi64_size", {kRoi64Size, kRoi64Size});
    append_bool(g_vision_runtime_config.udp_web_tcp_send_ipm_track_valid, "ipm_track_valid", ipm_track_valid);
    append_int(g_vision_runtime_config.udp_web_tcp_send_ipm_track_method, "ipm_track_method",
               static_cast<int>(vision_image_processor_ipm_line_error_method()));
    append_int(g_vision_runtime_config.udp_web_tcp_send_ipm_centerline_source, "ipm_centerline_source",
               static_cast<int>(vision_image_processor_ipm_line_error_source()));
    const int straight_selected_centerline_count = vision_image_processor_ipm_selected_centerline_count();
    const int straight_required_last_index = vision_image_processor_ipm_straight_required_last_index();
    const float straight_abs_error_sum = vision_image_processor_ipm_straight_abs_error_sum();
    const int straight_min_centerline_points =
        std::max(1, g_vision_runtime_config.route_straight_min_centerline_points);
    const bool straight_state_ready_now =
        (straight_selected_centerline_count >= straight_min_centerline_points) &&
        (straight_abs_error_sum < g_vision_runtime_config.route_straight_abs_error_sum_max);
    append_bool(true, "straight_state_ready_now", straight_state_ready_now);
    append_int(true, "straight_selected_centerline_count", straight_selected_centerline_count);
    append_int(true, "straight_min_centerline_points", straight_min_centerline_points);
    append_int(true, "straight_required_last_index", straight_required_last_index);
    append_float(true, "straight_abs_error_sum", straight_abs_error_sum);
    append_float(true, "straight_abs_error_sum_max", g_vision_runtime_config.route_straight_abs_error_sum_max);
    append_int(true, "straight_enter_consecutive_frames", g_vision_runtime_config.route_straight_enter_consecutive_frames);
    append_int(true, "route_main_state", vision_image_processor_route_main_state());
    append_int(true, "route_sub_state", vision_image_processor_route_sub_state());
    append_int(true, "route_preferred_source", vision_image_processor_route_preferred_source());
    append_int(true, "route_encoder_since_enter", static_cast<int>(vision_image_processor_route_encoder_since_state_enter()));
    append_int(true, "route_cross_loss_count", vision_image_processor_route_cross_loss_count());
    append_int(true, "route_left_loss_count", vision_image_processor_route_left_loss_count());
    append_int(true, "route_left_gain_count", vision_image_processor_route_left_gain_count());
    append_int(true, "route_right_loss_count", vision_image_processor_route_right_loss_count());
    append_int(true, "route_right_gain_count", vision_image_processor_route_right_gain_count());
    append_bool(true, "route_cross_detection_enabled", g_vision_runtime_config.route_cross_detection_enabled);
    const bool cross_state_entry_ready_now =
        g_vision_runtime_config.route_cross_detection_enabled &&
        cross_lower_left_found &&
        cross_lower_right_found &&
        cross_left_corner_post_frame_wall_rows >= g_vision_runtime_config.route_cross_entry_corner_post_frame_wall_rows_min &&
        cross_right_corner_post_frame_wall_rows >= g_vision_runtime_config.route_cross_entry_corner_post_frame_wall_rows_min;
    const bool cross_state_stage2_ready_now =
        ((cross_lower_left_found && cross_lower_left_y >= g_vision_runtime_config.route_cross_stage1_enter_corner_y_min) ||
         (cross_lower_right_found && cross_lower_right_y >= g_vision_runtime_config.route_cross_stage1_enter_corner_y_min));
    const bool cross_state_stage3_ready_now =
        (src_left_start_frame_wall_rows >= g_vision_runtime_config.route_cross_stage2_enter_start_frame_wall_rows_min &&
         src_right_start_frame_wall_rows >= g_vision_runtime_config.route_cross_stage2_enter_start_frame_wall_rows_min);
    const bool cross_state_exit_ready_now =
        (cross_start_boundary_gap_x > 0) &&
        (cross_start_boundary_gap_x < g_vision_runtime_config.route_cross_exit_start_gap_x_max);
    append_bool(true, "cross_state_entry_ready_now", cross_state_entry_ready_now);
    append_bool(true, "cross_state_stage2_ready_now", cross_state_stage2_ready_now);
    append_bool(true, "cross_state_stage3_ready_now", cross_state_stage3_ready_now);
    append_bool(true, "cross_state_exit_ready_now", cross_state_exit_ready_now);
    append_int(true, "cross_left_corner_post_frame_wall_rows", cross_left_corner_post_frame_wall_rows);
    append_int(true, "cross_right_corner_post_frame_wall_rows", cross_right_corner_post_frame_wall_rows);
    append_int(true, "cross_start_boundary_gap_x", cross_start_boundary_gap_x);
    append_int(true, "route_cross_entry_corner_post_frame_wall_rows_min", g_vision_runtime_config.route_cross_entry_corner_post_frame_wall_rows_min);
    append_int(true, "route_cross_stage2_enter_start_frame_wall_rows_min", g_vision_runtime_config.route_cross_stage2_enter_start_frame_wall_rows_min);
    append_int(true, "route_cross_stage1_enter_corner_y_min", g_vision_runtime_config.route_cross_stage1_enter_corner_y_min);
    append_int(true, "route_cross_exit_start_gap_x_max", g_vision_runtime_config.route_cross_exit_start_gap_x_max);
    append_int(true, "route_cross_stage3_jump_x_threshold_px", g_vision_runtime_config.route_cross_stage3_jump_x_threshold_px);
    append_int(true, "route_cross_stage3_cut_forward_points", g_vision_runtime_config.route_cross_stage3_cut_forward_points);
    append_int(true, "route_cross_stage3_left_anchor_x", g_vision_runtime_config.route_cross_stage3_left_anchor_x);
    append_int(true, "route_cross_stage3_left_anchor_y", g_vision_runtime_config.route_cross_stage3_left_anchor_y);
    append_int(true, "route_cross_stage3_right_anchor_x", g_vision_runtime_config.route_cross_stage3_right_anchor_x);
    append_int(true, "route_cross_stage3_right_anchor_y", g_vision_runtime_config.route_cross_stage3_right_anchor_y);
    append_bool(true, "route_circle_detection_enabled", g_vision_runtime_config.route_circle_detection_enabled);
    append_int(true, "route_circle_entry_min_boundary_count", g_vision_runtime_config.route_circle_entry_min_boundary_count);
    append_int(true, "route_circle_entry_corner_tail_margin", g_vision_runtime_config.route_circle_entry_corner_tail_margin);
    append_int(true, "route_circle_entry_corner_row_offset", g_vision_runtime_config.route_circle_entry_corner_row_offset);
    append_int(true, "route_circle_entry_gap_check_rows", g_vision_runtime_config.route_circle_entry_gap_check_rows);
    append_int(true, "route_circle_entry_min_raw_boundary_gap", g_vision_runtime_config.route_circle_entry_min_raw_boundary_gap);
    append_int(true, "route_circle_entry_corner_y_min", g_vision_runtime_config.route_circle_entry_corner_y_min);
    append_int(true, "route_circle_stage_frame_wall_rows_enter", g_vision_runtime_config.route_circle_stage_frame_wall_rows_enter);
    append_int(true, "route_circle_stage3_frame_wall_rows_trigger", g_vision_runtime_config.route_circle_stage3_frame_wall_rows_trigger);
    append_int(true, "route_circle_stage6_maze_start_row", g_vision_runtime_config.route_circle_stage6_maze_start_row);
    append_int(true, "circle_guide_min_frame_wall_segment_len", g_vision_runtime_config.circle_guide_min_frame_wall_segment_len);
    append_int(true, "circle_guide_target_offset_stage3", g_vision_runtime_config.circle_guide_target_offset_stage3);
    append_int(true, "circle_guide_anchor_offset_stage5", g_vision_runtime_config.circle_guide_anchor_offset_stage5);
    append_int(true, "route_circle_apply_touch_margin_px", g_vision_runtime_config.route_circle_apply_touch_margin_px);
    append_bool(true, "left_boundary_straight", src_left_boundary_straight);
    append_bool(true, "right_boundary_straight", src_right_boundary_straight);
    append_bool(true, "src_left_trace_has_frame_wall", src_left_trace_has_frame_wall);
    append_bool(true, "src_right_trace_has_frame_wall", src_right_trace_has_frame_wall);
    append_int(true, "left_start_frame_wall_rows", src_left_start_frame_wall_rows);
    append_int(true, "right_start_frame_wall_rows", src_right_start_frame_wall_rows);
    append_bool(true, "left_circle_entry_raw_gap_ok", left_circle_raw_gap_debug.ok);
    append_bool(true, "right_circle_entry_raw_gap_ok", right_circle_raw_gap_debug.ok);
    append_int(true, "left_circle_entry_raw_gap_min_px", left_circle_raw_gap_debug.min_observed_gap_px);
    append_int(true, "right_circle_entry_raw_gap_min_px", right_circle_raw_gap_debug.min_observed_gap_px);
    append_int(true, "left_circle_entry_raw_gap_checked_rows", left_circle_raw_gap_debug.checked_rows);
    append_int(true, "right_circle_entry_raw_gap_checked_rows", right_circle_raw_gap_debug.checked_rows);
    append_int(true, "left_circle_entry_raw_gap_failed_row_y", left_circle_raw_gap_debug.failed_row_y);
    append_int(true, "right_circle_entry_raw_gap_failed_row_y", right_circle_raw_gap_debug.failed_row_y);
    append_bool(true, "left_circle_entry_raw_gap_missing_point", left_circle_raw_gap_debug.missing_point);
    append_bool(true, "right_circle_entry_raw_gap_missing_point", right_circle_raw_gap_debug.missing_point);
    append_int(g_vision_runtime_config.udp_web_tcp_send_ipm_track_index, "ipm_track_index", ipm_track_index);
    append_int_array(g_vision_runtime_config.udp_web_tcp_send_ipm_track_point, "ipm_track_point",
                     {ipm_track_x, ipm_track_y});
    auto append_points = [&append_key, &line](bool enabled, const char *name, uint16 *xs, uint16 *ys, uint16 n, int max_w, int max_h) {
        if (!enabled)
        {
            return;
        }
        append_key(name);
        line += "[";
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

    auto append_u8_array = [&append_key, &line](bool enabled, const char *name, const uint8 *values, uint16 n) {
        if (!enabled)
        {
            return;
        }
        append_key(name);
        line += "[";
        for (uint16 i = 0; i < n; ++i)
        {
            if (i > 0)
            {
                line += ",";
            }
            line += std::to_string(static_cast<int>(values ? values[i] : 255));
        }
        line += "]";
    };

    append_points(g_vision_runtime_config.udp_web_tcp_send_left_boundary,
                  "left_boundary", x1, y1, left_dot_num, VISION_DOWNSAMPLED_WIDTH, VISION_DOWNSAMPLED_HEIGHT);
    append_points(g_vision_runtime_config.udp_web_tcp_send_right_boundary,
                  "right_boundary", x3, y3, right_dot_num, VISION_DOWNSAMPLED_WIDTH, VISION_DOWNSAMPLED_HEIGHT);
    append_points(true,
                  "left_circle_guide_line",
                  src_left_circle_guide_x,
                  src_left_circle_guide_y,
                  src_left_circle_guide_num,
                  VISION_DOWNSAMPLED_WIDTH,
                  VISION_DOWNSAMPLED_HEIGHT);
    append_points(true,
                  "right_circle_guide_line",
                  src_right_circle_guide_x,
                  src_right_circle_guide_y,
                  src_right_circle_guide_num,
                  VISION_DOWNSAMPLED_WIDTH,
                  VISION_DOWNSAMPLED_HEIGHT);
    append_points(true,
                  "cross_left_aux_trace",
                  cross_left_aux_trace_x,
                  cross_left_aux_trace_y,
                  cross_left_aux_trace_num,
                  VISION_DOWNSAMPLED_WIDTH,
                  VISION_DOWNSAMPLED_HEIGHT);
    append_points(true,
                  "cross_right_aux_trace",
                  cross_right_aux_trace_x,
                  cross_right_aux_trace_y,
                  cross_right_aux_trace_num,
                  VISION_DOWNSAMPLED_WIDTH,
                  VISION_DOWNSAMPLED_HEIGHT);
    append_u8_array(true, "cross_left_aux_trace_dir", cross_left_aux_trace_dir, cross_left_aux_trace_num);
    append_u8_array(true, "cross_right_aux_trace_dir", cross_right_aux_trace_dir, cross_right_aux_trace_num);
    append_points(true,
                  "cross_left_aux_regular",
                  cross_left_aux_regular_x,
                  cross_left_aux_regular_y,
                  cross_left_aux_regular_num,
                  VISION_DOWNSAMPLED_WIDTH,
                  VISION_DOWNSAMPLED_HEIGHT);
    append_points(true,
                  "cross_right_aux_regular",
                  cross_right_aux_regular_x,
                  cross_right_aux_regular_y,
                  cross_right_aux_regular_num,
                  VISION_DOWNSAMPLED_WIDTH,
                  VISION_DOWNSAMPLED_HEIGHT);
    append_points(g_vision_runtime_config.udp_web_tcp_send_left_boundary,
                  "eight_left_trace", eight_left_x, eight_left_y, eight_left_num, VISION_DOWNSAMPLED_WIDTH, VISION_DOWNSAMPLED_HEIGHT);
    append_points(g_vision_runtime_config.udp_web_tcp_send_right_boundary,
                  "eight_right_trace", eight_right_x, eight_right_y, eight_right_num, VISION_DOWNSAMPLED_WIDTH, VISION_DOWNSAMPLED_HEIGHT);
    append_u8_array(g_vision_runtime_config.udp_web_tcp_send_left_boundary,
                    "eight_left_trace_dir", eight_left_dir, eight_left_num);
    append_u8_array(g_vision_runtime_config.udp_web_tcp_send_right_boundary,
                    "eight_right_trace_dir", eight_right_dir, eight_right_num);
    append_int(g_vision_runtime_config.udp_web_tcp_send_left_boundary,
               "eight_left_first_frame_touch_index",
               eight_left_first_frame_touch_index);
    append_int(g_vision_runtime_config.udp_web_tcp_send_right_boundary,
               "eight_right_first_frame_touch_index",
               eight_right_first_frame_touch_index);
    const bool eight_left_touch_valid =
        eight_left_first_frame_touch_index >= 0 &&
        eight_left_first_frame_touch_index < static_cast<int>(eight_left_num) &&
        eight_left_x != nullptr &&
        eight_left_y != nullptr;
    const bool eight_right_touch_valid =
        eight_right_first_frame_touch_index >= 0 &&
        eight_right_first_frame_touch_index < static_cast<int>(eight_right_num) &&
        eight_right_x != nullptr &&
        eight_right_y != nullptr;
    append_int_array(g_vision_runtime_config.udp_web_tcp_send_left_boundary && eight_left_touch_valid,
                     "eight_left_first_frame_touch_point",
                     {eight_left_x[eight_left_first_frame_touch_index], eight_left_y[eight_left_first_frame_touch_index]});
    append_int_array(g_vision_runtime_config.udp_web_tcp_send_right_boundary && eight_right_touch_valid,
                     "eight_right_first_frame_touch_point",
                     {eight_right_x[eight_right_first_frame_touch_index], eight_right_y[eight_right_first_frame_touch_index]});
    append_bool(true, "cross_lower_corner_dir_enabled", g_vision_runtime_config.cross_lower_corner_dir_enabled);
    append_int(true, "cross_lower_corner_pre_window", g_vision_runtime_config.cross_lower_corner_pre_window);
    append_int(true, "cross_lower_corner_post_window", g_vision_runtime_config.cross_lower_corner_post_window);
    append_int(true, "cross_lower_corner_pre_min_votes", g_vision_runtime_config.cross_lower_corner_pre_min_votes);
    append_int(true, "cross_lower_corner_post_min_votes", g_vision_runtime_config.cross_lower_corner_post_min_votes);
    append_int(true, "cross_lower_corner_transition_max_len", g_vision_runtime_config.cross_lower_corner_transition_max_len);
    append_int(true, "cross_lower_corner_pair_y_diff_max", g_vision_runtime_config.cross_lower_corner_pair_y_diff_max);
    append_int(true, "cross_aux_vertical_scan_max_rows", g_vision_runtime_config.cross_aux_vertical_scan_max_rows);
    append_int(true, "cross_aux_trace_max_points", g_vision_runtime_config.cross_aux_trace_max_points);
    append_int(true, "cross_aux_trace_upward_rows_max", g_vision_runtime_config.cross_aux_trace_upward_rows_max);
    append_int(true, "cross_upper_dir4_pre_run_len", g_vision_runtime_config.cross_upper_dir4_pre_run_len);
    append_int(true, "cross_upper_transition_max_len", g_vision_runtime_config.cross_upper_transition_max_len);
    append_int(true, "cross_upper_dir6_post_run_len", g_vision_runtime_config.cross_upper_dir6_post_run_len);
    append_bool(true, "cross_lower_left_corner_found", cross_lower_left_found);
    append_bool(true, "cross_lower_right_corner_found", cross_lower_right_found);
    append_bool(true, "cross_lower_corner_pair_valid", cross_lower_pair_valid);
    append_int(true, "cross_lower_left_corner_index", cross_lower_left_index);
    append_int(true, "cross_lower_right_corner_index", cross_lower_right_index);
    append_int_array(cross_lower_left_found, "cross_lower_left_corner_point", {cross_lower_left_x, cross_lower_left_y});
    append_int_array(cross_lower_right_found, "cross_lower_right_corner_point", {cross_lower_right_x, cross_lower_right_y});
    append_bool(true, "cross_left_aux_found", cross_left_aux_found);
    append_bool(true, "cross_right_aux_found", cross_right_aux_found);
    append_int(true, "cross_left_aux_trace_count", static_cast<int>(cross_left_aux_trace_num));
    append_int(true, "cross_right_aux_trace_count", static_cast<int>(cross_right_aux_trace_num));
    append_int(true, "cross_left_aux_regular_count", static_cast<int>(cross_left_aux_regular_num));
    append_int(true, "cross_right_aux_regular_count", static_cast<int>(cross_right_aux_regular_num));
    append_int(true, "cross_left_aux_dir5_count", cross_left_aux_dir5_count);
    append_int(true, "cross_right_aux_dir5_count", cross_right_aux_dir5_count);
    append_int_array(cross_left_aux_found, "cross_left_aux_transition_point", {cross_left_aux_transition_x, cross_left_aux_transition_y});
    append_int_array(cross_right_aux_found, "cross_right_aux_transition_point", {cross_right_aux_transition_x, cross_right_aux_transition_y});
    append_bool(true, "cross_left_upper_corner_found", cross_left_upper_found);
    append_bool(true, "cross_right_upper_corner_found", cross_right_upper_found);
    append_int(true, "cross_left_upper_corner_index", cross_left_upper_index);
    append_int(true, "cross_right_upper_corner_index", cross_right_upper_index);
    append_int_array(cross_left_upper_found, "cross_left_upper_corner_point", {cross_left_upper_x, cross_left_upper_y});
    append_int_array(cross_right_upper_found, "cross_right_upper_corner_point", {cross_right_upper_x, cross_right_upper_y});
    append_bool(true, "cross_stage2_frozen_left_corner_found", cross_stage2_frozen_left_found);
    append_bool(true, "cross_stage2_frozen_right_corner_found", cross_stage2_frozen_right_found);
    append_int_array(cross_stage2_frozen_left_found,
                     "cross_stage2_frozen_left_corner_point",
                     {cross_stage2_frozen_left_x, cross_stage2_frozen_left_y});
    append_int_array(cross_stage2_frozen_right_found,
                     "cross_stage2_frozen_right_corner_point",
                     {cross_stage2_frozen_right_x, cross_stage2_frozen_right_y});
    append_points(g_vision_runtime_config.udp_web_tcp_send_ipm_left_boundary,
                  "ipm_left_boundary", ipm_x1, ipm_y1, ipm_left_dot_num, kIpmCanvasWidth, kIpmCanvasHeight);
    append_points(g_vision_runtime_config.udp_web_tcp_send_ipm_right_boundary,
                  "ipm_right_boundary", ipm_x3, ipm_y3, ipm_right_dot_num, kIpmCanvasWidth, kIpmCanvasHeight);

    const bool selected_is_right =
        (vision_image_processor_ipm_line_error_source() == VISION_IPM_LINE_ERROR_FROM_RIGHT_SHIFT);
    uint16 *selected_ipm_center_x = selected_is_right ? ipm_center_right_x : ipm_center_left_x;
    uint16 *selected_ipm_center_y = selected_is_right ? ipm_center_right_y : ipm_center_left_y;
    uint16 selected_ipm_center_num = selected_is_right ? ipm_center_right_num : ipm_center_left_num;
    uint16 *selected_src_center_x = selected_is_right ? src_center_right_x : src_center_left_x;
    uint16 *selected_src_center_y = selected_is_right ? src_center_right_y : src_center_left_y;
    uint16 selected_src_center_num = selected_is_right ? src_center_right_num : src_center_left_num;

    const bool send_selected_ipm_centerline =
        g_vision_runtime_config.udp_web_tcp_send_ipm_centerline_selected_shift;
    const bool send_selected_src_centerline =
        g_vision_runtime_config.udp_web_tcp_send_src_centerline_selected_shift;

    append_points(send_selected_ipm_centerline,
                  "ipm_centerline_selected_shift",
                  selected_ipm_center_x,
                  selected_ipm_center_y,
                  selected_ipm_center_num,
                  kIpmCanvasWidth,
                  kIpmCanvasHeight);
    append_points(send_selected_src_centerline,
                  "src_centerline_selected_shift",
                  selected_src_center_x,
                  selected_src_center_y,
                  selected_src_center_num,
                  VISION_DOWNSAMPLED_WIDTH,
                  VISION_DOWNSAMPLED_HEIGHT);
    append_int(g_vision_runtime_config.udp_web_tcp_send_ipm_centerline_selected_count,
               "ipm_centerline_selected_count",
               selected_ipm_center_num);
    append_int(g_vision_runtime_config.udp_web_tcp_send_src_centerline_selected_count,
               "src_centerline_selected_count",
               selected_src_center_num);
    append_int_array(g_vision_runtime_config.udp_web_tcp_send_gray_size, "gray_size",
                     {VISION_DOWNSAMPLED_WIDTH, VISION_DOWNSAMPLED_HEIGHT});
    append_int_array(g_vision_runtime_config.udp_web_tcp_send_ipm_size, "ipm_size",
                     {kIpmCanvasWidth, kIpmCanvasHeight});

    line += "}";
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
        seekfree_assistant_camera_send();
        auto t1 = std::chrono::steady_clock::now();
        g_last_send_tick_us.store(static_cast<uint64>(
            std::chrono::duration_cast<std::chrono::microseconds>(t1.time_since_epoch()).count()));
        g_last_send_time_us.store(static_cast<uint32>(
            std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count()));
    }

    if (try_acquire_udp_send_slot())
    {
        if (g_udp_enabled.load() && g_vision_runtime_config.udp_web_send_gray_jpeg)
        {
            std::vector<uint8> gray_image;
            int width = 0;
            int height = 0;
            uint8 mode = 0;
            const vision_web_image_format_enum format =
                sanitize_web_image_format(g_vision_runtime_config.udp_web_gray_image_format);
            if (build_gray_image(&gray_image, &width, &height, &mode))
            {
                send_udp_frame(gray_image, width, height, mode, format);
            }
        }
        if (g_udp_enabled.load() && g_vision_runtime_config.udp_web_send_binary_jpeg)
        {
            std::vector<uint8> binary_image;
            int width = 0;
            int height = 0;
            uint8 mode = 0;
            const vision_web_image_format_enum format =
                sanitize_web_image_format(g_vision_runtime_config.udp_web_binary_image_format);
            if (build_binary_image(&binary_image, &width, &height, &mode))
            {
                send_udp_frame(binary_image, width, height, mode, format);
            }
        }
        if (g_udp_enabled.load() && g_vision_runtime_config.udp_web_send_rgb_jpeg)
        {
            std::vector<uint8> rgb_image;
            int width = 0;
            int height = 0;
            uint8 mode = 0;
            const vision_web_image_format_enum format =
                sanitize_web_image_format(g_vision_runtime_config.udp_web_rgb_image_format);
            if (build_rgb_image(&rgb_image, &width, &height, &mode))
            {
                send_udp_frame(rgb_image, width, height, mode, format);
            }
        }
        if (g_udp_enabled.load())
        {
            std::vector<uint8> roi64_image;
            int width = 0;
            int height = 0;
            uint8 mode = 0;
            const vision_web_image_format_enum format =
                sanitize_web_image_format(g_vision_runtime_config.udp_web_rgb_image_format);
            if (build_roi64_image(&roi64_image, &width, &height, &mode))
            {
                send_udp_frame(roi64_image, width, height, mode, format);
            }
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

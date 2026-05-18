#include "driver/vision/vision_transport.h"

#include "imu_thread.h"
#include "motor_thread.h"
#include "line_follow_thread.h"
#include "driver/vision/vision_config.h"
#include "driver/vision/vision_frame_capture.h"
#include "driver/vision/vision_infer_async.h"
#include "driver/vision/vision_image_processor.h"
#include "driver/vision/vision_pipeline.h"
#include "vision_thread.h"

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
constexpr uint32 kUdpDefaultMaxFps = 60;
constexpr uint32 kUdpMaxFpsUpper = 120;
constexpr uint32 kMaxUdpPayload = 1200;
constexpr uint32 kUdpHeaderSize = 22;
constexpr uint32 kMagic = 0x56535545;
constexpr int kDefaultJpegQuality = 85;

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
    uint8 format;
    uint16 status_len;
};
#pragma pack(pop)

std::atomic<bool> g_udp_enabled(false);
std::atomic<uint32> g_udp_max_fps(kUdpDefaultMaxFps);
std::atomic<uint64> g_udp_last_send_tick_us(0);
std::atomic<uint32> g_udp_frame_id(0);
std::atomic<uint32> g_udp_tx_fps(0);

std::mutex g_init_mutex;
bool g_udp_ready = false;
char g_server_ip[64] = {0};
uint16 g_video_port = 0;

static uint64 now_us();

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
        enc_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality};
    }
    return cv::imencode(opencv_ext_for_web_image_format(format), img, *image_out, enc_params);
}

static bool build_web_image(std::vector<uint8> *image_out, int *width, int *height, uint8 *mode_out)
{
    if (image_out == nullptr || width == nullptr || height == nullptr || mode_out == nullptr)
    {
        return false;
    }
    const int send_mode = g_vision_runtime_config.web_send_mode;
    const int w = VISION_DOWNSAMPLED_WIDTH;
    const int h = VISION_DOWNSAMPLED_HEIGHT;
    const vision_web_image_format_enum fmt =
        sanitize_web_image_format(g_vision_runtime_config.web_image_format);
    const int quality = g_vision_runtime_config.web_jpeg_quality > 0
                            ? g_vision_runtime_config.web_jpeg_quality
                            : kDefaultJpegQuality;

    const uint8 *src = nullptr;
    uint8 mode_val = 0;
    if (send_mode == 0)
    {
        src = vision_image_processor_binary_downsampled_u8_image();
        mode_val = 0;
    }
    else
    {
        src = vision_image_processor_gray_downsampled_image();
        mode_val = 1;
    }

    if (src == nullptr || !encode_gray_like_for_web(src, w, h, fmt, quality, image_out))
    {
        return false;
    }
    *width = w;
    *height = h;
    *mode_out = mode_val;
    return true;
}

// 作用：构建紧凑 JSON 状态（视觉 + 控制关键字段）。
static std::string build_status_json()
{
    const uint32 capture_thread_fps = vision_frame_capture_fps();
    const uint32 vision_process_fps = vision_thread_process_fps();
    const uint32 udp_tx_fps = g_udp_tx_fps.load();

    const int current_line_error = ::line_error;
    bool red_found = false;
    int red_x = 0, red_y = 0, red_w = 0, red_h = 0, red_cx = 0, red_cy = 0;
    vision_image_processor_get_red_rect(&red_found, &red_x, &red_y, &red_w, &red_h, &red_cx, &red_cy);

    vision_infer_async_result_t infer_result{};
    const bool has_infer_result = vision_infer_async_fetch_latest(&infer_result);

    const int route_main_state = vision_image_processor_route_main_state();
    const int route_sub_state = vision_image_processor_route_sub_state();
    const int route_preferred_source = vision_image_processor_route_preferred_source();
    const int zebra_cross_count = vision_image_processor_zebra_cross_count();

    const int ipm_centerline_source = static_cast<int>(vision_image_processor_ipm_line_error_source());
    const int ipm_selected_count = vision_image_processor_ipm_selected_centerline_count();
    bool ipm_track_valid = false;
    int ipm_track_x = 0, ipm_track_y = 0;
    vision_image_processor_get_ipm_line_error_track_point(&ipm_track_valid, &ipm_track_x, &ipm_track_y);
    const int ipm_track_index = vision_image_processor_ipm_line_error_track_index();

    LineFollowPidDebugStatus line_follow_pid_debug{};
    MotorPidDebugStatus motor_pid_debug{};
    line_follow_thread_get_pid_debug_status(line_follow_pid_debug);
    motor_thread_get_pid_debug_status(motor_pid_debug);

    const float base_speed = line_follow_thread_normal_speed_reference();
    const float adjusted_base_speed = line_follow_thread_applied_base_speed();
    const float gyro_z = imu_thread_gyro_z_dps();
    const int64_t ts_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::steady_clock::now().time_since_epoch())
                              .count();

    char buf[2048];
    std::snprintf(buf, sizeof(buf),
        "{"
        "\"ts\":%lld,"
        "\"fps\":{\"capture\":%u,\"vision\":%u,\"tx\":%u},"
        "\"line_error\":%d,"
        "\"elements\":{"
          "\"red_found\":%d,"
          "\"red\":[%d,%d,%d,%d,%d,%d],"
          "\"ncnn_label\":\"%s\","
          "\"ncnn_score\":%.3f,"
          "\"ncnn_class_id\":%d"
        "},"
        "\"detour\":{"
          "\"route_main\":%d,"
          "\"route_sub\":%d,"
          "\"preferred_source\":%d,"
          "\"zebra\":%d"
        "},"
        "\"centerline\":{"
          "\"source\":%d,"
          "\"selected_count\":%d,"
          "\"track_valid\":%d,"
          "\"track_point\":[%d,%d],"
          "\"track_index\":%d"
        "},"
        "\"pid\":{"
          "\"pos\":{\"error\":%.3f,\"output\":%.3f,\"integral\":%.3f,\"kp_dynamic\":%.3f},"
          "\"yaw\":{\"ref\":%.3f,\"error\":%.3f,\"output\":%.3f,\"integral\":%.3f},"
          "\"steering\":%.3f"
        "},"
        "\"wheels\":{"
          "\"left\":{\"target\":%.3f,\"current\":%.3f,\"error\":%.3f,\"duty\":%.3f},"
          "\"right\":{\"target\":%.3f,\"current\":%.3f,\"error\":%.3f,\"duty\":%.3f}"
        "},"
        "\"speed\":{\"base\":%.3f,\"adjusted\":%.3f},"
        "\"imu\":{\"gyro_z\":%.3f,\"yaw_ref\":%.3f,\"yaw_error\":%.3f}"
        "}",
        static_cast<long long>(ts_ms),
        capture_thread_fps, vision_process_fps, udp_tx_fps,
        current_line_error,
        red_found ? 1 : 0, red_x, red_y, red_w, red_h, red_cx, red_cy,
        (has_infer_result && infer_result.ncnn_infer_valid) ? infer_result.ncnn_top_label : "",
        has_infer_result ? static_cast<double>(infer_result.ncnn_top_score) : 0.0,
        has_infer_result ? infer_result.ncnn_top_class_id : -1,
        route_main_state, route_sub_state, route_preferred_source, zebra_cross_count,
        ipm_centerline_source, ipm_selected_count,
        ipm_track_valid ? 1 : 0, ipm_track_x, ipm_track_y, ipm_track_index,
        static_cast<double>(line_follow_pid_debug.position_pid_error),
        static_cast<double>(line_follow_pid_debug.position_pid_output),
        static_cast<double>(line_follow_pid_debug.position_pid_integral),
        static_cast<double>(line_follow_pid_debug.dynamic_position_kp),
        static_cast<double>(line_follow_pid_debug.yaw_rate_ref_dps),
        static_cast<double>(line_follow_pid_debug.yaw_pid_error),
        static_cast<double>(line_follow_pid_debug.yaw_pid_output),
        static_cast<double>(line_follow_pid_debug.yaw_pid_integral),
        static_cast<double>(line_follow_pid_debug.applied_steering_output),
        static_cast<double>(motor_pid_debug.runtime.left_target_count),
        static_cast<double>(motor_pid_debug.runtime.left_current_count),
        static_cast<double>(motor_pid_debug.runtime.left_error),
        static_cast<double>(motor_pid_debug.runtime.left_duty),
        static_cast<double>(motor_pid_debug.runtime.right_target_count),
        static_cast<double>(motor_pid_debug.runtime.right_current_count),
        static_cast<double>(motor_pid_debug.runtime.right_error),
        static_cast<double>(motor_pid_debug.runtime.right_duty),
        static_cast<double>(base_speed),
        static_cast<double>(adjusted_base_speed),
        static_cast<double>(gyro_z),
        static_cast<double>(line_follow_pid_debug.yaw_rate_ref_dps),
        static_cast<double>(line_follow_pid_debug.yaw_rate_error_dps)
    );
    return std::string(buf);
}

// 作用：发送组合后的 UDP 分片帧（status JSON + image）。
static void send_combined_udp(const std::string &status_json,
                               const std::vector<uint8> &image_bytes,
                               int width,
                               int height,
                               uint8 mode,
                               vision_web_image_format_enum format)
{
    if (!g_udp_ready)
    {
        return;
    }

    // UDP 发送 FPS 统计。
    static uint32 window_frames = 0;
    static uint64 window_start_us = 0;
    const uint64 now = now_us();
    if (window_start_us == 0)
    {
        window_start_us = now;
    }
    ++window_frames;
    const uint64 elapsed_us = now - window_start_us;
    if (elapsed_us >= 1000000ULL)
    {
        const uint32 fps = static_cast<uint32>(
            (static_cast<uint64>(window_frames) * 1000000ULL + elapsed_us / 2ULL) / elapsed_us);
        g_udp_tx_fps.store(fps);
        window_frames = 0;
        window_start_us = now;
    }

    // 拼接完整 payload: status_json + image_bytes。
    const uint32 status_len = static_cast<uint32>(status_json.size());
    const uint32 total_size = status_len + static_cast<uint32>(image_bytes.size());
    const uint32 frame_id = g_udp_frame_id.fetch_add(1);
    const uint32 max_chunk_data = kMaxUdpPayload - kUdpHeaderSize;
    const uint32 chunk_total = (total_size + max_chunk_data - 1U) / max_chunk_data;
    std::vector<uint8> packet;
    packet.resize(kMaxUdpPayload);

    // 将 status + image 放入连续 buffer 便于分片读取。
    std::vector<uint8> combined;
    combined.reserve(total_size);
    combined.insert(combined.end(), status_json.begin(), status_json.end());
    combined.insert(combined.end(), image_bytes.begin(), image_bytes.end());

    for (uint32 i = 0; i < chunk_total; ++i)
    {
        const uint32 offset = i * max_chunk_data;
        const uint32 remain = total_size - offset;
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
        hdr.format = static_cast<uint8>(format);
        hdr.status_len = (i == 0) ? htons(static_cast<uint16>(status_len)) : 0;

        std::memcpy(packet.data(), &hdr, sizeof(hdr));
        std::memcpy(packet.data() + sizeof(hdr), combined.data() + offset, payload_len);
        udp_send_data(packet.data(), payload_len + sizeof(hdr));
    }
}

// ---------- public API ----------

} // namespace

void vision_transport_init()
{
    g_last_send_mode.store(-1);
}

void vision_transport_send_step()
{
    // 客户端发送（逐飞助手）。
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

    // 统一 UDP 发送（图像 + 状态）。
    if (g_udp_enabled.load() && try_acquire_udp_send_slot())
    {
        std::string status_json = build_status_json();
        std::vector<uint8> image_bytes;
        int width = 0;
        int height = 0;
        uint8 mode = 0;
        const vision_web_image_format_enum fmt =
            sanitize_web_image_format(g_vision_runtime_config.web_image_format);
        if (build_web_image(&image_bytes, &width, &height, &mode))
        {
            send_combined_udp(status_json, image_bytes, width, height, mode, fmt);
        }
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

bool vision_transport_udp_init(const char *server_ip, uint16 video_port)
{
    std::lock_guard<std::mutex> lk(g_init_mutex);
    if (server_ip == nullptr || server_ip[0] == '\0' || video_port == 0)
    {
        return false;
    }
    std::snprintf(g_server_ip, sizeof(g_server_ip), "%s", server_ip);
    g_video_port = video_port;

    g_udp_ready = (udp_init(g_server_ip, g_video_port) == 0);
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


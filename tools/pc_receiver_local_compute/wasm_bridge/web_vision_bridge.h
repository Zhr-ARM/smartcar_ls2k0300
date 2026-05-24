#ifndef WEB_VISION_BRIDGE_H_
#define WEB_VISION_BRIDGE_H_

#include <cstddef>
#include <cstdint>

static constexpr int WEB_VISION_MAX_POINTS = 512;
static constexpr int WEB_VISION_STATUS_MESSAGE_LEN = 192;

typedef struct
{
    int width;
    int height;
    const uint8_t *data;
    size_t data_size;
    bool is_gray;
} web_vision_frame_t;

typedef struct
{
    float left_current_count;
    float right_current_count;
    float left_target_count;
    float right_target_count;
    float left_filtered_count;
    float right_filtered_count;
    float base_speed;
    float adjusted_base_speed;
    int otsu_threshold;
    int64_t ts_ms;
} web_vision_raw_status_t;

typedef struct
{
    int count;
    int x[WEB_VISION_MAX_POINTS];
    int y[WEB_VISION_MAX_POINTS];
} web_vision_point_set_t;

typedef struct
{
    bool valid;
    int line_error;

    web_vision_point_set_t left_boundary;
    web_vision_point_set_t right_boundary;
    web_vision_point_set_t ipm_left_boundary;
    web_vision_point_set_t ipm_right_boundary;
    web_vision_point_set_t src_centerline_selected_shift;
    web_vision_point_set_t centerline_selected_shift;

    int ipm_track_index;
    int ipm_track_x;
    int ipm_track_y;
    bool ipm_track_valid;

    char status_message[WEB_VISION_STATUS_MESSAGE_LEN];
} web_vision_result_t;

// 说明：
// 这里只定义“浏览器侧复算”的稳定接口。
// 真正目标不是写一份新的 JS 算法，而是只读复用 project/ 中现有 C++ 算法源码，
// 再编译出 WebAssembly 供浏览器 worker 调用。
bool web_vision_bridge_process(const web_vision_frame_t *frame,
                               const web_vision_raw_status_t *status,
                               web_vision_result_t *result);

#endif

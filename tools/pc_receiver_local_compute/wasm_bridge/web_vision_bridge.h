#ifndef WEB_VISION_BRIDGE_H_
#define WEB_VISION_BRIDGE_H_

#include <cstddef>
#include <cstdint>

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
    float base_speed;
    int otsu_threshold;
    int64_t ts_ms;
} web_vision_raw_status_t;

typedef struct
{
    int line_error;
    bool valid;
} web_vision_result_t;

// 说明：
// 这里只定义“浏览器侧复算”的稳定接口，
// 具体实现后续应尽量复用 project/ 中现有算法源码，而不是重写一份。
bool web_vision_bridge_process(const web_vision_frame_t *frame,
                               const web_vision_raw_status_t *status,
                               web_vision_result_t *result);

#endif

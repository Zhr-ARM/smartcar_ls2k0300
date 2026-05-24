#include "web_vision_bridge.h"
#include "driver/vision/vision_image_processor.h"
#include "line_follow_thread.h"
#include "driver/vision/vision_frame_capture.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace
{
bool g_bridge_initialized = false;

void web_vision_reset_result(web_vision_result_t *result)
{
    if (result == nullptr)
    {
        return;
    }
    std::memset(result, 0, sizeof(*result));
    result->valid = false;
    result->ipm_track_index = -1;
    std::snprintf(result->status_message,
                  sizeof(result->status_message),
                  "%s",
                  "WASM bridge stub: real project algorithm is not linked yet.");
}

void copy_point_set(const uint16 *xs,
                    const uint16 *ys,
                    uint16 count,
                    web_vision_point_set_t *dst)
{
    if (dst == nullptr)
    {
        return;
    }
    dst->count = 0;
    if (xs == nullptr || ys == nullptr || count == 0)
    {
        return;
    }

    const int n = std::min<int>(count, WEB_VISION_MAX_POINTS);
    dst->count = n;
    for (int i = 0; i < n; ++i)
    {
        dst->x[i] = static_cast<int>(xs[i]);
        dst->y[i] = static_cast<int>(ys[i]);
    }
}

} // namespace

bool web_vision_bridge_process(const web_vision_frame_t *frame,
                               const web_vision_raw_status_t *status,
                               web_vision_result_t *result)
{
    if (result == nullptr)
    {
        return false;
    }

    web_vision_reset_result(result);

    if (frame == nullptr || status == nullptr || frame->data == nullptr || frame->data_size == 0)
    {
        std::snprintf(result->status_message,
                      sizeof(result->status_message),
                      "%s",
                      "WASM bridge stub: invalid input frame or raw status.");
        return false;
    }

    // 先把网页收到的原始速度喂进 compat 层。
    // 后续真实 `vision_line_error_layer.cpp` 接入后，会直接读到这份状态。
    line_follow_thread_set_base_speed(status->base_speed);
    wasm_compat_line_follow_thread_set_adjusted_base_speed(status->adjusted_base_speed);
    wasm_compat_vision_frame_capture_set_bgr_frame(frame->data, frame->data_size);

    if (!g_bridge_initialized)
    {
        g_bridge_initialized = vision_image_processor_init(nullptr);
        if (!g_bridge_initialized)
        {
            std::snprintf(result->status_message,
                          sizeof(result->status_message),
                          "%s",
                          "WASM bridge: vision_image_processor_init failed.");
            return false;
        }
    }

    if (!vision_image_processor_process_step())
    {
        std::snprintf(result->status_message,
                      sizeof(result->status_message),
                      "%s",
                      "WASM bridge: vision_image_processor_process_step returned no frame.");
        return false;
    }

    result->line_error = line_error;
    result->valid = true;
    result->ipm_track_index = vision_image_processor_ipm_line_error_track_index();
    vision_image_processor_get_ipm_line_error_track_point(&result->ipm_track_valid,
                                                          &result->ipm_track_x,
                                                          &result->ipm_track_y);

    uint16 *left_x = nullptr;
    uint16 *center_x = nullptr;
    uint16 *right_x = nullptr;
    uint16 *left_y = nullptr;
    uint16 *center_y = nullptr;
    uint16 *right_y = nullptr;
    uint16 boundary_count = 0;
    vision_image_processor_get_boundaries(&left_x, &center_x, &right_x,
                                          &left_y, &center_y, &right_y,
                                          &boundary_count);
    copy_point_set(left_x, left_y, boundary_count, &result->left_boundary);
    copy_point_set(right_x, right_y, boundary_count, &result->right_boundary);

    vision_image_processor_get_ipm_boundaries(&left_x, &center_x, &right_x,
                                              &left_y, &center_y, &right_y,
                                              &boundary_count);
    copy_point_set(left_x, left_y, boundary_count, &result->ipm_left_boundary);
    copy_point_set(right_x, right_y, boundary_count, &result->ipm_right_boundary);

    uint16 *shift_x = nullptr;
    uint16 *shift_y = nullptr;
    uint16 shift_num = 0;
    const vision_ipm_line_error_source_enum selected_source = vision_image_processor_ipm_line_error_source();
    if (selected_source == VISION_IPM_LINE_ERROR_FROM_RIGHT_SHIFT)
    {
        vision_image_processor_get_ipm_shifted_centerline_from_right(&shift_x, &shift_y, &shift_num);
        copy_point_set(shift_x, shift_y, shift_num, &result->centerline_selected_shift);
        vision_image_processor_get_src_shifted_centerline_from_right(&shift_x, &shift_y, &shift_num);
        copy_point_set(shift_x, shift_y, shift_num, &result->src_centerline_selected_shift);
    }
    else
    {
        vision_image_processor_get_ipm_shifted_centerline_from_left(&shift_x, &shift_y, &shift_num);
        copy_point_set(shift_x, shift_y, shift_num, &result->centerline_selected_shift);
        vision_image_processor_get_src_shifted_centerline_from_left(&shift_x, &shift_y, &shift_num);
        copy_point_set(shift_x, shift_y, shift_num, &result->src_centerline_selected_shift);
    }

    std::snprintf(result->status_message,
                  sizeof(result->status_message),
                  "WASM bridge: real algorithm executed (w=%d h=%d gray=%d otsu=%d line_error=%d).",
                  frame->width,
                  frame->height,
                  frame->is_gray ? 1 : 0,
                  static_cast<int>(vision_image_processor_get_last_otsu_threshold()),
                  result->line_error);
    return true;
}

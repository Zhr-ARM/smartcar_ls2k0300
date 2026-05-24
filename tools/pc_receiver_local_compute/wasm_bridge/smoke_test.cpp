#include "web_vision_bridge.h"

#include <cstdio>
#include <vector>

int main()
{
    std::vector<unsigned char> bgr(160 * 120 * 3, 0);

    web_vision_frame_t frame = {};
    frame.width = 160;
    frame.height = 120;
    frame.data = bgr.data();
    frame.data_size = bgr.size();
    frame.is_gray = false;

    web_vision_raw_status_t status = {};
    status.base_speed = 120.0f;
    status.adjusted_base_speed = 115.0f;
    status.otsu_threshold = 127;
    status.ts_ms = 1;

    web_vision_result_t result = {};
    const bool ok = web_vision_bridge_process(&frame, &status, &result);

    std::printf("ok=%d valid=%d line_error=%d track_valid=%d msg=%s\n",
                ok ? 1 : 0,
                result.valid ? 1 : 0,
                result.line_error,
                result.ipm_track_valid ? 1 : 0,
                result.status_message);
    std::printf("left=%d right=%d center=%d center_curv=%d left_angle=%d\n",
                result.left_boundary.count,
                result.right_boundary.count,
                result.centerline_selected_shift.count,
                result.centerline_curvature.count,
                result.left_boundary_angle_cos.count);
    return ok ? 0 : 1;
}

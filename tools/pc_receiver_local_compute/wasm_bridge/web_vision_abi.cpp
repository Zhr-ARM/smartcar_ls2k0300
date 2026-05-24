#include "web_vision_bridge.h"

#include <cstdio>
#include <string>

namespace
{
web_vision_result_t g_last_result = {};
std::string g_last_result_json = "{}";

void append_json_escaped(std::string &out, const char *text)
{
    out += "\"";
    if (text != nullptr)
    {
        for (const char *p = text; *p != '\0'; ++p)
        {
            const char ch = *p;
            if (ch == '\\' || ch == '"')
            {
                out += '\\';
                out += ch;
            }
            else if (ch == '\n')
            {
                out += "\\n";
            }
            else
            {
                out += ch;
            }
        }
    }
    out += "\"";
}

void append_point_set(std::string &out, const web_vision_point_set_t &pts)
{
    out += "[";
    for (int i = 0; i < pts.count; ++i)
    {
        if (i > 0) out += ",";
        out += "[";
        out += std::to_string(pts.x[i]);
        out += ",";
        out += std::to_string(pts.y[i]);
        out += "]";
    }
    out += "]";
}

void rebuild_last_result_json()
{
    std::string out;
    out.reserve(32768);
    out += "{";

    auto append_key = [&out](const char *name) {
        if (out.size() > 1) out += ",";
        out += "\"";
        out += name;
        out += "\":";
    };

    append_key("valid");
    out += g_last_result.valid ? "true" : "false";
    append_key("line_error");
    out += std::to_string(g_last_result.line_error);
    append_key("ipm_track_valid");
    out += g_last_result.ipm_track_valid ? "true" : "false";
    append_key("ipm_track_index");
    out += std::to_string(g_last_result.ipm_track_index);
    append_key("ipm_track_point");
    out += "[";
    out += std::to_string(g_last_result.ipm_track_x);
    out += ",";
    out += std::to_string(g_last_result.ipm_track_y);
    out += "]";

    append_key("leftBoundary");
    append_point_set(out, g_last_result.left_boundary);
    append_key("rightBoundary");
    append_point_set(out, g_last_result.right_boundary);
    append_key("ipmLeftBoundary");
    append_point_set(out, g_last_result.ipm_left_boundary);
    append_key("ipmRightBoundary");
    append_point_set(out, g_last_result.ipm_right_boundary);
    append_key("centerline");
    append_point_set(out, g_last_result.src_centerline_selected_shift);
    append_key("ipmCenterline");
    append_point_set(out, g_last_result.centerline_selected_shift);

    append_key("summary");
    append_json_escaped(out, g_last_result.status_message);
    append_key("status_message");
    append_json_escaped(out, g_last_result.status_message);

    out += "}";
    g_last_result_json.swap(out);
}
} // namespace

extern "C" {

int web_vision_abi_process_bgr(const unsigned char *bgr,
                               int width,
                               int height,
                               int is_gray,
                               unsigned int data_size,
                               float left_current_count,
                               float right_current_count,
                               float left_target_count,
                               float right_target_count,
                               float left_filtered_count,
                               float right_filtered_count,
                               float base_speed,
                               float adjusted_base_speed,
                               int otsu_threshold,
                               long long ts_ms)
{
    web_vision_frame_t frame = {};
    frame.width = width;
    frame.height = height;
    frame.data = bgr;
    frame.data_size = static_cast<size_t>(data_size);
    frame.is_gray = (is_gray != 0);

    web_vision_raw_status_t status = {};
    status.left_current_count = left_current_count;
    status.right_current_count = right_current_count;
    status.left_target_count = left_target_count;
    status.right_target_count = right_target_count;
    status.left_filtered_count = left_filtered_count;
    status.right_filtered_count = right_filtered_count;
    status.base_speed = base_speed;
    status.adjusted_base_speed = adjusted_base_speed;
    status.otsu_threshold = otsu_threshold;
    status.ts_ms = ts_ms;

    const bool ok = web_vision_bridge_process(&frame, &status, &g_last_result);
    rebuild_last_result_json();
    return ok ? 1 : 0;
}

const char *web_vision_abi_last_result_json()
{
    return g_last_result_json.c_str();
}

const char *web_vision_abi_version()
{
    return "vision_web_bridge_abi_v1";
}

}

#include "driver/vision/vision_line_error_layer.h"

#include "driver/vision/vision_config.h"
#include "driver/vision/vision_image_processor.h"
#include "line_follow_thread.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <mutex>

namespace
{
std::atomic<int> g_ipm_line_error_source(static_cast<int>(g_vision_runtime_config.ipm_line_error_source));
std::atomic<int> g_ipm_line_error_method(static_cast<int>(g_vision_runtime_config.ipm_line_error_method));
std::atomic<int> g_ipm_line_error_fixed_index(g_vision_runtime_config.ipm_line_error_fixed_index);
std::atomic<float> g_ipm_line_error_speed_k(g_vision_runtime_config.ipm_line_error_speed_k);
std::atomic<float> g_ipm_line_error_speed_b(g_vision_runtime_config.ipm_line_error_speed_b);
std::atomic<int> g_ipm_line_error_index_min(g_vision_runtime_config.ipm_line_error_index_min);
std::atomic<int> g_ipm_line_error_index_max(g_vision_runtime_config.ipm_line_error_index_max);
std::atomic<bool> g_centerline_curvature_enabled(g_vision_runtime_config.ipm_centerline_curvature_enabled);

std::mutex g_ipm_line_error_weighted_points_mutex;
std::array<int, VISION_LINE_ERROR_MAX_WEIGHTED_POINTS> g_ipm_line_error_point_indices = {
    g_vision_runtime_config.ipm_line_error_point_indices[0],
    g_vision_runtime_config.ipm_line_error_point_indices[1],
    g_vision_runtime_config.ipm_line_error_point_indices[2]
};
std::array<float, VISION_LINE_ERROR_MAX_WEIGHTED_POINTS> g_ipm_line_error_weights = {
    g_vision_runtime_config.ipm_line_error_weights[0],
    g_vision_runtime_config.ipm_line_error_weights[1],
    g_vision_runtime_config.ipm_line_error_weights[2]
};
size_t g_ipm_line_error_weighted_point_count = g_vision_runtime_config.ipm_line_error_weighted_point_count;

std::atomic<int> g_ipm_line_error_weighted_first_point_error(0);
std::atomic<int> g_centerline_curvature_step(std::max(1, g_vision_runtime_config.ipm_centerline_curvature_step));

bool g_ipm_weighted_decision_point_valid = false;
int g_ipm_weighted_decision_point_x = 0;
int g_ipm_weighted_decision_point_y = 0;
bool g_src_weighted_decision_point_valid = false;
int g_src_weighted_decision_point_x = 0;
int g_src_weighted_decision_point_y = 0;

bool g_ipm_line_error_track_valid = false;
int g_ipm_line_error_track_index = -1;
int g_ipm_line_error_track_x = 0;
int g_ipm_line_error_track_y = 0;
std::array<float, VISION_MAX_PROC_HEIGHT * 2> g_selected_centerline_curvature = {};
int g_selected_centerline_curvature_count = 0;
float g_mean_abs_offset = 0.0f;

static void compute_selected_centerline_curvature(const uint16 *xs, const uint16 *ys, int count)
{
    g_selected_centerline_curvature.fill(0.0f);
    g_selected_centerline_curvature_count = 0;
    if (!g_centerline_curvature_enabled.load() || xs == nullptr || ys == nullptr || count <= 0)
    {
        return;
    }

    const int n = std::clamp(count, 0, static_cast<int>(g_selected_centerline_curvature.size()));
    g_selected_centerline_curvature_count = n;
    const int span = std::max(1, g_centerline_curvature_step.load());
    if (n < 2)
    {
        return;
    }

    const float h = 1.0f;
    const float inv_2sh = 1.0f / (2.0f * static_cast<float>(span) * h);
    const float inv_sh2 = 1.0f / (static_cast<float>(span * span) * h * h);

    for (int i = 0; i < n; ++i)
    {
        const int im = std::clamp(i - span, 0, n - 1);
        const int ip = std::clamp(i + span, 0, n - 1);
        if (im == ip)
        {
            continue;
        }

        const float xm = static_cast<float>(xs[im]);
        const float x0 = static_cast<float>(xs[i]);
        const float xp = static_cast<float>(xs[ip]);
        const float ym = static_cast<float>(ys[im]);
        const float y0 = static_cast<float>(ys[i]);
        const float yp = static_cast<float>(ys[ip]);

        const float x1 = (xp - xm) * inv_2sh;
        const float y1 = (yp - ym) * inv_2sh;
        const float x2 = (xp - 2.0f * x0 + xm) * inv_sh2;
        const float y2 = (yp - 2.0f * y0 + ym) * inv_sh2;

        g_selected_centerline_curvature[i] = x1 * y2 - x2 * y1;
    }

    std::array<float, VISION_MAX_PROC_HEIGHT * 2> curvature_src = g_selected_centerline_curvature;
    for (int i = 1; i + 1 < n; ++i)
    {
        g_selected_centerline_curvature[i] = (curvature_src[i - 1] +
                                              2.0f * curvature_src[i] +
                                              curvature_src[i + 1]) * 0.25f;
    }
}

static void reset_ipm_line_error_weighted_points_to_default()
{
    const std::lock_guard<std::mutex> lock(g_ipm_line_error_weighted_points_mutex);
    g_ipm_line_error_point_indices.fill(0);
    g_ipm_line_error_weights.fill(0.0f);
    for (size_t i = 0; i < g_vision_runtime_config.ipm_line_error_weighted_point_count; ++i)
    {
        g_ipm_line_error_point_indices[i] = std::max(0, g_vision_runtime_config.ipm_line_error_point_indices[i]);
        g_ipm_line_error_weights[i] = g_vision_runtime_config.ipm_line_error_weights[i];
    }
    g_ipm_line_error_weighted_point_count = g_vision_runtime_config.ipm_line_error_weighted_point_count;
}

} // namespace

void vision_line_error_layer_reset()
{
    g_ipm_line_error_source.store(static_cast<int>(g_vision_runtime_config.ipm_line_error_source));
    g_ipm_line_error_method.store(static_cast<int>(g_vision_runtime_config.ipm_line_error_method));
    g_ipm_line_error_fixed_index.store(g_vision_runtime_config.ipm_line_error_fixed_index);
    g_ipm_line_error_speed_k.store(g_vision_runtime_config.ipm_line_error_speed_k);
    g_ipm_line_error_speed_b.store(g_vision_runtime_config.ipm_line_error_speed_b);
    g_ipm_line_error_index_min.store(g_vision_runtime_config.ipm_line_error_index_min);
    g_ipm_line_error_index_max.store(g_vision_runtime_config.ipm_line_error_index_max);
    reset_ipm_line_error_weighted_points_to_default();
    g_ipm_line_error_weighted_first_point_error.store(0);
    g_centerline_curvature_enabled.store(g_vision_runtime_config.ipm_centerline_curvature_enabled);
    g_centerline_curvature_step.store(std::max(1, g_vision_runtime_config.ipm_centerline_curvature_step));
    g_ipm_weighted_decision_point_valid = false;
    g_ipm_weighted_decision_point_x = 0;
    g_ipm_weighted_decision_point_y = 0;
    g_src_weighted_decision_point_valid = false;
    g_src_weighted_decision_point_x = 0;
    g_src_weighted_decision_point_y = 0;
    g_ipm_line_error_track_valid = false;
    g_ipm_line_error_track_index = -1;
    g_ipm_line_error_track_x = 0;
    g_ipm_line_error_track_y = 0;
    g_selected_centerline_curvature.fill(0.0f);
    g_selected_centerline_curvature_count = 0;
    g_mean_abs_offset = 0.0f;
}

int vision_line_error_layer_compute_from_ipm_shifted_centerline(const uint16 *ipm_center_x,
                                                                 const uint16 *ipm_center_y,
                                                                 int ipm_center_count,
                                                                 const uint16 *src_center_x,
                                                                 const uint16 *src_center_y,
                                                                 int src_center_count,
                                                                 int ipm_center_x_ref)
{
    g_ipm_line_error_track_valid = false;
    g_ipm_line_error_track_index = -1;
    g_ipm_line_error_track_x = 0;
    g_ipm_line_error_track_y = 0;
    g_ipm_weighted_decision_point_valid = false;
    g_ipm_weighted_decision_point_x = 0;
    g_ipm_weighted_decision_point_y = 0;
    g_src_weighted_decision_point_valid = false;
    g_src_weighted_decision_point_x = 0;
    g_src_weighted_decision_point_y = 0;

    const int center_count = std::clamp(ipm_center_count, 0, static_cast<int>(g_selected_centerline_curvature.size()));
    if (center_count <= 0)
    {
        return 0;
    }
    const uint16 *xs = ipm_center_x;
    const uint16 *ys = ipm_center_y;
    const int count = center_count;
    const uint16 *src_xs = src_center_x;
    const uint16 *src_ys = src_center_y;
    const int src_count = std::max(0, src_center_count);
    if (xs == nullptr || ys == nullptr || count <= 0)
    {
        g_selected_centerline_curvature.fill(0.0f);
        g_selected_centerline_curvature_count = 0;
        g_mean_abs_offset = 0.0f;
        return 0;
    }

    {
        double sum_abs = 0.0;
        for (int i = 0; i < count; ++i)
        {
            sum_abs += std::fabs(static_cast<double>(xs[i]) - static_cast<double>(ipm_center_x_ref));
        }
        g_mean_abs_offset = (count > 0) ? static_cast<float>(sum_abs / static_cast<double>(count)) : 0.0f;
    }

    compute_selected_centerline_curvature(xs, ys, count);

    float x = 0.0f;
    float y = 0.0f;
    const vision_ipm_line_error_method_enum method =
        static_cast<vision_ipm_line_error_method_enum>(g_ipm_line_error_method.load());
    if (method == VISION_IPM_LINE_ERROR_FIXED_INDEX)
    {
        const int idx = std::clamp(g_ipm_line_error_fixed_index.load(), 0, count - 1);
        g_ipm_line_error_track_index = idx;
        x = static_cast<float>(xs[idx]);
        y = static_cast<float>(ys[idx]);
    }
    else if (method == VISION_IPM_LINE_ERROR_SPEED_INDEX)
    {
        const float speed_k = g_ipm_line_error_speed_k.load();
        const float speed_b = g_ipm_line_error_speed_b.load();
        const float current_speed = line_follow_thread_applied_base_speed();
        const int cfg_index_min = g_ipm_line_error_index_min.load();
        const int cfg_index_max = g_ipm_line_error_index_max.load();
        const int range_min = std::min(cfg_index_min, cfg_index_max);
        const int range_max = std::max(cfg_index_min, cfg_index_max);
        const float idx_by_speed = speed_k * current_speed + speed_b;
        const int idx = std::clamp(static_cast<int>(std::lround(idx_by_speed)),
                                   range_min,
                                   std::min(range_max, count - 1));
        g_ipm_line_error_track_index = idx;
        x = static_cast<float>(xs[idx]);
        y = static_cast<float>(ys[idx]);
    }
    else
    {
        std::array<int, VISION_LINE_ERROR_MAX_WEIGHTED_POINTS> point_indices = {};
        std::array<float, VISION_LINE_ERROR_MAX_WEIGHTED_POINTS> weights = {};
        size_t point_count = 0;
        {
            const std::lock_guard<std::mutex> lock(g_ipm_line_error_weighted_points_mutex);
            point_indices = g_ipm_line_error_point_indices;
            weights = g_ipm_line_error_weights;
            point_count = g_ipm_line_error_weighted_point_count;
        }

        int decision_index = -1;
        for (size_t i = 0; i < point_count; ++i)
        {
            const int idx = point_indices[i];
            if (idx >= 0 && idx < count)
            {
                decision_index = idx;
                break;
            }
        }
        if (decision_index >= 0)
        {
            const int decision_point_error = static_cast<int>(xs[decision_index]) - ipm_center_x_ref;
            g_ipm_line_error_weighted_first_point_error.store(decision_point_error);
            // 动态间距模式已移除：原始加权模式固定使用配置索引点。
            g_ipm_weighted_decision_point_valid = true;
            g_ipm_weighted_decision_point_x = static_cast<int>(xs[decision_index]);
            g_ipm_weighted_decision_point_y = static_cast<int>(ys[decision_index]);
            if (src_xs != nullptr && src_ys != nullptr && decision_index < src_count)
            {
                g_src_weighted_decision_point_valid = true;
                g_src_weighted_decision_point_x = static_cast<int>(src_xs[decision_index]);
                g_src_weighted_decision_point_y = static_cast<int>(src_ys[decision_index]);
            }
        }
        else
        {
            g_ipm_line_error_weighted_first_point_error.store(0);
        }

        float total_weight = 0.0f;
        float valid_weight = 0.0f;
        for (size_t i = 0; i < point_count; ++i)
        {
            total_weight += weights[i];
            if (point_indices[i] >= 0 && point_indices[i] < count)
            {
                valid_weight += weights[i];
            }
        }
        if (point_count == 0 || std::fabs(valid_weight) <= 1e-6f)
        {
            return 0;
        }

        const float weight_scale = total_weight / valid_weight;
        float weighted_index = 0.0f;
        for (size_t i = 0; i < point_count; ++i)
        {
            const int idx = point_indices[i];
            if (idx < 0 || idx >= count)
            {
                continue;
            }
            const float effective_weight = weights[i] * weight_scale;
            weighted_index += static_cast<float>(idx) * effective_weight;
            x += static_cast<float>(xs[idx]) * effective_weight;
            y += static_cast<float>(ys[idx]) * effective_weight;
        }
        g_ipm_line_error_track_index = std::clamp(static_cast<int>(std::lround(weighted_index)), 0, count - 1);
    }

    g_ipm_line_error_track_valid = true;
    g_ipm_line_error_track_x = static_cast<int>(std::lround(x));
    g_ipm_line_error_track_y = static_cast<int>(std::lround(y));
    return static_cast<int>(std::lround(x - static_cast<float>(ipm_center_x_ref)));
}

void vision_line_error_layer_set_source(int source)
{
    if (source == static_cast<int>(VISION_IPM_LINE_ERROR_FROM_LEFT_SHIFT) ||
        source == static_cast<int>(VISION_IPM_LINE_ERROR_FROM_RIGHT_SHIFT) ||
        source == static_cast<int>(VISION_IPM_LINE_ERROR_FROM_AUTO))
    {
        g_ipm_line_error_source.store(source);
        return;
    }
    g_ipm_line_error_source.store(static_cast<int>(VISION_IPM_LINE_ERROR_FROM_LEFT_SHIFT));
}

int vision_line_error_layer_source()
{
    return g_ipm_line_error_source.load();
}

void vision_line_error_layer_set_method(int method)
{
    if (method >= static_cast<int>(VISION_IPM_LINE_ERROR_FIXED_INDEX) &&
        method <= static_cast<int>(VISION_IPM_LINE_ERROR_SPEED_INDEX))
    {
        g_ipm_line_error_method.store(method);
        return;
    }
    g_ipm_line_error_method.store(static_cast<int>(VISION_IPM_LINE_ERROR_WEIGHTED_INDEX));
}

int vision_line_error_layer_method()
{
    return g_ipm_line_error_method.load();
}

void vision_line_error_layer_set_fixed_index(int point_index)
{
    g_ipm_line_error_fixed_index.store(std::max(0, point_index));
}

int vision_line_error_layer_fixed_index()
{
    return g_ipm_line_error_fixed_index.load();
}

void vision_line_error_layer_set_weighted_points(const int *point_indices,
                                                 const float *weights,
                                                 size_t count)
{
    if (point_indices == nullptr || weights == nullptr || count == 0)
    {
        reset_ipm_line_error_weighted_points_to_default();
        return;
    }

    const size_t clamped_count = std::min(count, static_cast<size_t>(VISION_LINE_ERROR_MAX_WEIGHTED_POINTS));
    const std::lock_guard<std::mutex> lock(g_ipm_line_error_weighted_points_mutex);
    g_ipm_line_error_point_indices.fill(0);
    g_ipm_line_error_weights.fill(0.0f);
    for (size_t i = 0; i < clamped_count; ++i)
    {
        g_ipm_line_error_point_indices[i] = std::max(0, point_indices[i]);
        g_ipm_line_error_weights[i] = weights[i];
    }
    g_ipm_line_error_weighted_point_count = clamped_count;
}

size_t vision_line_error_layer_weighted_point_count()
{
    const std::lock_guard<std::mutex> lock(g_ipm_line_error_weighted_points_mutex);
    return g_ipm_line_error_weighted_point_count;
}

void vision_line_error_layer_set_speed_formula(float speed_k, float speed_b)
{
    g_ipm_line_error_speed_k.store(speed_k);
    g_ipm_line_error_speed_b.store(speed_b);
}

void vision_line_error_layer_get_speed_formula(float *speed_k, float *speed_b)
{
    if (speed_k) *speed_k = g_ipm_line_error_speed_k.load();
    if (speed_b) *speed_b = g_ipm_line_error_speed_b.load();
}

void vision_line_error_layer_set_index_range(int index_min, int index_max)
{
    g_ipm_line_error_index_min.store(index_min);
    g_ipm_line_error_index_max.store(index_max);
}

void vision_line_error_layer_get_index_range(int *index_min, int *index_max)
{
    if (index_min) *index_min = g_ipm_line_error_index_min.load();
    if (index_max) *index_max = g_ipm_line_error_index_max.load();
}

void vision_line_error_layer_get_track_point(bool *valid, int *x, int *y)
{
    if (valid) *valid = g_ipm_line_error_track_valid;
    if (x) *x = g_ipm_line_error_track_x;
    if (y) *y = g_ipm_line_error_track_y;
}

int vision_line_error_layer_track_index()
{
    return g_ipm_line_error_track_index;
}

void vision_line_error_layer_set_curvature_step(int step)
{
    g_centerline_curvature_step.store(std::max(1, step));
}

int vision_line_error_layer_curvature_step()
{
    return g_centerline_curvature_step.load();
}

void vision_line_error_layer_set_curvature_enabled(bool enabled)
{
    g_centerline_curvature_enabled.store(enabled);
}

bool vision_line_error_layer_curvature_enabled()
{
    return g_centerline_curvature_enabled.load();
}

void vision_line_error_layer_get_selected_centerline_curvature(const float **curvature, int *count)
{
    if (curvature) *curvature = g_selected_centerline_curvature.data();
    if (count) *count = g_selected_centerline_curvature_count;
}

float vision_line_error_layer_mean_abs_offset()
{
    return g_mean_abs_offset;
}

int vision_line_error_layer_weighted_first_point_error()
{
    return g_ipm_line_error_weighted_first_point_error.load();
}

void vision_line_error_layer_get_ipm_weighted_decision_point(bool *valid, int *x, int *y)
{
    if (valid) *valid = g_ipm_weighted_decision_point_valid;
    if (x) *x = g_ipm_weighted_decision_point_x;
    if (y) *y = g_ipm_weighted_decision_point_y;
}

void vision_line_error_layer_get_src_weighted_decision_point(bool *valid, int *x, int *y)
{
    if (valid) *valid = g_src_weighted_decision_point_valid;
    if (x) *x = g_src_weighted_decision_point_x;
    if (y) *y = g_src_weighted_decision_point_y;
}

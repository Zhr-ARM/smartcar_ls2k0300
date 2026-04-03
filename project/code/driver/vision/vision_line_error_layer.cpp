#include "driver/vision/vision_line_error_layer.h"

#include "driver/vision/vision_config.h"
#include "driver/vision/vision_image_processor.h"
#include "line_follow_thread.h"
#include "motor_thread.h"

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
std::atomic<int> g_ipm_line_error_weighted_current_spacing(1);
std::atomic<int> g_centerline_curvature_step(std::max(1, g_vision_runtime_config.ipm_centerline_curvature_step));
std::atomic<float> g_curvature_lookahead_lambda(std::max(0.0f, g_vision_runtime_config.ipm_curvature_lookahead_lambda));
std::atomic<float> g_curvature_lookahead_mu(std::max(0.0f, g_vision_runtime_config.ipm_curvature_lookahead_mu));
constexpr float kCurvatureWeightedSigma = 3.0f;

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
std::array<float, VISION_DOWNSAMPLED_HEIGHT * 2> g_selected_centerline_curvature = {};
int g_selected_centerline_curvature_count = 0;
float g_curvature_lookahead_speed_v = 0.0f;
float g_curvature_lookahead_k_eff = 0.0f;
float g_curvature_lookahead_eta = 0.0f;
int g_curvature_lookahead_index = -1;
float g_curvature_weighted_error = 0.0f;
bool g_curvature_lookahead_point_valid = false;
int g_curvature_lookahead_point_x = 0;
int g_curvature_lookahead_point_y = 0;
float g_curvature_kappa_max = 0.0f;
float g_curvature_delta_kappa_max = 0.0f;
float g_curvature_base_speed_curve = 0.0f;
float g_curvature_v_curve_raw = 0.0f;
float g_curvature_v_curve_after_dkappa = 0.0f;
float g_curvature_v_error_limit = 0.0f;
float g_curvature_v_target = 0.0f;
float g_mean_abs_offset = 0.0f;

static void compute_selected_centerline_curvature(const uint16 *xs, const uint16 *ys, int count)
{
    g_selected_centerline_curvature.fill(0.0f);
    g_selected_centerline_curvature_count = std::clamp(count, 0, static_cast<int>(g_selected_centerline_curvature.size()));
    if (xs == nullptr || ys == nullptr || count <= 0)
    {
        return;
    }

    const int step = std::max(1, g_centerline_curvature_step.load());
    const int start_idx = step;
    const int end_idx = count - step;
    if (start_idx >= end_idx)
    {
        return;
    }

    for (int i = start_idx; i < end_idx; ++i)
    {
        const int i0 = i - step;
        const int i1 = i;
        const int i2 = i + step;

        const double x0 = static_cast<double>(xs[i0]);
        const double y0 = static_cast<double>(ys[i0]);
        const double x1 = static_cast<double>(xs[i1]);
        const double y1 = static_cast<double>(ys[i1]);
        const double x2 = static_cast<double>(xs[i2]);
        const double y2 = static_cast<double>(ys[i2]);

        const double ax = x1 - x0;
        const double ay = y1 - y0;
        const double bx = x2 - x1;
        const double by = y2 - y1;

        const double len_a = std::sqrt(ax * ax + ay * ay);
        const double len_b = std::sqrt(bx * bx + by * by);
        const double cx = x2 - x0;
        const double cy = y2 - y0;
        const double len_c = std::sqrt(cx * cx + cy * cy);
        const double denom = len_a * len_b * len_c;
        if (denom <= 1e-9)
        {
            g_selected_centerline_curvature[i] = 0.0f;
            continue;
        }

        const double cross = ax * by - ay * bx;
        const double curvature = (2.0 * cross) / denom;
        g_selected_centerline_curvature[i] = static_cast<float>(curvature);
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

static void compute_curvature_lookahead_index_and_weighted_error(const uint16 *xs, int count, int ipm_center_x_ref)
{
    g_curvature_lookahead_speed_v = 0.0f;
    g_curvature_lookahead_k_eff = 0.0f;
    g_curvature_lookahead_eta = 0.0f;
    g_curvature_lookahead_index = -1;
    g_curvature_weighted_error = 0.0f;
    g_curvature_lookahead_point_valid = false;
    g_curvature_lookahead_point_x = 0;
    g_curvature_lookahead_point_y = 0;
    if (xs == nullptr || count <= 0)
    {
        return;
    }

    const float v_l = motor_thread_left_count();
    const float v_r = motor_thread_right_count();
    const float v = 0.5f * (v_l + v_r);
    g_curvature_lookahead_speed_v = v;

    const float lambda = std::max(0.0f, g_curvature_lookahead_lambda.load());
    double sum_weight = 0.0;
    double sum_weighted_abs_k = 0.0;
    const int curvature_count = std::clamp(g_selected_centerline_curvature_count, 0, count);
    for (int i = 0; i < curvature_count; ++i)
    {
        const double weight = std::exp(-static_cast<double>(lambda) * static_cast<double>(i));
        sum_weight += weight;
        sum_weighted_abs_k += std::abs(static_cast<double>(g_selected_centerline_curvature[i])) * weight;
    }

    const float k_eff = (sum_weight > 1e-9) ? static_cast<float>(sum_weighted_abs_k / sum_weight) : 0.0f;
    g_curvature_lookahead_k_eff = k_eff;

    const float mu = std::max(0.0f, g_curvature_lookahead_mu.load());
    const float v_max = std::max(1e-3f, std::fabs(line_follow_thread_base_speed()));
    const float eta = std::clamp((v / v_max) * std::exp(-mu * k_eff), 0.0f, 1.0f);
    g_curvature_lookahead_eta = eta;

    const int cfg_index_min = g_ipm_line_error_index_min.load();
    const int cfg_index_max = g_ipm_line_error_index_max.load();
    const int range_min = std::clamp(std::min(cfg_index_min, cfg_index_max), 0, count - 1);
    const int range_max = std::clamp(std::max(cfg_index_min, cfg_index_max), 0, count - 1);
    const float i_center = static_cast<float>(range_min) + eta * static_cast<float>(range_max - range_min);
    g_curvature_lookahead_index = std::clamp(static_cast<int>(std::lround(i_center)), range_min, range_max);

    const float sigma = std::max(1e-3f, kCurvatureWeightedSigma);
    const float denom_sigma = 2.0f * sigma * sigma;
    double sum_w = 0.0;
    double sum_w_offset = 0.0;
    for (int i = range_min; i <= range_max; ++i)
    {
        const float d = static_cast<float>(i) - i_center;
        const float w = std::exp(-(d * d) / denom_sigma);
        const float offset = static_cast<float>(xs[i]) - static_cast<float>(ipm_center_x_ref);
        sum_w += w;
        sum_w_offset += static_cast<double>(w) * static_cast<double>(offset);
    }
    if (sum_w > 1e-9)
    {
        g_curvature_weighted_error = static_cast<float>(sum_w_offset / sum_w);
    }
}

static void compute_curvature_based_speed_limit(int count)
{
    g_curvature_kappa_max = 0.0f;
    g_curvature_delta_kappa_max = 0.0f;
    g_curvature_base_speed_curve = 0.0f;
    g_curvature_v_curve_raw = 0.0f;
    g_curvature_v_curve_after_dkappa = 0.0f;
    g_curvature_v_error_limit = 0.0f;
    g_curvature_v_target = 0.0f;
    if (count <= 0)
    {
        return;
    }

    const float v_max_global = std::max(0.0f, std::fabs(line_follow_thread_base_speed()));
    const float v_min_global = std::max(0.0f, g_vision_runtime_config.ipm_curve_speed_v_min_global);
    const float v_low = std::min(v_min_global, v_max_global);
    const float v_high = std::max(v_min_global, v_max_global);

    const int cfg_index_min = g_ipm_line_error_index_min.load();
    const int cfg_index_max = g_ipm_line_error_index_max.load();
    const int idx_min = std::clamp(std::min(cfg_index_min, cfg_index_max), 0, count - 1);
    const int idx_max = std::clamp(std::max(cfg_index_min, cfg_index_max), 0, count - 1);
    const int cur_idx = idx_min;
    const int look_idx = std::clamp(g_curvature_lookahead_index, idx_min, idx_max);
    const int extra = std::max(0, g_vision_runtime_config.ipm_curve_speed_extra_plan_points);
    const int start_idx = std::clamp(cur_idx, 0, count - 1);
    const int end_idx = std::clamp(look_idx + extra, 0, count - 1);
    if (end_idx <= start_idx)
    {
        return;
    }

    float kappa_max = 0.0f;
    float delta_kappa_max = 0.0f;
    for (int i = start_idx + 1; i <= end_idx; ++i)
    {
        const float k_raw = g_selected_centerline_curvature[i];
        kappa_max = std::max(kappa_max, std::fabs(k_raw));
        delta_kappa_max = std::max(delta_kappa_max, std::fabs(k_raw - g_selected_centerline_curvature[i - 1]));
    }

    const float ay_allow = std::max(0.0f, g_vision_runtime_config.ipm_curve_speed_ay_allow);
    const float eps = std::max(1e-6f, g_vision_runtime_config.ipm_curve_speed_kappa_epsilon);
    const float k_delta = std::max(0.0f, g_vision_runtime_config.ipm_curve_speed_delta_kappa_gain);
    const float speed_gain = std::max(0.0f, g_vision_runtime_config.ipm_curve_speed_gain);
    const float v_curve_raw = std::sqrt(std::max(0.0f, ay_allow / (kappa_max + eps))) * speed_gain;
    const float v_curve_after_dkappa = v_curve_raw / (1.0f + k_delta * delta_kappa_max);
    const float v_curve = std::clamp(v_curve_after_dkappa, v_low, v_high);

    const float k_error = std::max(0.0f, g_vision_runtime_config.ipm_curve_speed_error_gain);
    const float error_deadband = std::max(0.0f, g_vision_runtime_config.ipm_curve_speed_error_deadband);
    float ae = std::fabs(g_curvature_weighted_error);
    float v_error_limit = v_high;
    if (ae > error_deadband)
    {
        ae -= error_deadband;
        v_error_limit = v_high * (1.0f - k_error * ae);
    }
    v_error_limit = std::clamp(v_error_limit, v_low, v_high);

    const float v_target = std::clamp(std::min(v_curve, v_error_limit), v_low, v_high);

    g_curvature_kappa_max = kappa_max;
    g_curvature_delta_kappa_max = delta_kappa_max;
    g_curvature_base_speed_curve = std::max(0.0f, v_target);
    g_curvature_v_curve_raw = std::max(0.0f, v_curve_raw);
    g_curvature_v_curve_after_dkappa = std::max(0.0f, v_curve_after_dkappa);
    g_curvature_v_error_limit = std::max(0.0f, v_error_limit);
    g_curvature_v_target = std::max(0.0f, v_target);
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
    g_ipm_line_error_weighted_current_spacing.store(1);
    g_centerline_curvature_step.store(std::max(1, g_vision_runtime_config.ipm_centerline_curvature_step));
    g_curvature_lookahead_lambda.store(std::max(0.0f, g_vision_runtime_config.ipm_curvature_lookahead_lambda));
    g_curvature_lookahead_mu.store(std::max(0.0f, g_vision_runtime_config.ipm_curvature_lookahead_mu));
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
    g_curvature_lookahead_speed_v = 0.0f;
    g_curvature_lookahead_k_eff = 0.0f;
    g_curvature_lookahead_eta = 0.0f;
    g_curvature_lookahead_index = -1;
    g_curvature_weighted_error = 0.0f;
    g_curvature_lookahead_point_valid = false;
    g_curvature_lookahead_point_x = 0;
    g_curvature_lookahead_point_y = 0;
    g_curvature_kappa_max = 0.0f;
    g_curvature_delta_kappa_max = 0.0f;
    g_curvature_base_speed_curve = 0.0f;
    g_curvature_v_curve_raw = 0.0f;
    g_curvature_v_curve_after_dkappa = 0.0f;
    g_curvature_v_error_limit = 0.0f;
    g_curvature_v_target = 0.0f;
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
    compute_curvature_lookahead_index_and_weighted_error(xs, count, ipm_center_x_ref);
    compute_curvature_based_speed_limit(count);
    if (g_curvature_lookahead_index >= 0 && g_curvature_lookahead_index < count)
    {
        g_curvature_lookahead_point_valid = true;
        g_curvature_lookahead_point_x = static_cast<int>(xs[g_curvature_lookahead_index]);
        g_curvature_lookahead_point_y = static_cast<int>(ys[g_curvature_lookahead_index]);
    }

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
        const float current_speed = line_follow_thread_base_speed();
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
            g_ipm_line_error_weighted_current_spacing.store(1);
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
            g_ipm_line_error_weighted_current_spacing.store(1);
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
    if (source == static_cast<int>(VISION_IPM_LINE_ERROR_FROM_RIGHT_SHIFT))
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

void vision_line_error_layer_get_selected_centerline_curvature(const float **curvature, int *count)
{
    if (curvature) *curvature = g_selected_centerline_curvature.data();
    if (count) *count = g_selected_centerline_curvature_count;
}

void vision_line_error_layer_get_curvature_lookahead_debug(float *speed_v,
                                                           float *k_eff,
                                                           float *eta,
                                                           int *lookahead_index)
{
    if (speed_v) *speed_v = g_curvature_lookahead_speed_v;
    if (k_eff) *k_eff = g_curvature_lookahead_k_eff;
    if (eta) *eta = g_curvature_lookahead_eta;
    if (lookahead_index) *lookahead_index = g_curvature_lookahead_index;
}

void vision_line_error_layer_get_curvature_weighted_error_debug(float *weighted_error,
                                                                bool *lookahead_point_valid,
                                                                int *lookahead_point_x,
                                                                int *lookahead_point_y)
{
    if (weighted_error) *weighted_error = g_curvature_weighted_error;
    if (lookahead_point_valid) *lookahead_point_valid = g_curvature_lookahead_point_valid;
    if (lookahead_point_x) *lookahead_point_x = g_curvature_lookahead_point_x;
    if (lookahead_point_y) *lookahead_point_y = g_curvature_lookahead_point_y;
}

void vision_line_error_layer_get_curvature_speed_limit_debug(float *kappa_max,
                                                             float *delta_kappa_max,
                                                             float *curve_base_speed,
                                                             float *v_curve_raw,
                                                             float *v_curve_after_dkappa,
                                                             float *v_error_limit,
                                                             float *v_target)
{
    if (kappa_max) *kappa_max = g_curvature_kappa_max;
    if (delta_kappa_max) *delta_kappa_max = g_curvature_delta_kappa_max;
    if (curve_base_speed) *curve_base_speed = g_curvature_base_speed_curve;
    if (v_curve_raw) *v_curve_raw = g_curvature_v_curve_raw;
    if (v_curve_after_dkappa) *v_curve_after_dkappa = g_curvature_v_curve_after_dkappa;
    if (v_error_limit) *v_error_limit = g_curvature_v_error_limit;
    if (v_target) *v_target = g_curvature_v_target;
}

float vision_line_error_layer_mean_abs_offset()
{
    return g_mean_abs_offset;
}

int vision_line_error_layer_weighted_first_point_error()
{
    return g_ipm_line_error_weighted_first_point_error.load();
}

int vision_line_error_layer_weighted_current_spacing()
{
    return g_ipm_line_error_weighted_current_spacing.load();
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

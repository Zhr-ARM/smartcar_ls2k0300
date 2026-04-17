#include "line_follow_thread.h"

#include "imu_thread.h"
#include "app/beep_thread/beep_thread.h"
#include "driver/vision/vision_config.h"
#include "driver/vision/vision_image_processor.h"
#include "driver/vision/vision_route_state_machine.h"
#include "motor_thread.h"
#include "pid.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <pthread.h>
#include <sys/syscall.h>
#include <thread>

namespace
{
// 控制周期：1ms(1000Hz)。巡线环频率高于电机速度环(5ms)的整数倍，能持续给出平滑转向目标。
constexpr int32 LINE_FOLLOW_PERIOD_MS = 20;
// 调度优先级：巡线线程作为中高优先级实时任务执行。
constexpr int32 LINE_FOLLOW_THREAD_PRIORITY = 8;
constexpr int32 LINE_FOLLOW_MAIN_STATE_SWITCH_BEEP_MS = 200;
constexpr float LINE_FOLLOW_LOOP_DT_SECONDS = LINE_FOLLOW_PERIOD_MS / 1000.0f;
constexpr float IMU_NOMINAL_DT_SECONDS = 0.005f;
constexpr float VISION_NOMINAL_DT_SECONDS = 1.0f / 60.0f;
constexpr float IMU_MAX_DT_SECONDS = 0.050f;
constexpr float VISION_MAX_DT_SECONDS = 0.200f;
constexpr float PID_MAX_DT_SECONDS = 0.200f;
constexpr float RAD_TO_DEG = 180.0f / 3.1415926f;

using RouteProfile = pid_tuning::route_line_follow::Profile;

std::thread g_line_follow_thread;
std::atomic<bool> g_line_follow_running(false);
std::atomic<int32> g_thread_tid(0);
std::atomic<int32> g_thread_policy(0);
std::atomic<int32> g_thread_priority(0);
std::atomic<float> g_normal_speed_reference(pid_tuning::route_line_follow::kNormalProfile.base_speed);
std::atomic<float> g_line_error_px(0.0f);
std::atomic<float> g_turn_output(0.0f);
std::atomic<bool> g_reload_from_globals_requested(false);

// 滤波后的归一化误差状态，跨周期保留。
// 这里刻意保留“状态记忆”，因为巡线不是单次运算，而是连续控制。
// 注意：状态只在“拿到新视觉帧”时推进一次，不会在 1ms 空循环里重复吃旧帧。
float g_filtered_error = 0.0f;
// 滤波后的横摆角速度状态：把 IMU 的瞬时抖动再收一层，减轻差速来回抽动。
// 注意：状态只在“拿到新 IMU 样本”时推进一次。
float g_filtered_yaw_rate_dps = 0.0f;
// 滤波后的目标点夹角：由跟踪点相对图像中垂线的偏转角得到。
// 注意：状态只在“拿到新视觉帧”时推进一次。
float g_filtered_track_point_angle_deg = 0.0f;
// 当前真正参与左右轮目标合成的基础速度状态：把“理想基础速度”做成缓增缓降，避免一帧一跳。
float g_applied_base_speed_state = -1.0f;
// 位置环与角速度环各自最近一次真实更新后的输出，样本未更新时沿用旧值。
float g_position_output_state = 0.0f;
float g_yaw_rate_output_state = 0.0f;
// 位置环用于动态Kd调度的误差变化率状态：只在拿到新视觉帧时推进一次。
float g_position_error_rate_px_per_second = 0.0f;
float g_last_position_control_error_px = 0.0f;
bool g_has_last_position_control_error = false;
// 最近一次已消费的 IMU / 视觉样本序号。
uint32 g_last_imu_sample_seq = 0;
uint32 g_last_vision_frame_seq = 0;
bool g_has_last_imu_update_time = false;
bool g_has_last_vision_update_time = false;
bool g_has_last_position_pid_time = false;
bool g_has_last_yaw_rate_pid_time = false;
std::chrono::steady_clock::time_point g_last_imu_update_time;
std::chrono::steady_clock::time_point g_last_vision_update_time;
std::chrono::steady_clock::time_point g_last_position_pid_time;
std::chrono::steady_clock::time_point g_last_yaw_rate_pid_time;
bool g_has_last_logged_route_state = false;
int g_last_logged_route_main_state = VISION_ROUTE_MAIN_NORMAL;
int g_last_logged_route_sub_state = VISION_ROUTE_SUB_NONE;
std::mutex g_pid_debug_mutex;
LineFollowPidDebugStatus g_pid_debug_status{};

struct RouteProfileSelection
{
    const RouteProfile &profile;
    const char *name;
};

struct ControlErrorState
{
    float filtered_error_px;
    float abs_filtered_error_px;
    float control_error_px;
};

float apply_iir_filter(float previous_value, float current_value, float alpha)
{
    return previous_value * (1.0f - alpha) + current_value * alpha;
}

/**
 * @brief 按“误差绝对值的一次函数”动态调整增益，并做上下限保护。
 *
 * 设计目的：
 * - 小误差时保持较温和的基础增益，抑制抖动与来回修正；
 * - 大误差时自动提高增益，增强纠偏力度与收敛速度；
 * - 始终把结果限制在可控范围内，避免参数突增导致控制过激。
 *
 * 数学形式：
 *   gain = clamp(base_gain + a * |e|, min_gain, max_gain)
 * 其中 e 为参与当前控制器增益调度的实际误差量。
 *
 * 参数说明：
 * @param base_gain        基础增益（e=0 时的起始值）。
 * @param linear_a         一次项系数，决定“误差增大时增益提升”的速度。
 * @param min_gain         输出增益下限，防止增益过小导致响应迟钝。
 * @param max_gain         输出增益上限，防止增益过大引发振荡或过冲。
 * @param error_value      参与增益调度的误差值；位置环这里用像素误差，角速度环这里用 dps 误差。
 *
 * 关键点：
 * - 使用 |e| 后，正负误差得到相同增益幅度，保证左右转向调节“对称”；
 * - 增益随 |e| 线性上升，调参更直观；
 * - clamp 是最后一道保护，确保结果始终落在调参可接受区间。
 */
float compute_linear_abs_gain(float base_gain,
                              float linear_a,
                              float min_gain,
                              float max_gain,
                              float error_value)
{
    // 误差绝对值项：只关心误差大小，不区分正负方向。
    const float abs_error = std::fabs(error_value);

    // 一次增益律 + 限幅保护。
    return std::clamp(base_gain + linear_a * abs_error, min_gain, max_gain);
}

/**
 * @brief 三段连续分段动态增益：不同误差段用不同斜率，但段间保持连续。
 *
 * 数学形式：
 * gain = base
 *      + a1 * clamp(|e|, 0, t1)
 *      + a2 * clamp(|e|-t1, 0, t2-t1)
 *      + a3 * max(|e|-t2, 0)
 */
float compute_piecewise_linear_abs_gain(float base_gain,
                                        float low_a,
                                        float low_threshold,
                                        float mid_a,
                                        float mid_threshold,
                                        float high_a,
                                        float min_gain,
                                        float max_gain,
                                        float error_value)
{
    const float abs_error = std::fabs(error_value);
    const float clamped_low_threshold = std::max(low_threshold, 0.0f);
    const float clamped_mid_threshold = std::max(mid_threshold, clamped_low_threshold);
    const float low_segment = std::clamp(abs_error, 0.0f, clamped_low_threshold);
    const float mid_segment =
        std::clamp(abs_error - clamped_low_threshold, 0.0f, clamped_mid_threshold - clamped_low_threshold);
    const float high_segment = std::max(abs_error - clamped_mid_threshold, 0.0f);
    const float gain = base_gain +
                       low_a * low_segment +
                       mid_a * mid_segment +
                       high_a * high_segment;
    return std::clamp(gain, min_gain, max_gain);
}
// 把滤波后的像素误差转换成真正参与位置环控制的像素 control_error_px，
// 同时保留便于调试的绝对误差量。
ControlErrorState compute_control_error_state(float filtered_error_px)
{
    const float abs_filtered_error_px = std::fabs(filtered_error_px);
    float control_error_px = filtered_error_px;

    if (abs_filtered_error_px < pid_tuning::line_follow::kErrorDeadzonePx)
    {
        control_error_px = 0.0f;
    }
    else if (abs_filtered_error_px < pid_tuning::line_follow::kErrorLowGainLimitPx)
    {
        control_error_px *= pid_tuning::line_follow::kErrorLowGain;
    }

    return {filtered_error_px, abs_filtered_error_px, control_error_px};
}

void reset_line_follow_runtime_state()
{
    g_thread_tid = 0;
    g_thread_policy = 0;
    g_thread_priority = 0;
    g_line_error_px.store(0.0f);
    g_turn_output.store(0.0f);
    g_filtered_error = 0.0f;
    g_filtered_yaw_rate_dps = 0.0f;
    g_filtered_track_point_angle_deg = 0.0f;
    g_applied_base_speed_state = -1.0f;
    g_position_output_state = 0.0f;
    g_yaw_rate_output_state = 0.0f;
    g_position_error_rate_px_per_second = 0.0f;
    g_last_position_control_error_px = 0.0f;
    g_has_last_position_control_error = false;
    g_last_imu_sample_seq = 0;
    g_last_vision_frame_seq = 0;
    g_has_last_imu_update_time = false;
    g_has_last_vision_update_time = false;
    g_has_last_position_pid_time = false;
    g_has_last_yaw_rate_pid_time = false;
    g_has_last_logged_route_state = false;
    g_last_logged_route_main_state = VISION_ROUTE_MAIN_NORMAL;
    g_last_logged_route_sub_state = VISION_ROUTE_SUB_NONE;
    std::lock_guard<std::mutex> lock(g_pid_debug_mutex);
    g_pid_debug_status = {};
}

const char *sched_policy_name(int32 policy)
{
    switch (policy)
    {
        case SCHED_FIFO:  return "SCHED_FIFO";
        case SCHED_RR:    return "SCHED_RR";
        case SCHED_OTHER: return "SCHED_OTHER";
#ifdef SCHED_BATCH
        case SCHED_BATCH: return "SCHED_BATCH";
#endif
#ifdef SCHED_IDLE
        case SCHED_IDLE:  return "SCHED_IDLE";
#endif
        default:          return "UNKNOWN";
    }
}

const char *route_main_state_name(int route_main_state)
{
    switch (route_main_state)
    {
        case VISION_ROUTE_MAIN_NORMAL:       return "NORMAL";
        case VISION_ROUTE_MAIN_STRAIGHT:     return "STRAIGHT";
        case VISION_ROUTE_MAIN_CIRCLE_LEFT:  return "CIRCLE_LEFT";
        case VISION_ROUTE_MAIN_CIRCLE_RIGHT: return "CIRCLE_RIGHT";
        case VISION_ROUTE_MAIN_CROSS:        return "CROSS";
        default:                             return "UNKNOWN_MAIN";
    }
}

const char *route_sub_state_name(int route_sub_state)
{
    switch (route_sub_state)
    {
        case VISION_ROUTE_SUB_NONE:                 return "NONE";
        case VISION_ROUTE_SUB_CROSS_1:              return "CROSS_1";
        case VISION_ROUTE_SUB_CROSS_2:              return "CROSS_2";
        case VISION_ROUTE_SUB_CROSS_3:              return "CROSS_3";
        case VISION_ROUTE_SUB_CIRCLE_LEFT_1:        return "CIRCLE_LEFT_1";
        case VISION_ROUTE_SUB_CIRCLE_LEFT_2:        return "CIRCLE_LEFT_2";
        case VISION_ROUTE_SUB_CIRCLE_LEFT_3:        return "CIRCLE_LEFT_3";
        case VISION_ROUTE_SUB_CIRCLE_LEFT_4:        return "CIRCLE_LEFT_4";
        case VISION_ROUTE_SUB_CIRCLE_LEFT_5:        return "CIRCLE_LEFT_5";
        case VISION_ROUTE_SUB_CIRCLE_LEFT_6:        return "CIRCLE_LEFT_6";
        case VISION_ROUTE_SUB_CIRCLE_RIGHT_1:       return "CIRCLE_RIGHT_1";
        case VISION_ROUTE_SUB_CIRCLE_RIGHT_2:       return "CIRCLE_RIGHT_2";
        case VISION_ROUTE_SUB_CIRCLE_RIGHT_3:       return "CIRCLE_RIGHT_3";
        case VISION_ROUTE_SUB_CIRCLE_RIGHT_4:       return "CIRCLE_RIGHT_4";
        case VISION_ROUTE_SUB_CIRCLE_RIGHT_5:       return "CIRCLE_RIGHT_5";
        case VISION_ROUTE_SUB_CIRCLE_RIGHT_6:       return "CIRCLE_RIGHT_6";
        default:                                    return "UNKNOWN_SUB";
    }
}

RouteProfileSelection select_route_profile_selection(int route_main_state, int route_sub_state)
{
    using pid_tuning::route_line_follow::kCircleEnterProfile;
    using pid_tuning::route_line_follow::kCircleExitProfile;
    using pid_tuning::route_line_follow::kCircleInsideProfile;
    using pid_tuning::route_line_follow::kCrossProfile;
    using pid_tuning::route_line_follow::kNormalProfile;
    using pid_tuning::route_line_follow::kStraightProfile;

    switch (route_main_state)
    {
        case VISION_ROUTE_MAIN_STRAIGHT:
            return {kStraightProfile, "STRAIGHT"};

        case VISION_ROUTE_MAIN_CROSS:
            return {kCrossProfile, "CROSS"};

        case VISION_ROUTE_MAIN_CIRCLE_LEFT:
        case VISION_ROUTE_MAIN_CIRCLE_RIGHT:
            switch (route_sub_state)
            {
                case VISION_ROUTE_SUB_CIRCLE_LEFT_1:
                case VISION_ROUTE_SUB_CIRCLE_RIGHT_1:
                    return {kCircleEnterProfile, "CIRCLE_ENTER"};

                case VISION_ROUTE_SUB_CIRCLE_LEFT_2:
                case VISION_ROUTE_SUB_CIRCLE_LEFT_3:
                case VISION_ROUTE_SUB_CIRCLE_LEFT_4:
                case VISION_ROUTE_SUB_CIRCLE_RIGHT_2:
                case VISION_ROUTE_SUB_CIRCLE_RIGHT_3:
                case VISION_ROUTE_SUB_CIRCLE_RIGHT_4:
                    return {kCircleInsideProfile, "CIRCLE_INSIDE"};

                case VISION_ROUTE_SUB_CIRCLE_LEFT_5:
                case VISION_ROUTE_SUB_CIRCLE_LEFT_6:
                case VISION_ROUTE_SUB_CIRCLE_RIGHT_5:
                case VISION_ROUTE_SUB_CIRCLE_RIGHT_6:
                    return {kCircleExitProfile, "CIRCLE_EXIT"};

                default:
                    return {kCircleEnterProfile, "CIRCLE_ENTER"};
            }

        default:
            return {kNormalProfile, "NORMAL"};
    }
}

void log_route_state_transition_if_changed(int route_main_state, int route_sub_state)
{
    if (!g_has_last_logged_route_state)
    {
        g_last_logged_route_main_state = route_main_state;
        g_last_logged_route_sub_state = route_sub_state;
        g_has_last_logged_route_state = true;
        return;
    }

    if ((route_main_state == g_last_logged_route_main_state) &&
        (route_sub_state == g_last_logged_route_sub_state))
    {
        return;
    }

    const RouteProfileSelection previous_selection =
        select_route_profile_selection(g_last_logged_route_main_state, g_last_logged_route_sub_state);
    const RouteProfileSelection current_selection =
        select_route_profile_selection(route_main_state, route_sub_state);

    const bool main_state_changed = (route_main_state != g_last_logged_route_main_state);

    printf("[LINE_FOLLOW STATE] main %s -> %s, sub %s -> %s, profile %s -> %s\r\n",
           route_main_state_name(g_last_logged_route_main_state),
           route_main_state_name(route_main_state),
           route_sub_state_name(g_last_logged_route_sub_state),
           route_sub_state_name(route_sub_state),
           previous_selection.name,
           current_selection.name);
    if (main_state_changed)
    {
        beep_thread_request_beep(LINE_FOLLOW_MAIN_STATE_SWITCH_BEEP_MS);
    }

    g_last_logged_route_main_state = route_main_state;
    g_last_logged_route_sub_state = route_sub_state;
}

float compute_signed_track_point_angle_deg(bool track_valid, int track_x, int track_y)
{
    if (!track_valid)
    {
        return 0.0f;
    }

    const float dx = static_cast<float>(track_x - (VISION_IPM_WIDTH / 2));
    const float dy = static_cast<float>((VISION_IPM_HEIGHT - 1) - track_y);
    const float clamped_dy = std::max(dy, 1.0f);
    const float angle_rad = atan2f(-dx, clamped_dy);
    return angle_rad * RAD_TO_DEG;
}

void configure_line_follow_controllers_for_profile(const RouteProfile &profile,
                                                   float position_kp,
                                                   float position_kd,
                                                   float yaw_rate_kp)
{
    position_pid1.set_params(position_kp, profile.position_ki, position_kd);
    position_pid1.set_integral_limit(profile.position_max_integral);
    position_pid1.set_output_limit(profile.position_max_output);

    position_pid2.set_params(yaw_rate_kp, profile.yaw_rate_ki, profile.yaw_rate_kd);
    position_pid2.set_integral_limit(profile.yaw_rate_max_integral);
    position_pid2.set_output_limit(profile.yaw_rate_max_output);
}

float compute_profile_base_speed_from_normal_reference(float normal_speed_reference,
                                                       const RouteProfile &profile)
{
    const float safe_normal_speed_reference = std::max(normal_speed_reference, 0.0f);
    const float normal_profile_base_speed =
        std::max(pid_tuning::route_line_follow::kNormalProfile.base_speed, 1.0f);
    const float runtime_scale_from_normal_speed =
        safe_normal_speed_reference / normal_profile_base_speed;
    return std::max(0.0f,
                    profile.base_speed *
                    pid_tuning::route_line_follow::kGlobalBaseSpeedScale *
                    runtime_scale_from_normal_speed);
}

float compute_progressive_slowdown_scale(float absolute_value,
                                         float slowdown_start,
                                         float slowdown_full,
                                         float min_speed_scale)
{
    if (slowdown_start <= 0.0f || slowdown_full <= slowdown_start)
    {
        return 1.0f;
    }

    if (absolute_value <= slowdown_start)
    {
        return 1.0f;
    }

    const float slowdown_ratio = std::clamp((absolute_value - slowdown_start) /
                                            (slowdown_full - slowdown_start),
                                            0.0f,
                                            1.0f);
    return 1.0f - slowdown_ratio * (1.0f - min_speed_scale);
}

bool should_force_full_speed(const RouteProfile &profile,
                             float mean_abs_path_error,
                             float abs_yaw_rate_ref_dps)
{
    const bool preview_turn_is_gentle =
        (profile.yaw_rate_ref_slowdown_start_dps <= 0.0f) ||
        (abs_yaw_rate_ref_dps < profile.yaw_rate_ref_slowdown_start_dps);
    return (profile.straight_full_speed_error_threshold_px > 0.0f) &&
           (mean_abs_path_error < profile.straight_full_speed_error_threshold_px) &&
           preview_turn_is_gentle;
}

float compute_desired_base_speed(float profile_base_speed,
                                 const RouteProfile &profile,
                                 float abs_filtered_error_px,
                                 float abs_yaw_rate_ref_dps,
                                 float mean_abs_path_error,
                                 bool &force_full_speed)
{
    force_full_speed = should_force_full_speed(profile, mean_abs_path_error, abs_yaw_rate_ref_dps);
    if (force_full_speed)
    {
        return profile_base_speed;
    }

    const float error_speed_scale =
        compute_progressive_slowdown_scale(abs_filtered_error_px,
                                           profile.turn_slowdown_start_px,
                                           profile.turn_slowdown_full_px,
                                           profile.turn_min_speed_scale);
    const float preview_speed_scale =
        compute_progressive_slowdown_scale(abs_yaw_rate_ref_dps,
                                           profile.yaw_rate_ref_slowdown_start_dps,
                                           profile.yaw_rate_ref_slowdown_full_dps,
                                           profile.turn_min_speed_scale);
    return profile_base_speed * std::min(error_speed_scale, preview_speed_scale);
}

float clamp_valid_dt_seconds(float dt_seconds, float fallback_dt_seconds, float max_dt_seconds)
{
    if (!std::isfinite(dt_seconds) || dt_seconds <= 1.0e-4f)
    {
        return fallback_dt_seconds;
    }

    return std::clamp(dt_seconds, 1.0e-4f, max_dt_seconds);
}

float compute_sample_dt_seconds(std::chrono::steady_clock::time_point now,
                                std::chrono::steady_clock::time_point &last_time,
                                bool &has_last_time,
                                float fallback_dt_seconds,
                                float max_dt_seconds)
{
    float dt_seconds = fallback_dt_seconds;
    if (has_last_time)
    {
        dt_seconds = std::chrono::duration_cast<std::chrono::duration<float>>(now - last_time).count();
    }
    last_time = now;
    has_last_time = true;
    return clamp_valid_dt_seconds(dt_seconds, fallback_dt_seconds, max_dt_seconds);
}

float update_position_error_rate_if_needed(bool vision_updated,
                                           float control_error_px,
                                           float fallback_dt_seconds)
{
    if (!vision_updated)
    {
        return g_position_error_rate_px_per_second;
    }

    const float safe_dt_seconds =
        clamp_valid_dt_seconds(fallback_dt_seconds, VISION_NOMINAL_DT_SECONDS, VISION_MAX_DT_SECONDS);

    if (!g_has_last_position_control_error)
    {
        g_position_error_rate_px_per_second = 0.0f;
    }
    else
    {
        g_position_error_rate_px_per_second =
            (control_error_px - g_last_position_control_error_px) / safe_dt_seconds;
    }

    g_last_position_control_error_px = control_error_px;
    g_has_last_position_control_error = true;
    return g_position_error_rate_px_per_second;
}

float update_pid_output_state_if_needed(bool should_update,
                                        float control_error,
                                        float fallback_dt_seconds,
                                        std::chrono::steady_clock::time_point &last_time,
                                        bool &has_last_time,
                                        float output_limit,
                                        PositionalPidController &controller,
                                        float current_state)
{
    if (!should_update)
    {
        return current_state;
    }

    const float dt_seconds =
        compute_sample_dt_seconds(std::chrono::steady_clock::now(),
                                  last_time,
                                  has_last_time,
                                  fallback_dt_seconds,
                                  PID_MAX_DT_SECONDS);
    return std::clamp(controller.compute_by_error(control_error, dt_seconds),
                      -output_limit,
                      output_limit);
}

float alpha_to_time_constant_seconds(float alpha, float nominal_dt_seconds)
{
    const float safe_alpha = std::clamp(alpha, 1.0e-3f, 0.999f);
    return nominal_dt_seconds * (1.0f - safe_alpha) / safe_alpha;
}

float compute_iir_alpha_from_dt(float dt_seconds, float time_constant_seconds)
{
    if (!std::isfinite(time_constant_seconds) || time_constant_seconds <= 1.0e-6f)
    {
        return 1.0f;
    }

    return std::clamp(dt_seconds / (time_constant_seconds + dt_seconds), 0.0f, 1.0f);
}

float update_applied_base_speed(float current_base_speed,
                                float profile_base_speed,
                                float desired_base_speed,
                                bool force_full_speed,
                                const RouteProfile &profile)
{
    if (current_base_speed < 0.0f)
    {
        return desired_base_speed;
    }

    if (force_full_speed)
    {
        return profile_base_speed;
    }

    const float max_drop = std::max(0.0f,
                                    current_base_speed *
                                    profile.turn_slowdown_max_drop_ratio_per_cycle);
    const float max_rise = std::max(0.0f,
                                    current_base_speed *
                                    profile.turn_slowdown_max_rise_ratio_per_cycle);
    return std::clamp(desired_base_speed,
                      current_base_speed - max_drop,
                      current_base_speed + max_rise);
}

float clamp_steering_to_wheel_room(float applied_base_speed, float raw_steering_output)
{
    const float max_left_turn_room = applied_base_speed - pid_tuning::line_follow::kTargetCountMin;
    const float max_right_turn_room = pid_tuning::line_follow::kTargetCountMax - applied_base_speed;
    const float available_steering_limit = std::max(0.0f, std::min(max_left_turn_room, max_right_turn_room));
    return std::clamp(raw_steering_output,
                      -available_steering_limit,
                      available_steering_limit);
}

bool update_filtered_yaw_rate_if_new_sample(float *sample_dt_seconds_out)
{
    const uint32 imu_sample_seq = imu_thread_gyro_z_sample_seq();
    if (imu_sample_seq == g_last_imu_sample_seq)
    {
        return false;
    }
    g_last_imu_sample_seq = imu_sample_seq;

    const auto now = std::chrono::steady_clock::now();
    const float sample_dt_seconds =
        compute_sample_dt_seconds(now,
                                  g_last_imu_update_time,
                                  g_has_last_imu_update_time,
                                  IMU_NOMINAL_DT_SECONDS,
                                  IMU_MAX_DT_SECONDS);
    if (sample_dt_seconds_out != nullptr)
    {
        *sample_dt_seconds_out = sample_dt_seconds;
    }

    const float measured_yaw_rate_dps =
        imu_thread_gyro_z_dps() * pid_tuning::imu::kGyroYawRateSign;
    const float gyro_filter_tau_seconds =
        alpha_to_time_constant_seconds(pid_tuning::imu::kGyroYawRateFilterAlpha, IMU_NOMINAL_DT_SECONDS);
    const float gyro_filter_alpha =
        compute_iir_alpha_from_dt(sample_dt_seconds, gyro_filter_tau_seconds);
    g_filtered_yaw_rate_dps = apply_iir_filter(g_filtered_yaw_rate_dps,
                                               measured_yaw_rate_dps,
                                               gyro_filter_alpha);
    return true;
}

bool update_filtered_vision_inputs_if_new_frame(float *frame_dt_seconds_out)
{
    const uint32 vision_frame_seq = vision_image_processor_processed_frame_seq();
    if (vision_frame_seq == g_last_vision_frame_seq)
    {
        return false;
    }
    g_last_vision_frame_seq = vision_frame_seq;

    const auto now = std::chrono::steady_clock::now();
    const float frame_dt_seconds =
        compute_sample_dt_seconds(now,
                                  g_last_vision_update_time,
                                  g_has_last_vision_update_time,
                                  VISION_NOMINAL_DT_SECONDS,
                                  VISION_MAX_DT_SECONDS);
    if (frame_dt_seconds_out != nullptr)
    {
        *frame_dt_seconds_out = frame_dt_seconds;
    }

    // 偏差来源固定使用 line_error（方案A）。
    const float selected_offset_error = static_cast<float>(line_error);
    // 方向约定：正偏差表示中线偏右（x_ref右侧），与控制器内部正方向相反，统一取负。
    const float raw_error_px = -selected_offset_error;
    g_line_error_px.store(raw_error_px);

    // 一阶 IIR 滤波只在“新视觉帧到来”时推进一次，避免在 1ms 循环里对旧帧重复滤波。
    // 这里直接在像素误差量纲下滤波，不再经过归一化。
    const float error_filter_tau_seconds =
        alpha_to_time_constant_seconds(pid_tuning::line_follow::kErrorFilterAlpha, VISION_NOMINAL_DT_SECONDS);
    const float error_filter_alpha =
        compute_iir_alpha_from_dt(frame_dt_seconds, error_filter_tau_seconds);
    g_filtered_error = apply_iir_filter(g_filtered_error,
                                        raw_error_px,
                                        error_filter_alpha);

    bool track_point_valid = false;
    int track_point_x = 0;
    int track_point_y = 0;
    vision_image_processor_get_ipm_line_error_track_point(&track_point_valid, &track_point_x, &track_point_y);
    //视觉跟踪点相对中线的夹角
    const float current_track_point_angle_deg =
        compute_signed_track_point_angle_deg(track_point_valid, track_point_x, track_point_y);
    const float track_point_filter_tau_seconds =
        alpha_to_time_constant_seconds(pid_tuning::yaw_rate_loop::kTrackPointAngleFilterAlpha,
                                       VISION_NOMINAL_DT_SECONDS);
    const float track_point_filter_alpha =
        compute_iir_alpha_from_dt(frame_dt_seconds, track_point_filter_tau_seconds);
    g_filtered_track_point_angle_deg = apply_iir_filter(g_filtered_track_point_angle_deg,
                                                        current_track_point_angle_deg,
                                                        track_point_filter_alpha);
    return true;
}

void refresh_thread_info()
{
    int policy = 0; // 获取到的当前线程调度策略
    struct sched_param param; // 获取到的当前线程调度参数

    memset(&param, 0, sizeof(param));

    g_thread_tid = (int32)syscall(SYS_gettid);

    if (0 == pthread_getschedparam(pthread_self(), &policy, &param))
    {
        g_thread_policy = policy;
        g_thread_priority = param.sched_priority;
    }
    else
    {
        g_thread_policy = 0;
        g_thread_priority = 0;
    }
}

/**
 * @brief 巡线控制线程主循环
 *
 * 控制链路：
 * 1) 视觉模块先给出采样行处的赛道中线偏差 line_error（像素）；
 * 2) 巡线线程只在“拿到新视觉帧 / 新 IMU 样本”时推进各自滤波状态；
 * 3) 再统一方向约定，并通过归一化、死区、小误差降增益把视觉噪声整形成“可控误差”；
 * 4) 位置环 PID 负责“回中线”，角速度环 PID 负责“按视觉期望横摆率转过去”；
 * 5) 两个环并级叠加成最终差速，再叠加基础速度与视觉弯道降速后下发左右轮目标。
 */
void line_follow_loop()
{
    struct sched_param sp; // 用于设置当前线程调度的参数结构体
    sp.sched_priority = LINE_FOLLOW_THREAD_PRIORITY;
    if (0 != pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp))
    {
        printf("line_follow set sched failed, fallback to current policy\r\n");
    }

    refresh_thread_info();

    while (g_line_follow_running.load())
    {
        if (g_reload_from_globals_requested.exchange(false))
        {
            position_pid1.reset();
            position_pid2.reset();
            g_normal_speed_reference.store(std::max(0.0f, pid_tuning::route_line_follow::kNormalProfile.base_speed));
        }

        // 这里的 normal_speed_reference 表示“当前希望的 NORMAL 档直道参考速度”。
        // 各状态实际基础速度由 pid_tuning 里的绝对速度档位给出，再按这个直道参考速度做整体缩放。
        const float current_normal_speed_reference = g_normal_speed_reference.load();
        const int route_main_state = vision_image_processor_route_main_state();//主状态获取
        const int route_sub_state = vision_image_processor_route_sub_state();//子状态获取
        const RouteProfileSelection route_selection =
            select_route_profile_selection(route_main_state, route_sub_state);//状态对应参数包
        const RouteProfile &route_profile = route_selection.profile;//取出参数包中数据
        const float profile_base_speed =
            compute_profile_base_speed_from_normal_reference(current_normal_speed_reference, route_profile);
        // 这些滤波器只在“数据源真的更新了”时推进一次；
        // 若当前 1ms 周期只是重复读到旧样本，就沿用上一份滤波状态，避免 alpha 被空转放大。
        float imu_sample_dt_seconds = IMU_NOMINAL_DT_SECONDS;
        float vision_frame_dt_seconds = VISION_NOMINAL_DT_SECONDS;
        const bool imu_updated = update_filtered_yaw_rate_if_new_sample(&imu_sample_dt_seconds);
        const bool vision_updated = update_filtered_vision_inputs_if_new_frame(&vision_frame_dt_seconds);
        if (vision_updated)
        {
            log_route_state_transition_if_changed(route_main_state, route_sub_state);
        }

        // 后面的 deadzone / 小误差降增益继续使用像素尺度判断，
        // 位置环当前也直接在像素量纲下工作。
        const ControlErrorState error_state = compute_control_error_state(g_filtered_error);

        // 动态 Kp：误差越大，比例增益越强。
        // 这里不再对动态 Kp 的输入额外归一化，直接用经过死区/低增益处理后的像素误差。
        const float dynamic_kp =
            compute_piecewise_linear_abs_gain(route_profile.position_dynamic_kp_base,
                                              route_profile.position_dynamic_kp_quad_a,
                                              route_profile.position_dynamic_kp_low_error_threshold_px,
                                              route_profile.position_dynamic_kp_mid_a,
                                              route_profile.position_dynamic_kp_mid_error_threshold_px,
                                              route_profile.position_dynamic_kp_high_a,
                                              route_profile.position_dynamic_kp_min,
                                              route_profile.position_dynamic_kp_max,
                                              error_state.control_error_px);
        const float position_error_rate_px_per_second =
            update_position_error_rate_if_needed(vision_updated,
                                                 error_state.control_error_px,
                                                 vision_frame_dt_seconds);
        const float dynamic_position_kd = compute_linear_abs_gain(route_profile.position_kd,
                                                                  route_profile.position_dynamic_kd_quad_a,
                                                                  route_profile.position_dynamic_kd_min,
                                                                  route_profile.position_dynamic_kd_max,
                                                                  position_error_rate_px_per_second);

        // 第二条支路：角速度环 PID。
        // 当前固定采用“跟踪点夹角 -> 目标横摆角速度”的路线：
        // 1) 跟踪点偏角负责告诉车头“该往哪边、该多快转”；
        // 2) IMU 负责反馈“车身现在实际转了多少”；
        // 3) 角速度环去逼近这个由夹角生成的目标横摆角速度。
        // 这样位置环和角速度环职责更清楚：位置环回中线，角速度环管车头朝向。
        const float yaw_rate_ref_dps = std::clamp(
            g_filtered_track_point_angle_deg * route_profile.yaw_rate_ref_from_track_point_gain_dps,
            -route_profile.yaw_rate_ref_limit_dps,
            route_profile.yaw_rate_ref_limit_dps);
        //角速度的差
        const float yaw_rate_error_dps = yaw_rate_ref_dps - g_filtered_yaw_rate_dps; // 目标角速度与实际角速度之差
        const float dynamic_yaw_rate_kp = compute_linear_abs_gain(route_profile.yaw_rate_kp,
                                                                  route_profile.yaw_rate_dynamic_kp_quad_a,
                                                                  route_profile.yaw_rate_dynamic_kp_min,
                                                                  route_profile.yaw_rate_dynamic_kp_max,
                                                                  yaw_rate_error_dps);
        const bool enable_yaw_rate_kp =
            (route_profile.yaw_rate_kp_enable_error_threshold_px <= 0.0f) ||
            (error_state.abs_filtered_error_px >= route_profile.yaw_rate_kp_enable_error_threshold_px);
        const float applied_yaw_rate_kp = enable_yaw_rate_kp ? dynamic_yaw_rate_kp : 0.0f;
        configure_line_follow_controllers_for_profile(route_profile,
                                                      dynamic_kp,
                                                      dynamic_position_kd,
                                                      applied_yaw_rate_kp);

        // ---------------- 并级控制核心 ----------------
        // 第一条支路：位置环 PID，只负责“把车拉回中线”。
        // 它盯的是横向误差 e，输出一份差速量。
        g_position_output_state = update_pid_output_state_if_needed(vision_updated,
                                                                    error_state.control_error_px,
                                                                    vision_frame_dt_seconds,
                                                                    g_last_position_pid_time,
                                                                    g_has_last_position_pid_time,
                                                                    route_profile.position_max_output,
                                                                    position_pid1,
                                                                    g_position_output_state);
        const float position_output = g_position_output_state;

        const float yaw_rate_pid_dt_fallback =
            vision_updated ? vision_frame_dt_seconds : imu_sample_dt_seconds;
        g_yaw_rate_output_state = update_pid_output_state_if_needed(vision_updated || imu_updated,
                                                                    yaw_rate_error_dps,
                                                                    yaw_rate_pid_dt_fallback,
                                                                    g_last_yaw_rate_pid_time,
                                                                    g_has_last_yaw_rate_pid_time,
                                                                    route_profile.yaw_rate_max_output,
                                                                    position_pid2,
                                                                    g_yaw_rate_output_state);
        const float yaw_rate_output = g_yaw_rate_output_state;

        // 并级的意思，就是两条支路独立算完后再直接相加：
        // - position_output 管“位置偏了多少”；
        // - yaw_rate_output 管“车身现在转得对不对”。
        const float raw_steering_output = std::clamp(position_output + yaw_rate_output,
                                                     -route_profile.steering_max_output,
                                                     route_profile.steering_max_output);
        const float abs_yaw_rate_ref_dps = std::fabs(yaw_rate_ref_dps);

        bool force_full_speed = false; // 是否强制满速(即车身状态良好处于直道)的标志位
        const float mean_abs_path_error = vision_image_processor_ipm_mean_abs_offset_error(); // 视觉给出的整条分析路径平均绝对偏差像素
        const float desired_base_speed = compute_desired_base_speed(profile_base_speed,
                                                                    route_profile,
                                                                    error_state.abs_filtered_error_px,
                                                                    abs_yaw_rate_ref_dps,
                                                                    mean_abs_path_error,
                                                                    force_full_speed);
        // 当前版本把“陀螺仪降速”整体拿掉，只保留视觉负责速度规划。
        // 若整条路径绝对偏差均值很小，且前方预瞄转向需求也不大，认为处于稳定直道，可直接给满基础速度。
        // 大弯主动降基础速度：
        // 如果还按直道速度冲，内外轮目标会跳得很大，速度环很容易跟不上，车体就会发飘。
        // 这里保留两类视觉降速：
        // 1) 当前横向误差已经变大时，按误差降速；
        // 2) 当前虽还贴线，但前方预瞄转向需求已经明显时，也提前降速，避免“看见弯还不收油”。
        g_applied_base_speed_state = update_applied_base_speed(g_applied_base_speed_state,
                                                               profile_base_speed,
                                                               desired_base_speed,
                                                               force_full_speed,
                                                               route_profile);
        const float applied_base_speed = std::clamp(g_applied_base_speed_state, // 经过所有降速和缓冲限幅后的最终下发基础速度
                                                    pid_tuning::line_follow::kTargetCountMin,
                                                    pid_tuning::line_follow::kTargetCountMax);
        const float steering_output = clamp_steering_to_wheel_room(applied_base_speed,
                                                                   raw_steering_output);

        // 差速映射：左轮=base-out，右轮=base+out。
        // steering_output 为正时，右轮更快、左轮更慢，车体会朝左修正；
        // steering_output 为负时，左轮更快、右轮更慢，车体会朝右修正。
        //
        // 这里先根据当前基础速度剩余的轮速余量，再收一次 steering_output，
        // 避免最后再由左右轮目标限幅“硬裁切”，让转向行为尽量保持可预期。
        const float left_target = std::clamp(applied_base_speed - steering_output, // 计算得到的电机左轮最终目标速度
                                             pid_tuning::line_follow::kTargetCountMin,
                                             pid_tuning::line_follow::kTargetCountMax);
        const float right_target = std::clamp(applied_base_speed + steering_output, // 计算得到的电机右轮最终目标速度
                                              pid_tuning::line_follow::kTargetCountMin,
                                              pid_tuning::line_follow::kTargetCountMax);
        // 重新用限幅后的左右轮目标反推实际差速，保证对外上报值和真正下发给电机的一致。
        const float applied_steering_output = (right_target - left_target) * 0.5f; // 反推出来并实际下放给电机的转向差速调整量
        motor_thread_set_target_count(left_target, right_target);

        // 对外发布关键调试量，供屏显/上位机读取。
        // 这样你在调试时能同时看到：看到了多大误差、实际打了多少差速、左右轮目标是多少。
        g_turn_output.store(applied_steering_output);

        bool track_point_valid = false;
        int track_point_x = 0;
        int track_point_y = 0;
        vision_image_processor_get_ipm_line_error_track_point(&track_point_valid, &track_point_x, &track_point_y);
        const float current_track_point_angle_deg =
            compute_signed_track_point_angle_deg(track_point_valid, track_point_x, track_point_y);
        const float raw_error_px = g_line_error_px.load();
        const float measured_yaw_rate_dps = imu_thread_gyro_z_dps() * pid_tuning::imu::kGyroYawRateSign;

        {
            std::lock_guard<std::mutex> lock(g_pid_debug_mutex);
            g_pid_debug_status.vision_updated = vision_updated;
            g_pid_debug_status.imu_updated = imu_updated;
            g_pid_debug_status.route_main_state = route_main_state;
            g_pid_debug_status.route_sub_state = route_sub_state;
            g_pid_debug_status.normal_speed_reference = current_normal_speed_reference;
            g_pid_debug_status.profile_base_speed = profile_base_speed;
            g_pid_debug_status.desired_base_speed = desired_base_speed;
            g_pid_debug_status.applied_base_speed = applied_base_speed;
            g_pid_debug_status.raw_error_px = raw_error_px;
            g_pid_debug_status.filtered_error_px = error_state.filtered_error_px;
            g_pid_debug_status.abs_filtered_error_px = error_state.abs_filtered_error_px;
            g_pid_debug_status.control_error_px = error_state.control_error_px;
            g_pid_debug_status.track_point_valid = track_point_valid;
            g_pid_debug_status.track_point_x = track_point_x;
            g_pid_debug_status.track_point_y = track_point_y;
            g_pid_debug_status.current_track_point_angle_deg = current_track_point_angle_deg;
            g_pid_debug_status.filtered_track_point_angle_deg = g_filtered_track_point_angle_deg;
            g_pid_debug_status.measured_yaw_rate_dps = measured_yaw_rate_dps;
            g_pid_debug_status.yaw_rate_ref_dps = yaw_rate_ref_dps;
            g_pid_debug_status.yaw_rate_error_dps = yaw_rate_error_dps;
            g_pid_debug_status.dynamic_position_kp = dynamic_kp;
            g_pid_debug_status.dynamic_yaw_rate_kp = dynamic_yaw_rate_kp;
            g_pid_debug_status.applied_yaw_rate_kp = applied_yaw_rate_kp;
            g_pid_debug_status.position_pid_kp = position_pid1.kp();
            g_pid_debug_status.position_pid_ki = position_pid1.ki();
            g_pid_debug_status.position_pid_kd = position_pid1.kd();
            g_pid_debug_status.position_pid_target = position_pid1.get_target();
            g_pid_debug_status.position_pid_error = position_pid1.get_error();
            g_pid_debug_status.position_pid_integral = position_pid1.integral();
            g_pid_debug_status.position_pid_output = position_pid1.get_output();
            g_pid_debug_status.position_pid_max_integral = position_pid1.max_integral();
            g_pid_debug_status.position_pid_max_output = position_pid1.max_output();
            g_pid_debug_status.yaw_pid_kp = position_pid2.kp();
            g_pid_debug_status.yaw_pid_ki = position_pid2.ki();
            g_pid_debug_status.yaw_pid_kd = position_pid2.kd();
            g_pid_debug_status.yaw_pid_target = position_pid2.get_target();
            g_pid_debug_status.yaw_pid_error = position_pid2.get_error();
            g_pid_debug_status.yaw_pid_integral = position_pid2.integral();
            g_pid_debug_status.yaw_pid_output = position_pid2.get_output();
            g_pid_debug_status.yaw_pid_max_integral = position_pid2.max_integral();
            g_pid_debug_status.yaw_pid_max_output = position_pid2.max_output();
            g_pid_debug_status.route_yaw_rate_ref_gain = route_profile.yaw_rate_ref_from_track_point_gain_dps;
            g_pid_debug_status.route_yaw_rate_ref_limit = route_profile.yaw_rate_ref_limit_dps;
            g_pid_debug_status.route_steering_max_output = route_profile.steering_max_output;
            g_pid_debug_status.route_yaw_rate_kp_enable_error_threshold_px =
                route_profile.yaw_rate_kp_enable_error_threshold_px;
            g_pid_debug_status.mean_abs_path_error = mean_abs_path_error;
            g_pid_debug_status.force_full_speed = force_full_speed;
            g_pid_debug_status.raw_steering_output = raw_steering_output;
            g_pid_debug_status.clamped_steering_output = steering_output;
            g_pid_debug_status.applied_steering_output = applied_steering_output;
            g_pid_debug_status.left_target_count = left_target;
            g_pid_debug_status.right_target_count = right_target;
            g_pid_debug_status.vision_dt_ms = vision_frame_dt_seconds * 1000.0f;
            g_pid_debug_status.imu_dt_ms = imu_sample_dt_seconds * 1000.0f;
        }

        system_delay_ms(LINE_FOLLOW_PERIOD_MS);
    }
}
}

bool line_follow_thread_init()
{
    if (g_line_follow_running.load())
    {
        return true;
    }

    reset_line_follow_runtime_state();

    const RouteProfile &default_profile = pid_tuning::route_line_follow::kNormalProfile;
    // 位置环：目标固定为 0，表示希望赛道中线最终回到图像中心。
    position_pid1.init(default_profile.position_dynamic_kp_base,
                       default_profile.position_ki,
                       default_profile.position_kd,
                       default_profile.position_max_integral,
                       default_profile.position_max_output);
    position_pid1.set_target(0.0f);
    // 角速度环：同样目标为 0，但真正计算时使用的是 compute_by_error(r_ref - r)。
    // 这里明确采用位置式 PID，把“目标角速度和实际角速度的差”直接变成一份差速补偿量。
    // 这样更符合它作为并级支路的角色，也更方便你后面直接看输出大小来调参。
    position_pid2.init(default_profile.yaw_rate_kp,
                       default_profile.yaw_rate_ki,
                       default_profile.yaw_rate_kd,
                       default_profile.yaw_rate_max_integral,
                       default_profile.yaw_rate_max_output);
    position_pid2.set_target(0.0f);

    g_line_follow_running = true;
    g_line_follow_thread = std::thread(line_follow_loop);
    return true;
}

void line_follow_thread_cleanup()
{
    if (!g_line_follow_running.load())
    {
        return;
    }

    // 先停循环，再join，保证线程退出与资源回收时序确定。
    g_line_follow_running = false;

    if (g_line_follow_thread.joinable())
    {
        g_line_follow_thread.join();
    }

    // 清掉 PID 和滤波残留，避免下次启动时把上一次的“方向记忆”带进来。
    position_pid1.reset();
    position_pid2.reset();
    reset_line_follow_runtime_state();
}

void line_follow_thread_print_info()
{
    const int32 tid = g_thread_tid.load(); // 缓存加载的当前巡线线程系统ID

    if (0 < tid)
    {
        printf("thread=%s tid=%d policy=%s priority=%d\r\n",
               "line_follow",
               tid,
               sched_policy_name(g_thread_policy.load()),
               g_thread_priority.load());
    }
    else
    {
        printf("thread=%s tid=unknown policy=unknown priority=unknown\r\n",
               "line_follow");
    }
}

void line_follow_thread_set_normal_speed_reference(float speed)
{
    g_normal_speed_reference.store(std::max(0.0f, speed));
}

float line_follow_thread_error()
{
    return g_line_error_px.load();
}

float line_follow_thread_turn_output()
{
    return g_turn_output.load();
}

float line_follow_thread_normal_speed_reference()
{
    return std::max(0.0f,
                    g_normal_speed_reference.load() *
                    pid_tuning::route_line_follow::kGlobalBaseSpeedScale);
}

float line_follow_thread_applied_base_speed()
{
    if (g_applied_base_speed_state >= 0.0f)
    {
        return g_applied_base_speed_state;
    }
    return line_follow_thread_normal_speed_reference();
}

bool line_follow_thread_get_pid_debug_status(LineFollowPidDebugStatus &status)
{
    std::lock_guard<std::mutex> lock(g_pid_debug_mutex);
    status = g_pid_debug_status;
    return true;
}

void line_follow_thread_request_reload_from_globals()
{
    g_reload_from_globals_requested.store(true);
}

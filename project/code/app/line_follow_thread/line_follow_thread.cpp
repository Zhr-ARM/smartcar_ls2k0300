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
constexpr int32 LINE_FOLLOW_PERIOD_MS = 1;
// 调度优先级：SCHED_RR prio=5，低于视觉线程(10)，保证视觉优先拿到 CPU。
constexpr int32 LINE_FOLLOW_THREAD_PRIORITY = 5;
constexpr int32 LINE_FOLLOW_MAIN_STATE_SWITCH_BEEP_MS = 200;
constexpr float LINE_FOLLOW_LOOP_DT_SECONDS = LINE_FOLLOW_PERIOD_MS / 1000.0f;
constexpr float IMU_NOMINAL_DT_SECONDS = 0.005f;
constexpr float VISION_NOMINAL_DT_SECONDS = 1.0f / 60.0f;
constexpr float IMU_MAX_DT_SECONDS = 0.050f;
constexpr float VISION_MAX_DT_SECONDS = 0.200f;
constexpr float PID_MAX_DT_SECONDS = 0.200f;
constexpr float POSITION_PID_DT_SECONDS = 0.010f;
constexpr float YAW_RATE_PID_DT_SECONDS = 0.005f;
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
// 滤波后的横摆角速度状态：把 IMU 的瞬时抖动再收一层，减轻差速来回抽动。
// 注意：状态只在“拿到新 IMU 样本”时推进一次。
float g_filtered_yaw_rate_dps = 0.0f;
// 滤波后的目标点夹角：由跟踪点相对图像中垂线的偏转角得到。
// 注意：状态只在“拿到新视觉帧”时推进一次。
float g_filtered_track_point_angle_deg = 0.0f;
// 位置环与角速度环各自最近一次真实更新后的输出，样本未更新时沿用旧值。
float g_position_output_state = 0.0f;
float g_yaw_rate_output_state = 0.0f;
// 最近一次已消费的 IMU / 视觉样本序号。
uint32 g_last_imu_sample_seq = 0;
uint32 g_last_vision_frame_seq = 0;
bool g_has_last_imu_update_time = false;
bool g_has_last_vision_update_time = false;
std::chrono::steady_clock::time_point g_last_imu_update_time;
std::chrono::steady_clock::time_point g_last_vision_update_time;
bool g_has_last_logged_route_state = false;
int g_last_logged_route_main_state = VISION_ROUTE_MAIN_NORMAL;
int g_last_logged_route_sub_state = VISION_ROUTE_SUB_NONE;
std::mutex g_pid_debug_mutex;
LineFollowPidDebugStatus g_pid_debug_status{};
int g_last_cascade_mode = 0;

struct RouteProfileSelection
{
    const RouteProfile &profile;
    const char *name;
};


enum CascadeMode
{
    CASCADE_MODE_NORMAL = 0,
    CASCADE_MODE_YAW_DEBUG = 1,
    CASCADE_MODE_SPEED_DEBUG = 2,
};

float apply_iir_filter(float previous_value, float current_value, float alpha)
{
    return previous_value * (1.0f - alpha) + current_value * alpha;
}


void reset_line_follow_runtime_state()
{
    g_thread_tid = 0;
    g_thread_policy = 0;
    g_thread_priority = 0;
    g_line_error_px.store(0.0f);
    g_turn_output.store(0.0f);
    g_filtered_yaw_rate_dps = 0.0f;
    g_filtered_track_point_angle_deg = 0.0f;
    g_position_output_state = 0.0f;
    g_yaw_rate_output_state = 0.0f;
    g_last_imu_sample_seq = 0;
    g_last_vision_frame_seq = 0;
    g_has_last_imu_update_time = false;
    g_has_last_vision_update_time = false;
    g_has_last_logged_route_state = false;
    g_last_logged_route_main_state = VISION_ROUTE_MAIN_NORMAL;
    g_last_logged_route_sub_state = VISION_ROUTE_SUB_NONE;
    g_last_cascade_mode = CASCADE_MODE_NORMAL;
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
        case VISION_ROUTE_MAIN_CIRCLE:       return "CIRCLE";
        case VISION_ROUTE_MAIN_STRAIGHT:     return "STRAIGHT_DISABLED";
        default:                             return "UNKNOWN_MAIN";
    }
}

const char *route_sub_state_name(int route_sub_state)
{
    switch (route_sub_state)
    {
        case VISION_ROUTE_SUB_NONE:                 return "NONE";
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
    (void)route_main_state;
    (void)route_sub_state;

    using pid_tuning::route_line_follow::kNormalProfile;
    return {kNormalProfile, "NORMAL"};
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

    bool track_point_valid = false;
    int track_point_x = 0;
    int track_point_y = 0;
    vision_image_processor_get_ipm_line_error_track_point(&track_point_valid, &track_point_x, &track_point_y);
    const float current_track_point_angle_deg =
        compute_signed_track_point_angle_deg(track_point_valid, track_point_x, track_point_y);
    g_filtered_track_point_angle_deg = current_track_point_angle_deg;
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
 * 4) 位置环 PID 输出目标横摆角速度，角速度环 PID 输出差速速度量；
 * 5) 最终按左右轮目标映射并结合降速策略下发到速度环。
 */
void line_follow_loop()
{
    struct sched_param sp; // 用于设置当前线程调度的参数结构体
    sp.sched_priority = LINE_FOLLOW_THREAD_PRIORITY;
    if (0 != pthread_setschedparam(pthread_self(), SCHED_RR, &sp))
    {
        printf("line_follow set sched SCHED_RR prio=%d failed, fallback to current policy\r\n", LINE_FOLLOW_THREAD_PRIORITY);
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

        // 这里的 normal_speed_reference 表示”当前希望的 NORMAL 档直道参考速度”。
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

        const float pos_error_px = g_line_error_px.load();

        const bool speed_loop_debug_enabled = pid_tuning::line_follow::kSpeedLoopDebugEnabled;
        const bool yaw_rate_debug_enabled = !speed_loop_debug_enabled &&
                                            pid_tuning::line_follow::kYawRateDebugEnabled;

        const float position_pid_output_limit = std::max(route_profile.position_max_output, 0.0f);
        configure_line_follow_controllers_for_profile(route_profile,
                                                      route_profile.position_kp,
                                                      route_profile.position_kd,
                                                      route_profile.yaw_rate_kp);

        bool pos_dummy_has_time = false;
        std::chrono::steady_clock::time_point pos_dummy_time;
        g_position_output_state = update_pid_output_state_if_needed(vision_updated,
                                                                    pos_error_px,
                                                                    POSITION_PID_DT_SECONDS,
                                                                    pos_dummy_time,
                                                                    pos_dummy_has_time,
                                                                    position_pid_output_limit,
                                                                    position_pid1,
                                                                    g_position_output_state);
        const float yaw_rate_ref_from_pos_dps = g_position_output_state;
        const float yaw_rate_ref_final_dps = yaw_rate_debug_enabled
                                                 ? pid_tuning::line_follow::kYawRateDebugTargetDps
                                                 : yaw_rate_ref_from_pos_dps;
        const float yaw_rate_error_dps = yaw_rate_ref_final_dps - g_filtered_yaw_rate_dps;
        position_pid2.set_params(route_profile.yaw_rate_kp, route_profile.yaw_rate_ki, route_profile.yaw_rate_kd);
        position_pid2.set_integral_limit(route_profile.yaw_rate_max_integral);
        position_pid2.set_output_limit(route_profile.yaw_rate_max_output);

        bool yaw_dummy_has_time = false;
        std::chrono::steady_clock::time_point yaw_dummy_time;
        g_yaw_rate_output_state = update_pid_output_state_if_needed(vision_updated || imu_updated,
                                                                    yaw_rate_error_dps,
                                                                    YAW_RATE_PID_DT_SECONDS,
                                                                    yaw_dummy_time,
                                                                    yaw_dummy_has_time,
                                                                    route_profile.yaw_rate_max_output,
                                                                    position_pid2,
                                                                    g_yaw_rate_output_state);
        const float delta_v_cmd = g_yaw_rate_output_state;

        const float speed_command_base = speed_loop_debug_enabled
                                             ? pid_tuning::line_follow::kSpeedLoopDebugBaseSpeed
                                             : profile_base_speed;
        const float speed_command_diff = speed_loop_debug_enabled
                                             ? pid_tuning::line_follow::kSpeedLoopDebugDiffSpeed
                                             : delta_v_cmd;

        // 速度环入口统一接收 base + diff。diff 为正时右轮更快、左轮更慢；
        // 最终左右轮目标和 diff 限幅都由 motor_thread 统一派生，避免上级和速度环各自分配。
        motor_thread_set_speed_command(speed_command_base, speed_command_diff);

        g_turn_output.store(motor_thread_diff_speed_command());

        bool track_point_valid = false;
        int track_point_x = 0;
        int track_point_y = 0;
        vision_image_processor_get_ipm_line_error_track_point(&track_point_valid, &track_point_x, &track_point_y);
        const float current_track_point_angle_deg =
            compute_signed_track_point_angle_deg(track_point_valid, track_point_x, track_point_y);
        const float measured_yaw_rate_dps = imu_thread_gyro_z_dps() * pid_tuning::imu::kGyroYawRateSign;

        {
            std::lock_guard<std::mutex> lock(g_pid_debug_mutex);
            g_pid_debug_status.vision_updated = vision_updated;
            g_pid_debug_status.imu_updated = imu_updated;
            g_pid_debug_status.route_main_state = route_main_state;
            g_pid_debug_status.route_sub_state = route_sub_state;
            g_pid_debug_status.normal_speed_reference = current_normal_speed_reference;
            g_pid_debug_status.profile_base_speed = profile_base_speed;
            g_pid_debug_status.desired_base_speed = profile_base_speed;
            g_pid_debug_status.applied_base_speed = profile_base_speed;
            g_pid_debug_status.raw_error_px = pos_error_px;
            g_pid_debug_status.filtered_error_px = pos_error_px;
            g_pid_debug_status.abs_filtered_error_px = std::fabs(pos_error_px);
            g_pid_debug_status.control_error_px = pos_error_px;
            g_pid_debug_status.track_point_valid = track_point_valid;
            g_pid_debug_status.track_point_x = track_point_x;
            g_pid_debug_status.track_point_y = track_point_y;
            g_pid_debug_status.current_track_point_angle_deg = current_track_point_angle_deg;
            g_pid_debug_status.filtered_track_point_angle_deg = g_filtered_track_point_angle_deg;
            g_pid_debug_status.measured_yaw_rate_dps = measured_yaw_rate_dps;
            g_pid_debug_status.yaw_rate_ref_from_pos_dps = yaw_rate_ref_from_pos_dps;
            g_pid_debug_status.yaw_rate_ref_final_dps = yaw_rate_ref_final_dps;
            g_pid_debug_status.yaw_rate_ref_dps = yaw_rate_ref_final_dps;
            g_pid_debug_status.yaw_rate_error_dps = yaw_rate_error_dps;
            g_pid_debug_status.delta_v_cmd = delta_v_cmd;
            g_pid_debug_status.target_yaw_rate_abs_filtered_dps = 0.0f;
            g_pid_debug_status.target_yaw_rate_speed_scale = 1.0f;
            g_pid_debug_status.dynamic_position_kp = route_profile.position_kp;
            g_pid_debug_status.dynamic_yaw_rate_kp = route_profile.yaw_rate_kp;
            g_pid_debug_status.applied_yaw_rate_kp = route_profile.yaw_rate_kp;
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
            g_pid_debug_status.route_yaw_rate_ref_gain = 1.0f;
            g_pid_debug_status.route_yaw_rate_ref_limit = route_profile.position_max_output;
            g_pid_debug_status.route_steering_max_output = 0.0f;
            g_pid_debug_status.route_yaw_rate_kp_enable_error_threshold_px = 0.0f;
            g_pid_debug_status.mean_abs_path_error = vision_image_processor_ipm_mean_abs_offset_error();
            g_pid_debug_status.speed_scheme_blended_abs_error_sum = 0.0f;
            g_pid_debug_status.speed_scheme_realtime_speed = 0.0f;
            g_pid_debug_status.speed_scheme_error_scale_raw = 1.0f;
            g_pid_debug_status.speed_scheme_final_speed_scale = 1.0f;
            g_pid_debug_status.speed_scheme_split_ratio = 0.0f;
            g_pid_debug_status.speed_scheme_point_count = 0;
            g_pid_debug_status.speed_scheme_ready = true;
            g_pid_debug_status.speed_scheme_triggered = false;
            g_pid_debug_status.speed_scheme_winner_branch = 0;
            g_pid_debug_status.speed_scheme_max_drop_ratio_per_cycle = 0.0f;
            g_pid_debug_status.speed_scheme_max_rise_ratio_per_cycle = 0.0f;
            g_pid_debug_status.force_full_speed = false;
            g_pid_debug_status.speed_command_base = motor_thread_base_speed_command();
            g_pid_debug_status.speed_command_diff = motor_thread_diff_speed_command();
            g_pid_debug_status.raw_steering_output = delta_v_cmd;
            g_pid_debug_status.clamped_steering_output = delta_v_cmd;
            g_pid_debug_status.applied_steering_output = motor_thread_diff_speed_command();
            g_pid_debug_status.left_target_count = motor_thread_left_target_count();
            g_pid_debug_status.right_target_count = motor_thread_right_target_count();
            g_pid_debug_status.speed_debug_left_target_applied = speed_loop_debug_enabled ? motor_thread_left_target_count() : 0.0f;
            g_pid_debug_status.speed_debug_right_target_applied = speed_loop_debug_enabled ? motor_thread_right_target_count() : 0.0f;
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
    position_pid1.init(default_profile.position_kp,
                       default_profile.position_ki,
                       default_profile.position_kd,
                       default_profile.position_max_integral,
                       default_profile.position_max_output);
    position_pid1.set_target(0.0f);
    // 角速度环：目标仍为 0，但运行时使用 compute_by_error(r_ref - r)。
    // 串级模式下它直接输出差速速度量（delta_v）。
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

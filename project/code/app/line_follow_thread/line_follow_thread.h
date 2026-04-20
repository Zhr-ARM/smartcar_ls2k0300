#ifndef LINE_FOLLOW_THREAD_H_
#define LINE_FOLLOW_THREAD_H_

#include "zf_common_headfile.h"

struct LineFollowPidDebugStatus
{
    bool vision_updated;
    bool imu_updated;
    int route_main_state;
    int route_sub_state;
    float normal_speed_reference;
    float profile_base_speed;
    float desired_base_speed;
    float applied_base_speed;
    float raw_error_px;
    float filtered_error_px;
    float abs_filtered_error_px;
    float control_error_px;
    bool track_point_valid;
    int track_point_x;
    int track_point_y;
    float current_track_point_angle_deg;
    float filtered_track_point_angle_deg;
    float measured_yaw_rate_dps;
    float yaw_rate_ref_dps;
    float yaw_rate_error_dps;
    float dynamic_position_kp;
    float dynamic_yaw_rate_kp;
    float applied_yaw_rate_kp;
    float position_pid_kp;
    float position_pid_ki;
    float position_pid_kd;
    float position_pid_target;
    float position_pid_error;
    float position_pid_integral;
    float position_pid_output;
    float position_pid_max_integral;
    float position_pid_max_output;
    float yaw_pid_kp;
    float yaw_pid_ki;
    float yaw_pid_kd;
    float yaw_pid_target;
    float yaw_pid_error;
    float yaw_pid_integral;
    float yaw_pid_output;
    float yaw_pid_max_integral;
    float yaw_pid_max_output;
    float route_yaw_rate_ref_gain;
    float route_yaw_rate_ref_limit;
    float route_steering_max_output;
    float route_yaw_rate_kp_enable_error_threshold_px;
    float mean_abs_path_error;
    float speed_scheme_blended_abs_error_sum;
    float speed_scheme_error_scale_raw;
    float speed_scheme_final_speed_scale;
    float speed_scheme_split_ratio;
    float speed_scheme_rear_exp_lambda;
    float speed_scheme_slowdown_start;
    float speed_scheme_slowdown_full;
    float speed_scheme_min_speed_scale;
    float speed_scheme_max_speed_scale;
    int speed_scheme_point_count;
    bool speed_scheme_ready;
    bool speed_scheme_triggered;
    int speed_scheme_winner_branch;
    float speed_scheme_max_drop_ratio_per_cycle;
    float speed_scheme_max_rise_ratio_per_cycle;
    int speed_scheme_centerline_count_lower;
    int speed_scheme_centerline_count_upper;
    float speed_scheme_centerline_scale_lower;
    float speed_scheme_centerline_scale_upper;
    float speed_scheme_centerline_scale_current;
    int speed_scheme_force_full_min_centerline_count;
    float speed_scheme_force_full_abs_error_sum_per_point;
    float speed_scheme_force_full_abs_error_sum_current;
    float speed_scheme_force_full_abs_error_sum_threshold;
    bool speed_scheme_force_full_centerline_ready;
    bool speed_scheme_force_full_error_ready;
    bool force_full_speed;
    float raw_steering_output;
    float clamped_steering_output;
    float applied_steering_output;
    float left_target_count;
    float right_target_count;
    float vision_dt_ms;
    float imu_dt_ms;
};

/**
 * @brief 初始化巡线控制线程
 * @return 初始化成功返回 true，失败返回 false
 */
bool line_follow_thread_init();

/**
 * @brief 停止巡线控制线程
 */
void line_follow_thread_cleanup();

/**
 * @brief 打印巡线线程的调度信息
 */
void line_follow_thread_print_info();

/**
 * @brief 设置巡线直道参考速度
 * @param speed NORMAL 档参考速度；其他状态会按各自 profile 相对缩放
 */
void line_follow_thread_set_normal_speed_reference(float speed);

/**
 * @brief 获取当前巡线误差，单位为像素
 * @return 正值表示赛道中心偏左，负值表示赛道中心偏右
 */
float line_follow_thread_error();

/**
 * @brief 获取当前位置环输出到左右轮的差速值
 * @return 当前差速目标值
 */
float line_follow_thread_turn_output();

/**
 * @brief 获取当前巡线直道参考速度
 * @return 当前 NORMAL 档参考速度设定值
 */
float line_follow_thread_normal_speed_reference();

/**
 * @brief 获取当前实际参与左右轮目标合成的基础速度
 * @return 当前真正下发给左右轮合成逻辑的基础速度；若尚未进入有效控制周期则返回直道参考速度
 */
float line_follow_thread_applied_base_speed();
bool line_follow_thread_get_pid_debug_status(LineFollowPidDebugStatus &status);
void line_follow_thread_request_reload_from_globals();

#endif

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
    float normalized_error;
    float filtered_error_norm;
    float filtered_error_px;
    float abs_filtered_error_px;
    float control_error_norm;
    float control_error_px;
    bool track_point_valid;
    int track_point_x;
    int track_point_y;
    float current_track_point_angle_deg;
    float filtered_track_point_angle_deg;
    float measured_yaw_rate_dps;
    float yaw_rate_ref_dps;
    float yaw_rate_error_dps;
    float yaw_rate_error_norm;
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

#endif

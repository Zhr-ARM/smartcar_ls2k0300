#ifndef __MOTOR_THREAD_H__
#define __MOTOR_THREAD_H__

#include "zf_common_headfile.h"

/**
 * @brief 电机双通道 PID 参数
 */
struct MotorPidParams
{
    float left_kp;   // 左侧比例系数
    float left_ki;   // 左侧积分系数
    float left_kd;   // 左侧微分系数
    float right_kp;  // 右侧比例系数
    float right_ki;  // 右侧积分系数
    float right_kd;  // 右侧微分系数
};

/**
 * @brief 电机串口状态数据
 */
struct MotorUartStatus
{
    float left_target_count;  // 左轮目标计数值
    float right_target_count; // 右轮目标计数值
    float left_error;         // 左轮当前误差
    float right_error;        // 右轮当前误差
    float left_current_count; // 左轮当前反馈计数值
    float right_current_count;// 右轮当前反馈计数值
    float left_duty;          // 左轮当前占空比
    float right_duty;         // 右轮当前占空比
};

/**
 * @brief 初始化电机控制线程
 * @return 成功返回 true，失败返回 false
 */
bool motor_thread_init();

/**
 * @brief 设置目标计数值
 * @param left_count 左轮目标计数(counts/5ms)
 * @param right_count 右轮目标计数(counts/5ms)
 */
void motor_thread_set_target_count(float left_count, float right_count);

/**
 * @brief 获取左轮当前反馈计数
 * @return 左轮实际编码器增量值
 */
float motor_thread_left_count();

/**
 * @brief 获取右轮当前反馈计数
 * @return 右轮实际编码器增量值
 */
float motor_thread_right_count();

/**
 * @brief 获取左轮当前占空比
 * @return 左轮输出的PWM占空比
 */
float motor_thread_left_duty();

/**
 * @brief 获取右轮当前占空比
 * @return 右轮输出的PWM占空比
 */
float motor_thread_right_duty();

/**
 * @brief 获取左轮目标计数值
 * @return 左轮设定的目标增量
 */
float motor_thread_left_target_count();

/**
 * @brief 获取右轮目标计数值
 * @return 右轮设定的目标增量
 */
float motor_thread_right_target_count();

/**
 * @brief 获取电机串口上报状态
 * @return 包含目标值和误差的状态数据
 */
MotorUartStatus motor_thread_uart_status();

/**
 * @brief 获取当前PID参数
 * @param params 用于接收 PID 参数的输出对象
 * @return 成功返回 true，失败返回 false
 */
bool motor_thread_get_pid_params(MotorPidParams &params);

/**
 * @brief 更新PID参数
 * @param params 新的 PID 参数
 * @return 参数合法并更新成功返回 true，失败返回 false
 */
bool motor_thread_set_pid_params(const MotorPidParams &params);

/**
 * @brief 清理电机控制线程
 */
void motor_thread_cleanup();

/**
 * @brief 打印调试信息
 */
void motor_thread_print_info();

#endif

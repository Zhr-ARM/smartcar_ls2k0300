#ifndef LINE_FOLLOW_THREAD_H_
#define LINE_FOLLOW_THREAD_H_

#include "zf_common_headfile.h"

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

#endif

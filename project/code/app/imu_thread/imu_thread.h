#ifndef __IMU_THREAD_H__
#define __IMU_THREAD_H__

#include "zf_common_headfile.h"
#include "imu660ra.h"

/**
 * @brief 初始化 IMU 数据采集线程
 * @return 初始化成功返回 true，失败返回 false
 */
bool imu_thread_init();

/**
 * @brief 获取当前姿态角
 * @return 当前欧拉角，单位为度
 */
Imu660raEuler imu_thread_attitude_deg();

/**
 * @brief 停止 IMU 数据采集线程
 */
void imu_thread_cleanup();

/**
 * @brief 打印 IMU 线程的调度信息
 */
void imu_thread_print_info();

#endif

#ifndef __IMU_THREAD_H__
#define __IMU_THREAD_H__

#include "zf_common_headfile.h"
#include "imu660ra.h"

/**
 * @brief 初始化 IMU 数据采集线程
 * 作用：完成 IMU 设备探测和驱动就绪，但不启动后台采集线程
 * @return 初始化成功返回 true，失败返回 false
 */
bool imu_thread_init();

/**
 * @brief 在指定时长内完成陀螺仪零偏标定，并启动后台采集线程
 * @param calibrate_duration_ms 标定时长，单位 ms
 * @return 标定并启动成功返回 true，失败返回 false
 */
bool imu_thread_calibrate_and_start(int32 calibrate_duration_ms);

/**
 * @brief 获取当前滤波并去零偏后的 Z 轴角速度
 * @return 当前 gyro_z，单位 deg/s
 */
float imu_thread_gyro_z_dps();

/**
 * @brief 停止 IMU 数据采集线程
 */
void imu_thread_cleanup();

/**
 * @brief 打印 IMU 线程的调度信息
 */
void imu_thread_print_info();

#endif

#ifndef IMU660RA_H_
#define IMU660RA_H_

#include "zf_common_headfile.h"

#define IMU660RA_GYRO_SCALE_NODE "in_anglvel_scale"
#define IMU660RA_DEFAULT_FILTER_ALPHA (0.70f)

// 与逐飞内核驱动中的默认量程保持一致:
// GYRO 默认 ±2000dps, raw / 16.4 = dps
#define IMU660RA_FALLBACK_GYRO_SCALE_RAD_S ((3.1415926535f / 180.0f) / 16.4f)

/**
 * @brief IMU660RA 原始角速度数据
 */
struct Imu660raData
{
    int16 gyro_x;
    int16 gyro_y;
    int16 gyro_z;
};

/**
 * @brief 三轴浮点向量
 */
struct Imu660raFloat3
{
    float x;
    float y;
    float z;
};

/**
 * @brief IMU660RA 角速度驱动
 * 作用：负责 IMU 采样、量程换算和角速度滤波
 */
class Imu660raDriver
{
public:
    /**
     * @brief 构造 IMU 驱动对象
     */
    Imu660raDriver();

    /**
     * @brief 初始化 IMU 设备和解算状态
     * @return 初始化成功返回 true，失败返回 false
     */
    bool init();

    /**
     * @brief 执行一次角速度更新
     * @param dt_s 本次更新周期，单位 s，传负值时自动估计
     * @return 更新成功返回 true，失败返回 false
     */
    bool update(float dt_s = -1.0f);

    /**
     * @brief 更新角速度原始数据
     * @return 更新成功返回 true，失败返回 false
     */
    bool update_gyro();

    /**
     * @brief 重新加载量程系数
     * @return 加载成功返回 true，失败返回 false
     */
    bool reload_scale();

    /**
     * @brief 清空滤波状态
     */
    void clear();

    /**
     * @brief 获取当前角速度量程系数
     * @return 每个 raw 对应的角速度，单位 rad/s
     */
    float gyro_scale_rad_s() const;

    /**
     * @brief 获取滤波后的 Z 轴角速度
     * @return Z 轴角速度，单位 deg/s
     */
    float filtered_gyro_z_deg_s() const;

    /**
     * @brief 获取最近一次初始化或运行错误信息
     * @return 错误信息字符串
     */
    const char *last_error() const;

private:
    /**
     * @brief 读取 sysfs 量程系数
     * @param path 量程节点路径
     * @param value 用于返回解析结果
     * @return 读取成功返回 true，失败返回 false
     */
    bool read_scale(const char *path, float &value) const;

    /**
     * @brief 将原始陀螺仪采样值转换为物理量
     */
    void convert_raw_gyro_data();

    /**
     * @brief 更新陀螺仪低通滤波结果
     */
    void apply_gyro_lowpass_filter();

    /**
     * @brief 更新最近一次错误信息
     * @param message 错误信息字符串
     */
    void set_error(const char *message);

    bool online_;
    bool filter_ready_;
    float filter_alpha_;
    float gyro_scale_;
    Imu660raData data_;
    Imu660raFloat3 gyro_rad_s_;
    Imu660raFloat3 filtered_gyro_rad_s_;
    char last_error_[96];
};

extern Imu660raDriver imu660ra_driver;

#endif

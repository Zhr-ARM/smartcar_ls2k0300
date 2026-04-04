#ifndef IMU660RA_H_
#define IMU660RA_H_

#include "zf_common_headfile.h"

#define IMU660RA_ACC_SCALE_NODE "in_accel_scale"
#define IMU660RA_GYRO_SCALE_NODE "in_anglvel_scale"

#define IMU660RA_DEFAULT_DT_S (0.01f)
#define IMU660RA_DEFAULT_FILTER_ALPHA (0.70f)
#define IMU660RA_DEFAULT_MAHONY_KP (0.65f)
#define IMU660RA_DEFAULT_MAHONY_KI (0.00f)
#define IMU660RA_DEFAULT_ERROR_LPF_HZ (1.00f)
#define IMU660RA_DEFAULT_ERROR_INTEGRAL_LIMIT (0.035f)
#define IMU660RA_GRAVITY_MS2 (9.80665f)
#define IMU660RA_ACC_VALID_MIN_G (0.80f)
#define IMU660RA_ACC_VALID_MAX_G (1.20f)
#define IMU660RA_ACC_AXIS_LIMIT_G (1.05f)

// 与逐飞内核驱动中的默认量程保持一致:
// ACC 默认 ±8g, raw / 4096 = g
// GYRO 默认 ±2000dps, raw / 16.4 = dps
#define IMU660RA_FALLBACK_ACC_SCALE_MS2 (9.80665f / 4096.0f)
#define IMU660RA_FALLBACK_GYRO_SCALE_RAD_S ((3.1415926535f / 180.0f) / 16.4f)

/**
 * @brief IMU660RA 原始六轴数据
 */
struct Imu660raData
{
    int16 acc_x;
    int16 acc_y;
    int16 acc_z;
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
 * @brief 欧拉角姿态数据
 * 作用：保存横滚、俯仰和航向角，单位为度
 */
struct Imu660raEuler
{
    float roll;
    float pitch;
    float yaw;
};

/**
 * @brief IMU660RA 姿态驱动
 * 作用：负责 IMU 采样、滤波和姿态角解算
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
     * @brief 执行一次完整姿态更新
     * @param dt_s 本次更新周期，单位 s，传负值时自动估计
     * @return 更新成功返回 true，失败返回 false
     */
    bool update(float dt_s = -1.0f);

    /**
     * @brief 更新加速度原始数据
     * @return 更新成功返回 true，失败返回 false
     */
    bool update_acc();

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
     * @brief 清空滤波和姿态状态
     */
    void clear();

    /**
     * @brief 设置采样周期
     * @param dt_s 采样周期，单位 s
     */
    void set_sample_period(float dt_s);

    /**
     * @brief 设置低通滤波系数
     * @param alpha 滤波系数
     */
    void set_filter_alpha(float alpha);

    /**
     * @brief 设置 Mahony 增益
     * @param kp 比例增益
     * @param ki 积分增益
     */
    void set_mahony_gain(float kp, float ki);

    /**
     * @brief 获取当前加速度物理量
     * @return 三轴加速度，单位 m/s^2
     */
    Imu660raFloat3 acc_ms2() const;

    /**
     * @brief 获取当前加速度量程系数
     * @return 每个 raw 对应的加速度，单位 m/s^2
     */
    float acc_scale_ms2() const;

    /**
     * @brief 获取当前角速度量程系数
     * @return 每个 raw 对应的角速度，单位 rad/s
     */
    float gyro_scale_rad_s() const;

    /**
     * @brief 获取当前姿态角
     * @return 欧拉角姿态，单位为度
     */
    Imu660raEuler attitude_deg() const;

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

    /**
     * @brief 获取 X 轴加速度原始值
     * @return X 轴加速度原始值
     */
    int16 acc_x() const;

    /**
     * @brief 获取 Y 轴加速度原始值
     * @return Y 轴加速度原始值
     */
    int16 acc_y() const;

    /**
     * @brief 获取 Z 轴加速度原始值
     * @return Z 轴加速度原始值
     */
    int16 acc_z() const;

    /**
     * @brief 获取 X 轴角速度原始值
     * @return X 轴角速度原始值
     */
    int16 gyro_x() const;

    /**
     * @brief 获取 Y 轴角速度原始值
     * @return Y 轴角速度原始值
     */
    int16 gyro_y() const;

    /**
     * @brief 获取 Z 轴角速度原始值
     * @return Z 轴角速度原始值
     */
    int16 gyro_z() const;

private:
    /**
     * @brief 读取 sysfs 量程系数
     * @param path 量程节点路径
     * @param value 用于返回解析结果
     * @return 读取成功返回 true，失败返回 false
     */
    bool read_scale(const char *path, float &value) const;

    /**
     * @brief 约束并平滑更新周期
     * @param dt_s 输入周期，单位 s
     * @return 处理后的周期值
     */
    float sanitize_dt(float dt_s);

    /**
     * @brief 估计当前运动强度
     * @param ax X 轴加速度
     * @param ay Y 轴加速度
     * @param az Z 轴加速度
     * @param gx X 轴角速度
     * @param gy Y 轴角速度
     * @param gz Z 轴角速度
     * @return 归一化运动强度
     */
    float calc_motion_factor(float ax, float ay, float az, float gx, float gy, float gz) const;

    /**
     * @brief 计算加速度修正权重
     * @param ax X 轴加速度
     * @param ay Y 轴加速度
     * @param az Z 轴加速度
     * @param gx X 轴角速度
     * @param gy Y 轴角速度
     * @param gz Z 轴角速度
     * @return 加速度修正权重
     */
    float calc_accel_weight(float ax, float ay, float az, float gx, float gy, float gz) const;

    /**
     * @brief 将原始采样值转换为物理量
     */
    void convert_raw_data();

    /**
     * @brief 更新低通滤波结果
     */
    void apply_lowpass_filter();

    /**
     * @brief 执行互补滤波更新
     * @param gx X 轴角速度，单位 rad/s
     * @param gy Y 轴角速度，单位 rad/s
     * @param gz Z 轴角速度，单位 rad/s
     * @param ax X 轴加速度，单位 m/s^2
     * @param ay Y 轴加速度，单位 m/s^2
     * @param az Z 轴加速度，单位 m/s^2
     * @param dt_s 本次积分周期，单位 s
     */
    void complementary_update(float gx, float gy, float gz, float ax, float ay, float az, float dt_s);

    /**
     * @brief 执行 Mahony 姿态更新
     * @param gx X 轴角速度，单位 rad/s
     * @param gy Y 轴角速度，单位 rad/s
     * @param gz Z 轴角速度，单位 rad/s
     * @param ax X 轴加速度，单位 m/s^2
     * @param ay Y 轴加速度，单位 m/s^2
     * @param az Z 轴加速度，单位 m/s^2
     * @param dt_s 本次积分周期，单位 s
     */
    void mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float dt_s);

    /**
     * @brief 更新重力误差估计
     * @param ax X 轴加速度，单位 m/s^2
     * @param ay Y 轴加速度，单位 m/s^2
     * @param az Z 轴加速度，单位 m/s^2
     * @param dt_s 本次更新周期，单位 s
     */
    void update_gravity_error(float ax, float ay, float az, float dt_s);

    /**
     * @brief 判断当前加速度是否可用于姿态修正
     * @param ax X 轴加速度，单位 m/s^2
     * @param ay Y 轴加速度，单位 m/s^2
     * @param az Z 轴加速度，单位 m/s^2
     * @return 可用于修正返回 true，否则返回 false
     */
    bool accel_correction_valid(float ax, float ay, float az) const;

    /**
     * @brief 将内部姿态状态更新为欧拉角
     */
    void update_euler();

    /**
     * @brief 更新最近一次错误信息
     * @param message 错误信息字符串
     */
    void set_error(const char *message);

    bool online_;
    bool filter_ready_;
    bool complementary_ready_;
    bool attitude_ready_;
    float sample_period_s_;
    float dt_estimate_s_;
    float filter_alpha_;
    float mahony_kp_;
    float mahony_ki_;
    float error_lpf_hz_;
    float error_integral_limit_;
    float motion_factor_;
    float accel_weight_;
    float adaptive_filter_alpha_;
    float acc_scale_;
    float gyro_scale_;
    float q0_;
    float q1_;
    float q2_;
    float q3_;
    Imu660raData data_;
    Imu660raFloat3 acc_ms2_;
    Imu660raFloat3 gyro_rad_s_;
    Imu660raFloat3 filtered_acc_ms2_;
    Imu660raFloat3 filtered_gyro_rad_s_;
    Imu660raFloat3 gravity_error_;
    Imu660raFloat3 integral_error_;
    Imu660raEuler attitude_deg_;
    Imu660raEuler complementary_deg_;
    char last_error_[96];
};

extern Imu660raDriver imu660ra_driver;

#endif

#include "imu660ra.h"

#include <math.h>
#include <stdlib.h>

#define IMU660RA_PI (3.1415926535f)
#define IMU660RA_RAD_TO_DEG (57.2957795f)
#define IMU660RA_MIN_DT_S (0.0001f)
#define IMU660RA_MAX_DT_SCALE (3.0f)
#define IMU660RA_DT_BLEND (0.20f)
#define IMU660RA_DYNAMIC_ALPHA_DROP (0.45f)
#define IMU660RA_MIN_FILTER_ALPHA (0.05f)
#define IMU660RA_GYRO_DYNAMIC_START_RAD_S (1.00f)
#define IMU660RA_GYRO_DYNAMIC_END_RAD_S (5.50f)
#define IMU660RA_ACC_DEV_START_MS2 (0.10f * IMU660RA_GRAVITY_MS2)
#define IMU660RA_ACC_DEV_END_MS2 (0.45f * IMU660RA_GRAVITY_MS2)
#define IMU660RA_COMPLEMENTARY_ALPHA_STATIC (0.960f)
#define IMU660RA_COMPLEMENTARY_ALPHA_DYNAMIC (0.995f)
#define IMU660RA_COMPLEMENTARY_BLEND_MAX (0.30f)
#define IMU660RA_INTEGRAL_LEAK_RATE (2.00f)
#define IMU660RA_MIN_CORRECTION_WEIGHT (0.05f)

/**
 * @brief 对浮点数进行区间限幅
 * @param value 输入值
 * @param lower 下限
 * @param upper 上限
 * @return 限幅后的数值
 */
static float clamp_float(float value, float lower, float upper)
{
    if (value < lower)
    {
        return lower;
    }
    if (value > upper)
    {
        return upper;
    }
    return value;
}

/**
 * @brief 对两个浮点值执行线性插值
 * @param start 起始值
 * @param end 结束值
 * @param weight 插值权重
 * @return 插值结果
 */
static float lerp_float(float start, float end, float weight)
{
    const float clamped_weight = clamp_float(weight, 0.0f, 1.0f);
    return start + (end - start) * clamped_weight;
}

/**
 * @brief 执行 smoothstep 平滑映射
 * @param lower 下限
 * @param upper 上限
 * @param value 输入值
 * @return 平滑映射结果
 */
static float smoothstep_float(float lower, float upper, float value)
{
    if (upper <= lower)
    {
        return (value >= upper) ? 1.0f : 0.0f;
    }

    const float x = clamp_float((value - lower) / (upper - lower), 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

/**
 * @brief 将角度包裹到 [-180, 180]
 * @param angle_deg 输入角度，单位为度
 * @return 包裹后的角度
 */
static float wrap_angle_deg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

/**
 * @brief 在角度空间执行插值
 * @param base_deg 基准角度
 * @param target_deg 目标角度
 * @param weight 插值权重
 * @return 插值后的角度
 */
static float blend_angle_deg(float base_deg, float target_deg, float weight)
{
    const float delta_deg = wrap_angle_deg(target_deg - base_deg);
    return wrap_angle_deg(base_deg + delta_deg * clamp_float(weight, 0.0f, 1.0f));
}

Imu660raDriver::Imu660raDriver()
    : online_(false),
      filter_ready_(false),
      complementary_ready_(false),
      attitude_ready_(false),
      sample_period_s_(IMU660RA_DEFAULT_DT_S),
      dt_estimate_s_(IMU660RA_DEFAULT_DT_S),
      filter_alpha_(IMU660RA_DEFAULT_FILTER_ALPHA),
      mahony_kp_(IMU660RA_DEFAULT_MAHONY_KP),
      mahony_ki_(IMU660RA_DEFAULT_MAHONY_KI),
      error_lpf_hz_(IMU660RA_DEFAULT_ERROR_LPF_HZ),
      error_integral_limit_(IMU660RA_DEFAULT_ERROR_INTEGRAL_LIMIT),
      motion_factor_(0.0f),
      accel_weight_(0.0f),
      adaptive_filter_alpha_(IMU660RA_DEFAULT_FILTER_ALPHA),
      acc_scale_(0.0f),
      gyro_scale_(0.0f),
      q0_(1.0f),
      q1_(0.0f),
      q2_(0.0f),
      q3_(0.0f)
{
    clear();
    set_error("IMU660RA 未初始化");
}

bool Imu660raDriver::init()
{
    clear();
    imu_get_dev_info();

    if (DEV_IMU660RA != imu_type)
    {
        online_ = false;

        if (DEV_IMU660RB == imu_type)
        {
            set_error("检测到 IMU660RB,当前库仅支持 IMU660RA");
        }
        else if (DEV_IMU963RA == imu_type)
        {
            set_error("检测到 IMU963RA,当前库仅支持 IMU660RA");
        }
        else
        {
            set_error("未检测到 IMU660RA");
        }
        return false;
    }

    online_ = true;
    if (!reload_scale())
    {
        online_ = false;
        return false;
    }

    set_error("");
    return update(sample_period_s_);
}

bool Imu660raDriver::update(float dt_s)
{
    if (!online_)
    {
        return false;
    }

    if (!update_acc() || !update_gyro())
    {
        return false;
    }

    const float step = sanitize_dt(dt_s);

    convert_raw_data();
    motion_factor_ = calc_motion_factor(acc_ms2_.x,
                                        acc_ms2_.y,
                                        acc_ms2_.z,
                                        gyro_rad_s_.x,
                                        gyro_rad_s_.y,
                                        gyro_rad_s_.z);
    apply_lowpass_filter();
    motion_factor_ = calc_motion_factor(filtered_acc_ms2_.x,
                                        filtered_acc_ms2_.y,
                                        filtered_acc_ms2_.z,
                                        filtered_gyro_rad_s_.x,
                                        filtered_gyro_rad_s_.y,
                                        filtered_gyro_rad_s_.z);
    accel_weight_ = calc_accel_weight(filtered_acc_ms2_.x,
                                      filtered_acc_ms2_.y,
                                      filtered_acc_ms2_.z,
                                      filtered_gyro_rad_s_.x,
                                      filtered_gyro_rad_s_.y,
                                      filtered_gyro_rad_s_.z);
    complementary_update(filtered_gyro_rad_s_.x,
                         filtered_gyro_rad_s_.y,
                         filtered_gyro_rad_s_.z,
                         filtered_acc_ms2_.x,
                         filtered_acc_ms2_.y,
                         filtered_acc_ms2_.z,
                         step);
    mahony_update(filtered_gyro_rad_s_.x,
                  filtered_gyro_rad_s_.y,
                  filtered_gyro_rad_s_.z,
                  filtered_acc_ms2_.x,
                  filtered_acc_ms2_.y,
                  filtered_acc_ms2_.z,
                  step);
    update_euler();

    return true;
}

bool Imu660raDriver::update_acc()
{
    if (!online_)
    {
        return false;
    }

    imu660ra_get_acc();
    data_.acc_x = imu660ra_acc_x;
    data_.acc_y = imu660ra_acc_y;
    data_.acc_z = imu660ra_acc_z;

    return true;
}

bool Imu660raDriver::update_gyro()
{
    if (!online_)
    {
        return false;
    }

    imu660ra_get_gyro();
    data_.gyro_x = imu660ra_gyro_x;
    data_.gyro_y = imu660ra_gyro_y;
    data_.gyro_z = imu660ra_gyro_z;

    return true;
}

bool Imu660raDriver::reload_scale()
{
    acc_scale_ = IMU660RA_FALLBACK_ACC_SCALE_MS2;
    gyro_scale_ = IMU660RA_FALLBACK_GYRO_SCALE_RAD_S;

    float value = 0.0f;
    if (read_scale(IMU660RA_ACC_SCALE_PATH, value))
    {
        acc_scale_ = value;
    }

    value = 0.0f;
    if (read_scale(IMU660RA_GYRO_SCALE_PATH, value))
    {
        gyro_scale_ = value;
    }

    return true;
}

void Imu660raDriver::clear()
{
    filter_ready_ = false;
    complementary_ready_ = false;
    attitude_ready_ = false;
    dt_estimate_s_ = sample_period_s_;
    motion_factor_ = 0.0f;
    accel_weight_ = 0.0f;
    adaptive_filter_alpha_ = filter_alpha_;
    q0_ = 1.0f;
    q1_ = 0.0f;
    q2_ = 0.0f;
    q3_ = 0.0f;

    memset(&data_, 0, sizeof(data_));
    memset(&acc_ms2_, 0, sizeof(acc_ms2_));
    memset(&gyro_rad_s_, 0, sizeof(gyro_rad_s_));
    memset(&filtered_acc_ms2_, 0, sizeof(filtered_acc_ms2_));
    memset(&filtered_gyro_rad_s_, 0, sizeof(filtered_gyro_rad_s_));
    memset(&gravity_error_, 0, sizeof(gravity_error_));
    memset(&integral_error_, 0, sizeof(integral_error_));
    memset(&attitude_deg_, 0, sizeof(attitude_deg_));
    memset(&complementary_deg_, 0, sizeof(complementary_deg_));
}

void Imu660raDriver::set_sample_period(float dt_s)
{
    if (dt_s >= IMU660RA_MIN_DT_S)
    {
        sample_period_s_ = dt_s;
        if (dt_estimate_s_ < IMU660RA_MIN_DT_S)
        {
            dt_estimate_s_ = dt_s;
        }
    }
}

void Imu660raDriver::set_filter_alpha(float alpha)
{
    filter_alpha_ = clamp_float(alpha, 0.0f, 0.99f);
}

void Imu660raDriver::set_mahony_gain(float kp, float ki)
{
    mahony_kp_ = (kp >= 0.0f) ? kp : 0.0f;
    mahony_ki_ = (ki >= 0.0f) ? ki : 0.0f;
}

Imu660raFloat3 Imu660raDriver::acc_ms2() const
{
    return acc_ms2_;
}

Imu660raEuler Imu660raDriver::attitude_deg() const
{
    return attitude_deg_;
}

int16 Imu660raDriver::acc_x() const
{
    return data_.acc_x;
}

int16 Imu660raDriver::acc_y() const
{
    return data_.acc_y;
}

int16 Imu660raDriver::acc_z() const
{
    return data_.acc_z;
}

int16 Imu660raDriver::gyro_x() const
{
    return data_.gyro_x;
}

int16 Imu660raDriver::gyro_y() const
{
    return data_.gyro_y;
}

int16 Imu660raDriver::gyro_z() const
{
    return data_.gyro_z;
}

/**
 * @brief 从 sysfs 节点读取量程系数
 * @param path 量程文件路径
 * @param value 用于返回解析结果
 * @return 读取成功返回 true，失败返回 false
 */
bool Imu660raDriver::read_scale(const char *path, float &value) const
{
    FILE *fp = fopen(path, "r");
    if (NULL == fp)
    {
        return false;
    }

    char str[32] = {0};
    int ret = fscanf(fp, "%31s", str);
    fclose(fp);

    if (1 != ret)
    {
        return false;
    }

    value = atof(str);
    return (value > 0.0f);
}

float Imu660raDriver::sanitize_dt(float dt_s)
{
    const float max_dt_s = fmaxf(sample_period_s_ * IMU660RA_MAX_DT_SCALE, IMU660RA_MIN_DT_S);
    float bounded_dt_s = dt_s;

    if (bounded_dt_s < IMU660RA_MIN_DT_S)
    {
        bounded_dt_s = (dt_estimate_s_ >= IMU660RA_MIN_DT_S) ? dt_estimate_s_ : sample_period_s_;
    }

    bounded_dt_s = clamp_float(bounded_dt_s, IMU660RA_MIN_DT_S, max_dt_s);
    dt_estimate_s_ += IMU660RA_DT_BLEND * (bounded_dt_s - dt_estimate_s_);

    return dt_estimate_s_;
}

float Imu660raDriver::calc_motion_factor(float ax, float ay, float az, float gx, float gy, float gz) const
{
    const float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    const float acc_dev = fabsf(acc_norm - IMU660RA_GRAVITY_MS2);
    const float gyro_norm = sqrtf(gx * gx + gy * gy + gz * gz);

    const float gyro_factor = smoothstep_float(IMU660RA_GYRO_DYNAMIC_START_RAD_S,
                                               IMU660RA_GYRO_DYNAMIC_END_RAD_S,
                                               gyro_norm);
    const float acc_factor = smoothstep_float(IMU660RA_ACC_DEV_START_MS2,
                                              IMU660RA_ACC_DEV_END_MS2,
                                              acc_dev);

    return clamp_float(0.60f * gyro_factor + 0.40f * acc_factor, 0.0f, 1.0f);
}

float Imu660raDriver::calc_accel_weight(float ax, float ay, float az, float gx, float gy, float gz) const
{
    const float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (acc_norm <= 0.15f * IMU660RA_GRAVITY_MS2)
    {
        return 0.0f;
    }

    const float axis_limit = IMU660RA_ACC_AXIS_LIMIT_G * IMU660RA_GRAVITY_MS2;
    const float axis_peak = fmaxf(fabsf(ax), fmaxf(fabsf(ay), fabsf(az)));
    if (axis_peak > axis_limit * 1.35f)
    {
        return 0.0f;
    }

    const float gyro_norm = sqrtf(gx * gx + gy * gy + gz * gz);
    const float acc_dev = fabsf(acc_norm - IMU660RA_GRAVITY_MS2);

    const float norm_weight = 1.0f - smoothstep_float(IMU660RA_ACC_DEV_START_MS2,
                                                      IMU660RA_ACC_DEV_END_MS2,
                                                      acc_dev);
    const float axis_weight = 1.0f - smoothstep_float(axis_limit * 0.70f,
                                                      axis_limit * 1.10f,
                                                      axis_peak);
    const float gyro_weight = 1.0f - 0.75f * smoothstep_float(IMU660RA_GYRO_DYNAMIC_START_RAD_S,
                                                              IMU660RA_GYRO_DYNAMIC_END_RAD_S,
                                                              gyro_norm);

    float weight = norm_weight * axis_weight * clamp_float(gyro_weight, 0.0f, 1.0f);
    if (!accel_correction_valid(ax, ay, az))
    {
        weight *= 0.35f;
    }

    return clamp_float(weight, 0.0f, 1.0f);
}

/**
 * @brief 将原始采样值转换为物理量
 */
void Imu660raDriver::convert_raw_data()
{
    acc_ms2_.x = data_.acc_x * acc_scale_;
    acc_ms2_.y = data_.acc_y * acc_scale_;
    acc_ms2_.z = data_.acc_z * acc_scale_;

    gyro_rad_s_.x = data_.gyro_x * gyro_scale_;
    gyro_rad_s_.y = data_.gyro_y * gyro_scale_;
    gyro_rad_s_.z = data_.gyro_z * gyro_scale_;
}

/**
 * @brief 更新低通滤波结果
 */
void Imu660raDriver::apply_lowpass_filter()
{
    if (!filter_ready_)
    {
        adaptive_filter_alpha_ = filter_alpha_;
        filtered_acc_ms2_ = acc_ms2_;
        filtered_gyro_rad_s_ = gyro_rad_s_;
        filter_ready_ = true;
        return;
    }

    const float min_alpha = clamp_float(filter_alpha_ - IMU660RA_DYNAMIC_ALPHA_DROP,
                                        IMU660RA_MIN_FILTER_ALPHA,
                                        filter_alpha_);
    adaptive_filter_alpha_ = lerp_float(filter_alpha_, min_alpha, motion_factor_);
    const float input_gain = 1.0f - adaptive_filter_alpha_;

    filtered_acc_ms2_.x = adaptive_filter_alpha_ * filtered_acc_ms2_.x + input_gain * acc_ms2_.x;
    filtered_acc_ms2_.y = adaptive_filter_alpha_ * filtered_acc_ms2_.y + input_gain * acc_ms2_.y;
    filtered_acc_ms2_.z = adaptive_filter_alpha_ * filtered_acc_ms2_.z + input_gain * acc_ms2_.z;

    filtered_gyro_rad_s_.x = adaptive_filter_alpha_ * filtered_gyro_rad_s_.x + input_gain * gyro_rad_s_.x;
    filtered_gyro_rad_s_.y = adaptive_filter_alpha_ * filtered_gyro_rad_s_.y + input_gain * gyro_rad_s_.y;
    filtered_gyro_rad_s_.z = adaptive_filter_alpha_ * filtered_gyro_rad_s_.z + input_gain * gyro_rad_s_.z;
}

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
void Imu660raDriver::complementary_update(float gx, float gy, float gz, float ax, float ay, float az, float dt_s)
{
    const float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (acc_norm <= 0.0f)
    {
        if (complementary_ready_)
        {
            complementary_deg_.roll = wrap_angle_deg(complementary_deg_.roll + gx * dt_s * IMU660RA_RAD_TO_DEG);
            complementary_deg_.pitch = wrap_angle_deg(complementary_deg_.pitch + gy * dt_s * IMU660RA_RAD_TO_DEG);
            complementary_deg_.yaw = wrap_angle_deg(complementary_deg_.yaw + gz * dt_s * IMU660RA_RAD_TO_DEG);
        }
        return;
    }

    const float acc_roll_deg = atan2f(ay, az) * IMU660RA_RAD_TO_DEG;
    const float acc_pitch_deg = atan2f(-ax, sqrtf(ay * ay + az * az)) * IMU660RA_RAD_TO_DEG;

    if (!complementary_ready_)
    {
        complementary_deg_.roll = acc_roll_deg;
        complementary_deg_.pitch = acc_pitch_deg;
        complementary_deg_.yaw = attitude_deg_.yaw;
        complementary_ready_ = true;
        return;
    }

    complementary_deg_.roll = wrap_angle_deg(complementary_deg_.roll + gx * dt_s * IMU660RA_RAD_TO_DEG);
    complementary_deg_.pitch = wrap_angle_deg(complementary_deg_.pitch + gy * dt_s * IMU660RA_RAD_TO_DEG);
    complementary_deg_.yaw = wrap_angle_deg(complementary_deg_.yaw + gz * dt_s * IMU660RA_RAD_TO_DEG);

    const float gyro_bias_weight = clamp_float((1.0f - accel_weight_) * 0.65f + motion_factor_ * 0.35f,
                                               0.0f,
                                               1.0f);
    const float alpha = lerp_float(IMU660RA_COMPLEMENTARY_ALPHA_STATIC,
                                   IMU660RA_COMPLEMENTARY_ALPHA_DYNAMIC,
                                   gyro_bias_weight);
    const float acc_gain = 1.0f - alpha;

    complementary_deg_.roll = blend_angle_deg(complementary_deg_.roll, acc_roll_deg, acc_gain);
    complementary_deg_.pitch = blend_angle_deg(complementary_deg_.pitch, acc_pitch_deg, acc_gain);
}

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
void Imu660raDriver::mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float dt_s)
{
    const float step = (dt_s >= IMU660RA_MIN_DT_S) ? dt_s : sample_period_s_;
    update_gravity_error(ax, ay, az, step);

    const float correction_weight = clamp_float(accel_weight_, 0.0f, 1.0f);
    const float integral_limit = error_integral_limit_ * (0.20f + 0.80f * correction_weight);
    const float integral_leak = clamp_float(1.0f - IMU660RA_INTEGRAL_LEAK_RATE * step * (1.0f - correction_weight),
                                            0.0f,
                                            1.0f);
    const float dynamic_ki = mahony_ki_ * correction_weight * correction_weight;
    const float dynamic_kp = mahony_kp_ * correction_weight;

    integral_error_.x *= integral_leak;
    integral_error_.y *= integral_leak;
    integral_error_.z *= integral_leak;

    if (dynamic_ki > 0.0f && correction_weight > IMU660RA_MIN_CORRECTION_WEIGHT)
    {
        integral_error_.x += dynamic_ki * gravity_error_.x * step;
        integral_error_.y += dynamic_ki * gravity_error_.y * step;
        integral_error_.z += dynamic_ki * gravity_error_.z * step;

        integral_error_.x = clamp_float(integral_error_.x, -integral_limit, integral_limit);
        integral_error_.y = clamp_float(integral_error_.y, -integral_limit, integral_limit);
        integral_error_.z = clamp_float(integral_error_.z, -integral_limit, integral_limit);
    }

    gx += integral_error_.x;
    gy += integral_error_.y;
    gz += integral_error_.z;

    gx += dynamic_kp * gravity_error_.x;
    gy += dynamic_kp * gravity_error_.y;
    gz += dynamic_kp * gravity_error_.z;

    gx *= 0.5f * step;
    gy *= 0.5f * step;
    gz *= 0.5f * step;

    const float qa = q0_;
    const float qb = q1_;
    const float qc = q2_;
    const float qd = q3_;

    q0_ += -qb * gx - qc * gy - qd * gz;
    q1_ += qa * gx + qc * gz - qd * gy;
    q2_ += qa * gy - qb * gz + qd * gx;
    q3_ += qa * gz + qb * gy - qc * gx;

    const float quat_norm = sqrtf(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    if (quat_norm <= 0.0f)
    {
        q0_ = 1.0f;
        q1_ = 0.0f;
        q2_ = 0.0f;
        q3_ = 0.0f;
        return;
    }

    const float inv_quat_norm = 1.0f / quat_norm;
    q0_ *= inv_quat_norm;
    q1_ *= inv_quat_norm;
    q2_ *= inv_quat_norm;
    q3_ *= inv_quat_norm;
    attitude_ready_ = true;
}

/**
 * @brief 更新重力误差估计
 * @param ax X 轴加速度，单位 m/s^2
 * @param ay Y 轴加速度，单位 m/s^2
 * @param az Z 轴加速度，单位 m/s^2
 * @param dt_s 本次更新周期，单位 s
 */
void Imu660raDriver::update_gravity_error(float ax, float ay, float az, float dt_s)
{
    const float filter_gain = clamp_float(error_lpf_hz_ * IMU660RA_PI * dt_s, 0.0f, 1.0f);
    const float correction_weight = clamp_float(accel_weight_, 0.0f, 1.0f);

    const float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (acc_norm <= 0.0f)
    {
        gravity_error_.x += filter_gain * (0.0f - gravity_error_.x);
        gravity_error_.y += filter_gain * (0.0f - gravity_error_.y);
        gravity_error_.z += filter_gain * (0.0f - gravity_error_.z);
        return;
    }

    const float inv_acc_norm = 1.0f / acc_norm;
    ax *= inv_acc_norm;
    ay *= inv_acc_norm;
    az *= inv_acc_norm;

    const float vx = 2.0f * (q1_ * q3_ - q0_ * q2_);
    const float vy = 2.0f * (q0_ * q1_ + q2_ * q3_);
    const float vz = q0_ * q0_ - q1_ * q1_ - q2_ * q2_ + q3_ * q3_;

    const float ex = ay * vz - az * vy;
    const float ey = az * vx - ax * vz;
    const float ez = ax * vy - ay * vx;

    gravity_error_.x += filter_gain * (correction_weight * ex - gravity_error_.x);
    gravity_error_.y += filter_gain * (correction_weight * ey - gravity_error_.y);
    gravity_error_.z += filter_gain * (correction_weight * ez - gravity_error_.z);
}

/**
 * @brief 判断当前加速度是否可用于姿态修正
 * @param ax X 轴加速度，单位 m/s^2
 * @param ay Y 轴加速度，单位 m/s^2
 * @param az Z 轴加速度，单位 m/s^2
 * @return 可用于修正返回 true，否则返回 false
 */
bool Imu660raDriver::accel_correction_valid(float ax, float ay, float az) const
{
    const float axis_limit = IMU660RA_ACC_AXIS_LIMIT_G * IMU660RA_GRAVITY_MS2;
    if (fabsf(ax) > axis_limit || fabsf(ay) > axis_limit || fabsf(az) > axis_limit)
    {
        return false;
    }

    const float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    const float min_norm = IMU660RA_ACC_VALID_MIN_G * IMU660RA_GRAVITY_MS2;
    const float max_norm = IMU660RA_ACC_VALID_MAX_G * IMU660RA_GRAVITY_MS2;

    return (acc_norm >= min_norm && acc_norm <= max_norm);
}

/**
 * @brief 将内部姿态状态更新为欧拉角
 */
void Imu660raDriver::update_euler()
{
    const float sin_pitch = 2.0f * (q0_ * q2_ - q3_ * q1_);
    const float limited_sin_pitch = clamp_float(sin_pitch, -1.0f, 1.0f);

    const float quat_roll_deg = atan2f(2.0f * (q0_ * q1_ + q2_ * q3_),
                                       1.0f - 2.0f * (q1_ * q1_ + q2_ * q2_)) * IMU660RA_RAD_TO_DEG;
    const float quat_pitch_deg = asinf(limited_sin_pitch) * IMU660RA_RAD_TO_DEG;
    const float quat_yaw_deg = atan2f(2.0f * (q0_ * q3_ + q1_ * q2_),
                                      1.0f - 2.0f * (q2_ * q2_ + q3_ * q3_)) * IMU660RA_RAD_TO_DEG;

    attitude_deg_.roll = quat_roll_deg;
    attitude_deg_.pitch = quat_pitch_deg;
    attitude_deg_.yaw = quat_yaw_deg;

    if (complementary_ready_)
    {
        const float complementary_blend = clamp_float((0.08f + 0.22f * accel_weight_) * (1.0f - 0.35f * motion_factor_),
                                                      0.0f,
                                                      IMU660RA_COMPLEMENTARY_BLEND_MAX);
        attitude_deg_.roll = blend_angle_deg(quat_roll_deg, complementary_deg_.roll, complementary_blend);
        attitude_deg_.pitch = blend_angle_deg(quat_pitch_deg, complementary_deg_.pitch, complementary_blend);
        complementary_deg_.yaw = blend_angle_deg(complementary_deg_.yaw, quat_yaw_deg, 0.05f);
    }
}

/**
 * @brief 更新最近一次错误信息
 * @param message 错误信息字符串
 */
void Imu660raDriver::set_error(const char *message)
{
    if (NULL == message)
    {
        last_error_[0] = '\0';
        return;
    }

    strncpy(last_error_, message, sizeof(last_error_) - 1);
    last_error_[sizeof(last_error_) - 1] = '\0';
}

Imu660raDriver imu660ra_driver;

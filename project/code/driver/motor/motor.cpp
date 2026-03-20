#include "motor.h"
#include <chrono>

namespace
{
constexpr float kControlPeriodS = 0.005f;
}

/**
 * @brief 构造单路电机对象
 * @param dir_path 方向控制 GPIO 节点路径
 * @param pwm_path PWM 设备节点路径
 * @param encoder_path 编码器节点路径
 * @param encoder_sign 编码器方向修正系数
 */
Motor::Motor(const char *dir_path, const char *pwm_path, const char *encoder_path, int8 encoder_sign)
    : dir_path_(dir_path),
      pwm_path_(pwm_path),
      encoder_path_(encoder_path),
      encoder_sign_(encoder_sign),
      encoder_raw_last_(0),
      encoder_5ms_count_(0),
      current_duty_percent_(0),
      actual_dt_s_(0.005f),
      last_update_time_(std::chrono::steady_clock::now())
{
    memset(&pwm_info_, 0, sizeof(pwm_info_));
}

/**
 * @brief 初始化电机输出与编码器基准
 */
void Motor::init()
{
    pwm_get_dev_info(pwm_path_, &pwm_info_);
    encoder_raw_last_ = read_encoder_raw();
    encoder_5ms_count_ = 0;
    actual_dt_s_ = kControlPeriodS;
    last_update_time_ = std::chrono::steady_clock::now();
    stop();
}

/**
 * @brief 更新编码器计数差值
 */
void Motor::update_encoder(uint32 elapsed_periods)
{
    auto now = std::chrono::steady_clock::now();
    float measured_dt_s = std::chrono::duration<float>(now - last_update_time_).count();
    last_update_time_ = now;

    if (0 == elapsed_periods)
    {
        elapsed_periods = 1;
    }

    // 以 timerfd 到期次数作为主参考，再用实际时间做小幅修正，降低调度抖动造成的速度估计毛刺。
    float expected_dt_s = kControlPeriodS * (float)elapsed_periods;
    if (measured_dt_s <= 0.000001f)
    {
        measured_dt_s = expected_dt_s;
    }

    float min_dt_s = expected_dt_s * 0.70f;
    float max_dt_s = expected_dt_s * 1.35f;
    if (measured_dt_s < min_dt_s)
    {
        measured_dt_s = min_dt_s;
    }
    else if (measured_dt_s > max_dt_s)
    {
        measured_dt_s = max_dt_s;
    }

    actual_dt_s_ = expected_dt_s * 0.75f + measured_dt_s * 0.25f;

    int32 encoder_raw_now = read_encoder_raw();
    encoder_5ms_count_ = encoder_raw_now - encoder_raw_last_;
    encoder_raw_last_ = encoder_raw_now;
}

/**
 * @brief 设置电机输出占空比
 * @param percent 目标占空比百分比
 */
void Motor::set_duty(float percent)
{
    apply_output(percent);
}

/**
 * @brief 停止电机输出
 */
void Motor::stop()
{
    apply_output(0.0f);
}

/**
 * @brief 获取当前编码器增量
 * @return 当前采样周期的编码器计数差
 */
int32 Motor::get_encoder() const
{
    return encoder_5ms_count_;
}

/**
 * @brief 获取标准化后的 5ms 编码器计数
 * @return 折算到 5ms 周期的编码器增量
 */
float Motor::get_count_5ms() const
{
    if (actual_dt_s_ <= 0.000001f)
    {
        return 0.0f;
    }
    return (float)encoder_5ms_count_ * (kControlPeriodS / actual_dt_s_);
}

/**
 * @brief 获取当前逻辑占空比
 * @return 当前占空比百分比
 */
float Motor::get_duty() const
{
    return current_duty_percent_;
}

/**
 * @brief 读取编码器原始累计值
 * @return 编码器累计计数
 */
int32 Motor::read_encoder_raw() const
{
    return encoder_sign_ * encoder_get_count(encoder_path_);
}

/**
 * @brief 约束占空比输出范围
 * @param percent 输入占空比百分比
 * @return 限幅后的占空比百分比
 */
float Motor::clamp_percent(float percent) const
{
    if (percent > (float)MOTOR_MAX_DUTY_PERCENT)
    {
        return (float)MOTOR_MAX_DUTY_PERCENT;
    }
    if (percent < -(float)MOTOR_MAX_DUTY_PERCENT)
    {
        return -(float)MOTOR_MAX_DUTY_PERCENT;
    }
    return percent;
}

/**
 * @brief 将占空比百分比换算为原始占空比
 * @param duty_percent 占空比百分比
 * @return 原始占空比值
 */
uint32 Motor::percent_to_raw_duty(float duty_percent) const
{
    float abs_percent = (duty_percent >= 0.0f) ? duty_percent : -duty_percent;
    return (uint32)(abs_percent * (float)pwm_info_.duty_max / 100.0f);
}

/**
 * @brief 将逻辑占空比写入底层硬件
 * @param duty_percent 目标占空比百分比
 */
void Motor::apply_output(float duty_percent)
{
    current_duty_percent_ = clamp_percent(duty_percent);
    float hardware_duty = -current_duty_percent_;
    gpio_set_level(dir_path_, (hardware_duty >= 0.0f) ? 1 : 0);
    pwm_set_duty(pwm_path_, percent_to_raw_duty(hardware_duty));
}

/**
 * @brief 构造双路电机驱动对象
 */
MotorDriver::MotorDriver()
    : left_(LEFT_MOTOR_DIR, LEFT_MOTOR_PWM, LEFT_ENCODER_PATH, LEFT_ENCODER_SIGN),
      right_(RIGHT_MOTOR_DIR, RIGHT_MOTOR_PWM, RIGHT_ENCODER_PATH, RIGHT_ENCODER_SIGN)
{
}

/**
 * @brief 初始化左右电机模块
 */
void MotorDriver::init()
{
    left_.init();
    right_.init();
}

/**
 * @brief 执行 5ms 周期反馈更新
 */
void MotorDriver::update_5ms(uint32 elapsed_periods)
{
    left_.update_encoder(elapsed_periods);
    right_.update_encoder(elapsed_periods);
}

/**
 * @brief 停止所有电机输出
 */
void MotorDriver::stop_all()
{
    left_.stop();
    right_.stop();
}

/**
 * @brief 设置左侧电机占空比
 * @param percent 目标占空比百分比
 */
void MotorDriver::set_left_duty(float percent)
{
    left_.set_duty(percent);
}

/**
 * @brief 设置右侧电机占空比
 * @param percent 目标占空比百分比
 */
void MotorDriver::set_right_duty(float percent)
{
    right_.set_duty(percent);
}

/**
 * @brief 获取左侧编码器增量
 * @return 左侧编码器计数差
 */
int32 MotorDriver::left_encoder() const
{
    return left_.get_encoder();
}

/**
 * @brief 获取右侧编码器增量
 * @return 右侧编码器计数差
 */
int32 MotorDriver::right_encoder() const
{
    return right_.get_encoder();
}

/**
 * @brief 获取左侧 5ms 标准化计数
 * @return 左侧折算后的 5ms 编码器增量
 */
float MotorDriver::left_count_5ms() const
{
    return left_.get_count_5ms();
}

/**
 * @brief 获取右侧 5ms 标准化计数
 * @return 右侧折算后的 5ms 编码器增量
 */
float MotorDriver::right_count_5ms() const
{
    return right_.get_count_5ms();
}

/**
 * @brief 获取左侧逻辑占空比
 * @return 左侧占空比百分比
 */
float MotorDriver::left_duty() const
{
    return left_.get_duty();
}

/**
 * @brief 获取右侧逻辑占空比
 * @return 右侧占空比百分比
 */
float MotorDriver::right_duty() const
{
    return right_.get_duty();
}

MotorDriver motor_driver;

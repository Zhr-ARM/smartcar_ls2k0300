#include "motor.h"

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
      encoder_5ms_count_(0),
      current_duty_percent_(0),
      current_hardware_duty_(0),
      current_dir_level_(1)
{
    memset(&pwm_info_, 0, sizeof(pwm_info_));
}

/**
 * @brief 初始化电机输出与编码器基准
 */
void Motor::init()
{
    pwm_get_dev_info(pwm_path_, &pwm_info_);
    encoder_5ms_count_ = 0;
    stop();
}

/**
 * @brief 更新编码器计数差值
 */
void Motor::update_encoder(uint32 elapsed_periods)
{
    (void)elapsed_periods;
    // 方向编码器驱动每次 read 返回的已经是当前测量值，不是累计计数。
    encoder_5ms_count_ = read_encoder_raw();
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
    return (float)encoder_5ms_count_;
}

/**
 * @brief 获取当前逻辑占空比
 * @return 当前占空比百分比
 */
float Motor::get_duty() const
{
    return current_duty_percent_;
}

float Motor::get_hardware_duty() const
{
    return current_hardware_duty_;
}

int Motor::get_dir_level() const
{
    return current_dir_level_;
}

/**
 * @brief 读取编码器当前输出值
 * @return 编码器当前速度测量值
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
    current_hardware_duty_ = -current_duty_percent_;
    current_dir_level_ = (current_hardware_duty_ >= 0.0f) ? 1 : 0;
    gpio_set_level(dir_path_, current_dir_level_);
    pwm_set_duty(pwm_path_, percent_to_raw_duty(current_hardware_duty_));
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

float MotorDriver::left_hardware_duty() const
{
    return left_.get_hardware_duty();
}

float MotorDriver::right_hardware_duty() const
{
    return right_.get_hardware_duty();
}

int MotorDriver::left_dir_level() const
{
    return left_.get_dir_level();
}

int MotorDriver::right_dir_level() const
{
    return right_.get_dir_level();
}

MotorDriver motor_driver;

#include "brushless.h"

/**
 * @brief 构造单路无刷驱动对象
 * @param pwm_path PWM 设备节点路径
 */
Brushless::Brushless(const char *pwm_path)
    : pwm_path_(pwm_path),
      current_duty_percent_(0),
      current_raw_duty_(BRUSHLESS_PWM_OFF_DUTY)
{
    memset(&pwm_info_, 0, sizeof(pwm_info_));
}

/**
 * @brief 初始化无刷电调输出
 */
void Brushless::init()
{
    pwm_get_dev_info(pwm_path_, &pwm_info_);
    stop();
}

/**
 * @brief 按百分比设置无刷输出
 * @param percent 目标占空比百分比
 */
void Brushless::set_duty(float percent)
{
    float limited = clamp_percent(percent);
    apply_raw_duty(percent_to_raw_duty(limited));
    current_duty_percent_ = limited;
}

/**
 * @brief 直接设置原始占空比
 * @param duty 原始占空比值
 */
void Brushless::set_raw_duty(uint32 duty)
{
    uint32 limited = clamp_raw_duty(duty);
    apply_raw_duty(limited);
    current_duty_percent_ = raw_duty_to_percent(limited);
}

/**
 * @brief 输出最小有效油门
 */
void Brushless::stop()
{
    apply_raw_duty(get_min_raw_duty());
    current_duty_percent_ = 0.0f;
}

/**
 * @brief 关闭 PWM 输出
 */
void Brushless::disable()
{
    apply_raw_duty(BRUSHLESS_PWM_OFF_DUTY);
    current_duty_percent_ = 0.0f;
}

/**
 * @brief 获取当前百分比输出
 * @return 当前占空比百分比
 */
float Brushless::get_duty() const
{
    return current_duty_percent_;
}

/**
 * @brief 获取当前原始占空比
 * @return 当前原始占空比值
 */
uint32 Brushless::get_raw_duty() const
{
    return current_raw_duty_;
}

/**
 * @brief 获取最小有效原始占空比
 * @return 最小有效原始占空比值
 */
uint32 Brushless::get_min_raw_duty() const
{
    return pulse_to_raw_duty(BRUSHLESS_PWM_MIN_PULSE_US);
}

/**
 * @brief 获取最大有效原始占空比
 * @return 最大有效原始占空比值
 */
uint32 Brushless::get_max_raw_duty() const
{
    return pulse_to_raw_duty(BRUSHLESS_PWM_MAX_PULSE_US);
}

/**
 * @brief 约束百分比输出范围
 * @param percent 输入占空比百分比
 * @return 限幅后的占空比百分比
 */
float Brushless::clamp_percent(float percent) const
{
    if (percent > (float)BRUSHLESS_MAX_DUTY_PERCENT)
    {
        return (float)BRUSHLESS_MAX_DUTY_PERCENT;
    }
    if (percent < (float)BRUSHLESS_MIN_DUTY_PERCENT)
    {
        return (float)BRUSHLESS_MIN_DUTY_PERCENT;
    }
    return percent;
}

/**
 * @brief 约束原始占空比范围
 * @param duty 输入原始占空比
 * @return 限幅后的原始占空比
 */
uint32 Brushless::clamp_raw_duty(uint32 duty) const
{
    const uint32 min_duty = get_min_raw_duty();
    const uint32 max_duty = get_max_raw_duty();

    if (duty < BRUSHLESS_PWM_OFF_DUTY)
    {
        return BRUSHLESS_PWM_OFF_DUTY;
    }
    if (BRUSHLESS_PWM_OFF_DUTY == duty)
    {
        return BRUSHLESS_PWM_OFF_DUTY;
    }
    if (duty < min_duty)
    {
        return min_duty;
    }
    if (duty > max_duty)
    {
        return max_duty;
    }
    return duty;
}

/**
 * @brief 获取当前 PWM 的有效最大占空比
 * @return 最大原始占空比值
 */
uint32 Brushless::effective_duty_max() const
{
    if (0 != pwm_info_.duty_max)
    {
        return pwm_info_.duty_max;
    }
    return 10000;
}

/**
 * @brief 获取 PWM 周期
 * @return PWM 周期，单位 ns
 */
uint32 Brushless::period_ns() const
{
    if (0 != pwm_info_.period_ns)
    {
        return pwm_info_.period_ns;
    }
    if (0 != pwm_info_.freq)
    {
        return 1000000000U / pwm_info_.freq;
    }
    return 1000000000U / BRUSHLESS_DEFAULT_FREQ_HZ;
}

/**
 * @brief 将脉宽换算为原始占空比
 * @param pulse_us 脉宽，单位 us
 * @return 原始占空比值
 */
uint32 Brushless::pulse_to_raw_duty(uint32 pulse_us) const
{
    const unsigned long long denominator = period_ns();
    const unsigned long long numerator = (unsigned long long)effective_duty_max() * pulse_us * 1000ULL;
    unsigned long long raw_duty = 0;

    if (0 != denominator)
    {
        raw_duty = (numerator + denominator / 2ULL) / denominator;
    }

    if (raw_duty > effective_duty_max())
    {
        raw_duty = effective_duty_max();
    }
    return (uint32)raw_duty;
}

/**
 * @brief 将百分比输出换算为原始占空比
 * @param duty_percent 占空比百分比
 * @return 原始占空比值
 */
uint32 Brushless::percent_to_raw_duty(float duty_percent) const
{
    const uint32 min_duty = get_min_raw_duty();
    const uint32 max_duty = get_max_raw_duty();
    const uint32 range = max_duty - min_duty;

    return (uint32)(min_duty + duty_percent * (float)range / (float)BRUSHLESS_MAX_DUTY_PERCENT);
}

/**
 * @brief 将原始占空比换算为百分比输出
 * @param duty 原始占空比值
 * @return 占空比百分比
 */
float Brushless::raw_duty_to_percent(uint32 duty) const
{
    const uint32 min_duty = get_min_raw_duty();
    const uint32 max_duty = get_max_raw_duty();

    if (duty <= min_duty)
    {
        return 0.0f;
    }
    if (duty >= max_duty)
    {
        return (float)BRUSHLESS_MAX_DUTY_PERCENT;
    }
    if (max_duty == min_duty)
    {
        return 0.0f;
    }

    return (float)(duty - min_duty) * (float)BRUSHLESS_MAX_DUTY_PERCENT / (float)(max_duty - min_duty);
}

/**
 * @brief 将原始占空比写入硬件
 * @param duty 原始占空比值
 */
void Brushless::apply_raw_duty(uint32 duty)
{
    pwm_set_duty(pwm_path_, duty);
    pwm_info_.duty = duty;
    current_raw_duty_ = duty;
}

/**
 * @brief 构造双路无刷驱动对象
 */
BrushlessDriver::BrushlessDriver()
    : left_(LEFT_BRUSHLESS_PWM),
      right_(RIGHT_BRUSHLESS_PWM)
{
}

/**
 * @brief 初始化左右无刷电调
 */
void BrushlessDriver::init()
{
    left_.init();
    right_.init();
}

/**
 * @brief 停止左右无刷输出
 */
void BrushlessDriver::stop_all()
{
    left_.stop();
    right_.stop();
}

/**
 * @brief 关闭左右无刷 PWM 输出
 */
void BrushlessDriver::disable_all()
{
    left_.disable();
    right_.disable();
}

/**
 * @brief 设置左侧无刷输出
 * @param percent 目标占空比百分比
 */
void BrushlessDriver::set_left_duty(float percent)
{
    left_.set_duty(percent);
}

/**
 * @brief 设置右侧无刷输出
 * @param percent 目标占空比百分比
 */
void BrushlessDriver::set_right_duty(float percent)
{
    right_.set_duty(percent);
}

/**
 * @brief 直接设置左侧原始占空比
 * @param duty 原始占空比值
 */
void BrushlessDriver::set_left_raw_duty(uint32 duty)
{
    left_.set_raw_duty(duty);
}

/**
 * @brief 直接设置右侧原始占空比
 * @param duty 原始占空比值
 */
void BrushlessDriver::set_right_raw_duty(uint32 duty)
{
    right_.set_raw_duty(duty);
}

/**
 * @brief 获取左侧无刷输出
 * @return 左侧占空比百分比
 */
float BrushlessDriver::left_duty() const
{
    return left_.get_duty();
}

/**
 * @brief 获取右侧无刷输出
 * @return 右侧占空比百分比
 */
float BrushlessDriver::right_duty() const
{
    return right_.get_duty();
}

/**
 * @brief 获取左侧原始占空比
 * @return 左侧原始占空比值
 */
uint32 BrushlessDriver::left_raw_duty() const
{
    return left_.get_raw_duty();
}

/**
 * @brief 获取右侧原始占空比
 * @return 右侧原始占空比值
 */
uint32 BrushlessDriver::right_raw_duty() const
{
    return right_.get_raw_duty();
}

BrushlessDriver brushless_driver;

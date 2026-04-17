#include "pid.h"
#include <cmath>

/**
 * @brief 构造增量式 PID 控制器
 */
PidController::PidController()
    : kp_(0), ki_(0), kd_(0),
      target_(0.0f), output_(0.0f),
      output_min_(0), output_max_(0),
      error_(0.0f), error_prev_(0.0f), error_prev2_(0.0f),
      integral_limit_(0.0f),
      last_output_(0.0f), max_output_step_(1000.0f) // 默认不限制
{
}

/**
 * @brief 初始化增量式 PID 参数
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param output_min 输出下限
 * @param output_max 输出上限
 */
void PidController::init(float kp, float ki, float kd, float output_min, float output_max)
{
    set_params(kp, ki, kd);
    set_output_limit(output_min, output_max);
    reset();
}

/**
 * @brief 设置目标值
 * @param target_value 目标值
 */
void PidController::set_target(float target_value)
{
    target_ = target_value;
}

/**
 * @brief 获取当前目标值
 * @return 当前目标值
 */
float PidController::get_target() const
{
    return target_;
}

/**
 * @brief 执行一次增量式 PID 计算
 * @param current_value 当前反馈值
 * @return 当前输出结果
 */
float PidController::compute(float current_value)
{
    error_ = target_ - current_value;

    float proportional_increment = kp_ * (error_ - error_prev_);
    float integral_error = error_;
    if (integral_limit_ > 0.0f)
    {
        integral_error = clamp(integral_error, -integral_limit_, integral_limit_);
    }
    float integral_increment = ki_ * integral_error;
    float derivative_increment = kd_ * (error_ - 2.0f * error_prev_ + error_prev2_);
    float delta_output = proportional_increment + integral_increment + derivative_increment;

    delta_output = clamp(delta_output, -max_output_step_, max_output_step_);
    output_ = last_output_ + delta_output;
    output_ = clamp(output_, output_min_, output_max_);
    last_output_ = output_;
    error_prev2_ = error_prev_;
    error_prev_ = error_;

    return output_;
}

/**
 * @brief 重置控制器内部状态
 */
void PidController::reset()
{
    error_      = 0.0f;
    error_prev_ = 0.0f;
    error_prev2_ = 0.0f;
    output_     = 0.0f;
    last_output_ = 0.0f;
}

/**
 * @brief 更新 PID 参数
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void PidController::set_params(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

/**
 * @brief 设置积分限幅
 * @param limit 限幅绝对值
 */
void PidController::set_integral_limit(float limit)
{
    integral_limit_ = fabsf(limit);
}

/**
 * @brief 设置单次输出变化限幅
 * @param step 最大变化步长
 */
void PidController::set_max_output_step(float step)
{
    max_output_step_ = fabsf(step);
}

/**
 * @brief 设置输出范围
 * @param min_val 输出下限
 * @param max_val 输出上限
 */
void PidController::set_output_limit(float min_val, float max_val)
{
    if (min_val > max_val)
    {
        const float temp = min_val;
        min_val = max_val;
        max_val = temp;
    }

    output_min_ = min_val;
    output_max_ = max_val;
    output_ = clamp(output_, output_min_, output_max_);
    last_output_ = clamp(last_output_, output_min_, output_max_);
}

/**
 * @brief 获取比例系数
 * @return 当前比例系数
 */
float PidController::kp() const
{
    return kp_;
}

/**
 * @brief 获取积分系数
 * @return 当前积分系数
 */
float PidController::ki() const
{
    return ki_;
}

/**
 * @brief 获取微分系数
 * @return 当前微分系数
 */
float PidController::kd() const
{
    return kd_;
}

float PidController::output_min() const
{
    return output_min_;
}

float PidController::output_max() const
{
    return output_max_;
}

float PidController::integral_limit() const
{
    return integral_limit_;
}

float PidController::max_output_step() const
{
    return max_output_step_;
}

/**
 * @brief 获取当前输出值
 * @return 当前输出结果
 */
float PidController::get_output() const
{
    return output_;
}

/**
 * @brief 获取当前误差
 * @return 当前控制误差
 */
float PidController::get_error() const
{
    return error_;
}

/**
 * @brief 用真实执行输出同步控制器内部状态
 * @param output_value 执行器最终生效的输出值
 */
void PidController::track_output(float output_value)
{
    output_ = clamp(output_value, output_min_, output_max_);
    last_output_ = output_;
}

/**
 * @brief 对数值进行区间限幅
 * @param value 输入值
 * @param min_val 下限
 * @param max_val 上限
 * @return 限幅后的数值
 */
float PidController::clamp(float value, float min_val, float max_val) const
{
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}

static constexpr float kOutputMin = -30.0f;
static constexpr float kOutputMax = 30.0f;

/**
 * @brief 构造双通道 PID 控制器
 */
DualPidController::DualPidController()
    : left_(), right_()
{
}

/**
 * @brief 初始化左右双通道 PID 参数
 * @param left_kp 左路比例系数
 * @param left_ki 左路积分系数
 * @param left_kd 左路微分系数
 * @param right_kp 右路比例系数
 * @param right_ki 右路积分系数
 * @param right_kd 右路微分系数
 */
void DualPidController::init(float left_kp, float left_ki, float left_kd,
                             float right_kp, float right_ki, float right_kd)
{
    left_.init(left_kp, left_ki, left_kd, kOutputMin, kOutputMax);
    right_.init(right_kp, right_ki, right_kd, kOutputMin, kOutputMax);
}

/**
 * @brief 设置左右目标值
 * @param left_target 左路目标值
 * @param right_target 右路目标值
 */
void DualPidController::set_target(float left_target, float right_target)
{
    left_.set_target(left_target);
    right_.set_target(right_target);
}

/**
 * @brief 计算左右双通道输出
 * @param left_value 左路反馈值
 * @param right_value 右路反馈值
 * @param left_duty 用于返回左路输出
 * @param right_duty 用于返回右路输出
 */
void DualPidController::compute(float left_value, float right_value,
                                float &left_duty, float &right_duty)
{
    left_duty  = left_.compute(left_value);
    right_duty = right_.compute(right_value);
}

/**
 * @brief 重置左右控制器状态
 */
void DualPidController::reset()
{
    left_.reset();
    right_.reset();
}

/**
 * @brief 设置双通道输出范围
 * @param min_val 输出下限
 * @param max_val 输出上限
 */
void DualPidController::set_output_limit(float min_val, float max_val)
{
    left_.set_output_limit(min_val, max_val);
    right_.set_output_limit(min_val, max_val);
}

/**
 * @brief 获取左路控制器
 * @return 左路 PID 控制器引用
 */
PidController &DualPidController::left_pid()
{
    return left_;
}

/**
 * @brief 获取左路控制器只读引用
 * @return 左路 PID 控制器只读引用
 */
const PidController &DualPidController::left_pid() const
{
    return left_;
}

/**
 * @brief 获取右路控制器
 * @return 右路 PID 控制器引用
 */
PidController &DualPidController::right_pid()
{
    return right_;
}

/**
 * @brief 获取右路控制器只读引用
 * @return 右路 PID 控制器只读引用
 */
const PidController &DualPidController::right_pid() const
{
    return right_;
}

/**
 * @brief 构造速度反馈滤波器
 */
MotorSpeedPidController::SpeedFeedbackFilter::SpeedFeedbackFilter()
    : raw_prev1(0.0f),
      raw_prev2(0.0f),
      averaged_sum(0.0f),
      averaged_count(0),
      averaged_index(0),
      filtered(0.0f),
      initialized(false)
{
    for (int32 i = 0; i < kFeedbackAverageWindowCapacity; ++i)
    {
        averaged_history[i] = 0.0f;
    }
}

/**
 * @brief 重置速度反馈滤波器状态
 */
void MotorSpeedPidController::SpeedFeedbackFilter::reset()
{
    raw_prev1 = 0.0f;
    raw_prev2 = 0.0f;
    averaged_sum = 0.0f;
    averaged_count = 0;
    averaged_index = 0;
    filtered = 0.0f;
    initialized = false;

    for (int32 i = 0; i < kFeedbackAverageWindowCapacity; ++i)
    {
        averaged_history[i] = 0.0f;
    }
}

int32 MotorSpeedPidController::feedback_average_window()
{
    return std::clamp(pid_tuning::motor_speed::kFeedbackAverageWindow,
                      1,
                      kFeedbackAverageWindowCapacity);
}

float MotorSpeedPidController::feedback_low_pass_alpha()
{
    return clamp_float(pid_tuning::motor_speed::kFeedbackLowPassAlpha, 0.0f, 1.0f);
}

/**
 * @brief 构造双轮速度环控制器
 */
MotorSpeedPidController::MotorSpeedPidController()
    : pid_(),
      config_(default_config(kOutputMax)),
      left_filter_(),
      right_filter_(),
      mutex_()
{
}

/**
 * @brief 生成默认速度环配置
 * @param duty_limit 输出占空比绝对值上限
 * @return 默认配置
 */
MotorSpeedPidConfig MotorSpeedPidController::default_config(float duty_limit)
{
    MotorSpeedPidConfig config;
    config.pid_params.left_kp = pid_tuning::motor_speed::kLeftKp;
    config.pid_params.left_ki = pid_tuning::motor_speed::kLeftKi;
    config.pid_params.left_kd = pid_tuning::motor_speed::kLeftKd;
    config.pid_params.right_kp = pid_tuning::motor_speed::kRightKp;
    config.pid_params.right_ki = pid_tuning::motor_speed::kRightKi;
    config.pid_params.right_kd = pid_tuning::motor_speed::kRightKd;
    config.integral_limit = pid_tuning::motor_speed::kIntegralLimit;
    config.max_output_step = pid_tuning::motor_speed::kMaxOutputStep;
    config.correction_limit = pid_tuning::motor_speed::kCorrectionLimit;
    config.duty_limit = duty_limit;
    config.left_feedforward_gain = pid_tuning::motor_speed::kLeftFeedforwardGain;
    config.right_feedforward_gain = pid_tuning::motor_speed::kRightFeedforwardGain;
    config.left_feedforward_bias = pid_tuning::motor_speed::kLeftFeedforwardBias;
    config.right_feedforward_bias = pid_tuning::motor_speed::kRightFeedforwardBias;
    config.feedforward_bias_threshold = pid_tuning::motor_speed::kFeedforwardBiasThreshold;
    config.decel_error_threshold = pid_tuning::motor_speed::kDecelErrorThreshold;
    config.decel_duty_gain = pid_tuning::motor_speed::kDecelDutyGain;
    config.decel_duty_limit = pid_tuning::motor_speed::kDecelDutyLimit;
    return config;
}

/**
 * @brief 初始化速度环配置的限幅和符号
 */
void MotorSpeedPidController::sanitize_config()
{
    config_.integral_limit = fabsf(config_.integral_limit);
    config_.max_output_step = fabsf(config_.max_output_step);
    config_.correction_limit = fabsf(config_.correction_limit);
    config_.duty_limit = fabsf(config_.duty_limit);
    config_.left_feedforward_gain = fabsf(config_.left_feedforward_gain);
    config_.right_feedforward_gain = fabsf(config_.right_feedforward_gain);
    config_.left_feedforward_bias = fabsf(config_.left_feedforward_bias);
    config_.right_feedforward_bias = fabsf(config_.right_feedforward_bias);
    config_.feedforward_bias_threshold = fabsf(config_.feedforward_bias_threshold);
    config_.decel_error_threshold = fabsf(config_.decel_error_threshold);
    config_.decel_duty_gain = fabsf(config_.decel_duty_gain);
    config_.decel_duty_limit = fabsf(config_.decel_duty_limit);

    if (config_.duty_limit <= 0.0f)
    {
        config_.duty_limit = fabsf(kOutputMax);
    }
    if (config_.correction_limit > config_.duty_limit)
    {
        config_.correction_limit = config_.duty_limit;
    }
}

/**
 * @brief 重置双侧反馈滤波器
 */
void MotorSpeedPidController::reset_filters()
{
    left_filter_.reset();
    right_filter_.reset();
}

/**
 * @brief 初始化速度环控制器
 * @param config 速度环配置
 */
void MotorSpeedPidController::init(const MotorSpeedPidConfig &config)
{
    std::lock_guard<std::mutex> lock(mutex_);

    config_ = config;
    sanitize_config();

    pid_.init(config_.pid_params.left_kp, config_.pid_params.left_ki, config_.pid_params.left_kd,
              config_.pid_params.right_kp, config_.pid_params.right_ki, config_.pid_params.right_kd);
    pid_.set_target(0.0f, 0.0f);
    pid_.set_output_limit(-config_.correction_limit, config_.correction_limit);
    pid_.left_pid().set_integral_limit(config_.integral_limit);
    pid_.right_pid().set_integral_limit(config_.integral_limit);
    pid_.left_pid().set_max_output_step(config_.max_output_step);
    pid_.right_pid().set_max_output_step(config_.max_output_step);
    pid_.reset();
    reset_filters();
}

/**
 * @brief 重置速度环内部状态
 */
void MotorSpeedPidController::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    pid_.reset();
    reset_filters();
}

/**
 * @brief 获取当前 PID 参数
 * @param params 输出参数
 */
void MotorSpeedPidController::get_pid_params(DualPidParams &params) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    params.left_kp = pid_.left_pid().kp();
    params.left_ki = pid_.left_pid().ki();
    params.left_kd = pid_.left_pid().kd();
    params.right_kp = pid_.right_pid().kp();
    params.right_ki = pid_.right_pid().ki();
    params.right_kd = pid_.right_pid().kd();
}

void MotorSpeedPidController::get_debug_info(MotorSpeedPidDebugInfo &info) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    info.left_kp = config_.pid_params.left_kp;
    info.left_ki = config_.pid_params.left_ki;
    info.left_kd = config_.pid_params.left_kd;
    info.right_kp = config_.pid_params.right_kp;
    info.right_ki = config_.pid_params.right_ki;
    info.right_kd = config_.pid_params.right_kd;
    info.integral_limit = config_.integral_limit;
    info.max_output_step = config_.max_output_step;
    info.correction_limit = config_.correction_limit;
    info.duty_limit = config_.duty_limit;
    info.left_feedforward_gain = config_.left_feedforward_gain;
    info.right_feedforward_gain = config_.right_feedforward_gain;
    info.left_feedforward_bias = config_.left_feedforward_bias;
    info.right_feedforward_bias = config_.right_feedforward_bias;
    info.feedforward_bias_threshold = config_.feedforward_bias_threshold;
    info.decel_error_threshold = config_.decel_error_threshold;
    info.decel_duty_gain = config_.decel_duty_gain;
    info.decel_duty_limit = config_.decel_duty_limit;

    const PidController &left = pid_.left_pid();
    const PidController &right = pid_.right_pid();
    info.left_pid_target = left.get_target();
    info.right_pid_target = right.get_target();
    info.left_pid_error = left.get_error();
    info.right_pid_error = right.get_error();
    info.left_pid_output = left.get_output();
    info.right_pid_output = right.get_output();
    info.left_pid_output_min = left.output_min();
    info.right_pid_output_min = right.output_min();
    info.left_pid_output_max = left.output_max();
    info.right_pid_output_max = right.output_max();
    info.left_pid_integral_limit = left.integral_limit();
    info.right_pid_integral_limit = right.integral_limit();
    info.left_pid_max_output_step = left.max_output_step();
    info.right_pid_max_output_step = right.max_output_step();
}

/**
 * @brief 校验 PID 参数是否有效
 * @param value 待校验的参数值
 * @return 合法返回 true
 */
bool MotorSpeedPidController::is_valid_pid_value(float value)
{
    return std::isfinite(value) && (value >= 0.0f);
}

/**
 * @brief 更新 PID 参数
 * @param params 新参数
 * @return 合法返回 true，否则返回 false
 */
bool MotorSpeedPidController::set_pid_params(const DualPidParams &params)
{
    if (!is_valid_pid_value(params.left_kp) ||
        !is_valid_pid_value(params.left_ki) ||
        !is_valid_pid_value(params.left_kd) ||
        !is_valid_pid_value(params.right_kp) ||
        !is_valid_pid_value(params.right_ki) ||
        !is_valid_pid_value(params.right_kd))
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    config_.pid_params = params;
    pid_.left_pid().set_params(params.left_kp, params.left_ki, params.left_kd);
    pid_.right_pid().set_params(params.right_kp, params.right_ki, params.right_kd);
    pid_.reset();
    return true;
}

/**
 * @brief 对浮点数进行限幅
 * @param value 输入值
 * @param min_value 下限
 * @param max_value 上限
 * @return 限幅后的值
 */
float MotorSpeedPidController::clamp_float(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

/**
 * @brief 计算三个采样值的中值
 * @param a 样本 a
 * @param b 样本 b
 * @param c 样本 c
 * @return 三点中值
 */
float MotorSpeedPidController::median_of_three(float a, float b, float c)
{
    if (((a >= b) && (a <= c)) || ((a >= c) && (a <= b)))
    {
        return a;
    }
    if (((b >= a) && (b <= c)) || ((b >= c) && (b <= a)))
    {
        return b;
    }
    return c;
}

/**
 * @brief 计算单轮前馈占空比
 * @param target_count 目标速度
 * @param gain 前馈增益
 * @param bias 静摩擦补偿
 * @return 限幅后的前馈占空比
 */
float MotorSpeedPidController::compute_feedforward_duty(float target_count, float gain, float bias) const
{
    float duty = target_count * gain;

    if (target_count > config_.feedforward_bias_threshold)
    {
        duty += bias;
    }
    else if (target_count < -config_.feedforward_bias_threshold)
    {
        duty -= bias;
    }

    return clamp_float(duty, -config_.duty_limit, config_.duty_limit);
}

/**
 * @brief 计算单轮减速辅助占空比
 * @param target_count 目标速度
 * @param feedback_count 当前反馈速度
 * @return 仅在目标显著低于当前速度时输出额外制动占空比
 */
float MotorSpeedPidController::compute_decel_assist_duty(float target_count, float feedback_count) const
{
    const float speed_error = target_count - feedback_count;

    if ((feedback_count > 0.0f) && (speed_error < -config_.decel_error_threshold))
    {
        const float assist = clamp_float(
            (-speed_error - config_.decel_error_threshold) * config_.decel_duty_gain,
            0.0f,
            config_.decel_duty_limit);
        return -assist;
    }

    if ((feedback_count < 0.0f) && (speed_error > config_.decel_error_threshold))
    {
        const float assist = clamp_float(
            (speed_error - config_.decel_error_threshold) * config_.decel_duty_gain,
            0.0f,
            config_.decel_duty_limit);
        return assist;
    }

    return 0.0f;
}

/**
 * @brief 更新单侧反馈滤波结果
 * @param filter 滤波器状态
 * @param raw_count 当前原始反馈
 * @return 滤波后的反馈值
 */
float MotorSpeedPidController::update_feedback_filter(SpeedFeedbackFilter &filter, float raw_count)
{
    const int32 active_feedback_average_window = feedback_average_window();
    const float active_feedback_low_pass_alpha = feedback_low_pass_alpha();

    if (!filter.initialized)
    {
        filter.raw_prev1 = raw_count;
        filter.raw_prev2 = raw_count;
        filter.averaged_history[0] = raw_count;
        filter.averaged_sum = raw_count;
        filter.averaged_count = 1;
        filter.averaged_index = 1 % active_feedback_average_window;
        filter.filtered = raw_count;
        filter.initialized = true;
        return raw_count;
    }

    const float median_value = median_of_three(raw_count, filter.raw_prev1, filter.raw_prev2);

    filter.raw_prev2 = filter.raw_prev1;
    filter.raw_prev1 = raw_count;

    if (filter.averaged_count < active_feedback_average_window)
    {
        filter.averaged_history[filter.averaged_count] = median_value;
        filter.averaged_sum += median_value;
        ++filter.averaged_count;
        filter.averaged_index = filter.averaged_count % active_feedback_average_window;
    }
    else
    {
        filter.averaged_sum -= filter.averaged_history[filter.averaged_index];
        filter.averaged_history[filter.averaged_index] = median_value;
        filter.averaged_sum += median_value;
        filter.averaged_index = (filter.averaged_index + 1) % active_feedback_average_window;
    }

    const float averaged_value = filter.averaged_sum / (float)filter.averaged_count;
    filter.filtered = filter.filtered * (1.0f - active_feedback_low_pass_alpha) + averaged_value * active_feedback_low_pass_alpha;
    return filter.filtered;
}

/**
 * @brief 执行一次双轮速度环计算
 * @param left_target 左轮目标速度
 * @param right_target 右轮目标速度
 * @param left_raw_feedback 左轮原始反馈
 * @param right_raw_feedback 右轮原始反馈
 * @return 速度环单次控制结果
 */
MotorSpeedControlState MotorSpeedPidController::compute(float left_target, float right_target,
                                                        float left_raw_feedback, float right_raw_feedback)
{
    std::lock_guard<std::mutex> lock(mutex_);

    MotorSpeedControlState state;
    state.left_feedback = update_feedback_filter(left_filter_, left_raw_feedback);
    state.right_feedback = update_feedback_filter(right_filter_, right_raw_feedback);
    state.left_error = left_target - state.left_feedback;
    state.right_error = right_target - state.right_feedback;

    state.left_feedforward = compute_feedforward_duty(left_target,
                                                      config_.left_feedforward_gain,
                                                      config_.left_feedforward_bias);
    state.right_feedforward = compute_feedforward_duty(right_target,
                                                       config_.right_feedforward_gain,
                                                       config_.right_feedforward_bias);
    state.left_decel_assist = compute_decel_assist_duty(left_target, state.left_feedback);
    state.right_decel_assist = compute_decel_assist_duty(right_target, state.right_feedback);

    pid_.set_target(left_target, right_target);

    state.left_correction = 0.0f;
    state.right_correction = 0.0f;
    pid_.compute(state.left_feedback, state.right_feedback, state.left_correction, state.right_correction);

    state.left_duty = state.left_feedforward + state.left_correction + state.left_decel_assist;
    state.right_duty = state.right_feedforward + state.right_correction + state.right_decel_assist;
    state.left_duty = clamp_float(state.left_duty, -config_.duty_limit, config_.duty_limit);
    state.right_duty = clamp_float(state.right_duty, -config_.duty_limit, config_.duty_limit);
    pid_.left_pid().track_output(state.left_duty - state.left_feedforward - state.left_decel_assist);
    pid_.right_pid().track_output(state.right_duty - state.right_feedforward - state.right_decel_assist);
    return state;
}

MotorSpeedPidController count_pid;

/**
 * @brief 构造位置式 PID 控制器
 */
PositionalPidController::PositionalPidController()
    : kp_(0), ki_(0), kd_(0),
      target_(0.0f), output_(0.0f),
      max_output_(0.0f), max_integral_(0.0f),
      error_(0.0f), last_error_(0.0f),
      integral_(0.0f)
{
}

/**
 * @brief 初始化位置式 PID 参数
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param max_integral 积分限幅
 * @param max_output 输出限幅
 */
void PositionalPidController::init(float kp, float ki, float kd, float max_integral, float max_output)
{
    set_params(kp, ki, kd);
    set_integral_limit(max_integral);
    set_output_limit(max_output);
    reset();
}

/**
 * @brief 设置目标值
 * @param target_value 目标值
 */
void PositionalPidController::set_target(float target_value)
{
    target_ = target_value;
}

/**
 * @brief 获取目标值
 * @return 当前目标值
 */
float PositionalPidController::get_target() const
{
    return target_;
}

/**
 * @brief 基于反馈值计算位置式 PID 输出
 * @param current_value 当前反馈值
 * @return 当前输出结果
 */
float PositionalPidController::compute(float current_value)
{
    float err = target_ - current_value;
    return compute_by_error(err);
}

/**
 * @brief 基于反馈值计算带 dt 的位置式 PID 输出
 * @param current_value 当前反馈值
 * @param dt_seconds 本次控制间隔
 * @return 当前输出结果
 */
float PositionalPidController::compute(float current_value, float dt_seconds)
{
    float err = target_ - current_value;
    return compute_by_error(err, dt_seconds);
}

/**
 * @brief 基于误差直接计算位置式 PID 输出
 * @param current_error 当前误差
 * @return 当前输出结果
 */
float PositionalPidController::compute_by_error(float current_error)
{
    return compute_by_error(current_error, 1.0f);
}

/**
 * @brief 基于误差直接计算带 dt 的位置式 PID 输出
 * @param current_error 当前误差
 * @param dt_seconds 本次控制间隔
 * @return 当前输出结果
 */
float PositionalPidController::compute_by_error(float current_error, float dt_seconds)
{
    const float safe_dt_seconds =
        (std::isfinite(dt_seconds) && dt_seconds > 1.0e-4f) ? dt_seconds : 1.0f;

    last_error_ = error_;
    error_ = current_error;

    float pout = error_ * kp_;
    float dout = ((error_ - last_error_) / safe_dt_seconds) * kd_;
    integral_ += error_ * ki_ * safe_dt_seconds;
    if (max_integral_ > 0.0f) {
        integral_ = clamp(integral_, -max_integral_, max_integral_);
    }
    output_ = pout + integral_ + dout;
    if (max_output_ > 0.0f) {
        output_ = clamp(output_, -max_output_, max_output_);
    }

    return output_;
}

/**
 * @brief 重置控制器状态
 */
void PositionalPidController::reset()
{
    error_ = 0.0f;
    last_error_ = 0.0f;
    integral_ = 0.0f;
    output_ = 0.0f;
}

/**
 * @brief 更新 PID 参数
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void PositionalPidController::set_params(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

/**
 * @brief 设置积分限幅
 * @param max_integral 积分限幅绝对值
 */
void PositionalPidController::set_integral_limit(float max_integral)
{
    max_integral_ = fabsf(max_integral);
}

/**
 * @brief 设置输出限幅
 * @param max_out 输出限幅绝对值
 */
void PositionalPidController::set_output_limit(float max_out)
{
    max_output_ = fabsf(max_out);
}

float PositionalPidController::kp() const
{
    return kp_;
}

float PositionalPidController::ki() const
{
    return ki_;
}

float PositionalPidController::kd() const
{
    return kd_;
}

float PositionalPidController::integral() const
{
    return integral_;
}

float PositionalPidController::max_integral() const
{
    return max_integral_;
}

float PositionalPidController::max_output() const
{
    return max_output_;
}

/**
 * @brief 获取当前输出值
 * @return 当前输出结果
 */
float PositionalPidController::get_output() const
{
    return output_;
}

/**
 * @brief 获取当前误差
 * @return 当前控制误差
 */
float PositionalPidController::get_error() const
{
    return error_;
}

/**
 * @brief 对数值进行区间限幅
 * @param value 输入值
 * @param min_val 下限
 * @param max_val 上限
 * @return 限幅后的数值
 */
float PositionalPidController::clamp(float value, float min_val, float max_val) const
{
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}

PositionalPidController position_pid1;
PositionalPidController position_pid2;

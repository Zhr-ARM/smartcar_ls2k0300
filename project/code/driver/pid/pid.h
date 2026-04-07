#ifndef __PID_H__
#define __PID_H__

#include "driver/pid/pid_tuning.h"
#include "zf_common_headfile.h"
#include <mutex>

/**
 * @brief 通用增量式 PID 控制器
 * 作用：处理目标与反馈的增量式闭环控制(输入输出单位自定)
 */
class PidController
{
public:
    PidController();

    /**
     * @brief 初始化PID参数
     * @param kp 比例系数
     * @param ki 积分系数
     * @param kd 微分系数
     * @param output_min 输出下限
     * @param output_max 输出上限
     */
    void init(float kp, float ki, float kd, float output_min, float output_max);

    /**
     * @brief 设置目标期望值
     * @param target_value 目标值
     */
    void set_target(float target_value);

    /**
     * @brief 获取当前目标值
     * @return 当前目标值
     */
    float get_target() const;

    /**
     * @brief 计算PID控制输出
     * @param current_value 当前实际反馈值
     * @return 增量累计后的总体输出百分比/数值
     */
    float compute(float current_value);

    /**
     * @brief 重置PID控制器历史状态
     */
    void reset();

    /**
     * @brief 动态更新PID参数
     * @param kp 比例系数
     * @param ki 积分系数
     * @param kd 微分系数
     */
    void set_params(float kp, float ki, float kd);

    /**
     * @brief 设置积分项累积限幅
     * @param limit 限幅绝对值，0表示无限制
     */
    void set_integral_limit(float limit);

    /**
     * @brief 设置单次计算最大输出允许变化量
     * @param step 最大步长限制
     */
    void set_max_output_step(float step);

    /**
     * @brief 设置整体输出限幅
     * @param min_val 输出下限
     * @param max_val 输出上限
     */
    void set_output_limit(float min_val, float max_val);

    /**
     * @brief 获取比例系数
     * @return 当前比例系数
     */
    float kp() const;

    /**
     * @brief 获取积分系数
     * @return 当前积分系数
     */
    float ki() const;

    /**
     * @brief 获取微分系数
     * @return 当前微分系数
     */
    float kd() const;

    /**
     * @brief 获取当前输出值
     * @return 当前输出结果
     */
    float get_output() const;

    /**
     * @brief 获取当前误差
     * @return 当前控制误差
     */
    float get_error() const;

    /**
     * @brief 用实际执行后的输出回写内部状态
     * @param output_value 执行器真实生效的控制输出
     */
    void track_output(float output_value);

private:
    float kp_, ki_, kd_;
    float target_, output_, output_min_, output_max_;
    float error_, error_prev_, error_prev2_, integral_limit_;
    float last_output_, max_output_step_;

    float clamp(float value, float min_val, float max_val) const;
};

/**
 * @brief 双通道PID控制器
 * 作用：封装一对增量式 PID 控制器，常用于左右双驱电机管理
 */
class DualPidController
{
public:
    DualPidController();

    /**
     * @brief 左右双通道同步初始化
     * @param left_kp 左路比例系数
     * @param left_ki 左路积分系数
     * @param left_kd 左路微分系数
     * @param right_kp 右路比例系数
     * @param right_ki 右路积分系数
     * @param right_kd 右路微分系数
     */
    void init(float left_kp, float left_ki, float left_kd, float right_kp, float right_ki, float right_kd);

    /**
     * @brief 设置左右轮目标期望值
     * @param left_target 左路目标值
     * @param right_target 右路目标值
     */
    void set_target(float left_target, float right_target);

    /**
     * @brief 双通道同步计算PID输出
     * @param left_value 左路当前反馈值
     * @param right_value 右路当前反馈值
     * @param left_duty 左路输出量引用
     * @param right_duty 右路输出量引用
     */
    void compute(float left_value, float right_value, float &left_duty, float &right_duty);

    /**
     * @brief 重置双通道控制器的历史状态
     */
    void reset();

    /**
     * @brief 同步设置双路的输出限幅
     * @param min_val 输出下限
     * @param max_val 输出上限
     */
    void set_output_limit(float min_val, float max_val);

    /**
     * @brief 获取左路控制器
     * @return 左路 PID 控制器引用
     */
    PidController &left_pid();

    /**
     * @brief 获取左路控制器只读引用
     * @return 左路 PID 控制器只读引用
     */
    const PidController &left_pid() const;

    /**
     * @brief 获取右路控制器
     * @return 右路 PID 控制器引用
     */
    PidController &right_pid();

    /**
     * @brief 获取右路控制器只读引用
     * @return 右路 PID 控制器只读引用
     */
    const PidController &right_pid() const;

private:
    PidController left_;
    PidController right_;
};

/**
 * @brief 双通道 PID 参数
 * 作用：统一描述左右轮的 PID 三参数
 */
struct DualPidParams
{
    float left_kp;   // 左轮比例系数
    float left_ki;   // 左轮积分系数
    float left_kd;   // 左轮微分系数
    float right_kp;  // 右轮比例系数
    float right_ki;  // 右轮积分系数
    float right_kd;  // 右轮微分系数
};

/**
 * @brief 电机速度环配置
 * 作用：集中管理双轮速度环的前馈、滤波与 PID 限制参数
 */
struct MotorSpeedPidConfig
{
    DualPidParams pid_params;          // 左右轮速度环 PID 三参数
    float integral_limit;             // 积分限幅，防止长期误差把积分项堆得过大
    float max_output_step;            // 单个 5ms 周期内 PID 修正项最大变化步长
    float correction_limit;           // PID 修正项总限幅，不包含前馈和减速辅助
    float duty_limit;                 // 最终输出 duty 绝对值上限
    float left_feedforward_gain;      // 左轮速度前馈斜率
    float right_feedforward_gain;     // 右轮速度前馈斜率
    float left_feedforward_bias;      // 左轮静摩擦补偿
    float right_feedforward_bias;     // 右轮静摩擦补偿
    float feedforward_bias_threshold; // 前馈静摩擦补偿触发阈值
    float decel_error_threshold;      // 减速辅助触发误差阈值
    float decel_duty_gain;            // 减速辅助增益
    float decel_duty_limit;           // 减速辅助最大输出
};

/**
 * @brief 电机速度环单次计算结果
 * 作用：返回滤波反馈、误差和最终占空比输出
 */
struct MotorSpeedControlState
{
    float left_feedback;
    float right_feedback;
    float left_error;
    float right_error;
    float left_feedforward;
    float right_feedforward;
    float left_correction;
    float right_correction;
    float left_decel_assist;
    float right_decel_assist;
    float left_duty;
    float right_duty;
};

/**
 * @brief 双轮电机速度环控制器
 * 作用：封装反馈滤波、前馈补偿与双通道增量式 PID 修正
 */
class MotorSpeedPidController
{
public:
    MotorSpeedPidController();

    /**
     * @brief 生成默认速度环配置
     * @param duty_limit 输出占空比绝对值上限
     * @return 默认配置对象
     */
    static MotorSpeedPidConfig default_config(float duty_limit);

    /**
     * @brief 初始化速度环控制器
     * @param config 速度环配置
     */
    void init(const MotorSpeedPidConfig &config);

    /**
     * @brief 重置控制器内部状态
     */
    void reset();

    /**
     * @brief 获取当前 PID 参数
     * @param params 用于接收参数的输出对象
     */
    void get_pid_params(DualPidParams &params) const;

    /**
     * @brief 更新 PID 参数
     * @param params 新的双通道 PID 参数
     * @return 参数合法返回 true，否则返回 false
     */
    bool set_pid_params(const DualPidParams &params);

    /**
     * @brief 执行一次双轮速度环计算
     * @param left_target 左轮目标速度
     * @param right_target 右轮目标速度
     * @param left_raw_feedback 左轮原始反馈
     * @param right_raw_feedback 右轮原始反馈
     * @return 单次控制结果
     */
    MotorSpeedControlState compute(float left_target, float right_target,
                                   float left_raw_feedback, float right_raw_feedback);

private:
    static constexpr int32 kFeedbackAverageWindow = pid_tuning::motor_speed::kFeedbackAverageWindow;
    static constexpr float kFeedbackLowPassAlpha = pid_tuning::motor_speed::kFeedbackLowPassAlpha;

    struct SpeedFeedbackFilter
    {
        float raw_prev1;
        float raw_prev2;
        float averaged_history[kFeedbackAverageWindow];
        float averaged_sum;
        int32 averaged_count;
        int32 averaged_index;
        float filtered;
        bool initialized;

        SpeedFeedbackFilter();
        void reset();
    };

    void sanitize_config();
    void reset_filters();
    float compute_feedforward_duty(float target_count, float gain, float bias) const;
    float compute_decel_assist_duty(float target_count, float feedback_count) const;
    float update_feedback_filter(SpeedFeedbackFilter &filter, float raw_count);
    static bool is_valid_pid_value(float value);
    static float clamp_float(float value, float min_value, float max_value);
    static float median_of_three(float a, float b, float c);

    DualPidController pid_;
    MotorSpeedPidConfig config_;
    SpeedFeedbackFilter left_filter_;
    SpeedFeedbackFilter right_filter_;
    mutable std::mutex mutex_;
};

/**
 * @brief 通用位置式 PID 控制器
 * 作用：处理目标与反馈的位置式闭环控制(相比增量式具备积分环节)
 */
class PositionalPidController
{
public:
    PositionalPidController();

    /**
     * @brief 初始化PID参数
     * @param kp 比例系数
     * @param ki 积分系数
     * @param kd 微分系数
     * @param max_integral 积分限幅
     * @param max_output 整体输出限幅
     */
    void init(float kp, float ki, float kd, float max_integral, float max_output);

    /**
     * @brief 设置目标值
     * @param target_value 目标值
     */
    void set_target(float target_value);

    /**
     * @brief 获取目标值
     * @return 当前目标值
     */
    float get_target() const;

    /**
     * @brief 基于当前反馈值计算PID位置式输出
     * @param current_value 实际反馈值
     * @return 控制器计算结果
     */
    float compute(float current_value);

    /**
     * @brief 基于当前反馈值计算带 dt 的位置式 PID 输出
     * @param current_value 实际反馈值
     * @param dt_seconds 本次控制间隔，单位秒
     * @return 控制器计算结果
     */
    float compute(float current_value, float dt_seconds);

    /**
     * @brief 纯误差驱动计算（常用于无明确稳态反馈的寻线模型）
     * @param current_error 偏差量
     * @return 控制器计算结果
     */
    float compute_by_error(float current_error);

    /**
     * @brief 纯误差驱动计算（带 dt 版本）
     * @param current_error 偏差量
     * @param dt_seconds 本次控制间隔，单位秒
     * @return 控制器计算结果
     */
    float compute_by_error(float current_error, float dt_seconds);

    /**
     * @brief 重置控制器状态
     */
    void reset();

    /**
     * @brief 更新 PID 参数
     * @param kp 比例系数
     * @param ki 积分系数
     * @param kd 微分系数
     */
    void set_params(float kp, float ki, float kd);

    /**
     * @brief 设置积分限幅
     * @param limit 积分限幅绝对值
     */
    void set_integral_limit(float limit);

    /**
     * @brief 设置输出限幅
     * @param max_out 输出限幅绝对值
     */
    void set_output_limit(float max_out);

    /**
     * @brief 获取当前输出值
     * @return 当前输出结果
     */
    float get_output() const;

    /**
     * @brief 获取当前误差
     * @return 当前控制误差
     */
    float get_error() const;

private:
    float kp_, ki_, kd_;
    float target_, output_;
    float max_output_, max_integral_;
    float error_, last_error_;
    float integral_;

    float clamp(float value, float min_val, float max_val) const;
};

extern MotorSpeedPidController count_pid;
extern PositionalPidController position_pid1;
extern PositionalPidController position_pid2;

#endif

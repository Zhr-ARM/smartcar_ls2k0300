#ifndef __BRUSHLESS_H__
#define __BRUSHLESS_H__

#include "zf_common_headfile.h"

#define BRUSHLESS1_PWM "/dev/zf_device_pwm_esc_1"
#define BRUSHLESS2_PWM "/dev/zf_device_pwm_esc_2"

#define LEFT_BRUSHLESS_PWM BRUSHLESS2_PWM
#define RIGHT_BRUSHLESS_PWM BRUSHLESS1_PWM

#define BRUSHLESS_MIN_DUTY_PERCENT (0)
#define BRUSHLESS_MAX_DUTY_PERCENT (100)
#define BRUSHLESS_PWM_OFF_DUTY (0)
#define BRUSHLESS_PWM_MIN_PULSE_US (1000)
#define BRUSHLESS_PWM_MAX_PULSE_US (2000)
#define BRUSHLESS_DEFAULT_FREQ_HZ (50)

/**
 * @brief 单路无刷电调驱动
 * 作用：封装 PWM 占空比与电调脉宽之间的换算和输出
 */
class Brushless
{
public:
    /**
     * @brief 构造单路无刷驱动对象
     * @param pwm_path PWM 设备节点路径
     */
    explicit Brushless(const char *pwm_path);

    /**
     * @brief 初始化无刷电调输出
     */
    void init();

    /**
     * @brief 按百分比设置输出
     * @param percent 目标占空比百分比
     */
    void set_duty(float percent);

    /**
     * @brief 直接设置底层原始占空比
     * @param duty 原始占空比值
     */
    void set_raw_duty(uint32 duty);

    /**
     * @brief 输出最小有效油门
     */
    void stop();

    /**
     * @brief 关闭 PWM 输出
     */
    void disable();

    /**
     * @brief 获取当前百分比输出
     * @return 当前占空比百分比
     */
    float get_duty() const;

    /**
     * @brief 获取当前原始占空比
     * @return 当前原始占空比值
     */
    uint32 get_raw_duty() const;

    /**
     * @brief 获取最小有效原始占空比
     * @return 最小有效原始占空比值
     */
    uint32 get_min_raw_duty() const;

    /**
     * @brief 获取最大有效原始占空比
     * @return 最大有效原始占空比值
     */
    uint32 get_max_raw_duty() const;

private:
    /**
     * @brief 约束百分比输出范围
     * @param percent 输入占空比百分比
     * @return 限幅后的占空比百分比
     */
    float clamp_percent(float percent) const;

    /**
     * @brief 约束原始占空比范围
     * @param duty 输入原始占空比
     * @return 限幅后的原始占空比
     */
    uint32 clamp_raw_duty(uint32 duty) const;

    /**
     * @brief 获取当前 PWM 的有效最大占空比
     * @return 最大原始占空比值
     */
    uint32 effective_duty_max() const;

    /**
     * @brief 获取 PWM 周期
     * @return PWM 周期，单位 ns
     */
    uint32 period_ns() const;

    /**
     * @brief 将脉宽换算为原始占空比
     * @param pulse_us 脉宽，单位 us
     * @return 原始占空比值
     */
    uint32 pulse_to_raw_duty(uint32 pulse_us) const;

    /**
     * @brief 将百分比输出换算为原始占空比
     * @param duty_percent 占空比百分比
     * @return 原始占空比值
     */
    uint32 percent_to_raw_duty(float duty_percent) const;

    /**
     * @brief 将原始占空比换算为百分比输出
     * @param duty 原始占空比值
     * @return 占空比百分比
     */
    float raw_duty_to_percent(uint32 duty) const;

    /**
     * @brief 将原始占空比写入硬件
     * @param duty 原始占空比值
     */
    void apply_raw_duty(uint32 duty);

    const char *pwm_path_;
    struct pwm_info pwm_info_;
    float current_duty_percent_;
    uint32 current_raw_duty_;
};

/**
 * @brief 双路无刷电调驱动
 * 作用：统一管理左右两路无刷输出
 */
class BrushlessDriver
{
public:
    /**
     * @brief 构造双路无刷驱动对象
     */
    BrushlessDriver();

    /**
     * @brief 初始化左右无刷电调
     */
    void init();

    /**
     * @brief 预留的 5ms 周期更新接口
     */
    void update_5ms();

    /**
     * @brief 停止左右无刷输出
     */
    void stop_all();

    /**
     * @brief 关闭左右无刷 PWM 输出
     */
    void disable_all();

    /**
     * @brief 设置左侧无刷输出
     * @param percent 目标占空比百分比
     */
    void set_left_duty(float percent);

    /**
     * @brief 设置右侧无刷输出
     * @param percent 目标占空比百分比
     */
    void set_right_duty(float percent);

    /**
     * @brief 直接设置左侧原始占空比
     * @param duty 原始占空比值
     */
    void set_left_raw_duty(uint32 duty);

    /**
     * @brief 直接设置右侧原始占空比
     * @param duty 原始占空比值
     */
    void set_right_raw_duty(uint32 duty);

    /**
     * @brief 获取左侧无刷输出
     * @return 左侧占空比百分比
     */
    float left_duty() const;

    /**
     * @brief 获取右侧无刷输出
     * @return 右侧占空比百分比
     */
    float right_duty() const;

    /**
     * @brief 获取左侧原始占空比
     * @return 左侧原始占空比值
     */
    uint32 left_raw_duty() const;

    /**
     * @brief 获取右侧原始占空比
     * @return 右侧原始占空比值
     */
    uint32 right_raw_duty() const;

private:
    Brushless left_;
    Brushless right_;
};

extern BrushlessDriver brushless_driver;

#endif

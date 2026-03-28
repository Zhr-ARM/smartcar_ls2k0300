#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "zf_common_headfile.h"

#define MOTOR1_DIR "/dev/zf_driver_gpio_motor_1"
#define MOTOR1_PWM "/dev/zf_device_pwm_motor_1"
#define MOTOR2_DIR "/dev/zf_driver_gpio_motor_2"
#define MOTOR2_PWM "/dev/zf_device_pwm_motor_2"

#define ENCODER_1 "/dev/zf_encoder_1"
#define ENCODER_2 "/dev/zf_encoder_2"

#define LEFT_MOTOR_DIR MOTOR1_DIR
#define LEFT_MOTOR_PWM MOTOR1_PWM
#define RIGHT_MOTOR_DIR MOTOR2_DIR
#define RIGHT_MOTOR_PWM MOTOR2_PWM

#define LEFT_ENCODER_PATH ENCODER_1
#define RIGHT_ENCODER_PATH ENCODER_2

#define MOTOR_MAX_DUTY_PERCENT (40)
#define LEFT_ENCODER_SIGN (1)
#define RIGHT_ENCODER_SIGN (-1)

/**
 * @brief 单路有刷电机驱动
 * 作用：封装方向控制、PWM 输出和编码器反馈
 */
class Motor
{
public:
    /**
     * @brief 构造单路电机对象
     * @param dir_path 方向控制 GPIO 节点路径
     * @param pwm_path PWM 设备节点路径
     * @param encoder_path 编码器节点路径
     * @param encoder_sign 编码器方向修正系数
     */
    Motor(const char *dir_path, const char *pwm_path, const char *encoder_path, int8 encoder_sign);

    /**
     * @brief 初始化电机输出与编码器基准
     */
    void init();

    /**
     * @brief 更新编码器计数差值
     * @param elapsed_periods 距上次更新实际经过的 5ms 控制周期数
     */
    void update_encoder(uint32 elapsed_periods = 1);

    /**
     * @brief 设置电机输出占空比
     * @param percent 目标占空比百分比
     */
    void set_duty(float percent);

    /**
     * @brief 停止电机输出
     */
    void stop();

    /**
     * @brief 获取当前编码器增量
     * @return 当前采样周期的编码器计数差
     */
    int32 get_encoder() const;

    /**
     * @brief 获取标准化后的 5ms 编码器计数
     * @return 折算到 5ms 周期的编码器增量
     */
    float get_count_5ms() const;

    /**
     * @brief 获取当前逻辑占空比
     * @return 当前占空比百分比
     */
    float get_duty() const;

private:
    /**
     * @brief 读取编码器当前输出值
     * @return 方向编码器当前速度测量值
     */
    int32 read_encoder_raw() const;

    /**
     * @brief 约束占空比输出范围
     * @param percent 输入占空比百分比
     * @return 限幅后的占空比百分比
     */
    float clamp_percent(float percent) const;

    /**
     * @brief 将占空比百分比换算为原始占空比
     * @param duty_percent 占空比百分比
     * @return 原始占空比值
     */
    uint32 percent_to_raw_duty(float duty_percent) const;

    /**
     * @brief 将逻辑占空比写入底层硬件
     * @param duty_percent 目标占空比百分比
     */
    void apply_output(float duty_percent);

    const char *dir_path_;
    const char *pwm_path_;
    const char *encoder_path_;
    int8 encoder_sign_;
    struct pwm_info pwm_info_;
    int32 encoder_5ms_count_;
    float current_duty_percent_;
};

/**
 * @brief 双路有刷电机驱动
 * 作用：统一管理左右电机的控制与反馈
 */
class MotorDriver
{
public:
    MotorDriver();

    /**
     * @brief 初始化左右电机模块
     */
    void init();

    /**
     * @brief 执行 5ms 周期反馈更新
     * @param elapsed_periods 实际经过的 5ms 控制周期数
     */
    void update_5ms(uint32 elapsed_periods = 1);

    /**
     * @brief 紧急停止所有电机输出
     */
    void stop_all();

    /**
     * @brief 设定左侧电机占空比
     * @param percent 期望输出的百分比
     */
    void set_left_duty(float percent);

    /**
     * @brief 设定右侧电机占空比
     * @param percent 期望输出的百分比
     */
    void set_right_duty(float percent);

    /**
     * @brief 获取左侧编码器增量
     * @return 左侧编码器计数差
     */
    int32 left_encoder() const;

    /**
     * @brief 获取右侧编码器增量
     * @return 右侧编码器计数差
     */
    int32 right_encoder() const;

    /**
     * @brief 获取左侧 5ms 标准化计数
     * @return 左侧折算后的 5ms 编码器增量
     */
    float left_count_5ms() const;

    /**
     * @brief 获取右侧 5ms 标准化计数
     * @return 右侧折算后的 5ms 编码器增量
     */
    float right_count_5ms() const;

    /**
     * @brief 获取左侧逻辑占空比
     * @return 左侧占空比百分比
     */
    float left_duty() const;

    /**
     * @brief 获取右侧逻辑占空比
     * @return 右侧占空比百分比
     */
    float right_duty() const;

private:
    Motor left_;
    Motor right_;
};

extern MotorDriver motor_driver;

#endif

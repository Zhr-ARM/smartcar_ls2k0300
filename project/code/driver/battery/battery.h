#ifndef __BATTERY_H__
#define __BATTERY_H__

#include <csignal>
#include "zf_common_headfile.h"

#define BATTERY_ADC_REG_PATH "/sys/bus/iio/devices/iio:device0/in_voltage7_raw"
#define BATTERY_ADC_SCALE_PATH "/sys/bus/iio/devices/iio:device0/in_voltage_scale"
#define BATTERY_VOLTAGE_DIVIDER_RATIO (11.4f)

/**
 * @brief 电池电压监测类
 * 作用：读取 ADC 原始值并换算为电池电压
 */
class BatteryMonitor
{
public:
    /**
     * @brief 构造电池监测对象
     * @param adc_reg_path ADC 原始值节点路径
     * @param adc_scale_path ADC 缩放系数节点路径
     * @param divider_ratio 分压比
     */
    BatteryMonitor(const char *adc_reg_path, const char *adc_scale_path, float divider_ratio);

    /**
     * @brief 初始化电池监测数据
     */
    void init();

    /**
     * @brief 刷新当前电池电压数据
     */
    void update();

    /**
     * @brief 获取最新 ADC 原始值
     * @return ADC 原始采样值
     */
    uint16 adc_raw() const;

    /**
     * @brief 获取最新 ADC 缩放系数
     * @return ADC 缩放系数
     */
    float adc_scale() const;

    /**
     * @brief 获取电池电压的毫伏值
     * @return 当前电池电压，单位 mV
     */
    uint32 voltage_mv() const;

    /**
     * @brief 获取电池电压的伏特值
     * @return 当前电池电压，单位 V
     */
    float voltage_v() const;

private:
    /**
     * @brief 根据 ADC 采样值换算电压
     * @param adc_raw ADC 原始值
     * @param adc_scale ADC 缩放系数
     * @return 换算后的电压值，单位 mV
     */
    uint32 calculate_voltage_mv(uint16 adc_raw, float adc_scale) const;

    const char *adc_reg_path_;
    const char *adc_scale_path_;
    float divider_ratio_;
    uint16 adc_raw_;
    float adc_scale_;
    uint32 voltage_mv_;
};

extern BatteryMonitor battery_monitor;

/**
 * @brief 初始化低压保护模块
 * 作用：初始化电池监测、低压判定状态和蜂鸣器默认状态
 */
void battery_low_voltage_protection_init();

/**
 * @brief 更新低压保护状态
 * 作用：刷新电池电压、执行滤波和连续低压确认
 * @return 若已经触发低压保护则返回 true
 */
bool battery_low_voltage_protection_update();

/**
 * @brief 获取低压保护是否已经锁存
 * @return 已触发并锁存返回 true
 */
bool battery_low_voltage_protection_latched();

/**
 * @brief 获取滤波后的电池电压
 * @return 当前用于低压判定的滤波电压，单位 V
 */
float battery_low_voltage_protection_filtered_voltage_v();

/**
 * @brief 获取低压保护阈值
 * @return 低压保护阈值，单位 V
 */
float battery_low_voltage_protection_threshold_v();

/**
 * @brief 获取建议的低压检测周期
 * @return 建议检测周期，单位 ms
 */
int battery_low_voltage_protection_check_period_ms();

/**
 * @brief 关闭蜂鸣器
 */
void battery_low_voltage_protection_silence_buzzer();

/**
 * @brief 进入低压蜂鸣报警循环
 * 作用：在主控制已停下后持续蜂鸣提醒，直到用户退出程序
 * @param exit_flag 外部退出标志；传空指针则表示不检查外部退出
 */
void battery_low_voltage_protection_run_alarm_loop(volatile sig_atomic_t *exit_flag);

#endif

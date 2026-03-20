#ifndef __BATTERY_H__
#define __BATTERY_H__

#include "zf_common_headfile.h"

#define BATTERY_ADC_REG_PATH "/sys/bus/iio/devices/iio:device0/in_voltage7_raw"
#define BATTERY_ADC_SCALE_PATH "/sys/bus/iio/devices/iio:device0/in_voltage_scale"
#define BATTERY_VOLTAGE_DIVIDER_RATIO (11.0f)

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

#endif

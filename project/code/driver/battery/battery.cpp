#include "battery.h"

/**
 * @brief 构造电池监测对象
 * @param adc_reg_path ADC 原始值节点路径
 * @param adc_scale_path ADC 缩放系数节点路径
 * @param divider_ratio 分压比
 */
BatteryMonitor::BatteryMonitor(const char *adc_reg_path, const char *adc_scale_path, float divider_ratio)
    : adc_reg_path_(adc_reg_path),
      adc_scale_path_(adc_scale_path),
      divider_ratio_(divider_ratio),
      adc_raw_(0),
      adc_scale_(0.0f),
      voltage_mv_(0)
{
}

/**
 * @brief 初始化电池电压数据
 */
void BatteryMonitor::init()
{
    update();
}

/**
 * @brief 刷新当前电池电压数据
 */
void BatteryMonitor::update()
{
    adc_raw_ = adc_convert(adc_reg_path_);
    adc_scale_ = adc_get_scale(adc_scale_path_);
    voltage_mv_ = calculate_voltage_mv(adc_raw_, adc_scale_);
}

/**
 * @brief 获取最新 ADC 原始值
 * @return ADC 原始采样值
 */
uint16 BatteryMonitor::adc_raw() const
{
    return adc_raw_;
}

/**
 * @brief 获取最新 ADC 缩放系数
 * @return ADC 缩放系数
 */
float BatteryMonitor::adc_scale() const
{
    return adc_scale_;
}

/**
 * @brief 获取电池电压的毫伏值
 * @return 当前电池电压，单位 mV
 */
uint32 BatteryMonitor::voltage_mv() const
{
    return voltage_mv_;
}

/**
 * @brief 获取电池电压的伏特值
 * @return 当前电池电压，单位 V
 */
float BatteryMonitor::voltage_v() const
{
    return (float)voltage_mv_ / 1000.0f;
}

/**
 * @brief 根据 ADC 采样值换算电池电压
 * @param adc_raw ADC 原始值
 * @param adc_scale ADC 缩放系数
 * @return 换算后的电压值，单位 mV
 */
uint32 BatteryMonitor::calculate_voltage_mv(uint16 adc_raw, float adc_scale) const
{
    const float voltage_mv = adc_raw * adc_scale * divider_ratio_;

    if (voltage_mv <= 0.0f)
    {
        return 0;
    }
    return (uint32)(voltage_mv + 0.5f);
}

BatteryMonitor battery_monitor(BATTERY_ADC_REG_PATH, BATTERY_ADC_SCALE_PATH, BATTERY_VOLTAGE_DIVIDER_RATIO);

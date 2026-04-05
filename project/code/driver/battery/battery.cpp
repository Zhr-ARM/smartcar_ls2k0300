#include "battery.h"

namespace
{
// 参考逐飞蜂鸣器例程：低电平点亮蜂鸣器，高电平关闭蜂鸣器。
constexpr const char *kBatteryBeepDevicePath = "/dev/zf_driver_gpio_beep";
// 低压后打印电压、停控制线程并进入蜂鸣报警，直到用户主动退出程序。
constexpr bool kBatteryProtectionEnabled = true;
constexpr bool kBatteryAlarmEnabled = kBatteryProtectionEnabled;
constexpr float kBatteryLowVoltageThresholdV = 10.3f;
constexpr float kBatteryVoltageFilterAlpha = 0.35f;
constexpr int kBatteryLowVoltageConfirmCount = 3;
constexpr int kBatteryCheckPeriodMs =1000;
constexpr int kBatteryStatusPrintPeriodMs = 2000;

float g_filtered_battery_voltage_v = 0.0f;
bool g_battery_filter_initialized = false;
int g_low_voltage_confirm_count = 0;
bool g_low_voltage_latched = false;

void battery_buzzer_set_enabled(bool enabled)
{
    gpio_set_level(kBatteryBeepDevicePath, (kBatteryAlarmEnabled && enabled) ? 0x0 : 0x1);
}

bool battery_exit_requested(volatile sig_atomic_t *exit_flag)
{
    return (nullptr != exit_flag) && (0 != *exit_flag);
}

bool battery_alarm_delay_ms(int total_ms, volatile sig_atomic_t *exit_flag)
{
    constexpr int kSliceMs = 20;
    int elapsed_ms = 0;
    while (!battery_exit_requested(exit_flag) && (elapsed_ms < total_ms))
    {
        const int remain_ms = total_ms - elapsed_ms;
        const int step_ms = (remain_ms < kSliceMs) ? remain_ms : kSliceMs;
        system_delay_ms(step_ms);
        elapsed_ms += step_ms;
    }
    return !battery_exit_requested(exit_flag);
}
} // namespace

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

void battery_low_voltage_protection_init()
{
    battery_monitor.init();
    g_filtered_battery_voltage_v = battery_monitor.voltage_v();
    g_battery_filter_initialized = true;
    g_low_voltage_latched = false;
    g_low_voltage_confirm_count = 0;
    battery_buzzer_set_enabled(false);

    if (!kBatteryProtectionEnabled)
    {
        printf("[BATTERY] low voltage protection disabled, monitor-only mode\r\n");
        return;
    }

    g_low_voltage_latched = (g_filtered_battery_voltage_v < kBatteryLowVoltageThresholdV);
    g_low_voltage_confirm_count = g_low_voltage_latched ? kBatteryLowVoltageConfirmCount : 0;
}

bool battery_low_voltage_protection_update()
{
    battery_monitor.update();

    const float raw_battery_voltage_v = battery_monitor.voltage_v();
    if (!g_battery_filter_initialized)
    {
        g_filtered_battery_voltage_v = raw_battery_voltage_v;
        g_battery_filter_initialized = true;
    }
    else
    {
        g_filtered_battery_voltage_v =
            g_filtered_battery_voltage_v * (1.0f - kBatteryVoltageFilterAlpha) +
            raw_battery_voltage_v * kBatteryVoltageFilterAlpha;
    }

    if (!kBatteryProtectionEnabled)
    {
        g_low_voltage_confirm_count = 0;
        g_low_voltage_latched = false;
        return false;
    }

    if (g_filtered_battery_voltage_v < kBatteryLowVoltageThresholdV)
    {
        ++g_low_voltage_confirm_count;
    }
    else
    {
        g_low_voltage_confirm_count = 0;
    }

    if (g_low_voltage_confirm_count >= kBatteryLowVoltageConfirmCount)
    {
        g_low_voltage_latched = true;
    }

    return g_low_voltage_latched;
}

bool battery_low_voltage_protection_latched()
{
    return g_low_voltage_latched;
}

float battery_low_voltage_protection_filtered_voltage_v()
{
    return g_filtered_battery_voltage_v;
}

float battery_low_voltage_protection_threshold_v()
{
    return kBatteryLowVoltageThresholdV;
}

int battery_low_voltage_protection_check_period_ms()
{
    return kBatteryCheckPeriodMs;
}

void battery_low_voltage_protection_silence_buzzer()
{
    battery_buzzer_set_enabled(false);
}

void battery_low_voltage_protection_run_alarm_loop(volatile sig_atomic_t *exit_flag)
{
    if (!kBatteryProtectionEnabled || !kBatteryAlarmEnabled)
    {
        battery_buzzer_set_enabled(false);
        return;
    }

    int elapsed_since_print_ms = kBatteryStatusPrintPeriodMs;
    battery_buzzer_set_enabled(false);

    while (!battery_exit_requested(exit_flag))
    {
        battery_monitor.update();
        const float current_voltage_v = battery_monitor.voltage_v();

        if (elapsed_since_print_ms >= kBatteryStatusPrintPeriodMs)
        {
            printf("[BATTERY] protection latched voltage=%.2fV, buzzer alarm active\r\n",
                   static_cast<double>(current_voltage_v));
            elapsed_since_print_ms = 0;
        }

        // 报警节奏：三声短鸣，再停顿一下，便于听觉上明显区分“低压告警”。
        for (int i = 0; (i < 3) && !battery_exit_requested(exit_flag); ++i)
        {
            battery_buzzer_set_enabled(true);
            if (!battery_alarm_delay_ms(140, exit_flag))
            {
                break;
            }

            battery_buzzer_set_enabled(false);
            if (!battery_alarm_delay_ms(120, exit_flag))
            {
                break;
            }

            elapsed_since_print_ms += 260;
        }

        if (battery_exit_requested(exit_flag))
        {
            break;
        }

        if (!battery_alarm_delay_ms(700, exit_flag))
        {
            break;
        }
        elapsed_since_print_ms += 700;
    }

    battery_buzzer_set_enabled(false);
}

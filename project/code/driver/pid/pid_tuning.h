#ifndef PID_TUNING_H_
#define PID_TUNING_H_

#include "zf_common_headfile.h"

// 说明：
// 1) 这个文件专门放“默认调参值”，方便集中修改。
// 2) 这里的参数会被速度环 PID 和巡线位置环在初始化时读取。
// 3) 如果后续串口动态改 PID，运行时值可能与这里不同；这里仍然是编译时默认值入口。

namespace pid_tuning
{
namespace imu
{
// IMU 启动零偏标定时长：利用主程序启动阶段的静止窗口估计 gyro_z 零偏。
inline constexpr int32 kStartupCalibrateDurationMs = 2000;
// gyro_z 方向校正：若接入后“越抗漂越甩”，优先把它从 1 改成 -1。
inline constexpr float kGyroYawRateSign = 1.0f;
// 巡线线程内对横摆角速度再做一层轻滤波，抑制偶发抖动。
// 约定：该滤波只在“收到新 IMU 样本”时推进一次，不按 1ms 控制循环空转。
inline constexpr float kGyroYawRateFilterAlpha = 0.45f;
} // namespace imu

namespace motor_speed
{
// 左轮速度环比例项：误差一出现就立即给修正，越大响应越快，但过大容易抖。
inline constexpr float kLeftKp = 0.0850f;
// 左轮速度环积分项：用来消除稳态误差，越大越容易追平长期偏差，但也更容易拖尾。
inline constexpr float kLeftKi = 0.00078f;
// 左轮速度环微分项：抑制误差变化过快，当前默认关闭。
inline constexpr float kLeftKd = 0.0f;

// 右轮速度环比例项：含义同左轮，用于补偿左右轮机械差异可单独设置。
inline constexpr float kRightKp = 0.0850f;
// 右轮速度环积分项：含义同左轮。
inline constexpr float kRightKi = 0.00078f;
// 右轮速度环微分项：当前默认关闭。
inline constexpr float kRightKd = 0.0f;

// 积分限幅：限制单次参与积分计算的误差幅度，避免积分累积过大。
inline constexpr float kIntegralLimit = 180.0f;
// 单周期最大修正步长：限制 PID 输出每个 5ms 周期最多变化多少，越大越跟手。
inline constexpr float kMaxOutputStep = 15.0f;
// PID 修正项总限幅：只限制 PID 修正部分，不包含前馈和减速辅助。
inline constexpr float kCorrectionLimit = 24.0f;

// 左轮速度前馈斜率：目标速度每增加 1 count/5ms，预先增加多少 duty。
inline constexpr float kLeftFeedforwardGain = 0.090f;
// 右轮速度前馈斜率：通常左右轮不完全对称，允许单独设置。
inline constexpr float kRightFeedforwardGain = 0.090f;
// 左轮静摩擦补偿：目标速度超过阈值后，额外补一点 duty 帮助起转。
inline constexpr float kLeftFeedforwardBias = 1.4f;
// 右轮静摩擦补偿：含义同左轮。
inline constexpr float kRightFeedforwardBias = 1.4f;
// 静摩擦补偿触发阈值：目标速度绝对值超过它时，才叠加 bias。
inline constexpr float kFeedforwardBiasThreshold = 5.0f;

// 减速辅助触发误差：实际速度明显高于目标这么多时，开始主动给反向刹车辅助。
inline constexpr float kDecelErrorThreshold = 5.0f;
// 减速辅助增益：超出的速度误差乘这个系数，生成额外减速 duty。
inline constexpr float kDecelDutyGain = 0.18f;
// 减速辅助最大值：防止减速辅助过猛。
inline constexpr float kDecelDutyLimit = 14.0f;

// 反馈平均窗口：越大越稳，越小越灵；当前用于速度反馈平滑。
inline constexpr int32 kFeedbackAverageWindow = 2;
// 反馈低通系数：越大越信任新值、响应更快；越小越平滑、但滞后更明显。
inline constexpr float kFeedbackLowPassAlpha = 0.95f;
} // namespace motor_speed

namespace yaw_rate_loop
{
// 视觉曲率滤波系数：让目标角速度更多跟随赛道整体趋势，而不是单帧角点毛刺。
// 约定：该滤波只在“收到新视觉帧”时推进一次。
inline constexpr float kVisualCurvatureFilterAlpha = 0.25f;
// 目标点夹角滤波系数：平滑单帧跟踪点跳动，避免目标角速度突然抽动。
// 约定：该滤波只在“收到新视觉帧”时推进一次。
inline constexpr float kTrackPointAngleFilterAlpha = 0.6f;
} // namespace yaw_rate_loop

namespace line_follow
{
// 视觉误差低通系数：越大越跟当前帧，越小越重视历史趋势。
// 约定：该滤波只在“收到新视觉帧”时推进一次。
inline constexpr float kErrorFilterAlpha = 0.95f;

// 左右轮目标最小值：允许轻微反转，方便大误差时快速拧回车头。
inline constexpr float kTargetCountMin = 20.0f;
// 左右轮目标最大值：限制巡线线程下发给速度环的目标上限。
inline constexpr float kTargetCountMax = 1550.0f;

// 误差死区：采样线附近的小抖动直接忽略，减少左右抽动。
inline constexpr float kErrorDeadzonePx = 0.6f;
// 小误差降增益区间：误差小于这个值时，先温和修正。
inline constexpr float kErrorLowGainLimitPx = 3.0f;
// 小误差降增益系数：进入小误差区间后，控制量乘这个比例。
inline constexpr float kErrorLowGain = 0.70f;
} // namespace line_follow

namespace route_line_follow
{
// 按路线状态切换的巡线调参档：
// 1) 统一放在这里，方便你后续只改一个头文件；
// 2) “前馈”这里主要指视觉生成目标横摆角速度时的参考增益；
// 3) 基础速度直接写成“各状态绝对速度”，再额外乘一个全局倍率。
struct Profile
{
    float base_speed;                              // 该状态下的绝对基础速度（单位同电机目标 count/5ms）
    float straight_full_speed_error_threshold_px;  // 均值路径误差低于该阈值时，允许直接给该状态满速；<=0 表示禁用

    float position_dynamic_kp_quad_a;              // 动态Kp一次项系数 a（当前参数名保留 quad_a，实际按 a*|x|+b 使用）
    float position_dynamic_kp_base;                // 动态Kp基准值：零误差附近从它起算，建议满足 min <= base <= max
    float position_dynamic_kp_min;                 // 动态Kp下限：如果不希望 base 被夹掉，需保证它不大于 base
    float position_dynamic_kp_max;                 // 动态Kp上限
    float position_ki;                             // 位置环积分项
    float position_kd;                             // 位置环微分项
    float position_max_integral;                   // 位置环积分限幅
    float position_max_output;                     // 位置环输出限幅
    float steering_max_output;                     // 并级合成后的总转向输出限幅

    float yaw_rate_ref_from_error_gain_dps;        // 视觉误差前馈到目标横摆角速度的增益
    float yaw_rate_ref_from_curvature_gain_dps;    // 视觉曲率前馈到目标横摆角速度的增益
    float yaw_rate_ref_from_track_point_gain_dps;  // 跟踪点夹角前馈到目标横摆角速度的增益
    float yaw_rate_ref_limit_dps;                  // 目标横摆角速度限幅
    float yaw_rate_kp;                             // 角速度环比例项（动态Kp基准值）
    float yaw_rate_dynamic_kp_quad_a;              // 角速度环动态Kp一次项系数 a（当前参数名保留 quad_a，实际按 a*|x|+b 使用）
    float yaw_rate_dynamic_kp_min;                 // 角速度环动态Kp下限
    float yaw_rate_dynamic_kp_max;                 // 角速度环动态Kp上限
    float yaw_rate_kp_enable_error_threshold_px;   // 横向误差绝对值超过该阈值时，才启用角速度环Kp；<=0 表示始终启用
    float yaw_rate_ki;                             // 角速度环积分项
    float yaw_rate_kd;                             // 角速度环微分项
    float yaw_rate_max_integral;                   // 角速度环积分限幅
    float yaw_rate_max_output;                     // 角速度环输出限幅

    float turn_slowdown_start_px;                  // 弯道降速起点
    float turn_slowdown_full_px;                   // 弯道降速满量程点
    float yaw_rate_ref_slowdown_start_dps;         // 预瞄目标横摆角速度降速起点
    float yaw_rate_ref_slowdown_full_dps;          // 预瞄目标横摆角速度降速满量程点
    float turn_min_speed_scale;                    // 弯道最低保速比例
    float turn_slowdown_max_drop_ratio_per_cycle;  // 单周期最大降速比例
    float turn_slowdown_max_rise_ratio_per_cycle;  // 单周期最大升速比例
};

// 全局基础速度倍率：
// 作用：在不改变各状态相对快慢关系的前提下，统一整体提速或降速。
inline constexpr float kGlobalBaseSpeedScale = 1.00f;

inline constexpr bool is_dynamic_kp_range_valid(const Profile &profile)
{
    return (profile.position_dynamic_kp_min <= profile.position_dynamic_kp_base) &&
           (profile.position_dynamic_kp_base <= profile.position_dynamic_kp_max) &&
           (profile.yaw_rate_dynamic_kp_min <= profile.yaw_rate_kp) &&
           (profile.yaw_rate_kp <= profile.yaw_rate_dynamic_kp_max);
}

inline constexpr bool is_preview_slowdown_range_valid(const Profile &profile)
{
    return (profile.yaw_rate_ref_slowdown_start_dps <= 0.0f) ||
           (profile.yaw_rate_ref_slowdown_start_dps < profile.yaw_rate_ref_slowdown_full_dps);
}

inline constexpr Profile make_normal_profile()
{
    Profile profile{};

    profile.base_speed = 280.0f; // NORMAL 档基础速度：非直道加速阶段默认按 300 count/5ms 行驶。
    profile.straight_full_speed_error_threshold_px = 0.0f; // NORMAL 档直道满速阈值：整条路径均值误差小于 1px 才允许直通满速。

    profile.position_dynamic_kp_quad_a = 4.4f; // NORMAL 档位置环动态Kp一次项：误差越大，Kp 按线性速度增长。
    profile.position_dynamic_kp_base = 1.2f; // NORMAL 档位置环动态Kp基础值：零误差附近从 1300 起步，避免“名义基础值”被下限吃掉。
    profile.position_dynamic_kp_min = 0.0f; // NORMAL 档位置环动态Kp下限：当前与基础值保持一致，表示零误差起点就是最小 Kp。
    profile.position_dynamic_kp_max = 90.0f; // NORMAL 档位置环动态Kp上限：防止比例项过强。
    profile.position_ki = 0.0f; // NORMAL 档位置环积分项：当前关闭，避免积分拖尾。
    profile.position_kd = 0.0f; // NORMAL 档位置环微分项：抑制转向过冲和来回摆动。
    profile.position_max_integral = 0.0f; // NORMAL 档位置环积分限幅：0 表示当前不启用额外积分限幅。
    profile.position_max_output = 280.0f; // NORMAL 档位置环输出限幅：位置支路允许的最大差速。
    profile.steering_max_output = 280.0f; // NORMAL 档总转向限幅：位置环与角速度环叠加后的总差速上限。

    profile.yaw_rate_ref_from_error_gain_dps = 0.0f; // NORMAL 档误差前馈增益：当前固定夹角模式，不参与目标横摆角速度生成，建议保持 0。
    profile.yaw_rate_ref_from_curvature_gain_dps = 0.0f; // NORMAL 档曲率前馈增益：当前固定夹角模式，不参与目标横摆角速度生成，建议保持 0。
    profile.yaw_rate_ref_from_track_point_gain_dps = 4.0f; // NORMAL 档跟踪点夹角前馈增益：目标点偏角映射到目标横摆角速度的比例。
    profile.yaw_rate_ref_limit_dps = 220.0f; // NORMAL 档目标横摆角速度限幅：防止视觉异常时姿态目标过大。
    profile.yaw_rate_kp = 1.2f; // NORMAL 档角速度环比例项：把弯道基础支撑补回来，避免入弯后车头跟不上。
    profile.yaw_rate_dynamic_kp_quad_a = 0.00f; // NORMAL 档角速度环动态Kp一次项：弯里误差增大时线性抬 Kp。
    profile.yaw_rate_dynamic_kp_min = 0.0f; // NORMAL 档角速度环动态Kp下限：与基础 Kp 对齐，保证一进角速度环就有足够支撑。
    profile.yaw_rate_dynamic_kp_max = 10.0f; // NORMAL 档角速度环动态Kp上限：给中大弯留更强纠偏空间，但仍比固定大 Kp 更可控。
    profile.yaw_rate_kp_enable_error_threshold_px = 0.0f; // NORMAL 档角速度环Kp启用阈值：更早介入，避免等偏差放大后才开始补转向。
    profile.yaw_rate_ki = 0.0f; // NORMAL 档角速度环积分项：当前关闭，避免姿态积分拖尾。
    profile.yaw_rate_kd = 0.02f; // NORMAL 档角速度环微分项：略回一点，帮助压住补强后可能带来的过冲。
    profile.yaw_rate_max_integral = 0.0f; // NORMAL 档角速度环积分限幅：0 表示当前不启用额外积分限幅。
    profile.yaw_rate_max_output = 240.0f; // NORMAL 档角速度环输出限幅：角速度支路允许的最大差速。

    profile.turn_slowdown_start_px = 10.0f; // NORMAL 档弯道降速起点：误差超过 50px 开始降速。
    profile.turn_slowdown_full_px = 25.0f; // NORMAL 档弯道降速满量程点：误差到 60px 时达到最大降速。
    profile.yaw_rate_ref_slowdown_start_dps = 0.0f; // NORMAL 档预瞄目标横摆角速度降速起点：0 表示关闭这条预瞄降速逻辑。
    profile.yaw_rate_ref_slowdown_full_dps = 0.0f; // NORMAL 档预瞄目标横摆角速度降速满量程点：关闭后保持 0。
    profile.turn_min_speed_scale = 0.90f; // NORMAL 档最低保速比例：再大误差也至少保留 7% 基础速度。
    profile.turn_slowdown_max_drop_ratio_per_cycle = 0.45f; // NORMAL 档单周期最大降速比例：限制一拍内速度下降过快。
    profile.turn_slowdown_max_rise_ratio_per_cycle = 0.01f; // NORMAL 档单周期最大升速比例：限制一拍内速度回升过快。

    return profile;
}

inline constexpr Profile kNormalProfile = make_normal_profile();
static_assert(is_dynamic_kp_range_valid(kNormalProfile), "kNormalProfile dynamic Kp range is invalid");
static_assert(is_preview_slowdown_range_valid(kNormalProfile), "kNormalProfile preview slowdown range is invalid");

// 直道加速阶段：
// 双边直道都稳定时进入；在 NORMAL 基础上提速，并单独增强位置环，避免高速直道时“只有姿态、没有回中线”。
inline constexpr Profile make_straight_profile()
{
    Profile profile = kNormalProfile;

    profile.base_speed = 500.0f; // NORMAL 档基础速度：非直道加速阶段默认按 300 count/5ms 行驶。
    profile.straight_full_speed_error_threshold_px = 0.0f; // NORMAL 档直道满速阈值：整条路径均值误差小于 1px 才允许直通满速。

    profile.position_dynamic_kp_quad_a = 0.0; // NORMAL 档位置环动态Kp一次项：误差越大，Kp 按线性速度增长。
    profile.position_dynamic_kp_base = 1.0f; // NORMAL 档位置环动态Kp基础值：零误差附近从 1300 起步，避免“名义基础值”被下限吃掉。
    profile.position_dynamic_kp_min = 0.0f; // NORMAL 档位置环动态Kp下限：当前与基础值保持一致，表示零误差起点就是最小 Kp。
    profile.position_dynamic_kp_max = 30.0f; // NORMAL 档位置环动态Kp上限：防止比例项过强。
    profile.position_ki = 0.0f; // NORMAL 档位置环积分项：当前关闭，避免积分拖尾。
    profile.position_kd = 0.0f; // NORMAL 档位置环微分项：抑制转向过冲和来回摆动。
    profile.position_max_integral = 0.0f; // NORMAL 档位置环积分限幅：0 表示当前不启用额外积分限幅。
    profile.position_max_output = 190.0f; // NORMAL 档位置环输出限幅：位置支路允许的最大差速。
    profile.steering_max_output = 190.0f; // NORMAL 档总转向限幅：位置环与角速度环叠加后的总差速上限。

    profile.yaw_rate_ref_from_error_gain_dps = 0.0f; // NORMAL 档误差前馈增益：当前固定夹角模式，不参与目标横摆角速度生成，建议保持 0。
    profile.yaw_rate_ref_from_curvature_gain_dps = 0.0f; // NORMAL 档曲率前馈增益：当前固定夹角模式，不参与目标横摆角速度生成，建议保持 0。
    profile.yaw_rate_ref_from_track_point_gain_dps = 4.0f; // NORMAL 档跟踪点夹角前馈增益：目标点偏角映射到目标横摆角速度的比例。
    profile.yaw_rate_ref_limit_dps = 220.0f; // NORMAL 档目标横摆角速度限幅：防止视觉异常时姿态目标过大。
    profile.yaw_rate_kp = 0.0f; // NORMAL 档角速度环比例项：把弯道基础支撑补回来，避免入弯后车头跟不上。
    profile.yaw_rate_dynamic_kp_quad_a = 0.0f; // NORMAL 档角速度环动态Kp一次项：弯里误差增大时线性抬 Kp。
    profile.yaw_rate_dynamic_kp_min = 0.0f; // NORMAL 档角速度环动态Kp下限：与基础 Kp 对齐，保证一进角速度环就有足够支撑。
    profile.yaw_rate_dynamic_kp_max = 10.0f; // NORMAL 档角速度环动态Kp上限：给中大弯留更强纠偏空间，但仍比固定大 Kp 更可控。
    profile.yaw_rate_kp_enable_error_threshold_px = 0.0f; // NORMAL 档角速度环Kp启用阈值：更早介入，避免等偏差放大后才开始补转向。
    profile.yaw_rate_ki = 0.0f; // NORMAL 档角速度环积分项：当前关闭，避免姿态积分拖尾。
    profile.yaw_rate_kd = 0.0f; // NORMAL 档角速度环微分项：略回一点，帮助压住补强后可能带来的过冲。
    profile.yaw_rate_max_integral = 0.0f; // NORMAL 档角速度环积分限幅：0 表示当前不启用额外积分限幅。
    profile.yaw_rate_max_output = 160.0f; // NORMAL 档角速度环输出限幅：角速度支路允许的最大差速。

    profile.turn_slowdown_start_px = 1.0f; // NORMAL 档弯道降速起点：误差超过 50px 开始降速。
    profile.turn_slowdown_full_px = 10.0f; // NORMAL 档弯道降速满量程点：误差到 60px 时达到最大降速。
    profile.yaw_rate_ref_slowdown_start_dps = 0.0f; // NORMAL 档预瞄目标横摆角速度降速起点：0 表示关闭这条预瞄降速逻辑。
    profile.yaw_rate_ref_slowdown_full_dps = 0.0f; // NORMAL 档预瞄目标横摆角速度降速满量程点：关闭后保持 0。
    profile.turn_min_speed_scale = 0.30f; // NORMAL 档最低保速比例：再大误差也至少保留 7% 基础速度。
    profile.turn_slowdown_max_drop_ratio_per_cycle = 0.95f; // NORMAL 档单周期最大降速比例：限制一拍内速度下降过快。
    profile.turn_slowdown_max_rise_ratio_per_cycle = 0.05f; // NORMAL 档单周期最大升速比例：限制一拍内速度回升过快。

    return profile;
}

inline constexpr Profile kStraightProfile = make_straight_profile();
static_assert(is_dynamic_kp_range_valid(kStraightProfile), "kStraightProfile dynamic Kp range is invalid");
static_assert(is_preview_slowdown_range_valid(kStraightProfile), "kStraightProfile preview slowdown range is invalid");

// 十字阶段：
// 十字两个子状态（CROSS_1/CROSS_2）共用一份参数，默认按直道档模板初始化，便于独立调参。
inline constexpr Profile make_cross_profile()
{
    Profile profile = kNormalProfile;
    profile.base_speed = 350.0f; // NORMAL 档基础速度：非直道加速阶段默认按 300 count/5ms 行驶。
    profile.straight_full_speed_error_threshold_px = 0.0f; // NORMAL 档直道满速阈值：整条路径均值误差小于 1px 才允许直通满速。

    profile.position_dynamic_kp_quad_a = 4.4f; // NORMAL 档位置环动态Kp一次项：误差越大，Kp 按线性速度增长。
    profile.position_dynamic_kp_base = 1.2f; // NORMAL 档位置环动态Kp基础值：零误差附近从 1300 起步，避免“名义基础值”被下限吃掉。
    profile.position_dynamic_kp_min = 0.0f; // NORMAL 档位置环动态Kp下限：当前与基础值保持一致，表示零误差起点就是最小 Kp。
    profile.position_dynamic_kp_max = 90.0f; // NORMAL 档位置环动态Kp上限：防止比例项过强。
    profile.position_ki = 0.0f; // NORMAL 档位置环积分项：当前关闭，避免积分拖尾。
    profile.position_kd = 0.0f; // NORMAL 档位置环微分项：抑制转向过冲和来回摆动。
    profile.position_max_integral = 0.0f; // NORMAL 档位置环积分限幅：0 表示当前不启用额外积分限幅。
    profile.position_max_output = 280.0f; // NORMAL 档位置环输出限幅：位置支路允许的最大差速。
    profile.steering_max_output = 280.0f; // NORMAL 档总转向限幅：位置环与角速度环叠加后的总差速上限。

    profile.yaw_rate_ref_from_error_gain_dps = 0.0f; // NORMAL 档误差前馈增益：当前固定夹角模式，不参与目标横摆角速度生成，建议保持 0。
    profile.yaw_rate_ref_from_curvature_gain_dps = 0.0f; // NORMAL 档曲率前馈增益：当前固定夹角模式，不参与目标横摆角速度生成，建议保持 0。
    profile.yaw_rate_ref_from_track_point_gain_dps = 4.0f; // NORMAL 档跟踪点夹角前馈增益：目标点偏角映射到目标横摆角速度的比例。
    profile.yaw_rate_ref_limit_dps = 220.0f; // NORMAL 档目标横摆角速度限幅：防止视觉异常时姿态目标过大。
    profile.yaw_rate_kp = 1.2f; // NORMAL 档角速度环比例项：把弯道基础支撑补回来，避免入弯后车头跟不上。
    profile.yaw_rate_dynamic_kp_quad_a = 0.00f; // NORMAL 档角速度环动态Kp一次项：弯里误差增大时线性抬 Kp。
    profile.yaw_rate_dynamic_kp_min = 0.0f; // NORMAL 档角速度环动态Kp下限：与基础 Kp 对齐，保证一进角速度环就有足够支撑。
    profile.yaw_rate_dynamic_kp_max = 10.0f; // NORMAL 档角速度环动态Kp上限：给中大弯留更强纠偏空间，但仍比固定大 Kp 更可控。
    profile.yaw_rate_kp_enable_error_threshold_px = 0.0f; // NORMAL 档角速度环Kp启用阈值：更早介入，避免等偏差放大后才开始补转向。
    profile.yaw_rate_ki = 0.0f; // NORMAL 档角速度环积分项：当前关闭，避免姿态积分拖尾。
    profile.yaw_rate_kd = 0.02f; // NORMAL 档角速度环微分项：略回一点，帮助压住补强后可能带来的过冲。
    profile.yaw_rate_max_integral = 0.0f; // NORMAL 档角速度环积分限幅：0 表示当前不启用额外积分限幅。
    profile.yaw_rate_max_output = 240.0f; // NORMAL 档角速度环输出限幅：角速度支路允许的最大差速。

    profile.turn_slowdown_start_px = 10.0f; // NORMAL 档弯道降速起点：误差超过 50px 开始降速。
    profile.turn_slowdown_full_px = 25.0f; // NORMAL 档弯道降速满量程点：误差到 60px 时达到最大降速。
    profile.yaw_rate_ref_slowdown_start_dps = 0.0f; // NORMAL 档预瞄目标横摆角速度降速起点：0 表示关闭这条预瞄降速逻辑。
    profile.yaw_rate_ref_slowdown_full_dps = 0.0f; // NORMAL 档预瞄目标横摆角速度降速满量程点：关闭后保持 0。
    profile.turn_min_speed_scale = 0.90f; // NORMAL 档最低保速比例：再大误差也至少保留 7% 基础速度。
    profile.turn_slowdown_max_drop_ratio_per_cycle = 0.45f; // NORMAL 档单周期最大降速比例：限制一拍内速度下降过快。
    profile.turn_slowdown_max_rise_ratio_per_cycle = 0.01f; // NORMAL 档单周期最大升速比例：限制一拍内速度回升过快。
    return profile;
}

inline constexpr Profile kCrossProfile = make_cross_profile();
static_assert(is_dynamic_kp_range_valid(kCrossProfile), "kCrossProfile dynamic Kp range is invalid");
static_assert(is_preview_slowdown_range_valid(kCrossProfile), "kCrossProfile preview slowdown range is invalid");

// 环岛入口：
// 适当降速，同时稍微增强位置环/角速度前馈，帮助更早建立转向。
inline constexpr Profile make_circle_enter_profile()
{
    Profile profile = kNormalProfile;

    //profile.base_speed = 250.0f; // 环岛入口档基础速度：入口默认按 225 count/5ms 行驶。
    // profile.straight_full_speed_error_threshold_px = 0.0f; // 环岛入口档直道满速阈值：禁用入口阶段满速直通。

    // profile.position_dynamic_kp_quad_a = 2.0f; // 环岛入口档位置环动态Kp一次项：大误差时更积极地拉回中线。
    // profile.position_dynamic_kp_base = 1500.0f; // 环岛入口档位置环动态Kp基础值：零误差附近就保持较强贴线，和实际生效起点一致。
    // profile.position_dynamic_kp_min = 1500.0f; // 环岛入口档位置环动态Kp下限：当前与基础值一致，避免基础值被下限覆盖。
    // profile.position_dynamic_kp_max = 6650.0f; // 环岛入口档位置环动态Kp上限：允许更强的极限纠偏。
    // profile.position_max_output = 2312.5f; // 环岛入口档位置环输出限幅：位置支路可给更大差速。
    // profile.steering_max_output = 2312.5f; // 环岛入口档总转向限幅：整体转向空间略放大。

    // profile.yaw_rate_ref_from_error_gain_dps = 0.0f; // 环岛入口档误差前馈增益：当前固定夹角模式，不参与目标横摆角速度生成，建议保持 0。
    // profile.yaw_rate_ref_from_curvature_gain_dps = 0.0f; // 环岛入口档曲率前馈增益：当前固定夹角模式，不参与目标横摆角速度生成，建议保持 0。
    // profile.yaw_rate_ref_from_track_point_gain_dps = 4.32f; // 环岛入口档跟踪点夹角前馈增益：目标点偏角带来的姿态目标更积极。
    // profile.yaw_rate_ref_limit_dps = 231.0f; // 环岛入口档目标横摆角速度限幅：允许更大的姿态目标。
    // profile.yaw_rate_kp = 0.0f; // 环岛入口档角速度环比例项：提高姿态跟踪强度，并保持在动态Kp允许区间内。
    // profile.yaw_rate_dynamic_kp_quad_a = 0.0f; // 环岛入口档角速度环动态Kp一次项：默认关闭动态。
    // profile.yaw_rate_dynamic_kp_min = 0.0f; // 环岛入口档角速度环动态Kp下限。
    // profile.yaw_rate_dynamic_kp_max = 0.0f; // 环岛入口档角速度环动态Kp上限。
    // profile.yaw_rate_max_output = 0.0f; // 环岛入口档角速度环输出限幅：角速度支路可给更大差速。

    // profile.turn_slowdown_start_px = 1.0f; // 环岛入口档弯道降速起点：较早触发降速进入环岛。
    // profile.turn_slowdown_full_px = 22.0f; // 环岛入口档弯道降速满量程点：22px 误差时达到最大降速。
    // profile.yaw_rate_ref_slowdown_start_dps = 0.0f; // 环岛入口档预瞄目标横摆角速度降速起点：0 表示关闭这条预瞄降速逻辑。
    // profile.yaw_rate_ref_slowdown_full_dps = 0.0f; // 环岛入口档预瞄目标横摆角速度降速满量程点：关闭后保持 0。
    // profile.turn_min_speed_scale = 0.75f; // 环岛入口档最低保速比例：入口至少保留 15% 基础速度。

    return profile;
}

inline constexpr Profile kCircleEnterProfile = make_circle_enter_profile();
static_assert(is_dynamic_kp_range_valid(kCircleEnterProfile), "kCircleEnterProfile dynamic Kp range is invalid");
static_assert(is_preview_slowdown_range_valid(kCircleEnterProfile), "kCircleEnterProfile preview slowdown range is invalid");

// 环岛内部：
// 这是最激进但也最慢的档位，优先保证能稳住单边界巡迹。
inline constexpr Profile make_circle_inside_profile()
{
    Profile profile = kNormalProfile;

    //profile.base_speed = 250.0f; // 环岛内部档基础速度：内部默认按 200 count/5ms 控速。
    // profile.straight_full_speed_error_threshold_px = 0.0f; // 环岛内部档直道满速阈值：禁用内部阶段满速直通。

    // profile.position_dynamic_kp_quad_a = 45.0f; // 环岛内部档位置环动态Kp一次项：大误差时更果断地拉回单边中线。
    // profile.position_dynamic_kp_base = 1800.0f; // 环岛内部档位置环动态Kp基础值：零误差附近也保持高强度贴线，和实际生效起点对齐。
    // profile.position_dynamic_kp_min =1800.0f; // 环岛内部档位置环动态Kp下限：当前与基础值一致，避免环内小误差区出现隐藏夹限。
    // profile.position_dynamic_kp_max = 6650.0f; // 环岛内部档位置环动态Kp上限：允许更强的极限纠偏。
    // profile.position_kd = 16.0f; // 环岛内部档位置环微分项：增强抑制过冲的能力。
    // profile.position_max_output = 2375.0f; // 环岛内部档位置环输出限幅：位置支路可给更大差速。
    // profile.steering_max_output = 2375.0f; // 环岛内部档总转向限幅：整体转向最激进。

    // profile.yaw_rate_ref_from_error_gain_dps = 0.0f; // 环岛内部档误差前馈增益：当前固定夹角模式，不参与目标横摆角速度生成，建议保持 0。
    // profile.yaw_rate_ref_from_curvature_gain_dps = 0.0f; // 环岛内部档曲率前馈增益：当前固定夹角模式，不参与目标横摆角速度生成，建议保持 0。
    // profile.yaw_rate_ref_from_track_point_gain_dps = 4.48f; // 环岛内部档跟踪点夹角前馈增益：目标点偏角带来的姿态目标更果断。
    // profile.yaw_rate_ref_limit_dps = 237.6f; // 环岛内部档目标横摆角速度限幅：允许更高姿态目标。
    // profile.yaw_rate_kp = 0.0; // 环岛内部档角速度环比例项。
    // profile.yaw_rate_dynamic_kp_quad_a = 0.0; // 环岛内部档角速度环动态Kp一次项。
    // profile.yaw_rate_dynamic_kp_min = 0.0; // 环岛内部档角速度环动态Kp下限。
    // profile.yaw_rate_dynamic_kp_max = 0.0; // 环岛内部档角速度环动态Kp上限。
    // profile.yaw_rate_ki = 0.0; // 环岛内部档角速度环积分项。
    // profile.yaw_rate_kd = 0.0; // 环岛内部档角速度环微分项。
    // profile.yaw_rate_max_integral = 0.0; // 环岛内部档角速度环积分限幅：与 NORMAL 对齐。
    // profile.yaw_rate_max_output = 0.0; // 环岛内部档角速度环输出限幅：与 NORMAL 对齐。

    // profile.turn_slowdown_start_px = 2.0f; // 环岛内部档弯道降速起点：更早降速，优先保证稳定性。
    // profile.turn_slowdown_full_px = 30.0f; // 环岛内部档弯道降速满量程点：较小误差就达到最大降速。
    // profile.yaw_rate_ref_slowdown_start_dps = 0.0f; // 环岛内部档预瞄目标横摆角速度降速起点：0 表示关闭这条预瞄降速逻辑。
    // profile.yaw_rate_ref_slowdown_full_dps = 0.0f; // 环岛内部档预瞄目标横摆角速度降速满量程点：关闭后保持 0。
    // profile.turn_min_speed_scale = 0.80f; // 环岛内部档最低保速比例：内部至少保留 12% 基础速度。

    return profile;
}

inline constexpr Profile kCircleInsideProfile = make_circle_inside_profile();
static_assert(is_dynamic_kp_range_valid(kCircleInsideProfile), "kCircleInsideProfile dynamic Kp range is invalid");
static_assert(is_preview_slowdown_range_valid(kCircleInsideProfile), "kCircleInsideProfile preview slowdown range is invalid");

// 环岛出环：
// 保持一定转向积极性，但开始恢复速度，为回到 NORMAL 做过渡。
inline constexpr Profile make_circle_exit_profile()
{
    Profile profile = kNormalProfile;

    // profile.base_speed = 250.0f; // 环岛出环档基础速度：出环默认按 250 count/5ms 过渡回直道。
    // profile.straight_full_speed_error_threshold_px = 0.0f; // 环岛出环档直道满速阈值：禁用出环阶段满速直通。

    // profile.position_dynamic_kp_quad_a = 60.0f; // 环岛出环档位置环动态Kp一次项：保持较强的大误差纠偏能力。
    // profile.position_dynamic_kp_base = 2000.0f; // 环岛出环档位置环动态Kp基础值：零误差附近直接从出环专用起点开始，帮助尽快摆正车头。
    // profile.position_dynamic_kp_min = 2000.0f; // 环岛出环档位置环动态Kp下限：当前与基础值一致，避免出环档基础值被隐藏覆盖。
    // profile.position_dynamic_kp_max = 6650.0f; // 环岛出环档位置环动态Kp上限：允许一定的强纠偏。
    // profile.position_max_output = 2375.0f; // 环岛出环档位置环输出限幅：位置支路仍保留较大的差速空间。
    // profile.steering_max_output = 2375.0f; // 环岛出环档总转向限幅：整体转向略高于 NORMAL。

    // profile.yaw_rate_ref_from_error_gain_dps = 0.0f; // 环岛出环档误差前馈增益：当前固定夹角模式，不参与目标横摆角速度生成，建议保持 0。
    // profile.yaw_rate_ref_from_curvature_gain_dps = 0.0f; // 环岛出环档曲率前馈增益：当前固定夹角模式，不参与目标横摆角速度生成，建议保持 0。
    // profile.yaw_rate_ref_from_track_point_gain_dps = 4.32f; // 环岛出环档跟踪点夹角前馈增益：目标点偏角仍能带来较明显姿态目标。
    // profile.yaw_rate_ref_limit_dps = 231.0f; // 环岛出环档目标横摆角速度限幅：允许略大的姿态目标。
    // profile.yaw_rate_kp = 0.0f; // 环岛出环档角速度环比例项：姿态跟踪比 NORMAL 略紧。
    // profile.yaw_rate_dynamic_kp_quad_a = 0.0f; // 环岛出环档角速度环动态Kp一次项：默认关闭动态。
    // profile.yaw_rate_dynamic_kp_min = 0.0f; // 环岛出环档角速度环动态Kp下限。
    // profile.yaw_rate_dynamic_kp_max = 0.0f; // 环岛出环档角速度环动态Kp上限。
    // profile.yaw_rate_max_output = 1273.0f; // 环岛出环档角速度环输出限幅：姿态支路差速上限略放大。

    // profile.turn_slowdown_start_px = 0.0f; // 环岛出环档弯道降速起点：比 NORMAL 更早开始控速，平稳退出。
    // profile.turn_slowdown_full_px = 25.0f; // 环岛出环档弯道降速满量程点：50px 误差时达到最大降速。
    // profile.yaw_rate_ref_slowdown_start_dps = 0.0f; // 环岛出环档预瞄目标横摆角速度降速起点：0 表示关闭这条预瞄降速逻辑。
    // profile.yaw_rate_ref_slowdown_full_dps = 0.0f; // 环岛出环档预瞄目标横摆角速度降速满量程点：关闭后保持 0。
    // profile.turn_min_speed_scale = 0.50f; // 环岛出环档最低保速比例：出环至少保留 14% 基础速度。

    return profile;
}

inline constexpr Profile kCircleExitProfile = make_circle_exit_profile();
static_assert(is_dynamic_kp_range_valid(kCircleExitProfile), "kCircleExitProfile dynamic Kp range is invalid");
static_assert(is_preview_slowdown_range_valid(kCircleExitProfile), "kCircleExitProfile preview slowdown range is invalid");
} // namespace route_line_follow
} // namespace pid_tuning

#endif

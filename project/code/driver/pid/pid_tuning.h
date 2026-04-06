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
inline constexpr float kGyroYawRateFilterAlpha = 0.30f;
} // namespace imu

namespace motor_speed
{
// 左轮速度环比例项：误差一出现就立即给修正，越大响应越快，但过大容易抖。
inline constexpr float kLeftKp = 0.0058f;
// 左轮速度环积分项：用来消除稳态误差，越大越容易追平长期偏差，但也更容易拖尾。
inline constexpr float kLeftKi = 0.00070f;
// 左轮速度环微分项：抑制误差变化过快，当前默认关闭。
inline constexpr float kLeftKd = 0.0f;

// 右轮速度环比例项：含义同左轮，用于补偿左右轮机械差异可单独设置。
inline constexpr float kRightKp = 0.0058f;
// 右轮速度环积分项：含义同左轮。
inline constexpr float kRightKi = 0.00070f;
// 右轮速度环微分项：当前默认关闭。
inline constexpr float kRightKd = 0.0f;

// 积分限幅：限制单次参与积分计算的误差幅度，避免积分累积过大。
inline constexpr float kIntegralLimit = 180.0f;
// 单周期最大修正步长：限制 PID 输出每个 5ms 周期最多变化多少，越大越跟手。
inline constexpr float kMaxOutputStep = 2.40f;
// PID 修正项总限幅：只限制 PID 修正部分，不包含前馈和减速辅助。
inline constexpr float kCorrectionLimit = 24.0f;

// 左轮速度前馈斜率：目标速度每增加 1 count/5ms，预先增加多少 duty。
inline constexpr float kLeftFeedforwardGain = 0.075f;
// 右轮速度前馈斜率：通常左右轮不完全对称，允许单独设置。
inline constexpr float kRightFeedforwardGain = 0.085f;
// 左轮静摩擦补偿：目标速度超过阈值后，额外补一点 duty 帮助起转。
inline constexpr float kLeftFeedforwardBias = 1.2f;
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
inline constexpr float kFeedbackLowPassAlpha = 0.80f;
} // namespace motor_speed

namespace position_loop
{
// 巡线位置环动态比例项：Kp = base + quad_a * error^2。
// 使用归一化误差，参数不随分辨率变化。
inline constexpr float kDynamicKpQuadA = 800.0f;
// 误差为 0 时的基础 Kp。
inline constexpr float kDynamicKpBase = 1000.0f;
// 动态 Kp 下限与上限：防止过大过小。
inline constexpr float kDynamicKpMin = 1300.0f;
inline constexpr float kDynamicKpMax = 2500.0f;
// 巡线位置环积分项：用于消除长期偏差，当前默认关闭。
inline constexpr float kKi = 0.0f;
// 巡线位置环微分项：抑制误差变化过快，缓和转向过冲。
inline constexpr float kKd = 920.0f; //800
// 巡线位置环输出限幅：限制最终差速大小，避免大舵过猛。
inline constexpr float kMaxOutput = 1250.0f;
} // namespace position_loop

namespace yaw_rate_loop
{
// 视觉曲率滤波系数：让目标角速度更多跟随赛道整体趋势，而不是单帧角点毛刺。
// 约定：该滤波只在“收到新视觉帧”时推进一次。
inline constexpr float kVisualCurvatureFilterAlpha = 0.25f;
// 目标点夹角滤波系数：平滑单帧跟踪点跳动，避免目标角速度突然抽动。
// 约定：该滤波只在“收到新视觉帧”时推进一次。
inline constexpr float kTrackPointAngleFilterAlpha = 0.25f;
// 视觉误差 -> 目标横摆角速度的映射增益。
// 误差越大，给角速度环的“该转多快”目标就越大。
inline constexpr float kRefFromErrorGainDps = 180.0f;
// 视觉曲率 -> 目标横摆角速度的映射增益。
// 这个量负责“提前量”，赛道曲率越大，越提前给出转向速度目标。
inline constexpr float kRefFromCurvatureGainDps = 1600.0f;
// 跟踪点夹角 -> 目标横摆角速度的映射增益。
// 夹角越大，说明目标点偏离中垂线越明显，需要建立更大的横摆速度。
inline constexpr float kRefFromTrackPointAngleGainDps = 4.0f;
// 目标横摆角速度上限，避免视觉异常时把角速度目标推得过猛。
inline constexpr float kRefLimitDps = 220.0f;

// 角速度环建议保持“位置式 PID”：
// 1) 它本身就是并级支路，直接输出一份差速量，位置式更直观；
// 2) IMU 信号更新快，位置式更容易和输出限幅、积分限幅配合；
// 3) 增量式更适合底层执行器或占空比直接调节，这里不是那个层级。
inline constexpr float kKp = 1.50f;
inline constexpr float kKi = 0.0f;
inline constexpr float kKd = 0.0f;
inline constexpr float kMaxIntegral = 0.0f;
inline constexpr float kMaxOutput = 260.0f;
} // namespace yaw_rate_loop

namespace line_follow
{
// 视觉误差低通系数：越大越跟当前帧，越小越重视历史趋势。
// 约定：该滤波只在“收到新视觉帧”时推进一次。
inline constexpr float kErrorFilterAlpha = 0.80f;

// 左右轮目标最小值：允许轻微反转，方便大误差时快速拧回车头。
inline constexpr float kTargetCountMin = -400.0f;
// 左右轮目标最大值：限制巡线线程下发给速度环的目标上限。
inline constexpr float kTargetCountMax = 1250.0f;

// 大弯降速起点：误差超过这个像素后，开始按比例降低基础速度。
inline constexpr float kTurnSlowdownStartPx = 50.0f;
// 大弯降速满量程点：误差超过这个像素后，降速比例达到上限。
inline constexpr float kTurnSlowdownFullPx = 60.0f;
// 大弯最低速度比例：基础速度最低会保留到这个比例，不会无限降。
inline constexpr float kTurnMinSpeedScale = 0.07f;
// 单周期最大降速比例：每次循环最多只允许比上一周期再降这么多。
inline constexpr float kTurnSlowdownMaxDropRatioPerCycle = 0.85f;
// 单周期最大加速比例：每次循环最多只允许比上一周期再升这么多。
inline constexpr float kTurnSlowdownMaxRiseRatioPerCycle = 0.02f;

// 归一化误差保护限幅：防止视觉异常值把控制链一下子打爆。
inline constexpr float kNormalizedErrorLimit = 1.2f;
// 误差死区：采样线附近的小抖动直接忽略，减少左右抽动。
inline constexpr float kErrorDeadzonePx = 0.6f;
// 小误差降增益区间：误差小于这个值时，先温和修正。
inline constexpr float kErrorLowGainLimitPx = 3.0f;
// 小误差降增益系数：进入小误差区间后，控制量乘这个比例。
inline constexpr float kErrorLowGain = 0.70f;
} // namespace line_follow
} // namespace pid_tuning

#endif

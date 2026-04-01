#ifndef PID_TUNING_H_
#define PID_TUNING_H_

#include "zf_common_headfile.h"

// 说明：
// 1) 这个文件专门放“默认调参值”，方便集中修改。
// 2) 这里的参数会被速度环 PID 和巡线位置环在初始化时读取。
// 3) 如果后续串口动态改 PID，运行时值可能与这里不同；这里仍然是编译时默认值入口。

namespace pid_tuning
{
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

namespace line_follow
{
// 视觉误差低通系数：越大越跟当前帧，越小越重视历史趋势。
inline constexpr float kErrorFilterAlpha = 0.80f;

// 巡线位置环比例项：横向误差一出现就打方向，越大回正越快。
inline constexpr float kPidKp = 1200.0f;
// 巡线位置环积分项：用于消除长期偏差，当前默认关闭。
inline constexpr float kPidKi = 0.0f;
// 巡线位置环微分项：抑制误差变化过快，缓和转向过冲。
inline constexpr float kPidKd = 800.0f;
// 巡线位置环输出限幅：限制最终差速大小，避免大舵过猛。
inline constexpr float kPidMaxOutput = 800.0f;

// 左右轮目标最小值：允许轻微反转，方便大误差时快速拧回车头。
inline constexpr float kTargetCountMin = 20.0f;
// 左右轮目标最大值：限制巡线线程下发给速度环的目标上限。
inline constexpr float kTargetCountMax = 1000.0f;

// 大弯降速起点：误差超过这个像素后，开始按比例降低基础速度。
inline constexpr float kTurnSlowdownStartPx = 2.0f;
// 大弯降速满量程点：误差超过这个像素后，降速比例达到上限。
inline constexpr float kTurnSlowdownFullPx = 20.0f;
// 大弯最低速度比例：基础速度最低会保留到这个比例，不会无限降。
inline constexpr float kTurnMinSpeedScale = 0.30f;
// 单周期最大降速比例：每次循环最多只允许比上一周期再降这么多。
inline constexpr float kTurnSlowdownMaxDropRatioPerCycle = 0.50f;
// 单周期最大加速比例：每次循环最多只允许比上一周期再升这么多。
inline constexpr float kTurnSlowdownMaxRiseRatioPerCycle = 0.10f;

// 归一化误差保护限幅：防止视觉异常值把控制链一下子打爆。
inline constexpr float kNormalizedErrorLimit = 1.2f;
// 误差死区：采样线附近的小抖动直接忽略，减少左右抽动。
inline constexpr float kErrorDeadzonePx = 0.5f;
// 小误差降增益区间：误差小于这个值时，先温和修正。
inline constexpr float kErrorLowGainLimitPx = 3.0f;
// 小误差降增益系数：进入小误差区间后，控制量乘这个比例。
inline constexpr float kErrorLowGain = 0.70f;
} // namespace line_follow
} // namespace pid_tuning

#endif

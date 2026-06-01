#ifndef PID_TUNING_H_
#define PID_TUNING_H_

#include "zf_common_headfile.h"

namespace pid_tuning
{
namespace imu
{
extern int32 kStartupCalibrateDurationMs;
extern float kGyroYawRateSign;
extern float kGyroYawRateFilterAlpha;
} // namespace imu

namespace motor_speed
{
extern float kLeftKp;
extern float kLeftKi;
extern float kLeftKd;
extern float kRightKp;
extern float kRightKi;
extern float kRightKd;
extern float kIntegralLimit;
extern float kMaxOutputStep;
extern float kCorrectionLimit;
extern float kLeftFeedforwardGain;
extern float kRightFeedforwardGain;
extern float kLeftFeedforwardBias;
extern float kRightFeedforwardBias;
extern float kFeedforwardBiasThreshold;
extern float kLeftFeedforwardLowSpeedBoost;
extern float kRightFeedforwardLowSpeedBoost;
extern float kLeftFeedforwardLowSpeedCutoff;
extern float kRightFeedforwardLowSpeedCutoff;
extern float kDecelErrorThreshold;
extern float kDecelDutyGain;
extern float kDecelDutyLimit;
extern int32 kFeedbackAverageWindow;
extern float kFeedbackLowPassAlpha;
extern bool kSpeedDebugEnabled;
} // namespace motor_speed

namespace brushless
{
extern bool kRealtimeEnabled;
extern float kLeftDutyPercent;
extern float kRightDutyPercent;
} // namespace brushless

namespace line_follow
{
extern float kTargetCountMin;
extern float kTargetCountMax;
extern bool kYawRateDebugEnabled;
extern float kYawRateDebugTargetDps;
extern bool kSpeedLoopDebugEnabled;
extern float kSpeedLoopDebugLeftTarget;
extern float kSpeedLoopDebugRightTarget;
extern float kSpeedLoopDebugBaseSpeed;
extern float kSpeedLoopDebugDiffSpeed;
} // namespace line_follow

namespace line_error_preview
{
inline constexpr size_t kWeightedPointCountMax = 16;

struct WeightedProfile
{
    size_t weighted_point_count;
    int point_indices[kWeightedPointCountMax];
    float weights[kWeightedPointCountMax];
};

bool is_weighted_profile_valid(const WeightedProfile &profile);

extern WeightedProfile kNormalWeightedProfile;
extern WeightedProfile kStraightWeightedProfile;
extern WeightedProfile kCrossWeightedProfile;
extern WeightedProfile kCircleEnterWeightedProfile;
extern WeightedProfile kCircleInsideWeightedProfile;
extern WeightedProfile kCircleExitWeightedProfile;
} // namespace line_error_preview

namespace route_line_follow
{
struct Profile
{
    float base_speed;

    float position_kp;
    float position_ki;
    float position_kd;
    float position_max_integral;
    float position_max_output;
    float yaw_rate_kp;
    float yaw_rate_ki;
    float yaw_rate_kd;
    float yaw_rate_max_integral;
    float yaw_rate_max_output;

    // line_error 前缀指数加权参数（新方案，按状态独立）。
    float line_error_prefix_ratio;
};

extern float kGlobalBaseSpeedScale;
extern float kCornerDecelMaxGyroDps;
extern float kCornerDecelMinSpeedScale;

bool is_line_error_prefix_exp_valid(const Profile &profile);

extern Profile kNormalProfile;
} // namespace route_line_follow
} // namespace pid_tuning

#endif

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
extern float kDecelErrorThreshold;
extern float kDecelDutyGain;
extern float kDecelDutyLimit;
extern int32 kFeedbackAverageWindow;
extern float kFeedbackLowPassAlpha;
} // namespace motor_speed

namespace brushless
{
extern bool kRealtimeEnabled;
extern float kLeftDutyPercent;
extern float kRightDutyPercent;
} // namespace brushless

namespace yaw_rate_loop
{
extern float kVisualCurvatureFilterAlpha;
extern float kTrackPointAngleFilterAlpha;
} // namespace yaw_rate_loop

namespace line_follow
{
extern float kErrorFilterAlpha;
extern float kTargetCountMin;
extern float kTargetCountMax;
extern float kErrorDeadzonePx;
extern float kErrorLowGainLimitPx;
extern float kErrorLowGain;
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

    float position_dynamic_kp_quad_a;
    float position_dynamic_kp_base;
    float position_dynamic_kp_min;
    float position_dynamic_kp_max;
    float position_dynamic_kp_low_error_threshold_px;
    float position_dynamic_kp_mid_a;
    float position_dynamic_kp_mid_error_threshold_px;
    float position_dynamic_kp_high_a;
    float position_ki;
    float position_kd;
    float position_dynamic_kd_quad_a;
    float position_dynamic_kd_min;
    float position_dynamic_kd_max;
    float position_max_integral;
    float position_max_output;
    float steering_max_output;

    float yaw_rate_ref_from_error_gain_dps;
    float yaw_rate_ref_from_curvature_gain_dps;
    float yaw_rate_ref_from_track_point_gain_dps;
    float yaw_rate_ref_limit_dps;
    float yaw_rate_kp;
    float yaw_rate_dynamic_kp_quad_a;
    float yaw_rate_dynamic_kp_min;
    float yaw_rate_dynamic_kp_max;
    float yaw_rate_kp_enable_error_threshold_px;
    float yaw_rate_ki;
    float yaw_rate_kd;
    float yaw_rate_max_integral;
    float yaw_rate_max_output;

    // line_error 前缀指数加权参数（新方案，按状态独立）。
    float line_error_prefix_ratio;
    float line_error_exp_lambda;

    // base_speed 速度方案参数（新方案，按状态独立）。
    // 速度方案后段目标点指数权重参数 lambda（>=0）：
    // 后段点按指数权重加权（权重和归一化为 1），用于生成速度决策目标点坐标。
    float speed_scheme_rear_exp_lambda;
    // 速度方案摩擦圆参数 n（>0）：
    // 速度决策公式为 v_target^2 + (angle_target * v_real)^2 = n。
    float speed_scheme_friction_circle_n;
    float speed_scheme_max_drop_ratio_per_cycle;
    float speed_scheme_max_rise_ratio_per_cycle;
};

extern float kGlobalBaseSpeedScale;

bool is_dynamic_kp_range_valid(const Profile &profile);
bool is_dynamic_position_kd_range_valid(const Profile &profile);
bool is_position_kp_piecewise_range_valid(const Profile &profile);
bool is_line_error_prefix_exp_valid(const Profile &profile);
bool is_speed_scheme_range_valid(const Profile &profile);

extern Profile kNormalProfile;
extern Profile kStraightProfile;
extern Profile kCrossProfile;
extern Profile kCircleEnterProfile;
extern Profile kCircleInsideProfile;
extern Profile kCircleExitProfile;
} // namespace route_line_follow
} // namespace pid_tuning

#endif

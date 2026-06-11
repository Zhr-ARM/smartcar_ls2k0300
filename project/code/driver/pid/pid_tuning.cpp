#include "driver/pid/pid_tuning.h"

namespace pid_tuning
{
namespace imu
{
int32 kStartupCalibrateDurationMs = 2000;
float kGyroYawRateSign = 1.0f;
float kGyroYawRateFilterAlpha = 0.45f;
} // namespace imu

namespace motor_speed
{
float kLeftKp = 0.1980f;
float kLeftKi = 0.00072f;
float kLeftKd = 0.0f;
float kRightKp = 0.2030f;
float kRightKi = 0.00078f;
float kRightKd = 0.0f;
float kIntegralLimit = 0.0f;
float kMaxOutputStep = 1000.0f;
float kCorrectionLimit = 100.0f;
float kLeftFeedforwardGain = 0.0f;
float kRightFeedforwardGain = 0.0f;
float kLeftFeedforwardBias = 0.0f;
float kRightFeedforwardBias = 0.0f;
float kFeedforwardBiasThreshold = 0.0f;
float kDecelErrorThreshold = 0.0f;
float kDecelDutyGain = 0.0f;
float kDecelDutyLimit = 0.0f;
int32 kFeedbackAverageWindow = 1;
float kFeedbackLowPassAlpha = 1.0f;
} // namespace motor_speed

namespace brushless
{
bool kRealtimeEnabled = false;
float kLeftDutyPercent = 0.0f;
float kRightDutyPercent = 0.0f;
} // namespace brushless

namespace yaw_rate_loop
{
float kVisualCurvatureFilterAlpha = 0.25f;
float kTrackPointAngleFilterAlpha = 0.6f;
} // namespace yaw_rate_loop

namespace line_follow
{
float kErrorFilterAlpha = 0.95f;
float kTargetCountMin = -200.0f;
float kTargetCountMax = 1550.0f;
float kErrorDeadzonePx = 0.6f;
float kErrorLowGainLimitPx = 3.0f;
float kErrorLowGain = 0.70f;
} // namespace line_follow

namespace line_error_preview
{
bool is_weighted_profile_valid(const WeightedProfile &profile)
{
    if (profile.weighted_point_count == 0 || profile.weighted_point_count > kWeightedPointCountMax)
    {
        return false;
    }

    float total_weight = 0.0f;
    for (size_t i = 0; i < profile.weighted_point_count; ++i)
    {
        if (profile.point_indices[i] < 0 || profile.weights[i] < 0.0f)
        {
            return false;
        }
        total_weight += profile.weights[i];
    }

    return total_weight > 0.0f;
}

WeightedProfile kNormalWeightedProfile = {
    3,
    {16, 20, 26},
    {0.5f, 0.25f, 0.25f},
};

WeightedProfile kStraightWeightedProfile = {
    3,
    {14, 22, 28},
    {0.45f, 0.35f, 0.20f},
};

WeightedProfile kCrossWeightedProfile = {
    3,
    {4, 8, 12},
    {0.5f, 0.4f, 0.1f},
};

WeightedProfile kCircleEnterWeightedProfile = {
    3,
    {5, 10, 15},
    {0.5f, 0.3f, 0.2f},
};

WeightedProfile kCircleInsideWeightedProfile = {
    3,
    {4, 8, 12},
    {0.55f, 0.30f, 0.15f},
};

WeightedProfile kCircleExitWeightedProfile = {
    3,
    {5, 10, 15},
    {0.5f, 0.3f, 0.2f},
};
} // namespace line_error_preview

namespace route_line_follow
{
float kGlobalBaseSpeedScale = 1.00f;

bool is_dynamic_kp_range_valid(const Profile &profile)
{
    return (profile.position_dynamic_kp_min <= profile.position_dynamic_kp_base) &&
           (profile.position_dynamic_kp_base <= profile.position_dynamic_kp_max) &&
           (profile.yaw_rate_dynamic_kp_min <= profile.yaw_rate_kp) &&
           (profile.yaw_rate_kp <= profile.yaw_rate_dynamic_kp_max) &&
           (profile.curve_position_dynamic_kp_min <= profile.curve_position_dynamic_kp_base) &&
           (profile.curve_position_dynamic_kp_base <= profile.curve_position_dynamic_kp_max) &&
           (profile.curve_yaw_rate_dynamic_kp_min <= profile.curve_yaw_rate_kp) &&
           (profile.curve_yaw_rate_kp <= profile.curve_yaw_rate_dynamic_kp_max);
}

bool is_position_kp_piecewise_range_valid(const Profile &profile)
{
    return (profile.position_dynamic_kp_low_error_threshold_px >= 0.0f) &&
           (profile.position_dynamic_kp_low_error_threshold_px <=
            profile.position_dynamic_kp_mid_error_threshold_px) &&
           (profile.curve_position_dynamic_kp_low_error_threshold_px >= 0.0f) &&
           (profile.curve_position_dynamic_kp_low_error_threshold_px <=
            profile.curve_position_dynamic_kp_mid_error_threshold_px);
}

bool is_line_error_prefix_exp_valid(const Profile &profile)
{
    return (profile.line_error_prefix_ratio > 0.0f) &&
           (profile.line_error_prefix_ratio <= 1.0f);
}

bool is_speed_scheme_range_valid(const Profile &profile)
{
    if (profile.speed_scheme_max_drop_ratio_per_cycle < 0.0f ||
        profile.speed_scheme_max_drop_ratio_per_cycle > 1.0f)
    {
        return false;
    }
    if (profile.speed_scheme_max_rise_ratio_per_cycle < 0.0f ||
        profile.speed_scheme_max_rise_ratio_per_cycle > 1.0f)
    {
        return false;
    }
    if (profile.speed_scheme_min_base_speed < 0.0f)
    {
        return false;
    }
    if (profile.speed_scheme_centerline_slope_change_rate_filter_alpha < 0.0f ||
        profile.speed_scheme_centerline_slope_change_rate_filter_alpha > 1.0f)
    {
        return false;
    }
    if (profile.slope_control_curve_threshold < 0.0f ||
        profile.slope_control_straight_confirm_frames < 1 ||
        profile.curve_base_speed < 0.0f ||
        profile.curve_speed_scheme_min_base_speed < 0.0f)
    {
        return false;
    }
    return true;
}

Profile kNormalProfile = [] {
    Profile p{};
    p.base_speed = 350.0f;
    p.position_dynamic_kp_quad_a = 3.0f;
    p.position_dynamic_kp_base = 2.1f;
    p.position_dynamic_kp_min = 0.0f;
    p.position_dynamic_kp_max = 50.0f;
    p.position_dynamic_kp_low_error_threshold_px = 3.0f;
    p.position_dynamic_kp_mid_a = 4.6f;
    p.position_dynamic_kp_mid_error_threshold_px = 10.0f;
    p.position_dynamic_kp_high_a = 5.6f;
    p.position_ki = 0.0f;
    p.position_kd = 0.15f;
    p.position_max_integral = 0.0f;
    p.position_max_output = 210.0f;
    p.steering_max_output = 210.0f;
    p.yaw_rate_ref_from_error_gain_dps = 0.0f;
    p.yaw_rate_ref_from_track_point_gain_dps = 7.0f;
    p.yaw_rate_ref_limit_dps = 360.0f;
    p.yaw_rate_kp = 1.0f;
    p.yaw_rate_dynamic_kp_quad_a = 0.0f;
    p.yaw_rate_dynamic_kp_min = 0.0f;
    p.yaw_rate_dynamic_kp_max = 10.0f;
    p.yaw_rate_kp_enable_error_threshold_px = 0.0f;
    p.yaw_rate_ki = 0.0f;
    p.yaw_rate_kd = 0.0f;
    p.yaw_rate_max_integral = 0.0f;
    p.yaw_rate_max_output = 200.0f;
    p.line_error_prefix_ratio = 0.6f;
    p.speed_scheme_max_drop_ratio_per_cycle = 0.82f;
    p.speed_scheme_max_rise_ratio_per_cycle = 0.01f;
    p.speed_scheme_min_base_speed = 270.0f;
    p.speed_scheme_centerline_slope_change_rate_enabled = true;
    p.speed_scheme_centerline_slope_change_rate_filter_alpha = 0.45f;
    p.slope_control_curve_threshold = 1.10f;
    p.slope_control_straight_confirm_frames = 10;
    p.curve_base_speed = p.speed_scheme_min_base_speed;
    p.curve_position_dynamic_kp_quad_a = p.position_dynamic_kp_quad_a;
    p.curve_position_dynamic_kp_base = p.position_dynamic_kp_base;
    p.curve_position_dynamic_kp_min = p.position_dynamic_kp_min;
    p.curve_position_dynamic_kp_max = p.position_dynamic_kp_max;
    p.curve_position_dynamic_kp_low_error_threshold_px = p.position_dynamic_kp_low_error_threshold_px;
    p.curve_position_dynamic_kp_mid_a = p.position_dynamic_kp_mid_a;
    p.curve_position_dynamic_kp_mid_error_threshold_px = p.position_dynamic_kp_mid_error_threshold_px;
    p.curve_position_dynamic_kp_high_a = p.position_dynamic_kp_high_a;
    p.curve_position_ki = p.position_ki;
    p.curve_position_kd = p.position_kd;
    p.curve_position_max_integral = p.position_max_integral;
    p.curve_position_max_output = p.position_max_output;
    p.curve_steering_max_output = p.steering_max_output;
    p.curve_yaw_rate_ref_from_error_gain_dps = p.yaw_rate_ref_from_error_gain_dps;
    p.curve_yaw_rate_ref_from_track_point_gain_dps = p.yaw_rate_ref_from_track_point_gain_dps;
    p.curve_yaw_rate_ref_limit_dps = p.yaw_rate_ref_limit_dps;
    p.curve_yaw_rate_kp = p.yaw_rate_kp;
    p.curve_yaw_rate_dynamic_kp_quad_a = p.yaw_rate_dynamic_kp_quad_a;
    p.curve_yaw_rate_dynamic_kp_min = p.yaw_rate_dynamic_kp_min;
    p.curve_yaw_rate_dynamic_kp_max = p.yaw_rate_dynamic_kp_max;
    p.curve_yaw_rate_kp_enable_error_threshold_px = p.yaw_rate_kp_enable_error_threshold_px;
    p.curve_yaw_rate_ki = p.yaw_rate_ki;
    p.curve_yaw_rate_kd = p.yaw_rate_kd;
    p.curve_yaw_rate_max_integral = p.yaw_rate_max_integral;
    p.curve_yaw_rate_max_output = p.yaw_rate_max_output;
    p.curve_speed_scheme_min_base_speed = p.speed_scheme_min_base_speed;
    return p;
}();

Profile kCircleProfile = kNormalProfile;
} // namespace route_line_follow
} // namespace pid_tuning

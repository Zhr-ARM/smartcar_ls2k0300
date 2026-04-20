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
float kLeftKp = 0.0850f;
float kLeftKi = 0.00078f;
float kLeftKd = 0.0f;
float kRightKp = 0.0850f;
float kRightKi = 0.00078f;
float kRightKd = 0.0f;
float kIntegralLimit = 180.0f;
float kMaxOutputStep = 15.0f;
float kCorrectionLimit = 24.0f;
float kLeftFeedforwardGain = 0.090f;
float kRightFeedforwardGain = 0.090f;
float kLeftFeedforwardBias = 1.4f;
float kRightFeedforwardBias = 1.4f;
float kFeedforwardBiasThreshold = 5.0f;
float kDecelErrorThreshold = 5.0f;
float kDecelDutyGain = 0.18f;
float kDecelDutyLimit = 14.0f;
int32 kFeedbackAverageWindow = 2;
float kFeedbackLowPassAlpha = 0.95f;
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
           (profile.yaw_rate_kp <= profile.yaw_rate_dynamic_kp_max);
}

bool is_dynamic_position_kd_range_valid(const Profile &profile)
{
    return (profile.position_dynamic_kd_min <= profile.position_kd) &&
           (profile.position_kd <= profile.position_dynamic_kd_max);
}

bool is_position_kp_piecewise_range_valid(const Profile &profile)
{
    return (profile.position_dynamic_kp_low_error_threshold_px >= 0.0f) &&
           (profile.position_dynamic_kp_low_error_threshold_px <=
            profile.position_dynamic_kp_mid_error_threshold_px);
}

bool is_line_error_prefix_exp_valid(const Profile &profile)
{
    return (profile.line_error_prefix_ratio > 0.0f) &&
           (profile.line_error_prefix_ratio <= 1.0f) &&
           (profile.line_error_exp_lambda >= 0.0f) &&
           (profile.line_error_exp_lambda <= 20.0f);
}

bool is_speed_scheme_range_valid(const Profile &profile)
{
    if (profile.speed_scheme_rear_exp_lambda < 0.0f ||
        profile.speed_scheme_rear_exp_lambda > 20.0f)
    {
        return false;
    }
    if (profile.speed_scheme_slowdown_full <= profile.speed_scheme_slowdown_start)
    {
        return false;
    }
    if (profile.speed_scheme_min_speed_scale < 0.0f ||
        profile.speed_scheme_min_speed_scale > 1.0f)
    {
        return false;
    }
    if (profile.speed_scheme_max_speed_scale < profile.speed_scheme_min_speed_scale ||
        profile.speed_scheme_max_speed_scale > 1.0f)
    {
        return false;
    }
    if (profile.speed_scheme_centerline_count_upper <= profile.speed_scheme_centerline_count_lower)
    {
        return false;
    }
    if (profile.speed_scheme_centerline_scale_lower < 0.0f ||
        profile.speed_scheme_centerline_scale_lower > 1.0f)
    {
        return false;
    }
    if (profile.speed_scheme_centerline_scale_upper < profile.speed_scheme_centerline_scale_lower ||
        profile.speed_scheme_centerline_scale_upper > 1.0f)
    {
        return false;
    }
    if (profile.speed_scheme_force_full_min_centerline_count < 0)
    {
        return false;
    }
    if (profile.speed_scheme_force_full_abs_error_sum_per_point < 0.0f)
    {
        return false;
    }
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
    return true;
}

Profile kNormalProfile = {
    350.0f,
    3.0f, 2.1f, 0.0f, 50.0f, 3.0f, 4.6f, 10.0f, 5.6f, 0.0f, 0.15f, 0.0f, 0.0f, 2.0f, 0.0f, 210.0f, 210.0f,
    0.0f, 0.0f, 7.0f, 360.0f, 1.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 200.0f,
    0.6f, 2.303f,
    1.609f, 1.0f, 40.0f, 0.7f, 1.0f,
    0, 30, 0.1f, 1.0f,
    30, 2.0f,
    0.82f, 0.01f
};

Profile kStraightProfile = {
    370.0f,
    0.0f, 1.0f, 0.0f, 30.0f, 3.0f, 0.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 300.0f, 600.0f,
    0.0f, 0.0f, 4.0f, 360.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 160.0f,
    0.6f, 2.303f,
    1.609f, 1.0f, 40.0f, 0.7f, 1.0f,
    0, 30, 0.1f, 1.0f,
    30, 2.0f,
    0.95f, 0.5f
};

Profile kCrossProfile = {
    350.0f,
    1.7f, 0.4f, 0.0f, 110.0f, 3.0f, 3.6f, 10.0f, 4.4f, 0.0f, 0.0f, 0.0f, 0.0f, 3.0f, 0.0f, 260.0f, 360.0f,
    0.0f, 0.0f, 7.0f, 360.0f, 1.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 300.0f,
    0.6f, 2.303f,
    1.609f, 1.0f, 40.0f, 0.7f, 1.0f,
    0, 30, 0.1f, 1.0f,
    30, 2.0f,
    0.82f, 0.01f
};

Profile kCircleEnterProfile = {
    320.0f,
    1.7f, 1.8f, 0.0f, 110.0f, 3.0f, 3.6f, 9.0f, 4.4f, 0.0f, 0.0f, 0.0f, 0.0f, 3.0f, 0.0f, 400.0f, 720.0f,
    0.0f, 0.0f, 7.0f, 360.0f, 1.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 320.0f,
    0.6f, 2.303f,
    1.609f, 1.0f, 40.0f, 0.7f, 1.0f,
    0, 30, 0.1f, 1.0f,
    30, 2.0f,
    0.75f, 0.01f
};

Profile kCircleInsideProfile = {
    320.0f,
    1.7f, 1.8f, 0.0f, 110.0f, 3.0f, 3.6f, 9.0f, 4.4f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 400.0f, 660.0f,
    0.0f, 0.0f, 7.0f, 360.0f, 1.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f, 0.02f, 0.0f, 300.0f,
    0.6f, 2.303f,
    1.609f, 1.0f, 40.0f, 0.7f, 1.0f,
    0, 30, 0.1f, 1.0f,
    30, 2.0f,
    0.82f, 0.01f
};

Profile kCircleExitProfile = {
    320.0f,
    1.7f, 1.8f, 0.0f, 110.0f, 3.0f, 3.6f, 10.0f, 4.4f, 0.0f, 0.0f, 0.0f, 0.0f, 3.0f, 0.0f, 400.0f, 660.0f,
    0.0f, 0.0f, 7.0f, 360.0f, 1.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f, 0.02f, 0.0f, 300.0f,
    0.6f, 2.303f,
    1.609f, 1.0f, 40.0f, 0.7f, 1.0f,
    0, 30, 0.1f, 1.0f,
    30, 2.0f,
    0.82f, 0.01f
};
} // namespace route_line_follow
} // namespace pid_tuning

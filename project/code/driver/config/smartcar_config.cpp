#include "driver/config/smartcar_config.h"

#include "app/line_follow_thread/line_follow_thread.h"
#include "app/motor_thread/motor_thread.h"
#include "app/vision_thread/vision_thread.h"
#include "driver/pid/pid_tuning.h"
#include "driver/vision/vision_config.h"
#include "driver/vision/vision_image_processor.h"
#include "driver/vision/vision_transport.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <limits.h>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <unistd.h>

namespace
{
struct StringStorage
{
    std::string udp_web_server_ip;
    std::string assistant_server_ip;
    std::array<std::string, VISION_NCNN_CONFIG_MAX_LABELS> ncnn_labels;
};

StringStorage g_string_storage;
std::mutex g_config_mutex;
std::string g_loaded_config_path;

using RawMap = std::unordered_map<std::string, std::string>;

struct PidSnapshot
{
    int32 imu_startup_calibrate_duration_ms = 0;
    float imu_gyro_yaw_rate_sign = 0.0f;
    float imu_gyro_yaw_rate_filter_alpha = 0.0f;

    float motor_left_kp = 0.0f;
    float motor_left_ki = 0.0f;
    float motor_left_kd = 0.0f;
    float motor_right_kp = 0.0f;
    float motor_right_ki = 0.0f;
    float motor_right_kd = 0.0f;
    float motor_integral_limit = 0.0f;
    float motor_max_output_step = 0.0f;
    float motor_correction_limit = 0.0f;
    float motor_left_feedforward_gain = 0.0f;
    float motor_right_feedforward_gain = 0.0f;
    float motor_left_feedforward_bias = 0.0f;
    float motor_right_feedforward_bias = 0.0f;
    float motor_feedforward_bias_threshold = 0.0f;
    float motor_decel_error_threshold = 0.0f;
    float motor_decel_duty_gain = 0.0f;
    float motor_decel_duty_limit = 0.0f;
    int32 motor_feedback_average_window = 0;
    float motor_feedback_low_pass_alpha = 0.0f;
    bool brushless_realtime_enabled = false;
    float brushless_left_duty_percent = 0.0f;
    float brushless_right_duty_percent = 0.0f;

    float yaw_rate_visual_curvature_filter_alpha = 0.0f;
    float yaw_rate_track_point_angle_filter_alpha = 0.0f;

    float line_follow_error_filter_alpha = 0.0f;
    float line_follow_target_count_min = 0.0f;
    float line_follow_target_count_max = 0.0f;
    float line_follow_error_deadzone_px = 0.0f;
    float line_follow_error_low_gain_limit_px = 0.0f;
    float line_follow_error_low_gain = 0.0f;

    pid_tuning::line_error_preview::WeightedProfile normal_weighted_profile{};
    pid_tuning::line_error_preview::WeightedProfile straight_weighted_profile{};
    pid_tuning::line_error_preview::WeightedProfile cross_weighted_profile{};
    pid_tuning::line_error_preview::WeightedProfile circle_enter_weighted_profile{};
    pid_tuning::line_error_preview::WeightedProfile circle_inside_weighted_profile{};
    pid_tuning::line_error_preview::WeightedProfile circle_exit_weighted_profile{};

    float route_global_base_speed_scale = 0.0f;
    pid_tuning::route_line_follow::Profile normal_profile{};
    pid_tuning::route_line_follow::Profile straight_profile{};
    pid_tuning::route_line_follow::Profile cross_profile{};
    pid_tuning::route_line_follow::Profile circle_enter_profile{};
    pid_tuning::route_line_follow::Profile circle_inside_profile{};
    pid_tuning::route_line_follow::Profile circle_exit_profile{};
};

struct ConfigSnapshot
{
    vision_runtime_config_t vision_runtime{};
    vision_processor_config_t vision_processor{};
    PidSnapshot pid{};
    StringStorage strings{};
    std::string loaded_path;
};

std::string executable_dir_config_path()
{
    char exe_path[PATH_MAX] = {0};
    const ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    if (len <= 0)
    {
        return "";
    }

    exe_path[len] = '\0';
    std::string full_path(exe_path);
    const size_t slash_pos = full_path.find_last_of('/');
    if (slash_pos == std::string::npos)
    {
        return "";
    }
    return full_path.substr(0, slash_pos + 1) + "smartcar_config.toml";
}

std::string trim(const std::string &value)
{
    size_t begin = 0;
    while (begin < value.size() && std::isspace(static_cast<unsigned char>(value[begin])))
    {
        ++begin;
    }

    size_t end = value.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1])))
    {
        --end;
    }
    return value.substr(begin, end - begin);
}

std::string strip_comment(const std::string &line)
{
    bool in_quotes = false;
    bool escaped = false;
    for (size_t i = 0; i < line.size(); ++i)
    {
        const char ch = line[i];
        if (escaped)
        {
            escaped = false;
            continue;
        }
        if (ch == '\\')
        {
            escaped = true;
            continue;
        }
        if (ch == '"')
        {
            in_quotes = !in_quotes;
            continue;
        }
        if (!in_quotes && ch == '#')
        {
            return line.substr(0, i);
        }
    }
    return line;
}

bool parse_key_values_text(const std::string &text, RawMap *values, std::string *error_message)
{
    std::string section;
    std::istringstream input(text);
    std::string line;
    int line_number = 0;
    while (std::getline(input, line))
    {
        ++line_number;
        line = trim(strip_comment(line));
        if (line.empty())
        {
            continue;
        }

        if (line.front() == '[')
        {
            if (line.back() != ']')
            {
                *error_message = "invalid section header at line " + std::to_string(line_number);
                return false;
            }
            section = trim(line.substr(1, line.size() - 2));
            if (section.empty())
            {
                *error_message = "empty section header at line " + std::to_string(line_number);
                return false;
            }
            continue;
        }

        const size_t eq_pos = line.find('=');
        if (eq_pos == std::string::npos)
        {
            *error_message = "missing '=' at line " + std::to_string(line_number);
            return false;
        }

        const std::string key = trim(line.substr(0, eq_pos));
        const std::string value = trim(line.substr(eq_pos + 1));
        if (key.empty() || value.empty())
        {
            *error_message = "invalid key/value at line " + std::to_string(line_number);
            return false;
        }

        const std::string full_key = section.empty() ? key : (section + "." + key);
        if (values->find(full_key) != values->end())
        {
            *error_message = "duplicate key: " + full_key;
            return false;
        }
        (*values)[full_key] = value;
    }

    return true;
}

bool parse_key_values(const std::string &path, RawMap *values, std::string *error_message)
{
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open())
    {
        *error_message = "cannot open file: " + path;
        return false;
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return parse_key_values_text(buffer.str(), values, error_message);
}

bool take_raw(const RawMap &values,
              std::set<std::string> *consumed,
              const std::string &key,
              std::string *raw,
              std::string *error_message)
{
    const auto it = values.find(key);
    if (it == values.end())
    {
        *error_message = "missing required key: " + key;
        return false;
    }
    consumed->insert(key);
    *raw = it->second;
    return true;
}

void consume_optional_key_if_present(const RawMap &values,
                                     std::set<std::string> *consumed,
                                     const std::string &key)
{
    if (values.find(key) != values.end())
    {
        consumed->insert(key);
    }
}

bool parse_bool_value(const std::string &raw, bool *value)
{
    if (raw == "true")
    {
        *value = true;
        return true;
    }
    if (raw == "false")
    {
        *value = false;
        return true;
    }
    return false;
}

bool parse_int_value(const std::string &raw, int *value)
{
    try
    {
        size_t consumed = 0;
        const long parsed = std::stol(raw, &consumed, 10);
        if (consumed != raw.size())
        {
            return false;
        }
        *value = static_cast<int>(parsed);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool parse_float_value(const std::string &raw, float *value)
{
    try
    {
        size_t consumed = 0;
        *value = std::stof(raw, &consumed);
        return consumed == raw.size();
    }
    catch (...)
    {
        return false;
    }
}

bool parse_size_t_value(const std::string &raw, size_t *value)
{
    try
    {
        size_t consumed = 0;
        const unsigned long long parsed = std::stoull(raw, &consumed, 10);
        if (consumed != raw.size())
        {
            return false;
        }
        *value = static_cast<size_t>(parsed);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool parse_string_value(const std::string &raw, std::string *value)
{
    if (raw.size() < 2 || raw.front() != '"' || raw.back() != '"')
    {
        return false;
    }

    value->clear();
    bool escaped = false;
    for (size_t i = 1; i + 1 < raw.size(); ++i)
    {
        const char ch = raw[i];
        if (escaped)
        {
            switch (ch)
            {
                case 'n': value->push_back('\n'); break;
                case 't': value->push_back('\t'); break;
                case '\\': value->push_back('\\'); break;
                case '"': value->push_back('"'); break;
                default: return false;
            }
            escaped = false;
            continue;
        }
        if (ch == '\\')
        {
            escaped = true;
            continue;
        }
        value->push_back(ch);
    }
    return !escaped;
}

bool split_array_items(const std::string &raw, std::vector<std::string> *items)
{
    if (raw.size() < 2 || raw.front() != '[' || raw.back() != ']')
    {
        return false;
    }

    items->clear();
    std::string current;
    bool in_quotes = false;
    bool escaped = false;
    for (size_t i = 1; i + 1 < raw.size(); ++i)
    {
        const char ch = raw[i];
        if (escaped)
        {
            current.push_back(ch);
            escaped = false;
            continue;
        }
        if (ch == '\\')
        {
            current.push_back(ch);
            escaped = true;
            continue;
        }
        if (ch == '"')
        {
            current.push_back(ch);
            in_quotes = !in_quotes;
            continue;
        }
        if (!in_quotes && ch == ',')
        {
            items->push_back(trim(current));
            current.clear();
            continue;
        }
        current.push_back(ch);
    }
    if (!trim(current).empty() || raw.find(',') != std::string::npos)
    {
        items->push_back(trim(current));
    }
    if (items->size() == 1 && items->front().empty())
    {
        items->clear();
    }
    return !in_quotes && !escaped;
}

bool require_bool(const RawMap &values, std::set<std::string> *consumed, const std::string &key, bool *target, std::string *error_message)
{
    std::string raw;
    if (!take_raw(values, consumed, key, &raw, error_message))
    {
        return false;
    }
    if (!parse_bool_value(raw, target))
    {
        *error_message = "invalid bool for key: " + key;
        return false;
    }
    return true;
}

bool require_int(const RawMap &values, std::set<std::string> *consumed, const std::string &key, int *target, std::string *error_message)
{
    std::string raw;
    if (!take_raw(values, consumed, key, &raw, error_message))
    {
        return false;
    }
    if (!parse_int_value(raw, target))
    {
        *error_message = "invalid int for key: " + key;
        return false;
    }
    return true;
}

bool require_size_t(const RawMap &values, std::set<std::string> *consumed, const std::string &key, size_t *target, std::string *error_message)
{
    std::string raw;
    if (!take_raw(values, consumed, key, &raw, error_message))
    {
        return false;
    }
    if (!parse_size_t_value(raw, target))
    {
        *error_message = "invalid size_t for key: " + key;
        return false;
    }
    return true;
}

bool require_float(const RawMap &values, std::set<std::string> *consumed, const std::string &key, float *target, std::string *error_message)
{
    std::string raw;
    if (!take_raw(values, consumed, key, &raw, error_message))
    {
        return false;
    }
    if (!parse_float_value(raw, target))
    {
        *error_message = "invalid float for key: " + key;
        return false;
    }
    return true;
}

bool require_string(const RawMap &values,
                    std::set<std::string> *consumed,
                    const std::string &key,
                    std::string *storage,
                    const char **target,
                    std::string *error_message)
{
    std::string raw;
    if (!take_raw(values, consumed, key, &raw, error_message))
    {
        return false;
    }
    if (!parse_string_value(raw, storage))
    {
        *error_message = "invalid string for key: " + key;
        return false;
    }
    *target = storage->c_str();
    return true;
}

template <size_t N>
bool require_int_array(const RawMap &values,
                       std::set<std::string> *consumed,
                       const std::string &key,
                       int (&target)[N],
                       std::string *error_message)
{
    std::string raw;
    if (!take_raw(values, consumed, key, &raw, error_message))
    {
        return false;
    }
    std::vector<std::string> items;
    if (!split_array_items(raw, &items) || items.size() != N)
    {
        *error_message = "invalid int array size for key: " + key;
        return false;
    }
    for (size_t i = 0; i < N; ++i)
    {
        if (!parse_int_value(items[i], &target[i]))
        {
            *error_message = "invalid int array item for key: " + key;
            return false;
        }
    }
    return true;
}

template <size_t N>
bool require_float_array(const RawMap &values,
                         std::set<std::string> *consumed,
                         const std::string &key,
                         float (&target)[N],
                         std::string *error_message)
{
    std::string raw;
    if (!take_raw(values, consumed, key, &raw, error_message))
    {
        return false;
    }
    std::vector<std::string> items;
    if (!split_array_items(raw, &items) || items.size() != N)
    {
        *error_message = "invalid float array size for key: " + key;
        return false;
    }
    for (size_t i = 0; i < N; ++i)
    {
        if (!parse_float_value(items[i], &target[i]))
        {
            *error_message = "invalid float array item for key: " + key;
            return false;
        }
    }
    return true;
}

template <size_t N>
bool require_double_array(const RawMap &values,
                          std::set<std::string> *consumed,
                          const std::string &key,
                          double (&target)[N],
                          std::string *error_message)
{
    std::string raw;
    if (!take_raw(values, consumed, key, &raw, error_message))
    {
        return false;
    }
    std::vector<std::string> items;
    if (!split_array_items(raw, &items) || items.size() != N)
    {
        *error_message = "invalid double array size for key: " + key;
        return false;
    }
    for (size_t i = 0; i < N; ++i)
    {
        float parsed = 0.0f;
        if (!parse_float_value(items[i], &parsed))
        {
            *error_message = "invalid double array item for key: " + key;
            return false;
        }
        target[i] = parsed;
    }
    return true;
}

bool require_string_array(const RawMap &values,
                          std::set<std::string> *consumed,
                          const std::string &key,
                          std::array<std::string, VISION_NCNN_CONFIG_MAX_LABELS> *storage,
                          const char *(&target)[VISION_NCNN_CONFIG_MAX_LABELS],
                          size_t *count,
                          std::string *error_message)
{
    std::string raw;
    if (!take_raw(values, consumed, key, &raw, error_message))
    {
        return false;
    }
    std::vector<std::string> items;
    if (!split_array_items(raw, &items) || items.size() > VISION_NCNN_CONFIG_MAX_LABELS)
    {
        *error_message = "invalid string array size for key: " + key;
        return false;
    }

    for (size_t i = 0; i < VISION_NCNN_CONFIG_MAX_LABELS; ++i)
    {
        (*storage)[i].clear();
        target[i] = nullptr;
    }

    for (size_t i = 0; i < items.size(); ++i)
    {
        if (!parse_string_value(items[i], &(*storage)[i]))
        {
            *error_message = "invalid string array item for key: " + key;
            return false;
        }
        target[i] = (*storage)[i].c_str();
    }
    *count = items.size();
    return true;
}

[[maybe_unused]] bool load_weighted_profile(const RawMap &values,
                                            std::set<std::string> *consumed,
                                            const std::string &prefix,
                                            pid_tuning::line_error_preview::WeightedProfile *profile,
                                            std::string *error_message)
{
    if (!require_size_t(values, consumed, prefix + ".weighted_point_count", &profile->weighted_point_count, error_message))
    {
        return false;
    }

    std::string raw_indices;
    if (!take_raw(values, consumed, prefix + ".point_indices", &raw_indices, error_message))
    {
        return false;
    }
    std::vector<std::string> index_items;
    if (!split_array_items(raw_indices, &index_items) || index_items.size() != profile->weighted_point_count)
    {
        *error_message = "point_indices size mismatch for " + prefix;
        return false;
    }

    std::string raw_weights;
    if (!take_raw(values, consumed, prefix + ".weights", &raw_weights, error_message))
    {
        return false;
    }
    std::vector<std::string> weight_items;
    if (!split_array_items(raw_weights, &weight_items) || weight_items.size() != profile->weighted_point_count)
    {
        *error_message = "weights size mismatch for " + prefix;
        return false;
    }

    for (size_t i = 0; i < pid_tuning::line_error_preview::kWeightedPointCountMax; ++i)
    {
        profile->point_indices[i] = 0;
        profile->weights[i] = 0.0f;
    }

    for (size_t i = 0; i < profile->weighted_point_count; ++i)
    {
        if (!parse_int_value(index_items[i], &profile->point_indices[i]) ||
            !parse_float_value(weight_items[i], &profile->weights[i]))
        {
            *error_message = "invalid weighted profile item for " + prefix;
            return false;
        }
    }
    return true;
}

bool load_route_profile(const RawMap &values,
                        std::set<std::string> *consumed,
                        const std::string &prefix,
                        pid_tuning::route_line_follow::Profile *profile,
                        std::string *error_message)
{
    return require_float(values, consumed, prefix + ".base_speed", &profile->base_speed, error_message) &&
           require_float(values, consumed, prefix + ".position_dynamic_kp_quad_a", &profile->position_dynamic_kp_quad_a, error_message) &&
           require_float(values, consumed, prefix + ".position_dynamic_kp_base", &profile->position_dynamic_kp_base, error_message) &&
           require_float(values, consumed, prefix + ".position_dynamic_kp_min", &profile->position_dynamic_kp_min, error_message) &&
           require_float(values, consumed, prefix + ".position_dynamic_kp_max", &profile->position_dynamic_kp_max, error_message) &&
           require_float(values, consumed, prefix + ".position_dynamic_kp_low_error_threshold_px", &profile->position_dynamic_kp_low_error_threshold_px, error_message) &&
           require_float(values, consumed, prefix + ".position_dynamic_kp_mid_a", &profile->position_dynamic_kp_mid_a, error_message) &&
           require_float(values, consumed, prefix + ".position_dynamic_kp_mid_error_threshold_px", &profile->position_dynamic_kp_mid_error_threshold_px, error_message) &&
           require_float(values, consumed, prefix + ".position_dynamic_kp_high_a", &profile->position_dynamic_kp_high_a, error_message) &&
           require_float(values, consumed, prefix + ".position_ki", &profile->position_ki, error_message) &&
           require_float(values, consumed, prefix + ".position_kd", &profile->position_kd, error_message) &&
           require_float(values, consumed, prefix + ".position_dynamic_kd_quad_a", &profile->position_dynamic_kd_quad_a, error_message) &&
           require_float(values, consumed, prefix + ".position_dynamic_kd_min", &profile->position_dynamic_kd_min, error_message) &&
           require_float(values, consumed, prefix + ".position_dynamic_kd_max", &profile->position_dynamic_kd_max, error_message) &&
           require_float(values, consumed, prefix + ".position_max_integral", &profile->position_max_integral, error_message) &&
           require_float(values, consumed, prefix + ".position_max_output", &profile->position_max_output, error_message) &&
           require_float(values, consumed, prefix + ".steering_max_output", &profile->steering_max_output, error_message) &&
           require_float(values, consumed, prefix + ".yaw_rate_ref_from_error_gain_dps", &profile->yaw_rate_ref_from_error_gain_dps, error_message) &&
           require_float(values, consumed, prefix + ".yaw_rate_ref_from_curvature_gain_dps", &profile->yaw_rate_ref_from_curvature_gain_dps, error_message) &&
           require_float(values, consumed, prefix + ".yaw_rate_ref_from_track_point_gain_dps", &profile->yaw_rate_ref_from_track_point_gain_dps, error_message) &&
           require_float(values, consumed, prefix + ".yaw_rate_ref_limit_dps", &profile->yaw_rate_ref_limit_dps, error_message) &&
           require_float(values, consumed, prefix + ".yaw_rate_kp", &profile->yaw_rate_kp, error_message) &&
           require_float(values, consumed, prefix + ".yaw_rate_dynamic_kp_quad_a", &profile->yaw_rate_dynamic_kp_quad_a, error_message) &&
           require_float(values, consumed, prefix + ".yaw_rate_dynamic_kp_min", &profile->yaw_rate_dynamic_kp_min, error_message) &&
           require_float(values, consumed, prefix + ".yaw_rate_dynamic_kp_max", &profile->yaw_rate_dynamic_kp_max, error_message) &&
           require_float(values, consumed, prefix + ".yaw_rate_kp_enable_error_threshold_px", &profile->yaw_rate_kp_enable_error_threshold_px, error_message) &&
           require_float(values, consumed, prefix + ".yaw_rate_ki", &profile->yaw_rate_ki, error_message) &&
           require_float(values, consumed, prefix + ".yaw_rate_kd", &profile->yaw_rate_kd, error_message) &&
           require_float(values, consumed, prefix + ".yaw_rate_max_integral", &profile->yaw_rate_max_integral, error_message) &&
           require_float(values, consumed, prefix + ".yaw_rate_max_output", &profile->yaw_rate_max_output, error_message) &&
           require_float(values, consumed, prefix + ".line_error_prefix_ratio", &profile->line_error_prefix_ratio, error_message) &&
           require_float(values, consumed, prefix + ".line_error_exp_lambda", &profile->line_error_exp_lambda, error_message) &&
           require_float(values, consumed, prefix + ".speed_scheme_rear_exp_lambda", &profile->speed_scheme_rear_exp_lambda, error_message) &&
           require_float(values, consumed, prefix + ".speed_scheme_friction_circle_n", &profile->speed_scheme_friction_circle_n, error_message) &&
           require_float(values, consumed, prefix + ".speed_scheme_max_drop_ratio_per_cycle", &profile->speed_scheme_max_drop_ratio_per_cycle, error_message) &&
           require_float(values, consumed, prefix + ".speed_scheme_max_rise_ratio_per_cycle", &profile->speed_scheme_max_rise_ratio_per_cycle, error_message);
}

PidSnapshot capture_pid_snapshot()
{
    PidSnapshot snapshot;
    snapshot.imu_startup_calibrate_duration_ms = pid_tuning::imu::kStartupCalibrateDurationMs;
    snapshot.imu_gyro_yaw_rate_sign = pid_tuning::imu::kGyroYawRateSign;
    snapshot.imu_gyro_yaw_rate_filter_alpha = pid_tuning::imu::kGyroYawRateFilterAlpha;

    snapshot.motor_left_kp = pid_tuning::motor_speed::kLeftKp;
    snapshot.motor_left_ki = pid_tuning::motor_speed::kLeftKi;
    snapshot.motor_left_kd = pid_tuning::motor_speed::kLeftKd;
    snapshot.motor_right_kp = pid_tuning::motor_speed::kRightKp;
    snapshot.motor_right_ki = pid_tuning::motor_speed::kRightKi;
    snapshot.motor_right_kd = pid_tuning::motor_speed::kRightKd;
    snapshot.motor_integral_limit = pid_tuning::motor_speed::kIntegralLimit;
    snapshot.motor_max_output_step = pid_tuning::motor_speed::kMaxOutputStep;
    snapshot.motor_correction_limit = pid_tuning::motor_speed::kCorrectionLimit;
    snapshot.motor_left_feedforward_gain = pid_tuning::motor_speed::kLeftFeedforwardGain;
    snapshot.motor_right_feedforward_gain = pid_tuning::motor_speed::kRightFeedforwardGain;
    snapshot.motor_left_feedforward_bias = pid_tuning::motor_speed::kLeftFeedforwardBias;
    snapshot.motor_right_feedforward_bias = pid_tuning::motor_speed::kRightFeedforwardBias;
    snapshot.motor_feedforward_bias_threshold = pid_tuning::motor_speed::kFeedforwardBiasThreshold;
    snapshot.motor_decel_error_threshold = pid_tuning::motor_speed::kDecelErrorThreshold;
    snapshot.motor_decel_duty_gain = pid_tuning::motor_speed::kDecelDutyGain;
    snapshot.motor_decel_duty_limit = pid_tuning::motor_speed::kDecelDutyLimit;
    snapshot.motor_feedback_average_window = pid_tuning::motor_speed::kFeedbackAverageWindow;
    snapshot.motor_feedback_low_pass_alpha = pid_tuning::motor_speed::kFeedbackLowPassAlpha;
    snapshot.brushless_realtime_enabled = pid_tuning::brushless::kRealtimeEnabled;
    snapshot.brushless_left_duty_percent = pid_tuning::brushless::kLeftDutyPercent;
    snapshot.brushless_right_duty_percent = pid_tuning::brushless::kRightDutyPercent;

    snapshot.yaw_rate_visual_curvature_filter_alpha = pid_tuning::yaw_rate_loop::kVisualCurvatureFilterAlpha;
    snapshot.yaw_rate_track_point_angle_filter_alpha = pid_tuning::yaw_rate_loop::kTrackPointAngleFilterAlpha;

    snapshot.line_follow_error_filter_alpha = pid_tuning::line_follow::kErrorFilterAlpha;
    snapshot.line_follow_target_count_min = pid_tuning::line_follow::kTargetCountMin;
    snapshot.line_follow_target_count_max = pid_tuning::line_follow::kTargetCountMax;
    snapshot.line_follow_error_deadzone_px = pid_tuning::line_follow::kErrorDeadzonePx;
    snapshot.line_follow_error_low_gain_limit_px = pid_tuning::line_follow::kErrorLowGainLimitPx;
    snapshot.line_follow_error_low_gain = pid_tuning::line_follow::kErrorLowGain;

    snapshot.normal_weighted_profile = pid_tuning::line_error_preview::kNormalWeightedProfile;
    snapshot.straight_weighted_profile = pid_tuning::line_error_preview::kStraightWeightedProfile;
    snapshot.cross_weighted_profile = pid_tuning::line_error_preview::kCrossWeightedProfile;
    snapshot.circle_enter_weighted_profile = pid_tuning::line_error_preview::kCircleEnterWeightedProfile;
    snapshot.circle_inside_weighted_profile = pid_tuning::line_error_preview::kCircleInsideWeightedProfile;
    snapshot.circle_exit_weighted_profile = pid_tuning::line_error_preview::kCircleExitWeightedProfile;

    snapshot.route_global_base_speed_scale = pid_tuning::route_line_follow::kGlobalBaseSpeedScale;
    snapshot.normal_profile = pid_tuning::route_line_follow::kNormalProfile;
    snapshot.straight_profile = pid_tuning::route_line_follow::kStraightProfile;
    snapshot.cross_profile = pid_tuning::route_line_follow::kCrossProfile;
    snapshot.circle_enter_profile = pid_tuning::route_line_follow::kCircleEnterProfile;
    snapshot.circle_inside_profile = pid_tuning::route_line_follow::kCircleInsideProfile;
    snapshot.circle_exit_profile = pid_tuning::route_line_follow::kCircleExitProfile;
    return snapshot;
}

void restore_pid_snapshot(const PidSnapshot &snapshot)
{
    pid_tuning::imu::kStartupCalibrateDurationMs = snapshot.imu_startup_calibrate_duration_ms;
    pid_tuning::imu::kGyroYawRateSign = snapshot.imu_gyro_yaw_rate_sign;
    pid_tuning::imu::kGyroYawRateFilterAlpha = snapshot.imu_gyro_yaw_rate_filter_alpha;

    pid_tuning::motor_speed::kLeftKp = snapshot.motor_left_kp;
    pid_tuning::motor_speed::kLeftKi = snapshot.motor_left_ki;
    pid_tuning::motor_speed::kLeftKd = snapshot.motor_left_kd;
    pid_tuning::motor_speed::kRightKp = snapshot.motor_right_kp;
    pid_tuning::motor_speed::kRightKi = snapshot.motor_right_ki;
    pid_tuning::motor_speed::kRightKd = snapshot.motor_right_kd;
    pid_tuning::motor_speed::kIntegralLimit = snapshot.motor_integral_limit;
    pid_tuning::motor_speed::kMaxOutputStep = snapshot.motor_max_output_step;
    pid_tuning::motor_speed::kCorrectionLimit = snapshot.motor_correction_limit;
    pid_tuning::motor_speed::kLeftFeedforwardGain = snapshot.motor_left_feedforward_gain;
    pid_tuning::motor_speed::kRightFeedforwardGain = snapshot.motor_right_feedforward_gain;
    pid_tuning::motor_speed::kLeftFeedforwardBias = snapshot.motor_left_feedforward_bias;
    pid_tuning::motor_speed::kRightFeedforwardBias = snapshot.motor_right_feedforward_bias;
    pid_tuning::motor_speed::kFeedforwardBiasThreshold = snapshot.motor_feedforward_bias_threshold;
    pid_tuning::motor_speed::kDecelErrorThreshold = snapshot.motor_decel_error_threshold;
    pid_tuning::motor_speed::kDecelDutyGain = snapshot.motor_decel_duty_gain;
    pid_tuning::motor_speed::kDecelDutyLimit = snapshot.motor_decel_duty_limit;
    pid_tuning::motor_speed::kFeedbackAverageWindow = snapshot.motor_feedback_average_window;
    pid_tuning::motor_speed::kFeedbackLowPassAlpha = snapshot.motor_feedback_low_pass_alpha;
    pid_tuning::brushless::kRealtimeEnabled = snapshot.brushless_realtime_enabled;
    pid_tuning::brushless::kLeftDutyPercent = snapshot.brushless_left_duty_percent;
    pid_tuning::brushless::kRightDutyPercent = snapshot.brushless_right_duty_percent;

    pid_tuning::yaw_rate_loop::kVisualCurvatureFilterAlpha = snapshot.yaw_rate_visual_curvature_filter_alpha;
    pid_tuning::yaw_rate_loop::kTrackPointAngleFilterAlpha = snapshot.yaw_rate_track_point_angle_filter_alpha;

    pid_tuning::line_follow::kErrorFilterAlpha = snapshot.line_follow_error_filter_alpha;
    pid_tuning::line_follow::kTargetCountMin = snapshot.line_follow_target_count_min;
    pid_tuning::line_follow::kTargetCountMax = snapshot.line_follow_target_count_max;
    pid_tuning::line_follow::kErrorDeadzonePx = snapshot.line_follow_error_deadzone_px;
    pid_tuning::line_follow::kErrorLowGainLimitPx = snapshot.line_follow_error_low_gain_limit_px;
    pid_tuning::line_follow::kErrorLowGain = snapshot.line_follow_error_low_gain;

    pid_tuning::line_error_preview::kNormalWeightedProfile = snapshot.normal_weighted_profile;
    pid_tuning::line_error_preview::kStraightWeightedProfile = snapshot.straight_weighted_profile;
    pid_tuning::line_error_preview::kCrossWeightedProfile = snapshot.cross_weighted_profile;
    pid_tuning::line_error_preview::kCircleEnterWeightedProfile = snapshot.circle_enter_weighted_profile;
    pid_tuning::line_error_preview::kCircleInsideWeightedProfile = snapshot.circle_inside_weighted_profile;
    pid_tuning::line_error_preview::kCircleExitWeightedProfile = snapshot.circle_exit_weighted_profile;

    pid_tuning::route_line_follow::kGlobalBaseSpeedScale = snapshot.route_global_base_speed_scale;
    pid_tuning::route_line_follow::kNormalProfile = snapshot.normal_profile;
    pid_tuning::route_line_follow::kStraightProfile = snapshot.straight_profile;
    pid_tuning::route_line_follow::kCrossProfile = snapshot.cross_profile;
    pid_tuning::route_line_follow::kCircleEnterProfile = snapshot.circle_enter_profile;
    pid_tuning::route_line_follow::kCircleInsideProfile = snapshot.circle_inside_profile;
    pid_tuning::route_line_follow::kCircleExitProfile = snapshot.circle_exit_profile;
}

ConfigSnapshot capture_config_snapshot()
{
    ConfigSnapshot snapshot;
    snapshot.vision_runtime = g_vision_runtime_config;
    snapshot.vision_processor = g_vision_processor_config;
    snapshot.pid = capture_pid_snapshot();
    snapshot.strings = g_string_storage;
    snapshot.loaded_path = g_loaded_config_path;
    return snapshot;
}

void restore_config_snapshot(const ConfigSnapshot &snapshot)
{
    g_string_storage = snapshot.strings;
    g_vision_runtime_config = snapshot.vision_runtime;
    g_vision_processor_config = snapshot.vision_processor;
    g_vision_runtime_config.udp_web_server_ip = g_string_storage.udp_web_server_ip.c_str();
    g_vision_runtime_config.assistant_server_ip = g_string_storage.assistant_server_ip.c_str();
    for (size_t i = 0; i < VISION_NCNN_CONFIG_MAX_LABELS; ++i)
    {
        g_vision_runtime_config.ncnn_labels[i] =
            g_string_storage.ncnn_labels[i].empty() ? nullptr : g_string_storage.ncnn_labels[i].c_str();
    }
    restore_pid_snapshot(snapshot.pid);
    g_loaded_config_path = snapshot.loaded_path;
}

void collect_restart_required_keys(const ConfigSnapshot &old_config,
                                   std::vector<std::string> *restart_required_keys)
{
    if (restart_required_keys == nullptr)
    {
        return;
    }
    restart_required_keys->clear();

    const auto push_if = [&](bool changed, const char *key) {
        if (changed)
        {
            restart_required_keys->emplace_back(key);
        }
    };

    push_if(old_config.vision_runtime.screen_display_enabled != g_vision_runtime_config.screen_display_enabled,
            "vision.runtime.screen_display_enabled");
    push_if(old_config.vision_runtime.udp_web_video_port != g_vision_runtime_config.udp_web_video_port,
            "vision.runtime.web.video_port");
    push_if(old_config.vision_runtime.udp_web_meta_port != g_vision_runtime_config.udp_web_meta_port,
            "vision.runtime.web.meta_port");
    push_if(std::string(old_config.vision_runtime.udp_web_server_ip ? old_config.vision_runtime.udp_web_server_ip : "") !=
                std::string(g_vision_runtime_config.udp_web_server_ip ? g_vision_runtime_config.udp_web_server_ip : ""),
            "vision.runtime.web.server_ip");
    push_if(old_config.vision_runtime.assistant_udp_enabled != g_vision_runtime_config.assistant_udp_enabled,
            "vision.runtime.assistant.enabled");
    push_if(old_config.vision_runtime.assistant_server_port != g_vision_runtime_config.assistant_server_port,
            "vision.runtime.assistant.server_port");
    push_if(std::string(old_config.vision_runtime.assistant_server_ip ? old_config.vision_runtime.assistant_server_ip : "") !=
                std::string(g_vision_runtime_config.assistant_server_ip ? g_vision_runtime_config.assistant_server_ip : ""),
            "vision.runtime.assistant.server_ip");
    push_if(old_config.vision_runtime.ncnn_input_width != g_vision_runtime_config.ncnn_input_width,
            "vision.runtime.ncnn.input_width");
    push_if(old_config.vision_runtime.ncnn_input_height != g_vision_runtime_config.ncnn_input_height,
            "vision.runtime.ncnn.input_height");
    push_if(old_config.vision_runtime.ncnn_label_count != g_vision_runtime_config.ncnn_label_count,
            "vision.runtime.ncnn.label_count");
    for (size_t i = 0; i < VISION_NCNN_CONFIG_MAX_LABELS; ++i)
    {
        const std::string old_label = old_config.vision_runtime.ncnn_labels[i] ? old_config.vision_runtime.ncnn_labels[i] : "";
        const std::string new_label = g_vision_runtime_config.ncnn_labels[i] ? g_vision_runtime_config.ncnn_labels[i] : "";
        if (old_label != new_label)
        {
            restart_required_keys->emplace_back("vision.runtime.ncnn.labels");
            break;
        }
    }
}

void apply_runtime_changes_after_commit()
{
    vision_transport_udp_set_enabled(g_vision_runtime_config.udp_web_enabled);
    vision_transport_udp_set_max_fps(g_vision_runtime_config.udp_web_max_fps);
    vision_transport_udp_set_tcp_enabled(g_vision_runtime_config.udp_web_tcp_enabled);

    vision_thread_set_send_mode(static_cast<vision_thread_send_mode_enum>(g_vision_runtime_config.send_mode));
    vision_thread_set_send_max_fps(g_vision_runtime_config.send_max_fps);
    vision_thread_set_infer_enabled(g_vision_runtime_config.infer_enabled);
    vision_thread_set_ncnn_enabled(g_vision_runtime_config.ncnn_enabled);
    vision_thread_set_client_sender_enabled(g_vision_runtime_config.client_sender_enabled);

    vision_image_processor_reload_config_from_globals();
    line_follow_thread_set_normal_speed_reference(pid_tuning::route_line_follow::kNormalProfile.base_speed);
    line_follow_thread_request_reload_from_globals();
    motor_thread_request_reload_from_globals();
}

bool load_from_path(const std::string &path, std::string *error_message)
{
    RawMap values;
    if (!parse_key_values(path, &values, error_message))
    {
        return false;
    }

    std::set<std::string> consumed;

    int schema_version = 0;
    if (!require_int(values, &consumed, "meta.schema_version", &schema_version, error_message))
    {
        return false;
    }
    if (schema_version != 1)
    {
        *error_message = "unsupported schema_version: " + std::to_string(schema_version);
        return false;
    }

    int send_max_fps = 0;
    int udp_web_max_fps = 0;
    int udp_web_video_port = 0;
    int udp_web_meta_port = 0;
    int assistant_server_port = 0;
    size_t ncnn_label_count = 0;
    if (!require_int(values, &consumed, "vision.runtime.send_mode", &g_vision_runtime_config.send_mode, error_message) ||
        !require_int(values, &consumed, "vision.runtime.send_max_fps", &send_max_fps, error_message) ||
        !require_bool(values, &consumed, "vision.runtime.infer_enabled", &g_vision_runtime_config.infer_enabled, error_message) ||
        !require_bool(values, &consumed, "vision.runtime.ncnn_enabled", &g_vision_runtime_config.ncnn_enabled, error_message) ||
        !require_int(values, &consumed, "vision.runtime.ncnn.input_width", &g_vision_runtime_config.ncnn_input_width, error_message) ||
        !require_int(values, &consumed, "vision.runtime.ncnn.input_height", &g_vision_runtime_config.ncnn_input_height, error_message) ||
        !require_size_t(values, &consumed, "vision.runtime.ncnn.label_count", &ncnn_label_count, error_message) ||
        !require_string_array(values, &consumed, "vision.runtime.ncnn.labels", &g_string_storage.ncnn_labels, g_vision_runtime_config.ncnn_labels, &g_vision_runtime_config.ncnn_label_count, error_message) ||
        !require_bool(values, &consumed, "vision.runtime.client_sender_enabled", &g_vision_runtime_config.client_sender_enabled, error_message) ||
        !require_bool(values, &consumed, "vision.runtime.screen_display_enabled", &g_vision_runtime_config.screen_display_enabled, error_message))
    {
        return false;
    }
    g_vision_runtime_config.send_max_fps = static_cast<uint32>(std::max(send_max_fps, 0));
    if (ncnn_label_count != g_vision_runtime_config.ncnn_label_count)
    {
        *error_message = "vision.runtime.ncnn.label_count does not match labels array length";
        return false;
    }

    if (!require_bool(values, &consumed, "vision.runtime.web.enabled", &g_vision_runtime_config.udp_web_enabled, error_message) ||
        !require_int(values, &consumed, "vision.runtime.web.max_fps", &udp_web_max_fps, error_message) ||
        !require_bool(values, &consumed, "vision.runtime.web.send_gray_jpeg", &g_vision_runtime_config.udp_web_send_gray_jpeg, error_message) ||
        !require_int(values, &consumed, "vision.runtime.web.gray_image_format", &g_vision_runtime_config.udp_web_gray_image_format, error_message) ||
        !require_bool(values, &consumed, "vision.runtime.web.send_binary_jpeg", &g_vision_runtime_config.udp_web_send_binary_jpeg, error_message) ||
        !require_int(values, &consumed, "vision.runtime.web.binary_image_format", &g_vision_runtime_config.udp_web_binary_image_format, error_message) ||
        !require_bool(values, &consumed, "vision.runtime.web.send_rgb_jpeg", &g_vision_runtime_config.udp_web_send_rgb_jpeg, error_message) ||
        !require_int(values, &consumed, "vision.runtime.web.rgb_image_format", &g_vision_runtime_config.udp_web_rgb_image_format, error_message) ||
        !require_int(values, &consumed, "vision.runtime.web.data_profile", &g_vision_runtime_config.udp_web_data_profile, error_message) ||
        !require_bool(values, &consumed, "vision.runtime.web.tcp_enabled", &g_vision_runtime_config.udp_web_tcp_enabled, error_message))
    {
        return false;
    }
    g_vision_runtime_config.udp_web_max_fps = static_cast<uint32>(std::max(udp_web_max_fps, 0));

#define REQUIRE_WEB_TCP_BOOL(name) \
    if (!require_bool(values, &consumed, "vision.runtime.web.tcp." #name, &g_vision_runtime_config.udp_web_tcp_send_##name, error_message)) return false

    REQUIRE_WEB_TCP_BOOL(ts_ms);
    REQUIRE_WEB_TCP_BOOL(line_error);
    REQUIRE_WEB_TCP_BOOL(cpu_usage_percent);
    REQUIRE_WEB_TCP_BOOL(mem_usage_percent);
    REQUIRE_WEB_TCP_BOOL(base_speed);
    REQUIRE_WEB_TCP_BOOL(left_target_count);
    REQUIRE_WEB_TCP_BOOL(right_target_count);
    REQUIRE_WEB_TCP_BOOL(left_current_count);
    REQUIRE_WEB_TCP_BOOL(right_current_count);
    REQUIRE_WEB_TCP_BOOL(left_filtered_count);
    REQUIRE_WEB_TCP_BOOL(right_filtered_count);
    REQUIRE_WEB_TCP_BOOL(left_error);
    REQUIRE_WEB_TCP_BOOL(right_error);
    REQUIRE_WEB_TCP_BOOL(left_feedforward);
    REQUIRE_WEB_TCP_BOOL(right_feedforward);
    REQUIRE_WEB_TCP_BOOL(left_correction);
    REQUIRE_WEB_TCP_BOOL(right_correction);
    REQUIRE_WEB_TCP_BOOL(left_decel_assist);
    REQUIRE_WEB_TCP_BOOL(right_decel_assist);
    REQUIRE_WEB_TCP_BOOL(left_duty);
    REQUIRE_WEB_TCP_BOOL(right_duty);
    REQUIRE_WEB_TCP_BOOL(left_hardware_duty);
    REQUIRE_WEB_TCP_BOOL(right_hardware_duty);
    REQUIRE_WEB_TCP_BOOL(left_dir_level);
    REQUIRE_WEB_TCP_BOOL(right_dir_level);
    REQUIRE_WEB_TCP_BOOL(otsu_threshold);
    REQUIRE_WEB_TCP_BOOL(perf_capture_wait_us);
    REQUIRE_WEB_TCP_BOOL(perf_preprocess_us);
    REQUIRE_WEB_TCP_BOOL(perf_otsu_us);
    REQUIRE_WEB_TCP_BOOL(perf_maze_us);
    REQUIRE_WEB_TCP_BOOL(perf_total_us);
    REQUIRE_WEB_TCP_BOOL(maze_left_points_raw);
    REQUIRE_WEB_TCP_BOOL(maze_right_points_raw);
    REQUIRE_WEB_TCP_BOOL(red_found);
    REQUIRE_WEB_TCP_BOOL(red_rect);
    REQUIRE_WEB_TCP_BOOL(roi_valid);
    REQUIRE_WEB_TCP_BOOL(roi_rect);
    REQUIRE_WEB_TCP_BOOL(ipm_track_valid);
    REQUIRE_WEB_TCP_BOOL(ipm_track_method);
    REQUIRE_WEB_TCP_BOOL(ipm_centerline_source);
    REQUIRE_WEB_TCP_BOOL(ipm_track_index);
    REQUIRE_WEB_TCP_BOOL(ipm_track_point);
    REQUIRE_WEB_TCP_BOOL(left_boundary);
    REQUIRE_WEB_TCP_BOOL(right_boundary);
    REQUIRE_WEB_TCP_BOOL(ipm_left_boundary);
    REQUIRE_WEB_TCP_BOOL(ipm_right_boundary);
    REQUIRE_WEB_TCP_BOOL(ipm_centerline_selected_shift);
    REQUIRE_WEB_TCP_BOOL(src_centerline_selected_shift);
    REQUIRE_WEB_TCP_BOOL(ipm_centerline_selected_count);
    REQUIRE_WEB_TCP_BOOL(src_centerline_selected_count);
    REQUIRE_WEB_TCP_BOOL(ipm_centerline_selected_curvature);
    REQUIRE_WEB_TCP_BOOL(gray_size);
    REQUIRE_WEB_TCP_BOOL(ipm_size);
#undef REQUIRE_WEB_TCP_BOOL

    if (!require_string(values, &consumed, "vision.runtime.web.server_ip", &g_string_storage.udp_web_server_ip, &g_vision_runtime_config.udp_web_server_ip, error_message) ||
        !require_int(values, &consumed, "vision.runtime.web.video_port", &udp_web_video_port, error_message) ||
        !require_int(values, &consumed, "vision.runtime.web.meta_port", &udp_web_meta_port, error_message) ||
        !require_bool(values, &consumed, "vision.runtime.assistant.enabled", &g_vision_runtime_config.assistant_udp_enabled, error_message) ||
        !require_string(values, &consumed, "vision.runtime.assistant.server_ip", &g_string_storage.assistant_server_ip, &g_vision_runtime_config.assistant_server_ip, error_message) ||
        !require_int(values, &consumed, "vision.runtime.assistant.server_port", &assistant_server_port, error_message))
    {
        return false;
    }
    g_vision_runtime_config.udp_web_video_port = static_cast<uint16>(std::max(udp_web_video_port, 0));
    g_vision_runtime_config.udp_web_meta_port = static_cast<uint16>(std::max(udp_web_meta_port, 0));
    g_vision_runtime_config.assistant_server_port = static_cast<uint16>(std::max(assistant_server_port, 0));

#define REQUIRE_RUNTIME_INT(name) \
    if (!require_int(values, &consumed, "vision.runtime." #name, &g_vision_runtime_config.name, error_message)) return false
#define REQUIRE_RUNTIME_BOOL(name) \
    if (!require_bool(values, &consumed, "vision.runtime." #name, &g_vision_runtime_config.name, error_message)) return false
#define REQUIRE_RUNTIME_FLOAT(name) \
    if (!require_float(values, &consumed, "vision.runtime." #name, &g_vision_runtime_config.name, error_message)) return false
#define REQUIRE_RUNTIME_SIZE_T(name) \
    if (!require_size_t(values, &consumed, "vision.runtime." #name, &g_vision_runtime_config.name, error_message)) return false

    REQUIRE_RUNTIME_INT(maze_start_row);
    REQUIRE_RUNTIME_INT(maze_trace_method);
    REQUIRE_RUNTIME_INT(maze_trace_y_fallback_stop_delta);
    REQUIRE_RUNTIME_BOOL(cross_lower_corner_dir_enabled);
    REQUIRE_RUNTIME_INT(cross_lower_corner_pre_window);
    REQUIRE_RUNTIME_INT(cross_lower_corner_post_window);
    REQUIRE_RUNTIME_INT(cross_lower_corner_pre_min_votes);
    REQUIRE_RUNTIME_INT(cross_lower_corner_post_min_votes);
    REQUIRE_RUNTIME_INT(cross_lower_corner_transition_max_len);
    REQUIRE_RUNTIME_INT(cross_lower_corner_pair_y_diff_max);
    REQUIRE_RUNTIME_BOOL(cross_lower_corner_extrapolate_enabled);
    REQUIRE_RUNTIME_INT(cross_lower_corner_extrapolate_min_y);
    REQUIRE_RUNTIME_INT(cross_lower_corner_extrapolate_y_span);
    REQUIRE_RUNTIME_INT(src_boundary_straight_check_count);
    REQUIRE_RUNTIME_FLOAT(src_boundary_straight_dir45_ratio_min);
    REQUIRE_RUNTIME_BOOL(undistort_enabled);
    REQUIRE_RUNTIME_BOOL(ipm_triangle_filter_enabled);
    REQUIRE_RUNTIME_BOOL(ipm_resample_enabled);
    REQUIRE_RUNTIME_FLOAT(ipm_resample_step_px);
    REQUIRE_RUNTIME_FLOAT(ipm_boundary_min_point_dist_px);
    REQUIRE_RUNTIME_FLOAT(ipm_boundary_spike_short_seg_max_px);
    REQUIRE_RUNTIME_FLOAT(ipm_boundary_spike_reverse_cos_threshold);
    REQUIRE_RUNTIME_INT(ipm_boundary_angle_step);
    REQUIRE_RUNTIME_FLOAT(ipm_boundary_corner_cos_threshold);
    REQUIRE_RUNTIME_INT(ipm_boundary_corner_nms_radius);
    REQUIRE_RUNTIME_BOOL(ipm_boundary_truncate_at_first_corner_enabled);
    REQUIRE_RUNTIME_INT(ipm_boundary_straight_min_points);
    REQUIRE_RUNTIME_INT(ipm_boundary_straight_check_count);
    REQUIRE_RUNTIME_FLOAT(ipm_boundary_straight_min_cos);
    REQUIRE_RUNTIME_FLOAT(ipm_track_width_px);
    REQUIRE_RUNTIME_FLOAT(ipm_center_target_offset_from_left_px);
    REQUIRE_RUNTIME_FLOAT(ipm_center_target_offset_weapons_from_left_px);
    REQUIRE_RUNTIME_FLOAT(ipm_center_target_offset_supplies_from_left_px);
    REQUIRE_RUNTIME_INT(infer_offset_vote_result_count);
    REQUIRE_RUNTIME_INT(infer_offset_restore_no_red_count);
    REQUIRE_RUNTIME_BOOL(ipm_centerline_postprocess_enabled);
    REQUIRE_RUNTIME_BOOL(ipm_centerline_triangle_filter_enabled);
    REQUIRE_RUNTIME_BOOL(ipm_centerline_resample_enabled);
    REQUIRE_RUNTIME_FLOAT(ipm_centerline_resample_step_px);
    REQUIRE_RUNTIME_BOOL(ipm_centerline_curvature_enabled);
    REQUIRE_RUNTIME_INT(ipm_centerline_curvature_step);
    REQUIRE_RUNTIME_BOOL(zebra_cross_detection_enabled);
    REQUIRE_RUNTIME_BOOL(keep_last_centerline_on_double_loss);
    REQUIRE_RUNTIME_BOOL(route_cross_detection_enabled);
    REQUIRE_RUNTIME_INT(route_cross_entry_corner_post_frame_wall_rows_min);
    REQUIRE_RUNTIME_INT(route_cross_stage2_enter_start_frame_wall_rows_min);
    REQUIRE_RUNTIME_INT(route_cross_stage1_enter_corner_y_min);
    REQUIRE_RUNTIME_INT(route_cross_exit_start_gap_x_max);
    REQUIRE_RUNTIME_INT(route_cross_stage3_jump_x_threshold_px);
    REQUIRE_RUNTIME_INT(route_cross_stage3_cut_forward_points);
    REQUIRE_RUNTIME_INT(route_cross_stage3_left_anchor_x);
    REQUIRE_RUNTIME_INT(route_cross_stage3_left_anchor_y);
    REQUIRE_RUNTIME_INT(route_cross_stage3_right_anchor_x);
    REQUIRE_RUNTIME_INT(route_cross_stage3_right_anchor_y);
    REQUIRE_RUNTIME_BOOL(route_circle_detection_enabled);
    REQUIRE_RUNTIME_INT(route_circle_entry_min_boundary_count);
    REQUIRE_RUNTIME_INT(route_circle_entry_corner_tail_margin);
    REQUIRE_RUNTIME_INT(route_circle_entry_corner_row_offset);
    REQUIRE_RUNTIME_INT(route_circle_entry_gap_check_rows);
    REQUIRE_RUNTIME_INT(route_circle_entry_min_raw_boundary_gap);
    REQUIRE_RUNTIME_INT(route_circle_entry_corner_y_min);
    REQUIRE_RUNTIME_INT(route_circle_stage_frame_wall_rows_enter);
    REQUIRE_RUNTIME_INT(route_circle_stage3_frame_wall_rows_trigger);
    REQUIRE_RUNTIME_INT(route_circle_stage6_maze_start_row);
    REQUIRE_RUNTIME_FLOAT(route_circle_center_target_offset_from_left_px);
    REQUIRE_RUNTIME_INT(circle_guide_min_frame_wall_segment_len);
    REQUIRE_RUNTIME_INT(circle_guide_target_offset_stage3);
    REQUIRE_RUNTIME_INT(circle_guide_anchor_offset_stage5);
    REQUIRE_RUNTIME_INT(route_circle_apply_touch_margin_px);
    REQUIRE_RUNTIME_INT(route_straight_min_centerline_points);
    REQUIRE_RUNTIME_INT(route_straight_enter_consecutive_frames);
    REQUIRE_RUNTIME_FLOAT(route_straight_abs_error_sum_max);
    REQUIRE_RUNTIME_INT(cross_aux_vertical_scan_max_rows);
    REQUIRE_RUNTIME_INT(cross_aux_trace_max_points);
    REQUIRE_RUNTIME_INT(cross_aux_trace_upward_rows_max);
    REQUIRE_RUNTIME_INT(cross_upper_dir4_pre_run_len);
    REQUIRE_RUNTIME_INT(cross_upper_transition_max_len);
    REQUIRE_RUNTIME_INT(cross_upper_dir6_post_run_len);
    REQUIRE_RUNTIME_INT(ipm_line_error_source);
    REQUIRE_RUNTIME_INT(ipm_line_error_method);

    // 兼容旧版本配置：以下键已弃用，若存在则仅消费避免触发 unknown-keys。
    consume_optional_key_if_present(values, &consumed, "vision.runtime.ipm_line_error_fixed_index");
    consume_optional_key_if_present(values, &consumed, "vision.runtime.ipm_line_error_weighted_point_count");
    consume_optional_key_if_present(values, &consumed, "vision.runtime.ipm_line_error_point_indices");
    consume_optional_key_if_present(values, &consumed, "vision.runtime.ipm_line_error_weights");
    consume_optional_key_if_present(values, &consumed, "vision.runtime.ipm_line_error_speed_k");
    consume_optional_key_if_present(values, &consumed, "vision.runtime.ipm_line_error_speed_b");
    consume_optional_key_if_present(values, &consumed, "vision.runtime.ipm_line_error_index_min");
    consume_optional_key_if_present(values, &consumed, "vision.runtime.ipm_line_error_index_max");
    consume_optional_key_if_present(values, &consumed, "vision.runtime.ipm_line_error_prefix_ratio");
    consume_optional_key_if_present(values, &consumed, "vision.runtime.ipm_line_error_exp_lambda");
    consume_optional_key_if_present(values, &consumed, "vision.runtime.ipm_line_error_linear_base_b");

#undef REQUIRE_RUNTIME_INT
#undef REQUIRE_RUNTIME_BOOL
#undef REQUIRE_RUNTIME_FLOAT
#undef REQUIRE_RUNTIME_SIZE_T

    if (!require_int(values, &consumed, "vision.processor.maze_trace_max_points", &g_vision_processor_config.maze_trace_max_points, error_message) ||
        !require_int(values, &consumed, "vision.processor.maze_lower_region_percent", &g_vision_processor_config.maze_lower_region_percent, error_message) ||
        !require_bool(values, &consumed, "vision.processor.demand_otsu_enable", &g_vision_processor_config.demand_otsu_enable, error_message) ||
        !require_bool(values, &consumed, "vision.processor.demand_otsu_keep_full_binary_cache", &g_vision_processor_config.demand_otsu_keep_full_binary_cache, error_message) ||
        !require_bool(values, &consumed, "vision.processor.enable_inverse_perspective", &g_vision_processor_config.enable_inverse_perspective, error_message) ||
        !require_int(values, &consumed, "vision.processor.ipm_output_width", &g_vision_processor_config.ipm_output_width, error_message) ||
        !require_int(values, &consumed, "vision.processor.ipm_output_height", &g_vision_processor_config.ipm_output_height, error_message))
    {
        return false;
    }

    double change_un_mat_flat[9] = {};
    double camera_matrix_flat[9] = {};
    if (!require_double_array(values, &consumed, "vision.processor.change_un_mat", change_un_mat_flat, error_message) ||
        !require_double_array(values, &consumed, "vision.processor.camera_matrix", camera_matrix_flat, error_message) ||
        !require_double_array(values, &consumed, "vision.processor.dist_coeffs", g_vision_processor_config.dist_coeffs, error_message) ||
        !require_int(values, &consumed, "vision.processor.undistort_move_x", &g_vision_processor_config.undistort_move_x, error_message) ||
        !require_int(values, &consumed, "vision.processor.undistort_move_y", &g_vision_processor_config.undistort_move_y, error_message) ||
        !require_float(values, &consumed, "vision.processor.default_line_sample_ratio", &g_vision_processor_config.default_line_sample_ratio, error_message) ||
        !require_int(values, &consumed, "vision.processor.default_maze_trace_x_min", &g_vision_processor_config.default_maze_trace_x_min, error_message) ||
        !require_int(values, &consumed, "vision.processor.default_maze_trace_x_max", &g_vision_processor_config.default_maze_trace_x_max, error_message))
    {
        return false;
    }
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            g_vision_processor_config.change_un_mat[row][col] = change_un_mat_flat[row * 3 + col];
            g_vision_processor_config.camera_matrix[row][col] = camera_matrix_flat[row * 3 + col];
        }
    }

    if (!require_int(values, &consumed, "pid.imu.startup_calibrate_duration_ms", &pid_tuning::imu::kStartupCalibrateDurationMs, error_message) ||
        !require_float(values, &consumed, "pid.imu.gyro_yaw_rate_sign", &pid_tuning::imu::kGyroYawRateSign, error_message) ||
        !require_float(values, &consumed, "pid.imu.gyro_yaw_rate_filter_alpha", &pid_tuning::imu::kGyroYawRateFilterAlpha, error_message))
    {
        return false;
    }

#define REQUIRE_PID_FLOAT(path, target) \
    if (!require_float(values, &consumed, path, &target, error_message)) return false
#define REQUIRE_PID_INT(path, target) \
    if (!require_int(values, &consumed, path, &target, error_message)) return false

    REQUIRE_PID_FLOAT("pid.motor_speed.left_kp", pid_tuning::motor_speed::kLeftKp);
    REQUIRE_PID_FLOAT("pid.motor_speed.left_ki", pid_tuning::motor_speed::kLeftKi);
    REQUIRE_PID_FLOAT("pid.motor_speed.left_kd", pid_tuning::motor_speed::kLeftKd);
    REQUIRE_PID_FLOAT("pid.motor_speed.right_kp", pid_tuning::motor_speed::kRightKp);
    REQUIRE_PID_FLOAT("pid.motor_speed.right_ki", pid_tuning::motor_speed::kRightKi);
    REQUIRE_PID_FLOAT("pid.motor_speed.right_kd", pid_tuning::motor_speed::kRightKd);
    REQUIRE_PID_FLOAT("pid.motor_speed.integral_limit", pid_tuning::motor_speed::kIntegralLimit);
    REQUIRE_PID_FLOAT("pid.motor_speed.max_output_step", pid_tuning::motor_speed::kMaxOutputStep);
    REQUIRE_PID_FLOAT("pid.motor_speed.correction_limit", pid_tuning::motor_speed::kCorrectionLimit);
    REQUIRE_PID_FLOAT("pid.motor_speed.left_feedforward_gain", pid_tuning::motor_speed::kLeftFeedforwardGain);
    REQUIRE_PID_FLOAT("pid.motor_speed.right_feedforward_gain", pid_tuning::motor_speed::kRightFeedforwardGain);
    REQUIRE_PID_FLOAT("pid.motor_speed.left_feedforward_bias", pid_tuning::motor_speed::kLeftFeedforwardBias);
    REQUIRE_PID_FLOAT("pid.motor_speed.right_feedforward_bias", pid_tuning::motor_speed::kRightFeedforwardBias);
    REQUIRE_PID_FLOAT("pid.motor_speed.feedforward_bias_threshold", pid_tuning::motor_speed::kFeedforwardBiasThreshold);
    REQUIRE_PID_FLOAT("pid.motor_speed.decel_error_threshold", pid_tuning::motor_speed::kDecelErrorThreshold);
    REQUIRE_PID_FLOAT("pid.motor_speed.decel_duty_gain", pid_tuning::motor_speed::kDecelDutyGain);
    REQUIRE_PID_FLOAT("pid.motor_speed.decel_duty_limit", pid_tuning::motor_speed::kDecelDutyLimit);
    REQUIRE_PID_INT("pid.motor_speed.feedback_average_window", pid_tuning::motor_speed::kFeedbackAverageWindow);
    REQUIRE_PID_FLOAT("pid.motor_speed.feedback_low_pass_alpha", pid_tuning::motor_speed::kFeedbackLowPassAlpha);
    if (!require_bool(values, &consumed, "pid.brushless.realtime_enabled", &pid_tuning::brushless::kRealtimeEnabled, error_message))
    {
        return false;
    }
    REQUIRE_PID_FLOAT("pid.brushless.left_duty_percent", pid_tuning::brushless::kLeftDutyPercent);
    REQUIRE_PID_FLOAT("pid.brushless.right_duty_percent", pid_tuning::brushless::kRightDutyPercent);
    REQUIRE_PID_FLOAT("pid.yaw_rate_loop.visual_curvature_filter_alpha", pid_tuning::yaw_rate_loop::kVisualCurvatureFilterAlpha);
    REQUIRE_PID_FLOAT("pid.yaw_rate_loop.track_point_angle_filter_alpha", pid_tuning::yaw_rate_loop::kTrackPointAngleFilterAlpha);
    REQUIRE_PID_FLOAT("pid.line_follow.error_filter_alpha", pid_tuning::line_follow::kErrorFilterAlpha);
    REQUIRE_PID_FLOAT("pid.line_follow.target_count_min", pid_tuning::line_follow::kTargetCountMin);
    REQUIRE_PID_FLOAT("pid.line_follow.target_count_max", pid_tuning::line_follow::kTargetCountMax);
    REQUIRE_PID_FLOAT("pid.line_follow.error_deadzone_px", pid_tuning::line_follow::kErrorDeadzonePx);
    REQUIRE_PID_FLOAT("pid.line_follow.error_low_gain_limit_px", pid_tuning::line_follow::kErrorLowGainLimitPx);
    REQUIRE_PID_FLOAT("pid.line_follow.error_low_gain", pid_tuning::line_follow::kErrorLowGain);
    REQUIRE_PID_FLOAT("pid.route_line_follow.global.base_speed_scale", pid_tuning::route_line_follow::kGlobalBaseSpeedScale);

#undef REQUIRE_PID_FLOAT
#undef REQUIRE_PID_INT

    if (!load_route_profile(values, &consumed, "pid.route_line_follow.normal", &pid_tuning::route_line_follow::kNormalProfile, error_message) ||
        !load_route_profile(values, &consumed, "pid.route_line_follow.straight", &pid_tuning::route_line_follow::kStraightProfile, error_message) ||
        !load_route_profile(values, &consumed, "pid.route_line_follow.cross", &pid_tuning::route_line_follow::kCrossProfile, error_message) ||
        !load_route_profile(values, &consumed, "pid.route_line_follow.circle_enter", &pid_tuning::route_line_follow::kCircleEnterProfile, error_message) ||
        !load_route_profile(values, &consumed, "pid.route_line_follow.circle_inside", &pid_tuning::route_line_follow::kCircleInsideProfile, error_message) ||
        !load_route_profile(values, &consumed, "pid.route_line_follow.circle_exit", &pid_tuning::route_line_follow::kCircleExitProfile, error_message))
    {
        return false;
    }

    if (g_vision_runtime_config.ncnn_label_count > VISION_NCNN_CONFIG_MAX_LABELS)
    {
        *error_message = "vision.runtime.ncnn.label_count exceeds max";
        return false;
    }

    const auto route_valid = [](const pid_tuning::route_line_follow::Profile &profile) {
        return pid_tuning::route_line_follow::is_dynamic_kp_range_valid(profile) &&
               pid_tuning::route_line_follow::is_dynamic_position_kd_range_valid(profile) &&
               pid_tuning::route_line_follow::is_position_kp_piecewise_range_valid(profile) &&
               pid_tuning::route_line_follow::is_line_error_prefix_exp_valid(profile) &&
               pid_tuning::route_line_follow::is_speed_scheme_range_valid(profile);
    };
    if (!route_valid(pid_tuning::route_line_follow::kNormalProfile) ||
        !route_valid(pid_tuning::route_line_follow::kStraightProfile) ||
        !route_valid(pid_tuning::route_line_follow::kCrossProfile) ||
        !route_valid(pid_tuning::route_line_follow::kCircleEnterProfile) ||
        !route_valid(pid_tuning::route_line_follow::kCircleInsideProfile) ||
        !route_valid(pid_tuning::route_line_follow::kCircleExitProfile))
    {
        *error_message = "pid.route_line_follow contains invalid profile";
        return false;
    }
    if (g_vision_runtime_config.route_straight_min_centerline_points < 1)
    {
        *error_message = "vision.runtime.route_straight_min_centerline_points must be >= 1";
        return false;
    }
    if (pid_tuning::brushless::kLeftDutyPercent < 0.0f || pid_tuning::brushless::kLeftDutyPercent > 100.0f ||
        pid_tuning::brushless::kRightDutyPercent < 0.0f || pid_tuning::brushless::kRightDutyPercent > 100.0f)
    {
        *error_message = "pid.brushless duty_percent must be in [0, 100]";
        return false;
    }

    if (consumed.size() != values.size())
    {
        std::ostringstream oss;
        bool first = true;
        for (const auto &entry : values)
        {
            if (consumed.find(entry.first) != consumed.end())
            {
                continue;
            }
            if (!first)
            {
                oss << ", ";
            }
            oss << entry.first;
            first = false;
        }
        *error_message = "unknown keys in config: " + oss.str();
        return false;
    }

    return true;
}

} // namespace

bool smartcar_config_load_from_default_locations(std::string *loaded_path, std::string *error_message)
{
    std::lock_guard<std::mutex> lock(g_config_mutex);
    const std::string target_path = executable_dir_config_path();
    if (target_path.empty())
    {
        if (error_message != nullptr)
        {
            *error_message = "cannot resolve config path from /proc/self/exe";
        }
        return false;
    }

    if (!load_from_path(target_path, error_message))
    {
        return false;
    }

    g_loaded_config_path = target_path;
    if (loaded_path != nullptr)
    {
        *loaded_path = target_path;
    }
    return true;
}

bool smartcar_config_apply_toml_text(const std::string &toml_text,
                                     std::vector<std::string> *restart_required_keys,
                                     std::string *error_message)
{
    std::lock_guard<std::mutex> lock(g_config_mutex);

    const ConfigSnapshot old_config = capture_config_snapshot();
    const std::string target_path = g_loaded_config_path.empty()
                                        ? executable_dir_config_path()
                                        : g_loaded_config_path;
    if (target_path.empty())
    {
        if (error_message != nullptr)
        {
            *error_message = "cannot resolve config path from /proc/self/exe";
        }
        return false;
    }
    const std::string validate_path = target_path + ".apply_validate";

    {
        std::ofstream output(validate_path, std::ios::binary | std::ios::trunc);
        if (!output.is_open())
        {
            if (error_message != nullptr)
            {
                *error_message = "cannot open validation file: " + validate_path;
            }
            return false;
        }
        output << toml_text;
        if (!output.good())
        {
            if (error_message != nullptr)
            {
                *error_message = "failed to write validation file: " + validate_path;
            }
            return false;
        }
    }

    if (!load_from_path(validate_path, error_message))
    {
        restore_config_snapshot(old_config);
        std::remove(validate_path.c_str());
        return false;
    }

    collect_restart_required_keys(old_config, restart_required_keys);

    if (std::rename(validate_path.c_str(), target_path.c_str()) != 0)
    {
        restore_config_snapshot(old_config);
        std::remove(validate_path.c_str());
        if (error_message != nullptr)
        {
            *error_message = "failed to replace config file";
        }
        return false;
    }

    g_loaded_config_path = target_path;
    apply_runtime_changes_after_commit();
    return true;
}

bool smartcar_config_read_loaded_text(std::string *toml_text,
                                      std::string *loaded_path,
                                      std::string *error_message)
{
    std::lock_guard<std::mutex> lock(g_config_mutex);
    const std::string path = g_loaded_config_path.empty()
                                 ? executable_dir_config_path()
                                 : g_loaded_config_path;
    if (path.empty())
    {
        if (error_message != nullptr)
        {
            *error_message = "cannot resolve config path from /proc/self/exe";
        }
        return false;
    }
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open())
    {
        if (error_message != nullptr)
        {
            *error_message = "cannot open config file: " + path;
        }
        return false;
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    if (toml_text != nullptr)
    {
        *toml_text = buffer.str();
    }
    if (loaded_path != nullptr)
    {
        *loaded_path = path;
    }
    return true;
}

std::string smartcar_config_loaded_path()
{
    std::lock_guard<std::mutex> lock(g_config_mutex);
    return g_loaded_config_path;
}

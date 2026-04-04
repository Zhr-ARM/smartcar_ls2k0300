#include "imu660ra.h"

#include <stdlib.h>

#define IMU660RA_RAD_TO_DEG (57.2957795f)

/**
 * @brief 对浮点数进行区间限幅
 * @param value 输入值
 * @param lower 下限
 * @param upper 上限
 * @return 限幅后的数值
 */
static float clamp_float(float value, float lower, float upper)
{
    if (value < lower)
    {
        return lower;
    }
    if (value > upper)
    {
        return upper;
    }
    return value;
}

Imu660raDriver::Imu660raDriver()
    : online_(false),
      filter_ready_(false),
      filter_alpha_(IMU660RA_DEFAULT_FILTER_ALPHA),
      gyro_scale_(0.0f)
{
    clear();
    set_error("IMU660RA 未初始化");
}

bool Imu660raDriver::init()
{
    clear();
    imu_get_dev_info();

    if (DEV_IMU660RA != imu_type)
    {
        online_ = false;

        if (DEV_IMU660RB == imu_type)
        {
            set_error("检测到 IMU660RB,当前库仅支持 IMU660RA");
        }
        else if (DEV_IMU963RA == imu_type)
        {
            set_error("检测到 IMU963RA,当前库仅支持 IMU660RA");
        }
        else if ('\0' != imu_dev_name[0])
        {
            char message[sizeof(last_error_)] = {0};
            snprintf(message, sizeof(message), "检测到未适配的 IMU 设备: %s", imu_dev_name);
            set_error(message);
        }
        else
        {
            set_error("未检测到 IMU660RA");
        }
        return false;
    }

    online_ = true;
    if (!reload_scale())
    {
        online_ = false;
        return false;
    }

    set_error("");
    return update();
}

bool Imu660raDriver::update(float dt_s)
{
    (void)dt_s;

    if (!online_)
    {
        return false;
    }

    if (!update_gyro())
    {
        return false;
    }

    convert_raw_gyro_data();
    apply_gyro_lowpass_filter();
    return true;
}

bool Imu660raDriver::update_gyro()
{
    if (!online_)
    {
        return false;
    }

    imu660ra_get_gyro();
    data_.gyro_x = imu660ra_gyro_x;
    data_.gyro_y = imu660ra_gyro_y;
    data_.gyro_z = imu660ra_gyro_z;
    return true;
}

bool Imu660raDriver::reload_scale()
{
    gyro_scale_ = IMU660RA_FALLBACK_GYRO_SCALE_RAD_S;

    char scale_path[IMU_SYSFS_PATH_MAX_LEN] = {0};
    float value = 0.0f;
    if (0 == imu_get_node_path(IMU660RA_GYRO_SCALE_NODE, scale_path, sizeof(scale_path)) &&
        read_scale(scale_path, value))
    {
        gyro_scale_ = value;
    }

    return true;
}

void Imu660raDriver::clear()
{
    filter_ready_ = false;
    memset(&data_, 0, sizeof(data_));
    memset(&gyro_rad_s_, 0, sizeof(gyro_rad_s_));
    memset(&filtered_gyro_rad_s_, 0, sizeof(filtered_gyro_rad_s_));
}

float Imu660raDriver::gyro_scale_rad_s() const
{
    return gyro_scale_;
}

float Imu660raDriver::filtered_gyro_z_deg_s() const
{
    return filtered_gyro_rad_s_.z * IMU660RA_RAD_TO_DEG;
}

const char *Imu660raDriver::last_error() const
{
    return last_error_;
}

/**
 * @brief 从 sysfs 节点读取量程系数
 * @param path 量程文件路径
 * @param value 用于返回解析结果
 * @return 读取成功返回 true，失败返回 false
 */
bool Imu660raDriver::read_scale(const char *path, float &value) const
{
    FILE *fp = fopen(path, "r");
    if (NULL == fp)
    {
        return false;
    }

    char str[32] = {0};
    const int ret = fscanf(fp, "%31s", str);
    fclose(fp);

    if (1 != ret)
    {
        return false;
    }

    value = atof(str);
    return (value > 0.0f);
}

void Imu660raDriver::convert_raw_gyro_data()
{
    gyro_rad_s_.x = data_.gyro_x * gyro_scale_;
    gyro_rad_s_.y = data_.gyro_y * gyro_scale_;
    gyro_rad_s_.z = data_.gyro_z * gyro_scale_;
}

void Imu660raDriver::apply_gyro_lowpass_filter()
{
    if (!filter_ready_)
    {
        filtered_gyro_rad_s_ = gyro_rad_s_;
        filter_ready_ = true;
        return;
    }

    const float history_gain = clamp_float(filter_alpha_, 0.0f, 0.99f);
    const float input_gain = 1.0f - history_gain;

    filtered_gyro_rad_s_.x = history_gain * filtered_gyro_rad_s_.x + input_gain * gyro_rad_s_.x;
    filtered_gyro_rad_s_.y = history_gain * filtered_gyro_rad_s_.y + input_gain * gyro_rad_s_.y;
    filtered_gyro_rad_s_.z = history_gain * filtered_gyro_rad_s_.z + input_gain * gyro_rad_s_.z;
}

void Imu660raDriver::set_error(const char *message)
{
    if (NULL == message)
    {
        last_error_[0] = '\0';
        return;
    }

    strncpy(last_error_, message, sizeof(last_error_) - 1);
    last_error_[sizeof(last_error_) - 1] = '\0';
}

Imu660raDriver imu660ra_driver;

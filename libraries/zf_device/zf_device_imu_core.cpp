#include "zf_device_imu_core.h"

#include <ctype.h>
#include <dirent.h>
#include <stdio.h>

#define IMU_IIO_ROOT_PATH       "/sys/bus/iio/devices"
#define IMU_DEFAULT_DEVICE_DIR  IMU_IIO_ROOT_PATH "/iio:device1"

uint8 imu_type = DEV_NO_FIND;
char imu_device_dir[IMU_SYSFS_PATH_MAX_LEN] = IMU_DEFAULT_DEVICE_DIR;
char imu_dev_name[IMU_DEVICE_NAME_MAX_LEN] = {0};

int16 imu_acc_x,  imu_acc_y,  imu_acc_z;
int16 imu_gyro_x, imu_gyro_y, imu_gyro_z;
int16 imu_mag_x,  imu_mag_y,  imu_mag_z;

char imu_file_path[9][IMU_SYSFS_PATH_MAX_LEN] =
{
    IMU_DEFAULT_DEVICE_DIR "/in_accel_x_raw",
    IMU_DEFAULT_DEVICE_DIR "/in_accel_y_raw",
    IMU_DEFAULT_DEVICE_DIR "/in_accel_z_raw",

    IMU_DEFAULT_DEVICE_DIR "/in_anglvel_x_raw",
    IMU_DEFAULT_DEVICE_DIR "/in_anglvel_y_raw",
    IMU_DEFAULT_DEVICE_DIR "/in_anglvel_z_raw",

    IMU_DEFAULT_DEVICE_DIR "/in_magn_x_raw",
    IMU_DEFAULT_DEVICE_DIR "/in_magn_y_raw",
    IMU_DEFAULT_DEVICE_DIR "/in_magn_z_raw",
};

/**
 * @brief 静默读取 sysfs 字符串，失败时不打印日志
 * @param path 节点路径
 * @param str 返回字符串缓冲区
 * @param size 缓冲区大小
 * @return 成功返回 0，失败返回 -1
 */
static int8 imu_read_string_quiet(const char *path, char *str, uint32 size)
{
    if (NULL == path || NULL == str || 0 == size)
    {
        return -1;
    }

    FILE *fp = fopen(path, "r");
    if (NULL == fp)
    {
        return -1;
    }

    str[0] = '\0';
    if (NULL == fgets(str, (int)size, fp))
    {
        fclose(fp);
        return -1;
    }
    fclose(fp);

    for (uint32 i = 0; '\0' != str[i]; ++i)
    {
        if ('\r' == str[i] || '\n' == str[i] || isspace((unsigned char)str[i]))
        {
            str[i] = '\0';
            break;
        }
    }

    return ('\0' != str[0]) ? 0 : -1;
}

/**
 * @brief 根据节点名拼接当前 IMU 对应的 sysfs 路径
 * @param node_name 节点名
 * @param path 返回路径缓冲区
 * @param path_size 缓冲区大小
 * @return 成功返回 0，失败返回 -1
 */
int8 imu_get_node_path(const char *node_name, char *path, uint32 path_size)
{
    if (NULL == node_name || NULL == path || 0 == path_size)
    {
        return -1;
    }

    const int length = snprintf(path, path_size, "%s/%s", imu_device_dir, node_name);
    if (length < 0 || (uint32)length >= path_size)
    {
        path[0] = '\0';
        return -1;
    }

    return 0;
}

/**
 * @brief 刷新原始数据节点缓存
 * @param device_dir IIO 设备目录
 */
static void imu_set_device_dir(const char *device_dir)
{
    static const char *node_name[9] =
    {
        "in_accel_x_raw",
        "in_accel_y_raw",
        "in_accel_z_raw",
        "in_anglvel_x_raw",
        "in_anglvel_y_raw",
        "in_anglvel_z_raw",
        "in_magn_x_raw",
        "in_magn_y_raw",
        "in_magn_z_raw",
    };

    if (NULL == device_dir || '\0' == device_dir[0])
    {
        device_dir = IMU_DEFAULT_DEVICE_DIR;
    }

    strncpy(imu_device_dir, device_dir, sizeof(imu_device_dir) - 1);
    imu_device_dir[sizeof(imu_device_dir) - 1] = '\0';

    for (uint32 i = 0; i < 9; ++i)
    {
        if (0 != imu_get_node_path(node_name[i], imu_file_path[i], sizeof(imu_file_path[i])))
        {
            imu_file_path[i][0] = '\0';
        }
    }
}

/**
 * @brief 判断设备名中是否包含目标字符串，大小写不敏感
 * @param src 原始设备名
 * @param target 目标关键字
 * @return 匹配返回 1，否则返回 0
 */
static int8 imu_name_contains_ignore_case(const char *src, const char *target)
{
    if (NULL == src || NULL == target || '\0' == target[0])
    {
        return 0;
    }

    for (uint32 i = 0; '\0' != src[i]; ++i)
    {
        uint32 j = 0;
        while ('\0' != target[j] &&
               '\0' != src[i + j] &&
               toupper((unsigned char)src[i + j]) == toupper((unsigned char)target[j]))
        {
            ++j;
        }

        if ('\0' == target[j])
        {
            return 1;
        }
    }

    return 0;
}

/**
 * @brief 根据设备名推断 IMU 类型
 * @param device_name sysfs name 节点内容
 * @return 匹配到的 IMU 类型
 */
static uint8 imu_detect_type_from_name(const char *device_name)
{
    if (imu_name_contains_ignore_case(device_name, "IMU660RA"))
    {
        return DEV_IMU660RA;
    }
    if (imu_name_contains_ignore_case(device_name, "IMU660RB"))
    {
        return DEV_IMU660RB;
    }
    if (imu_name_contains_ignore_case(device_name, "IMU963RA"))
    {
        return DEV_IMU963RA;
    }

    return DEV_NO_FIND;
}

/**
 * @brief 判断某个 IIO 设备目录是否具备 IMU 原始数据节点
 * @param device_dir IIO 设备目录
 * @return 具备加速度和角速度节点返回 1，否则返回 0
 */
static int8 imu_device_has_sensor_nodes(const char *device_dir)
{
    char acc_path[IMU_SYSFS_PATH_MAX_LEN] = {0};
    char gyro_path[IMU_SYSFS_PATH_MAX_LEN] = {0};

    if (NULL == device_dir)
    {
        return 0;
    }

    if (snprintf(acc_path, sizeof(acc_path), "%s/%s", device_dir, "in_accel_x_raw") < 0)
    {
        return 0;
    }
    if (snprintf(gyro_path, sizeof(gyro_path), "%s/%s", device_dir, "in_anglvel_x_raw") < 0)
    {
        return 0;
    }

    return (0 == access(acc_path, R_OK) && 0 == access(gyro_path, R_OK)) ? 1 : 0;
}

/**
 * @brief 缓存识别到的设备目录、设备名和设备类型
 * @param device_dir IIO 设备目录
 * @param device_name 设备名
 * @param type 识别到的设备类型
 */
static void imu_record_device(const char *device_dir, const char *device_name, uint8 type)
{
    imu_type = type;
    imu_set_device_dir(device_dir);

    if (NULL == device_name)
    {
        imu_dev_name[0] = '\0';
        return;
    }

    strncpy(imu_dev_name, device_name, sizeof(imu_dev_name) - 1);
    imu_dev_name[sizeof(imu_dev_name) - 1] = '\0';
}

/**
 * @brief 回退探测默认的 iio:device1
 * @return 成功读取到设备名返回 0，否则返回 -1
 */
static int8 imu_probe_default_device(void)
{
    char name_path[IMU_SYSFS_PATH_MAX_LEN] = {0};
    char device_name[IMU_DEVICE_NAME_MAX_LEN] = {0};

    if (snprintf(name_path, sizeof(name_path), "%s/%s", IMU_DEFAULT_DEVICE_DIR, "name") < 0)
    {
        return -1;
    }

    if (0 != imu_read_string_quiet(name_path, device_name, sizeof(device_name)))
    {
        return -1;
    }

    imu_record_device(IMU_DEFAULT_DEVICE_DIR,
                      device_name,
                      imu_detect_type_from_name(device_name));
    return 0;
}

void imu_get_dev_info()
{
    imu_type = DEV_NO_FIND;
    imu_dev_name[0] = '\0';
    imu_set_device_dir(IMU_DEFAULT_DEVICE_DIR);

    DIR *dir = opendir(IMU_IIO_ROOT_PATH);
    if (NULL != dir)
    {
        struct dirent *entry = NULL;
        char fallback_dir[IMU_SYSFS_PATH_MAX_LEN] = {0};
        char fallback_name[IMU_DEVICE_NAME_MAX_LEN] = {0};

        while (NULL != (entry = readdir(dir)))
        {
            if (0 != strncmp(entry->d_name, "iio:device", 10))
            {
                continue;
            }

            char device_dir[IMU_SYSFS_PATH_MAX_LEN] = {0};
            char name_path[IMU_SYSFS_PATH_MAX_LEN] = {0};
            char device_name[IMU_DEVICE_NAME_MAX_LEN] = {0};

            if (snprintf(device_dir, sizeof(device_dir), "%s/%s", IMU_IIO_ROOT_PATH, entry->d_name) < 0)
            {
                continue;
            }
            if (snprintf(name_path, sizeof(name_path), "%s/%s", device_dir, "name") < 0)
            {
                continue;
            }
            if (0 != imu_read_string_quiet(name_path, device_name, sizeof(device_name)))
            {
                continue;
            }

            const uint8 detected_type = imu_detect_type_from_name(device_name);
            if (DEV_NO_FIND != detected_type)
            {
                closedir(dir);
                imu_record_device(device_dir, device_name, detected_type);
                return;
            }

            if ('\0' == fallback_dir[0] && imu_device_has_sensor_nodes(device_dir))
            {
                strncpy(fallback_dir, device_dir, sizeof(fallback_dir) - 1);
                fallback_dir[sizeof(fallback_dir) - 1] = '\0';
                strncpy(fallback_name, device_name, sizeof(fallback_name) - 1);
                fallback_name[sizeof(fallback_name) - 1] = '\0';
            }
        }

        closedir(dir);

        if ('\0' != fallback_dir[0])
        {
            imu_record_device(fallback_dir, fallback_name, DEV_NO_FIND);
            return;
        }
    }

    if (0 == imu_probe_default_device())
    {
        return;
    }

    printf("imu init error\r\n");
}


int16 imu_get_raw(const char *path)
{
    char str[20] = {0};
    if (0 != imu_read_string_quiet(path, str, sizeof(str)))
    {
        return 0;
    }

    return atoi(str);
}



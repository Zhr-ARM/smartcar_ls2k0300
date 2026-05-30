#include "zf_device_imu_core.h"

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <stdio.h>

#define IMU_IIO_ROOT_PATH       "/sys/bus/iio/devices"
#define IMU_DEFAULT_DEVICE_DIR  IMU_IIO_ROOT_PATH "/iio:device1"

uint8 imu_type = DEV_NO_FIND;
char imu_device_dir[IMU_SYSFS_PATH_MAX_LEN] = IMU_DEFAULT_DEVICE_DIR;
char imu_dev_name[IMU_DEVICE_NAME_MAX_LEN] = {0};
char imu_probe_reason[IMU_PROBE_REASON_MAX_LEN] = {0};

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
 * @brief 更新最近一次 IMU 探测诊断原因
 * @param format printf 风格格式串
 */
static void imu_set_probe_reason(const char *format, ...)
{
    if (NULL == format)
    {
        imu_probe_reason[0] = '\0';
        return;
    }

    va_list args;
    va_start(args, format);
    vsnprintf(imu_probe_reason, sizeof(imu_probe_reason), format, args);
    va_end(args);
    imu_probe_reason[sizeof(imu_probe_reason) - 1] = '\0';
}

/**
 * @brief 获取 errno 的可打印文本
 * @param error_code errno 数值
 * @return strerror 文本
 */
static const char *imu_errno_text(int error_code)
{
    return (0 != error_code) ? strerror(error_code) : "no errno";
}

static const char *imu_safe_cstr(const char *text)
{
    return (NULL != text && '\0' != text[0]) ? text : "(empty)";
}

static void imu_copy_string(char *dst, uint32 dst_size, const char *src)
{
    if (NULL == dst || 0 == dst_size)
    {
        return;
    }
    if (NULL == src)
    {
        dst[0] = '\0';
        return;
    }

    uint32 i = 0;
    for (; i + 1 < dst_size && '\0' != src[i]; ++i)
    {
        dst[i] = src[i];
    }
    dst[i] = '\0';
}

/**
 * @brief 静默读取 sysfs 字符串，失败时不打印日志
 * @param path 节点路径
 * @param str 返回字符串缓冲区
 * @param size 缓冲区大小
 * @param error_out 返回失败 errno，成功时写入 0
 * @return 成功返回 0，失败返回 -1
 */
static int8 imu_read_string_quiet_errno(const char *path, char *str, uint32 size, int *error_out)
{
    if (NULL == path || NULL == str || 0 == size)
    {
        if (NULL != error_out)
        {
            *error_out = EINVAL;
        }
        return -1;
    }

    errno = 0;
    FILE *fp = fopen(path, "r");
    if (NULL == fp)
    {
        if (NULL != error_out)
        {
            *error_out = errno;
        }
        return -1;
    }

    str[0] = '\0';
    if (NULL == fgets(str, (int)size, fp))
    {
        const int saved_errno = (0 != ferror(fp) && 0 != errno) ? errno : ENODATA;
        fclose(fp);
        if (NULL != error_out)
        {
            *error_out = saved_errno;
        }
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

    if ('\0' == str[0])
    {
        if (NULL != error_out)
        {
            *error_out = ENODATA;
        }
        return -1;
    }

    if (NULL != error_out)
    {
        *error_out = 0;
    }
    return 0;
}

/**
 * @brief 静默读取 sysfs 字符串，失败时不打印日志
 * @param path 节点路径
 * @param str 返回字符串缓冲区
 * @param size 缓冲区大小
 * @return 成功返回 0，失败返回 -1
 */
static int8 imu_read_string_quiet(const char *path, char *str, uint32 size)
{
    return imu_read_string_quiet_errno(path, str, size, NULL);
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

    imu_copy_string(imu_device_dir, sizeof(imu_device_dir), device_dir);

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
static int8 imu_find_missing_sensor_node(const char *device_dir,
                                         char *missing_path,
                                         uint32 missing_path_size,
                                         int *error_out)
{
    static const char *node_name[] =
    {
        "in_accel_x_raw",
        "in_accel_y_raw",
        "in_accel_z_raw",
        "in_anglvel_x_raw",
        "in_anglvel_y_raw",
        "in_anglvel_z_raw",
    };

    if (NULL == device_dir)
    {
        if (NULL != error_out)
        {
            *error_out = EINVAL;
        }
        return -1;
    }

    for (uint32 i = 0; i < sizeof(node_name) / sizeof(node_name[0]); ++i)
    {
        char path[IMU_SYSFS_PATH_MAX_LEN] = {0};
        const int length = snprintf(path, sizeof(path), "%s/%s", device_dir, node_name[i]);
        if (length < 0 || (uint32)length >= sizeof(path))
        {
            if (NULL != missing_path && 0 < missing_path_size)
            {
                missing_path[0] = '\0';
            }
            if (NULL != error_out)
            {
                *error_out = ENAMETOOLONG;
            }
            return -1;
        }

        errno = 0;
        if (0 != access(path, R_OK))
        {
            if (NULL != missing_path && 0 < missing_path_size)
            {
                imu_copy_string(missing_path, missing_path_size, path);
            }
            if (NULL != error_out)
            {
                *error_out = errno;
            }
            return -1;
        }
    }

    if (NULL != error_out)
    {
        *error_out = 0;
    }
    return 0;
}

/**
 * @brief 判断某个 IIO 设备目录是否具备 IMU 原始数据节点
 * @param device_dir IIO 设备目录
 * @return 具备加速度和角速度节点返回 1，否则返回 0
 */
static int8 imu_device_has_sensor_nodes(const char *device_dir)
{
    return (0 == imu_find_missing_sensor_node(device_dir, NULL, 0, NULL)) ? 1 : 0;
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

    imu_copy_string(imu_dev_name, sizeof(imu_dev_name), device_name);
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
        imu_set_probe_reason("默认 IIO 设备路径过长: %s/name", IMU_DEFAULT_DEVICE_DIR);
        return -1;
    }

    int read_errno = 0;
    if (0 != imu_read_string_quiet_errno(name_path, device_name, sizeof(device_name), &read_errno))
    {
        imu_set_probe_reason("默认设备 name 节点读取失败 path=%s errno=%d(%s)",
                             name_path,
                             read_errno,
                             imu_errno_text(read_errno));
        return -1;
    }

    const uint8 detected_type = imu_detect_type_from_name(device_name);
    if (DEV_IMU660RA == detected_type)
    {
        char missing_path[IMU_SYSFS_PATH_MAX_LEN] = {0};
        int node_errno = 0;
        if (0 != imu_find_missing_sensor_node(IMU_DEFAULT_DEVICE_DIR,
                                             missing_path,
                                             sizeof(missing_path),
                                             &node_errno))
        {
            imu_record_device(IMU_DEFAULT_DEVICE_DIR, device_name, DEV_NO_FIND);
            imu_set_probe_reason("默认设备已识别为 IMU660RA，但原始数据节点尚未就绪 path=%s errno=%d(%s)",
                                 ('\0' != missing_path[0]) ? missing_path : "(path-too-long)",
                                 node_errno,
                                 imu_errno_text(node_errno));
            return 0;
        }
    }

    imu_record_device(IMU_DEFAULT_DEVICE_DIR, device_name, detected_type);
    if (DEV_NO_FIND == detected_type)
    {
        char missing_path[IMU_SYSFS_PATH_MAX_LEN] = {0};
        int node_errno = 0;
        if (0 == imu_find_missing_sensor_node(IMU_DEFAULT_DEVICE_DIR,
                                             missing_path,
                                             sizeof(missing_path),
                                             &node_errno))
        {
            imu_set_probe_reason("默认设备 name=%s 具备 IMU raw 节点，但名称不包含 IMU660RA",
                                 device_name);
        }
        else
        {
            imu_set_probe_reason("默认设备 name=%s 不匹配 IMU660RA，且原始数据节点缺失或不可读 path=%s errno=%d(%s)",
                                 device_name,
                                 ('\0' != missing_path[0]) ? missing_path : "(path-too-long)",
                                 node_errno,
                                 imu_errno_text(node_errno));
        }
    }
    else
    {
        imu_set_probe_reason("默认设备匹配 name=%s dir=%s", device_name, IMU_DEFAULT_DEVICE_DIR);
    }
    return 0;
}

void imu_get_dev_info()
{
    imu_type = DEV_NO_FIND;
    imu_dev_name[0] = '\0';
    imu_probe_reason[0] = '\0';
    imu_set_device_dir(IMU_DEFAULT_DEVICE_DIR);

    DIR *dir = opendir(IMU_IIO_ROOT_PATH);
    if (NULL != dir)
    {
        struct dirent *entry = NULL;
        char fallback_dir[IMU_SYSFS_PATH_MAX_LEN] = {0};
        char fallback_name[IMU_DEVICE_NAME_MAX_LEN] = {0};
        char first_device_dir[IMU_SYSFS_PATH_MAX_LEN] = {0};
        char first_device_name[IMU_DEVICE_NAME_MAX_LEN] = {0};
        char first_name_path[IMU_SYSFS_PATH_MAX_LEN] = {0};
        char first_missing_node[IMU_SYSFS_PATH_MAX_LEN] = {0};
        char pending_imu_dir[IMU_SYSFS_PATH_MAX_LEN] = {0};
        char pending_imu_name[IMU_DEVICE_NAME_MAX_LEN] = {0};
        char pending_imu_missing_node[IMU_SYSFS_PATH_MAX_LEN] = {0};
        int first_name_errno = 0;
        int first_missing_errno = 0;
        int pending_imu_missing_errno = 0;
        uint32 device_count = 0;
        uint32 name_read_fail_count = 0;
        uint32 unsupported_count = 0;

        while (NULL != (entry = readdir(dir)))
        {
            if (0 != strncmp(entry->d_name, "iio:device", 10))
            {
                continue;
            }

            char device_dir[IMU_SYSFS_PATH_MAX_LEN] = {0};
            char name_path[IMU_SYSFS_PATH_MAX_LEN] = {0};
            char device_name[IMU_DEVICE_NAME_MAX_LEN] = {0};

            int length = snprintf(device_dir, sizeof(device_dir), "%s/%s", IMU_IIO_ROOT_PATH, entry->d_name);
            if (length < 0 || (uint32)length >= sizeof(device_dir))
            {
                imu_set_probe_reason("IIO 设备路径拼接失败 root=%s name=%s", IMU_IIO_ROOT_PATH, entry->d_name);
                continue;
            }
            length = snprintf(name_path, sizeof(name_path), "%s/%s", device_dir, "name");
            if (length < 0 || (uint32)length >= sizeof(name_path))
            {
                imu_set_probe_reason("IIO name 节点路径拼接失败 dir=%s", device_dir);
                continue;
            }
            ++device_count;
            if ('\0' == first_device_dir[0])
            {
                imu_copy_string(first_device_dir, sizeof(first_device_dir), device_dir);
            }

            int read_errno = 0;
            if (0 != imu_read_string_quiet_errno(name_path, device_name, sizeof(device_name), &read_errno))
            {
                ++name_read_fail_count;
                if ('\0' == first_name_path[0])
                {
                    imu_copy_string(first_name_path, sizeof(first_name_path), name_path);
                    first_name_errno = read_errno;
                }
                continue;
            }
            if ('\0' == first_device_name[0])
            {
                imu_copy_string(first_device_name, sizeof(first_device_name), device_name);
            }

            const uint8 detected_type = imu_detect_type_from_name(device_name);
            if (DEV_NO_FIND != detected_type)
            {
                if (DEV_IMU660RA == detected_type)
                {
                    int node_errno = 0;
                    char missing_path[IMU_SYSFS_PATH_MAX_LEN] = {0};
                    if (0 != imu_find_missing_sensor_node(device_dir,
                                                         missing_path,
                                                         sizeof(missing_path),
                                                         &node_errno))
                    {
                        if ('\0' == pending_imu_dir[0])
                        {
                            imu_copy_string(pending_imu_dir, sizeof(pending_imu_dir), device_dir);
                            imu_copy_string(pending_imu_name, sizeof(pending_imu_name), device_name);
                            imu_copy_string(pending_imu_missing_node, sizeof(pending_imu_missing_node), missing_path);
                            pending_imu_missing_errno = node_errno;
                        }
                        continue;
                    }
                }

                closedir(dir);
                imu_record_device(device_dir, device_name, detected_type);
                imu_set_probe_reason("匹配到 IIO 设备 name=%s dir=%s", device_name, device_dir);
                return;
            }

            ++unsupported_count;
            if ('\0' == fallback_dir[0] && imu_device_has_sensor_nodes(device_dir))
            {
                imu_copy_string(fallback_dir, sizeof(fallback_dir), device_dir);
                imu_copy_string(fallback_name, sizeof(fallback_name), device_name);
            }
            else if ('\0' == first_missing_node[0])
            {
                int node_errno = 0;
                char missing_path[IMU_SYSFS_PATH_MAX_LEN] = {0};
                if (0 != imu_find_missing_sensor_node(device_dir,
                                                     missing_path,
                                                     sizeof(missing_path),
                                                     &node_errno))
                {
                    imu_copy_string(first_missing_node, sizeof(first_missing_node), missing_path);
                    first_missing_errno = node_errno;
                }
            }
        }

        closedir(dir);

        if ('\0' != pending_imu_dir[0])
        {
            imu_record_device(pending_imu_dir, pending_imu_name, DEV_NO_FIND);
            imu_set_probe_reason("已识别到 IMU660RA，但原始数据节点尚未就绪 path=%s errno=%d(%s) dir=%s",
                                 ('\0' != pending_imu_missing_node[0]) ? pending_imu_missing_node : "(path-too-long)",
                                 pending_imu_missing_errno,
                                 imu_errno_text(pending_imu_missing_errno),
                                 pending_imu_dir);
            return;
        }

        if ('\0' != fallback_dir[0])
        {
            imu_record_device(fallback_dir, fallback_name, DEV_NO_FIND);
            imu_set_probe_reason("扫描到具备 IMU raw 节点的 IIO 设备，但 name=%s 不包含 IMU660RA dir=%s",
                                 fallback_name,
                                 fallback_dir);
            return;
        }

        if (0 == device_count)
        {
            imu_set_probe_reason("%s 存在，但没有 iio:device* 设备目录", IMU_IIO_ROOT_PATH);
        }
        else if (name_read_fail_count == device_count)
        {
            imu_set_probe_reason("扫描到 %u 个 IIO 设备，但 name 节点均读取失败 first=%s errno=%d(%s)",
                                 (unsigned int)device_count,
                                 imu_safe_cstr(first_name_path),
                                 first_name_errno,
                                 imu_errno_text(first_name_errno));
            imu_record_device(first_device_dir, NULL, DEV_NO_FIND);
        }
        else if (0 < unsupported_count)
        {
            if ('\0' != first_missing_node[0])
            {
                imu_set_probe_reason("扫描到 %u 个 IIO 设备，首个 name=%s 不匹配 IMU660RA，且缺少/不可读节点 %s errno=%d(%s)",
                                     (unsigned int)device_count,
                                     imu_safe_cstr(first_device_name),
                                     first_missing_node,
                                     first_missing_errno,
                                     imu_errno_text(first_missing_errno));
            }
            else
            {
                imu_set_probe_reason("扫描到 %u 个 IIO 设备，但设备名均不包含 IMU660RA，首个 name=%s dir=%s",
                                     (unsigned int)device_count,
                                     imu_safe_cstr(first_device_name),
                                     imu_safe_cstr(first_device_dir));
            }
            imu_record_device(first_device_dir, first_device_name, DEV_NO_FIND);
        }
    }
    else
    {
        const int open_errno = errno;
        imu_set_probe_reason("无法打开 %s errno=%d(%s)",
                             IMU_IIO_ROOT_PATH,
                             open_errno,
                             imu_errno_text(open_errno));
    }

    char scan_reason[IMU_PROBE_REASON_MAX_LEN] = {0};
    imu_copy_string(scan_reason, sizeof(scan_reason), imu_probe_reason);

    if (0 == imu_probe_default_device())
    {
        return;
    }
    if ('\0' != scan_reason[0])
    {
        imu_copy_string(imu_probe_reason, sizeof(imu_probe_reason), scan_reason);
    }

    return;
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



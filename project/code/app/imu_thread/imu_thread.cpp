#include "imu_thread.h"

#include <algorithm>
#include <atomic>
#include <pthread.h>
#include <sys/syscall.h>
#include <thread>

namespace
{
constexpr int32 IMU_PERIOD_MS = 5;
// 调度优先级：低于速度环(10)和巡线线程(8)，但高于普通后台线程。
constexpr int32 IMU_THREAD_PRIORITY = 7;
constexpr float IMU_RAD_TO_DEG = 57.2957795f;

std::thread g_imu_thread;
std::atomic<bool> g_imu_initialized(false);
std::atomic<bool> g_imu_running(false);
std::atomic<int32> g_thread_tid(0);
std::atomic<int32> g_thread_policy(0);
std::atomic<int32> g_thread_priority(0);
std::atomic<float> g_gyro_z_dps(0.0f);
std::atomic<float> g_gyro_z_bias_dps(0.0f);

/**
 * @brief 将调度策略枚举转换为字符串
 * @param policy 调度策略值
 * @return 对应的策略名称
 */
const char *sched_policy_name(int32 policy)
{
    switch (policy)
    {
        case SCHED_FIFO:  return "SCHED_FIFO";
        case SCHED_RR:    return "SCHED_RR";
        case SCHED_OTHER: return "SCHED_OTHER";
#ifdef SCHED_BATCH
        case SCHED_BATCH: return "SCHED_BATCH";
#endif
#ifdef SCHED_IDLE
        case SCHED_IDLE:  return "SCHED_IDLE";
#endif
        default:          return "UNKNOWN";
    }
}

const char *safe_cstr(const char *text)
{
    return (NULL != text && '\0' != text[0]) ? text : "(empty)";
}

const char *imu_type_name(uint8 type)
{
    switch (type)
    {
        case DEV_IMU660RA: return "IMU660RA";
        case DEV_IMU660RB: return "IMU660RB";
        case DEV_IMU963RA: return "IMU963RA";
        case DEV_NO_FIND:  return "NO_FIND";
        default:           return "UNKNOWN";
    }
}

void print_imu_device_summary(const char *stage, bool success, const char *reason)
{
    printf("[IMU INIT] stage=%s result=%s reason=%s type=%s(0x%02X) name=%s dir=%s\r\n",
           safe_cstr(stage),
           success ? "ok" : "fail",
           safe_cstr(reason),
           imu_type_name(imu_type),
           static_cast<unsigned int>(imu_type),
           safe_cstr(imu_dev_name),
           safe_cstr(imu_device_dir));
}

void print_imu_scale_summary()
{
    char gyro_scale_path[IMU_SYSFS_PATH_MAX_LEN] = {0};
    const bool gyro_scale_node_found =
        (0 == imu_get_node_path(IMU660RA_GYRO_SCALE_NODE, gyro_scale_path, sizeof(gyro_scale_path)));

    printf("[IMU INIT] scale gyro=%.8f(rad/s/raw)=%.5f(deg/s/raw) gyro_src=%s\r\n",
           static_cast<double>(imu660ra_driver.gyro_scale_rad_s()),
           static_cast<double>(imu660ra_driver.gyro_scale_rad_s() * IMU_RAD_TO_DEG),
           gyro_scale_node_found ? gyro_scale_path : "fallback_default");
}

void print_imu_init_hint(uint8 type)
{
    switch (type)
    {
        case DEV_NO_FIND:
            printf("[IMU INIT] hint=未识别到支持的 IIO IMU 设备，请检查硬件连接、驱动加载以及 /sys/bus/iio/devices 是否存在 imu660ra 节点\r\n");
            break;
        case DEV_IMU660RB:
        case DEV_IMU963RA:
            printf("[IMU INIT] hint=当前工程驱动只支持 IMU660RA,如硬件型号不同，需要切换对应驱动或适配型号判断\r\n");
            break;
        default:
            printf("[IMU INIT] hint=若设备已识别但仍初始化失败，优先检查量程节点、原始数据节点访问权限以及首帧采样是否正常\r\n");
            break;
    }
}

/**
 * @brief 刷新当前线程的调度信息缓存
 */
void refresh_thread_info()
{
    int policy = 0;
    struct sched_param param;
    memset(&param, 0, sizeof(param));

    g_thread_tid = (int32)syscall(SYS_gettid);

    if (0 == pthread_getschedparam(pthread_self(), &policy, &param))
    {
        g_thread_policy = policy;
        g_thread_priority = param.sched_priority;
    }
    else
    {
        g_thread_policy = 0;
        g_thread_priority = 0;
    }
}

void publish_gyro_state()
{
    const float corrected_gyro_z_dps =
        imu660ra_driver.filtered_gyro_z_deg_s() - g_gyro_z_bias_dps.load();
    g_gyro_z_dps.store(corrected_gyro_z_dps);
}

int32 calc_bias_sample_count(int32 calibrate_duration_ms)
{
    if (calibrate_duration_ms <= 0)
    {
        return 0;
    }

    return std::max<int32>(1, (calibrate_duration_ms + IMU_PERIOD_MS - 1) / IMU_PERIOD_MS);
}

bool calibrate_gyro_bias_for_duration(int32 calibrate_duration_ms,
                                      float *bias_dps_out,
                                      int32 *valid_samples_out,
                                      int32 *failed_samples_out,
                                      int32 *expected_samples_out)
{
    float gyro_bias_sum = 0.0f;
    int32 valid_samples = 0;
    int32 failed_samples = 0;
    const int32 expected_samples = calc_bias_sample_count(calibrate_duration_ms);

    if (expected_samples_out)
    {
        *expected_samples_out = expected_samples;
    }

    for (int32 i = 0; i < expected_samples; ++i)
    {
        if (imu660ra_driver.update(IMU_PERIOD_MS / 1000.0f))
        {
            gyro_bias_sum += imu660ra_driver.filtered_gyro_z_deg_s();
            ++valid_samples;
        }
        else
        {
            ++failed_samples;
        }
        system_delay_ms(IMU_PERIOD_MS);
    }

    if (valid_samples_out)
    {
        *valid_samples_out = valid_samples;
    }
    if (failed_samples_out)
    {
        *failed_samples_out = failed_samples;
    }

    if (valid_samples <= 0)
    {
        if (bias_dps_out)
        {
            *bias_dps_out = 0.0f;
        }
        return false;
    }

    const float gyro_bias_dps = gyro_bias_sum / (float)valid_samples;
    if (bias_dps_out)
    {
        *bias_dps_out = gyro_bias_dps;
    }
    g_gyro_z_bias_dps.store(gyro_bias_dps);
    publish_gyro_state();
    g_gyro_z_dps.store(0.0f);
    return true;
}

/**
 * @brief IMU 数据采集线程主循环
 */
void imu_loop()
{
    struct sched_param sp;
    sp.sched_priority = IMU_THREAD_PRIORITY;
    if (0 != pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp))
    {
        printf("imu set sched failed, fallback to current policy\r\n");
    }

    refresh_thread_info();

    while (g_imu_running.load())
    {
        if (imu660ra_driver.update(IMU_PERIOD_MS / 1000.0f))
        {
            publish_gyro_state();
        }

        system_delay_ms(IMU_PERIOD_MS);
    }
}
}

bool imu_thread_init()
{
    if (g_imu_running.load() || g_imu_initialized.load())
    {
        return true;
    }

    printf("[IMU INIT] begin sample_period_ms=%d mode=gyro_z_only\r\n",
           IMU_PERIOD_MS);

    if (!imu660ra_driver.init())
    {
        const char *error = imu660ra_driver.last_error();
        print_imu_device_summary("driver_init", false, safe_cstr(error));
        print_imu_init_hint(imu_type);
        return false;
    }
    print_imu_device_summary("driver_init", true, "device probe and first update ok");
    print_imu_scale_summary();
    printf("[IMU INIT] stage=driver_ready result=ok wait_calibration=1\r\n");

    g_imu_initialized = true;
    g_gyro_z_dps.store(0.0f);
    g_gyro_z_bias_dps.store(0.0f);

    return true;
}

bool imu_thread_calibrate_and_start(int32 calibrate_duration_ms)
{
    g_gyro_z_dps.store(0.0f);
    g_gyro_z_bias_dps.store(0.0f);

    if (g_imu_running.load())
    {
        return true;
    }

    if (!g_imu_initialized.load() && !imu_thread_init())
    {
        return false;
    }

    float gyro_bias_dps = 0.0f;
    int32 bias_valid_samples = 0;
    int32 bias_failed_samples = 0;
    int32 bias_expected_samples = 0;
    printf("[IMU INIT] stage=gyro_bias_calibration begin duration_ms=%d expected_samples=%d period_ms=%d\r\n",
           calibrate_duration_ms,
           calc_bias_sample_count(calibrate_duration_ms),
           IMU_PERIOD_MS);
    if (!calibrate_gyro_bias_for_duration(calibrate_duration_ms,
                                          &gyro_bias_dps,
                                          &bias_valid_samples,
                                          &bias_failed_samples,
                                          &bias_expected_samples))
    {
        printf("[IMU INIT] stage=gyro_bias_calibration result=fail reason=未获取到有效陀螺样本 valid=%d failed=%d expected=%d last_error=%s\r\n",
               bias_valid_samples,
               bias_failed_samples,
               bias_expected_samples,
               safe_cstr(imu660ra_driver.last_error()));
        printf("[IMU INIT] hint=请让车辆在上电初始化时保持静止，并检查 IMU 原始数据节点是否能持续读到新值\r\n");
        return false;
    }
    printf("[IMU INIT] stage=gyro_bias_calibration result=ok bias_z=%.3f(dps) valid=%d failed=%d expected=%d duration_ms=%d\r\n",
           static_cast<double>(gyro_bias_dps),
           bias_valid_samples,
           bias_failed_samples,
           bias_expected_samples,
           calibrate_duration_ms);

    g_imu_running = true;
    g_imu_thread = std::thread(imu_loop);
    printf("[IMU INIT] stage=thread_start result=ok period_ms=%d mode=gyro_z_only\r\n", IMU_PERIOD_MS);

    return true;
}

float imu_thread_gyro_z_dps()
{
    return g_gyro_z_dps.load();
}

void imu_thread_cleanup()
{
    if (!g_imu_running.load())
    {
        g_gyro_z_dps.store(0.0f);
        g_gyro_z_bias_dps.store(0.0f);
        g_imu_initialized = false;
        return;
    }

    g_imu_running = false;

    if (g_imu_thread.joinable())
    {
        g_imu_thread.join();
    }

    g_gyro_z_dps.store(0.0f);
    g_gyro_z_bias_dps.store(0.0f);
    g_imu_initialized = false;
}

void imu_thread_print_info()
{
    int32 tid = g_thread_tid.load();

    if (0 < tid)
    {
        printf("thread=%s tid=%d policy=%s priority=%d\r\n",
               "imu_update",
               tid,
               sched_policy_name(g_thread_policy.load()),
               g_thread_priority.load());
    }
    else
    {
        printf("thread=%s tid=unknown policy=unknown priority=unknown\r\n",
               "imu_update");
    }
}

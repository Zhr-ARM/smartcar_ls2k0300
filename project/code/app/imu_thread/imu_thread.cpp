#include "imu_thread.h"

#include <atomic>
#include <mutex>
#include <pthread.h>
#include <sys/syscall.h>
#include <thread>

namespace
{
constexpr int32 IMU_PERIOD_MS = 10;

std::thread g_imu_thread;
std::atomic<bool> g_imu_running(false);
std::atomic<int32> g_thread_tid(0);
std::atomic<int32> g_thread_policy(0);
std::atomic<int32> g_thread_priority(0);

// 姿态角共享数据保护。
std::mutex g_attitude_mutex;
Imu660raEuler g_attitude = {0.0f, 0.0f, 0.0f};

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

/**
 * @brief IMU 数据采集线程主循环
 */
void imu_loop()
{
    refresh_thread_info();

    while (g_imu_running.load())
    {
        imu660ra_driver.update(IMU_PERIOD_MS / 1000.0f);

        {
            std::lock_guard<std::mutex> lock(g_attitude_mutex);
            g_attitude = imu660ra_driver.attitude_deg();
        }

        system_delay_ms(IMU_PERIOD_MS);
    }
}
}

bool imu_thread_init()
{
    if (g_imu_running.load())
    {
        return true;
    }

    if (!imu660ra_driver.init())
    {
        printf("imu thread init failed\r\n");
        return false;
    }
    imu660ra_driver.set_sample_period(IMU_PERIOD_MS / 1000.0f);

    g_imu_running = true;
    g_imu_thread = std::thread(imu_loop);

    return true;
}

Imu660raEuler imu_thread_attitude_deg()
{
    std::lock_guard<std::mutex> lock(g_attitude_mutex);
    return g_attitude;
}

void imu_thread_cleanup()
{
    if (!g_imu_running.load())
    {
        return;
    }

    g_imu_running = false;

    if (g_imu_thread.joinable())
    {
        g_imu_thread.join();
    }
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

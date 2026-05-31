#include "cpu_monitor_thread.h"

#include "driver/config/smartcar_config.h"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <pthread.h>
#include <sys/syscall.h>
#include <thread>

namespace
{
constexpr int32 kCpuMonitorThreadPriority = 1;
constexpr int kPrintIntervalMs = 2000;

std::thread g_cpu_monitor_thread;
std::atomic<bool> g_running(false);
std::atomic<int32> g_thread_tid(0);
std::atomic<int32> g_thread_policy(0);
std::atomic<int32> g_thread_priority(0);

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

void refresh_thread_info()
{
    int policy = 0;
    struct sched_param param;
    memset(&param, 0, sizeof(param));

    g_thread_tid = static_cast<int32>(syscall(SYS_gettid));
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

bool read_cpu_sample(unsigned long long *total_ticks, unsigned long long *idle_ticks)
{
    FILE *fp = std::fopen("/proc/stat", "r");
    if (fp == nullptr)
    {
        return false;
    }

    char line[256] = {0};
    const char *result = std::fgets(line, sizeof(line), fp);
    std::fclose(fp);
    if (result == nullptr)
    {
        return false;
    }

    unsigned long long user = 0, nice = 0, system = 0, idle = 0;
    unsigned long long iowait = 0, irq = 0, softirq = 0, steal = 0;
    const int parsed = std::sscanf(line,
                                   "cpu %llu %llu %llu %llu %llu %llu %llu %llu",
                                   &user, &nice, &system, &idle,
                                   &iowait, &irq, &softirq, &steal);
    if (parsed < 4)
    {
        return false;
    }

    *idle_ticks = idle + iowait;
    *total_ticks = user + nice + system + idle + iowait + irq + softirq + steal;
    return true;
}

void cpu_monitor_loop()
{
    struct sched_param sp;
    sp.sched_priority = kCpuMonitorThreadPriority;
    if (0 != pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp))
    {
        printf("cpu_monitor_thread set sched failed, fallback to current policy\r\n");
    }

    refresh_thread_info();

    unsigned long long prev_total = 0;
    unsigned long long prev_idle = 0;
    bool has_prev = false;

    while (g_running.load())
    {
        if (!g_cpu_monitor_enabled)
        {
            system_delay_ms(kPrintIntervalMs);
            has_prev = false;
            continue;
        }

        system_delay_ms(kPrintIntervalMs);

        unsigned long long total = 0;
        unsigned long long idle = 0;
        if (!read_cpu_sample(&total, &idle))
        {
            continue;
        }

        if (has_prev && total > prev_total)
        {
            const unsigned long long total_delta = total - prev_total;
            const unsigned long long idle_delta = idle - prev_idle;
            if (total_delta > 0)
            {
                const double busy_ratio = 1.0 - static_cast<double>(idle_delta) / static_cast<double>(total_delta);
                const double cpu_percent = std::clamp(busy_ratio * 100.0, 0.0, 100.0);
                printf("[CPU] usage=%.1f%%\r\n", cpu_percent);
            }
        }

        prev_total = total;
        prev_idle = idle;
        has_prev = true;
    }

    g_thread_tid = 0;
    g_thread_policy = 0;
    g_thread_priority = 0;
}

} // namespace

bool cpu_monitor_thread_init()
{
    if (g_running.load())
    {
        return true;
    }

    g_running = true;
    g_cpu_monitor_thread = std::thread(cpu_monitor_loop);
    return true;
}

void cpu_monitor_thread_cleanup()
{
    if (!g_running.load())
    {
        return;
    }

    g_running = false;
    if (g_cpu_monitor_thread.joinable())
    {
        g_cpu_monitor_thread.join();
    }
}

void cpu_monitor_thread_print_info()
{
    const int32 tid = g_thread_tid.load();
    if (0 < tid)
    {
        printf("thread=%s tid=%d policy=%s priority=%d\r\n",
               "cpu_monitor",
               tid,
               sched_policy_name(g_thread_policy.load()),
               g_thread_priority.load());
    }
    else
    {
        printf("thread=%s tid=unknown policy=unknown priority=unknown\r\n", "cpu_monitor");
    }
}

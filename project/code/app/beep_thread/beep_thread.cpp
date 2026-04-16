#include "beep_thread.h"

#include "zf_driver_gpio.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <pthread.h>
#include <sys/syscall.h>
#include <thread>

namespace
{
constexpr const char *kBeepDevicePath = "/dev/zf_driver_gpio_beep";
constexpr int32 kBeepThreadPriority = 1;
constexpr int kBeepThreadPeriodMs = 10;
constexpr int kAlarmPeriodMs = 400;
constexpr int kAlarmOnDurationMs = 220;

std::thread g_beep_thread;
std::atomic<bool> g_beep_running(false);
std::atomic<bool> g_alarm_enabled(false);
std::atomic<int32> g_thread_tid(0);
std::atomic<int32> g_thread_policy(0);
std::atomic<int32> g_thread_priority(0);
std::atomic<uint64_t> g_beep_deadline_ms(0);

uint64_t monotonic_time_ms()
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

void buzzer_set_enabled(bool enabled)
{
    gpio_set_level(kBeepDevicePath, enabled ? 0x1 : 0x0);
}

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

bool alarm_output_active(uint64_t now_ms)
{
    if (!g_alarm_enabled.load())
    {
        return false;
    }

    return (now_ms % static_cast<uint64_t>(kAlarmPeriodMs)) <
           static_cast<uint64_t>(kAlarmOnDurationMs);
}

bool request_output_active(uint64_t now_ms)
{
    return now_ms < g_beep_deadline_ms.load();
}

void beep_loop()
{
    struct sched_param sp;
    sp.sched_priority = kBeepThreadPriority;
    if (0 != pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp))
    {
        printf("beep_thread set sched failed, fallback to current policy\r\n");
    }

    refresh_thread_info();

    while (g_beep_running.load())
    {
        const uint64_t now_ms = monotonic_time_ms();
        const bool output_enabled = alarm_output_active(now_ms) || request_output_active(now_ms);
        buzzer_set_enabled(output_enabled);
        system_delay_ms(kBeepThreadPeriodMs);
    }

    buzzer_set_enabled(false);
    g_thread_tid = 0;
    g_thread_policy = 0;
    g_thread_priority = 0;
}
} // namespace

bool beep_thread_init()
{
    if (g_beep_running.load())
    {
        return true;
    }

    g_alarm_enabled = false;
    g_beep_deadline_ms = 0;
    buzzer_set_enabled(false);

    g_beep_running = true;
    g_beep_thread = std::thread(beep_loop);
    return true;
}

void beep_thread_cleanup()
{
    g_alarm_enabled = false;
    g_beep_deadline_ms = 0;

    if (!g_beep_running.load())
    {
        buzzer_set_enabled(false);
        return;
    }

    g_beep_running = false;
    if (g_beep_thread.joinable())
    {
        g_beep_thread.join();
    }

    buzzer_set_enabled(false);
}

void beep_thread_request_beep(int duration_ms)
{
    if (!g_beep_running.load())
    {
        return;
    }

    const int safe_duration_ms = std::max(duration_ms, 0);
    if (safe_duration_ms <= 0)
    {
        return;
    }

    const uint64_t now_ms = monotonic_time_ms();
    const uint64_t current_deadline_ms = g_beep_deadline_ms.load();
    const uint64_t new_deadline_ms =
        std::max(current_deadline_ms, now_ms) + static_cast<uint64_t>(safe_duration_ms);
    g_beep_deadline_ms.store(new_deadline_ms);
}

void beep_thread_set_alarm_enabled(bool enabled)
{
    g_alarm_enabled = enabled;
    if (enabled)
    {
        g_beep_deadline_ms = 0;
    }
}

bool beep_thread_alarm_enabled()
{
    return g_alarm_enabled.load();
}

void beep_thread_print_info()
{
    const int32 tid = g_thread_tid.load();
    if (0 < tid)
    {
        printf("thread=%s tid=%d policy=%s priority=%d\r\n",
               "beep",
               tid,
               sched_policy_name(g_thread_policy.load()),
               g_thread_priority.load());
    }
    else
    {
        printf("thread=%s tid=unknown policy=unknown priority=unknown\r\n", "beep");
    }
}

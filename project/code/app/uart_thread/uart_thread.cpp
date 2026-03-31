#include "uart_thread.h"
#include "uart_pid_protocol.h"
#include "zf_common_headfile.h"
#include "uart.h"
#include "motor_thread.h"
#include "motor.h"
#include "line_follow_thread.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <cmath>
#include <pthread.h>
#include <sys/syscall.h>
#include <thread>

namespace
{
Uart uart1(UART_DEFAULT_DEVICE, UART_DEFAULT_BAUDRATE);

constexpr int32 kStatusPushPeriodMs = 10;

std::thread g_status_thread;
std::atomic<bool> g_status_running(false);
std::atomic<int32> g_status_tid(0);
std::atomic<int32> g_status_policy(0);
std::atomic<int32> g_status_priority(0);

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
 * @brief 同时输出日志到控制台和串口
 * @param message 待发送的日志字符串
 */
void uart_send_log(const char *message)
{
    printf("%s", message);
    if (uart1.is_open())
    {
        uart1.send_string(message);
    }
}

/**
 * @brief 输出单行线程调度信息
 * @param name 线程名称
 * @param tid 线程 ID
 * @param policy 调度策略
 * @param priority 线程优先级
 * @param valid 线程信息是否有效
 */
void print_thread_info_line(const char *name, int32 tid, int32 policy, int32 priority, bool valid)
{
    char buffer[160];

    if (valid)
    {
        snprintf(buffer, sizeof(buffer),
                 "thread=%s tid=%d policy=%s priority=%d\r\n",
                 name, tid, sched_policy_name(policy), priority);
    }
    else
    {
        snprintf(buffer, sizeof(buffer),
                 "thread=%s tid=unknown policy=unknown priority=unknown\r\n",
                 name);
    }

    uart_send_log(buffer);
}

/**
 * @brief 打印主线程调度信息
 */
void print_main_thread_info()
{
    int policy = 0;
    struct sched_param param;

    memset(&param, 0, sizeof(param));

    if (0 == pthread_getschedparam(pthread_self(), &policy, &param))
    {
        print_thread_info_line("main",
                               (int32)syscall(SYS_gettid),
                               policy,
                               param.sched_priority,
                               true);
    }
    else
    {
        print_thread_info_line("main", 0, 0, 0, false);
    }
}

/**
 * @brief 刷新状态发送线程的调度信息缓存
 */
void refresh_status_thread_info()
{
    int policy = 0;
    struct sched_param param;

    memset(&param, 0, sizeof(param));

    g_status_tid = (int32)syscall(SYS_gettid);
    if (0 == pthread_getschedparam(pthread_self(), &policy, &param))
    {
        g_status_policy = policy;
        g_status_priority = param.sched_priority;
    }
    else
    {
        g_status_policy = 0;
        g_status_priority = 0;
    }
}

#if ENABLE_UART_TUNING
/**
 * @brief 协议层发送回调
 * @param data 待发送数据缓冲区
 * @param length 数据长度
 * @return 发送完整返回 true，否则返回 false
 */
bool uart_send_protocol(const uint8* data, size_t length)
{
    if (uart1.is_open())
    {
        return uart1.send(data, length) == (int32)length;
    }
    return false;
}

/**
 * @brief 串口接收回调
 * @param data 接收到的数据缓冲区
 * @param length 数据长度
 */
void uart_recv_callback(const uint8 *data, int32 length)
{
    g_pid_protocol.receive_data(data, length);
}
#endif

/**
 * @brief 状态主动上报线程主循环
 */
void status_push_loop()
{
    refresh_status_thread_info();
    int print_counter = 0;

    while (g_status_running.load())
    {
#if ENABLE_UART_TUNING
        g_pid_protocol.send_status_push();
#endif
        
        // if (++print_counter >= 2)
        // {
        //     print_counter = 0;
        //     char buf[192];
        //     char term_buf[256];
        //     const MotorUartStatus status = motor_thread_uart_status();
        //     const float line_error = line_follow_thread_error();

        //     snprintf(buf, sizeof(buf),
        //              "%7.1f,%7.1f,%6.2f,%7.1f,"
        //              "%7.1f,%7.1f,%6.2f,%7.1f\r\n",
        //              status.left_current_count,
        //              status.left_target_count,
        //              status.left_duty,
        //              status.left_error,
        //              status.right_current_count,
        //              status.right_target_count,
        //              status.right_duty,
        //              status.right_error);

        //     snprintf(term_buf, sizeof(term_buf),
        //              "[CTRL] LineErr=%7.1f | EncL=%7.1f TarL=%7.1f ErrL=%7.1f DutyL=%6.2f | EncR=%7.1f TarR=%7.1f ErrR=%7.1f DutyR=%6.2f\n",
        //              line_error,
        //              status.left_current_count,
        //              status.left_target_count,
        //              status.left_error,
        //              status.left_duty,
        //              status.right_current_count,
        //              status.right_target_count,
        //              status.right_error,
        //              status.right_duty);
        //     printf("%s", term_buf);

        //     if (uart1.is_open())
        //     {
        //         uart1.send_string(buf);
        //     }
        // }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(kStatusPushPeriodMs));
    }

    g_status_tid = 0;
    g_status_policy = 0;
    g_status_priority = 0;
}
}

bool uart_thread_init()
{
    if (!uart1.init())
    {
        printf("uart init failed\r\n");
        return false;
    }

#if ENABLE_UART_TUNING
    g_pid_protocol.init(uart_send_protocol);

    uart1.set_receive_callback(uart_recv_callback);
    if (!uart1.start_receive_callback())
    {
        printf("uart callback start failed\r\n");
        uart1.close();
        return false;
    }

    g_pid_protocol.send_boot_report();
#endif
    g_status_running = true;
    g_status_thread = std::thread(status_push_loop);

    return true;
}

void uart_thread_cleanup()
{
    g_status_running = false;

    if (g_status_thread.joinable())
    {
        g_status_thread.join();
    }

#if ENABLE_UART_TUNING
    uart1.stop_receive_callback();
#endif
    uart1.close();

#if ENABLE_UART_TUNING
    g_pid_protocol.cleanup();
#endif
}

void uart_thread_print_threads(timer_fd *motor_timer)
{
    print_main_thread_info();

    if (motor_timer != nullptr)
    {
        timer_fd::ThreadInfo timer_info = motor_timer->thread_info();
        print_thread_info_line("motor_timer",
                               timer_info.tid,
                               timer_info.policy,
                               timer_info.priority,
                               timer_info.valid);
    }

#if ENABLE_UART_TUNING
    Uart::ThreadInfo uart_info = uart1.rx_thread_info();
    print_thread_info_line("uart_rx",
                           uart_info.tid,
                           uart_info.policy,
                           uart_info.priority,
                           uart_info.valid);

    print_thread_info_line("uart_tx",
                           g_status_tid.load(),
                           g_status_policy.load(),
                           g_status_priority.load(),
                           (0 < g_status_tid.load()));
#endif
}

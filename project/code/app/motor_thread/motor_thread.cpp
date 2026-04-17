#include "motor_thread.h"

#include "motor.h"
#include "brushless.h"
#include "pid.h"

#include <atomic>
#include <mutex>
#include <pthread.h>
#include <sys/syscall.h>
#include <sys/timerfd.h>
#include <thread>
#include <unistd.h>

namespace
{
constexpr int32 MOTOR_PERIOD_MS = 5;
constexpr float TARGET_COUNT_MIN = -8.0f;
constexpr float TARGET_COUNT_MAX = 1500.0f;

std::thread g_motor_thread;
std::atomic<bool> g_motor_running(false);
std::atomic<int32> g_thread_tid(0);
std::atomic<int32> g_thread_policy(0);
std::atomic<int32> g_thread_priority(0);

// 目标计数保护
std::mutex g_target_mutex;
float g_target_left_count = 0.0f;
float g_target_right_count = 0.0f;

// 对外调试口默认发布原始编码器计数；滤波后的反馈仅供速度环内部使用。
std::atomic<float> g_left_raw_count{0.0f};
std::atomic<float> g_right_raw_count{0.0f};
std::atomic<float> g_left_filtered_count{0.0f};
std::atomic<float> g_right_filtered_count{0.0f};
std::atomic<float> g_left_error{0.0f};
std::atomic<float> g_right_error{0.0f};
std::atomic<float> g_left_feedforward{0.0f};
std::atomic<float> g_right_feedforward{0.0f};
std::atomic<float> g_left_correction{0.0f};
std::atomic<float> g_right_correction{0.0f};
std::atomic<float> g_left_decel_assist{0.0f};
std::atomic<float> g_right_decel_assist{0.0f};
std::atomic<float> g_left_duty{0.0f};
std::atomic<float> g_right_duty{0.0f};
std::atomic<float> g_left_hardware_duty{0.0f};
std::atomic<float> g_right_hardware_duty{0.0f};
std::atomic<int> g_left_dir_level{1};
std::atomic<int> g_right_dir_level{1};
std::atomic<bool> g_reload_from_globals_requested(false);

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
 * @brief 创建 5ms 周期定时器
 * @return 成功返回定时器文件描述符，失败返回 -1
 */
int init_timer_fd()
{
    int fd = timerfd_create(CLOCK_MONOTONIC, 0);
    if (fd == -1)
    {
        perror("motor timerfd_create");
        return -1;
    }

    struct itimerspec its;
    its.it_value.tv_sec = 0;
    its.it_value.tv_nsec = MOTOR_PERIOD_MS * 1000000;
    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = MOTOR_PERIOD_MS * 1000000;

    if (timerfd_settime(fd, 0, &its, NULL) == -1)
    {
        perror("motor timerfd_settime");
        close(fd);
        return -1;
    }
    return fd;
}

/**
 * @brief 电机闭环控制线程主循环
 */
void motor_loop()
{
    struct sched_param sp;
    sp.sched_priority = 10;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    refresh_thread_info();

    int fd = init_timer_fd();
    if (fd == -1)
    {
        return;
    }

    uint64_t expirations;
    while (g_motor_running.load())
    {
        if (read(fd, &expirations, sizeof(expirations)) != (ssize_t)sizeof(expirations))
        {
            perror("motor timer read");
            break;
        }

        const uint32 elapsed_periods = (expirations > 0U) ? (uint32)expirations : 1U;
        motor_driver.update_5ms(elapsed_periods);

        float target_l_count, target_r_count;
        {
            std::lock_guard<std::mutex> lock(g_target_mutex);
            target_l_count = g_target_left_count;
            target_r_count = g_target_right_count;
        }

        const float left_raw_count = motor_driver.left_count_5ms();
        const float right_raw_count = motor_driver.right_count_5ms();

        MotorSpeedControlState control_state =
            count_pid.compute(target_l_count, target_r_count,
                              left_raw_count,
                              right_raw_count);

        if (g_reload_from_globals_requested.exchange(false))
        {
            count_pid.init(MotorSpeedPidController::default_config((float)MOTOR_MAX_DUTY_PERCENT));
        }

        motor_driver.set_left_duty(control_state.left_duty);
        motor_driver.set_right_duty(control_state.right_duty);

        g_left_raw_count.store(left_raw_count);
        g_right_raw_count.store(right_raw_count);
        g_left_filtered_count.store(control_state.left_feedback);
        g_right_filtered_count.store(control_state.right_feedback);
        g_left_error.store(control_state.left_error);
        g_right_error.store(control_state.right_error);
        g_left_feedforward.store(control_state.left_feedforward);
        g_right_feedforward.store(control_state.right_feedforward);
        g_left_correction.store(control_state.left_correction);
        g_right_correction.store(control_state.right_correction);
        g_left_decel_assist.store(control_state.left_decel_assist);
        g_right_decel_assist.store(control_state.right_decel_assist);
        g_left_duty.store(control_state.left_duty);
        g_right_duty.store(control_state.right_duty);
        g_left_hardware_duty.store(motor_driver.left_hardware_duty());
        g_right_hardware_duty.store(motor_driver.right_hardware_duty());
        g_left_dir_level.store(motor_driver.left_dir_level());
        g_right_dir_level.store(motor_driver.right_dir_level());
    }

    close(fd);

    motor_driver.stop_all();
}
}

bool motor_thread_init()
{
    if (g_motor_running.load())
    {
        return true;
    }

    motor_driver.init();
    brushless_driver.init();
    count_pid.init(MotorSpeedPidController::default_config((float)MOTOR_MAX_DUTY_PERCENT));

    brushless_driver.set_left_duty(0);
    brushless_driver.set_right_duty(0);

    g_motor_running = true;
    g_motor_thread = std::thread(motor_loop);

    return true;
}

void motor_thread_set_target_count(float left_count, float right_count)
{
    std::lock_guard<std::mutex> lock(g_target_mutex);
    g_target_left_count = std::clamp(left_count, TARGET_COUNT_MIN, TARGET_COUNT_MAX);
    g_target_right_count = std::clamp(right_count, TARGET_COUNT_MIN, TARGET_COUNT_MAX);
}

float motor_thread_left_count()
{
    return g_left_raw_count.load();
}

float motor_thread_right_count()
{
    return g_right_raw_count.load();
}

float motor_thread_left_filtered_count()
{
    return g_left_filtered_count.load();
}

float motor_thread_right_filtered_count()
{
    return g_right_filtered_count.load();
}

float motor_thread_left_error()
{
    return g_left_error.load();
}

float motor_thread_right_error()
{
    return g_right_error.load();
}

float motor_thread_left_feedforward()
{
    return g_left_feedforward.load();
}

float motor_thread_right_feedforward()
{
    return g_right_feedforward.load();
}

float motor_thread_left_correction()
{
    return g_left_correction.load();
}

float motor_thread_right_correction()
{
    return g_right_correction.load();
}

float motor_thread_left_decel_assist()
{
    return g_left_decel_assist.load();
}

float motor_thread_right_decel_assist()
{
    return g_right_decel_assist.load();
}

float motor_thread_left_duty()
{
    return g_left_duty.load();
}

float motor_thread_right_duty()
{
    return g_right_duty.load();
}

float motor_thread_left_hardware_duty()
{
    return g_left_hardware_duty.load();
}

float motor_thread_right_hardware_duty()
{
    return g_right_hardware_duty.load();
}

int motor_thread_left_dir_level()
{
    return g_left_dir_level.load();
}

int motor_thread_right_dir_level()
{
    return g_right_dir_level.load();
}

float motor_thread_left_target_count()
{
    std::lock_guard<std::mutex> lock(g_target_mutex);
    return g_target_left_count;
}

float motor_thread_right_target_count()
{
    std::lock_guard<std::mutex> lock(g_target_mutex);
    return g_target_right_count;
}

MotorUartStatus motor_thread_uart_status()
{
    MotorUartStatus status;

    {
        std::lock_guard<std::mutex> lock(g_target_mutex);
        status.left_target_count = g_target_left_count;
        status.right_target_count = g_target_right_count;
    }

    status.left_feedback = g_left_filtered_count.load();
    status.right_feedback = g_right_filtered_count.load();
    status.left_error = g_left_error.load();
    status.right_error = g_right_error.load();
    status.left_feedforward = g_left_feedforward.load();
    status.right_feedforward = g_right_feedforward.load();
    status.left_correction = g_left_correction.load();
    status.right_correction = g_right_correction.load();
    status.left_decel_assist = g_left_decel_assist.load();
    status.right_decel_assist = g_right_decel_assist.load();
    status.left_current_count = g_left_raw_count.load();
    status.right_current_count = g_right_raw_count.load();
    status.left_duty = g_left_duty.load();
    status.right_duty = g_right_duty.load();
    status.left_hardware_duty = g_left_hardware_duty.load();
    status.right_hardware_duty = g_right_hardware_duty.load();
    status.left_dir_level = g_left_dir_level.load();
    status.right_dir_level = g_right_dir_level.load();
    return status;
}

bool motor_thread_get_pid_params(MotorPidParams &params)
{
    DualPidParams pid_params;
    count_pid.get_pid_params(pid_params);

    params.left_kp = pid_params.left_kp;
    params.left_ki = pid_params.left_ki;
    params.left_kd = pid_params.left_kd;
    params.right_kp = pid_params.right_kp;
    params.right_ki = pid_params.right_ki;
    params.right_kd = pid_params.right_kd;
    return true;
}

bool motor_thread_set_pid_params(const MotorPidParams &params)
{
    DualPidParams pid_params;
    pid_params.left_kp = params.left_kp;
    pid_params.left_ki = params.left_ki;
    pid_params.left_kd = params.left_kd;
    pid_params.right_kp = params.right_kp;
    pid_params.right_ki = params.right_ki;
    pid_params.right_kd = params.right_kd;
    return count_pid.set_pid_params(pid_params);
}

bool motor_thread_get_pid_debug_status(MotorPidDebugStatus &status)
{
    MotorSpeedPidDebugInfo info{};
    count_pid.get_debug_info(info);

    status.params.left_kp = info.left_kp;
    status.params.left_ki = info.left_ki;
    status.params.left_kd = info.left_kd;
    status.params.right_kp = info.right_kp;
    status.params.right_ki = info.right_ki;
    status.params.right_kd = info.right_kd;
    status.integral_limit = info.integral_limit;
    status.max_output_step = info.max_output_step;
    status.correction_limit = info.correction_limit;
    status.duty_limit = info.duty_limit;
    status.left_feedforward_gain = info.left_feedforward_gain;
    status.right_feedforward_gain = info.right_feedforward_gain;
    status.left_feedforward_bias = info.left_feedforward_bias;
    status.right_feedforward_bias = info.right_feedforward_bias;
    status.feedforward_bias_threshold = info.feedforward_bias_threshold;
    status.decel_error_threshold = info.decel_error_threshold;
    status.decel_duty_gain = info.decel_duty_gain;
    status.decel_duty_limit = info.decel_duty_limit;
    status.left_pid_target = info.left_pid_target;
    status.right_pid_target = info.right_pid_target;
    status.left_pid_error = info.left_pid_error;
    status.right_pid_error = info.right_pid_error;
    status.left_pid_output = info.left_pid_output;
    status.right_pid_output = info.right_pid_output;
    status.left_pid_output_min = info.left_pid_output_min;
    status.right_pid_output_min = info.right_pid_output_min;
    status.left_pid_output_max = info.left_pid_output_max;
    status.right_pid_output_max = info.right_pid_output_max;
    status.left_pid_integral_limit = info.left_pid_integral_limit;
    status.right_pid_integral_limit = info.right_pid_integral_limit;
    status.left_pid_max_output_step = info.left_pid_max_output_step;
    status.right_pid_max_output_step = info.right_pid_max_output_step;
    status.runtime = motor_thread_uart_status();
    return true;
}

void motor_thread_request_reload_from_globals()
{
    g_reload_from_globals_requested.store(true);
}

void motor_thread_cleanup()
{
    if (!g_motor_running.load())
    {
        return;
    }

    g_motor_running = false;

    if (g_motor_thread.joinable())
    {
        g_motor_thread.join();
    }

    motor_driver.stop_all();
    brushless_driver.stop_all();
}

void motor_thread_print_info()
{
    int32 tid = g_thread_tid.load();

    if (0 < tid)
    {
        printf("thread=%s tid=%d policy=%s priority=%d\r\n",
               "motor_ctrl",
               tid,
               sched_policy_name(g_thread_policy.load()),
               g_thread_priority.load());
    }
    else
    {
        printf("thread=%s tid=unknown policy=unknown priority=unknown\r\n",
               "motor_ctrl");
    }
}

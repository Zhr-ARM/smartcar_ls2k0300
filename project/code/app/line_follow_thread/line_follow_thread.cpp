#include "line_follow_thread.h"

#include "driver/vision/vision_config.h"
#include "driver/vision/vision_image_processor.h"
#include "motor_thread.h"
#include "pid.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <pthread.h>
#include <sys/syscall.h>
#include <thread>

namespace
{
// 控制周期：10ms(100Hz)。巡线环频率高于电机速度环(5ms)的整数倍，能持续给出平滑转向目标。
constexpr int32 LINE_FOLLOW_PERIOD_MS = 20;
// 调度优先级：巡线线程作为中高优先级实时任务执行。
constexpr int32 LINE_FOLLOW_THREAD_PRIORITY = 8;

std::thread g_line_follow_thread;
std::atomic<bool> g_line_follow_running(false);
std::atomic<int32> g_thread_tid(0);
std::atomic<int32> g_thread_policy(0);
std::atomic<int32> g_thread_priority(0);
std::atomic<float> g_base_speed(500.0f);
std::atomic<float> g_line_error_px(0.0f);
std::atomic<float> g_turn_output(0.0f);

// 滤波后的归一化误差状态，跨周期保留。
// 这里刻意保留“状态记忆”，因为巡线不是单次运算，而是连续控制。
float g_filtered_error = 0.0f;
// 误差降速后的基础速度状态：把“理想基础速度”做成缓增缓降，避免一帧一跳。
float g_adjusted_base_speed_state = -1.0f;

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
 * @brief 巡线控制线程主循环
 *
 * 控制链路：
 * 1) 视觉模块先给出采样行处的赛道中线偏差 line_error（像素）；
 * 2) 巡线线程统一方向约定，并把误差归一化、限幅；
 * 3) 通过低通滤波、死区、小误差降增益把视觉噪声整形成“可控误差”；
 * 4) 位置环 PID 把横向误差变成转向差速；
 * 5) 再叠加基础速度，并在大弯时主动降速，最终下发左右轮目标转速。
 */
void line_follow_loop()
{
    struct sched_param sp;
    sp.sched_priority = LINE_FOLLOW_THREAD_PRIORITY;
    if (0 != pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp))
    {
        printf("line_follow set sched failed, fallback to current policy\r\n");
    }

    refresh_thread_info();

    while (g_line_follow_running.load())
    {
        // base_speed 相当于“直道巡航速度”。
        // 真正送到电机线程的值，还会被后面的弯道降速逻辑二次修正。
        const float current_base_speed = g_base_speed.load();

        // 误差来源也随基准速度策略模式切换：
        // mode=0 使用当前 line_error；
        // mode=1 使用视觉层计算的加权积分偏差。
        float selected_offset_error = static_cast<float>(line_error);
        if (pid_tuning::line_follow::kBaseSpeedPlanMode == 1)
        {
            float weighted_error = 0.0f;
            bool lookahead_point_valid = false;
            int lookahead_point_x = 0;
            int lookahead_point_y = 0;
            vision_image_processor_get_ipm_curvature_weighted_error_debug(&weighted_error,
                                                                           &lookahead_point_valid,
                                                                           &lookahead_point_x,
                                                                           &lookahead_point_y);
            selected_offset_error = weighted_error;
        }

        // 方向约定：正偏差表示中线偏右（x_ref右侧），与控制器内部正方向相反，统一取负。
        const float raw_error_px = -selected_offset_error;

        // 以半幅宽度归一化到近似[-1,1]范围，并对异常值做保护限幅。
        // 这样 PID 参数可以脱离具体分辨率，后续改摄像头宽度时更容易复用。
        const float normalized_error = std::clamp(
            raw_error_px / ((float)VISION_DOWNSAMPLED_WIDTH * 0.5f),
            -pid_tuning::line_follow::kNormalizedErrorLimit,
            pid_tuning::line_follow::kNormalizedErrorLimit);

        // 一阶 IIR 滤波：抑制视觉噪声，避免转向输出高频抖动。
        // 这一步相当于告诉控制器：“相信趋势，不要被单帧跳动牵着走”。
        g_filtered_error =
            g_filtered_error * (1.0f - pid_tuning::line_follow::kErrorFilterAlpha) +
            normalized_error * pid_tuning::line_follow::kErrorFilterAlpha;

        // 后面的 deadzone / 小误差降增益继续使用像素尺度判断，
        // 是为了让阈值更直观，便于你结合画面观察实际偏差量。
        const float filtered_error_px = g_filtered_error * ((float)VISION_DOWNSAMPLED_WIDTH * 0.5f);
        const float abs_filtered_error_px = std::fabs(filtered_error_px);
        float control_error = g_filtered_error;

        // 先做死区：图像中心附近的轻微抖动直接忽略，避免小车左右来回“抽动”。
        if (std::fabs(filtered_error_px) < pid_tuning::line_follow::kErrorDeadzonePx)
        {
            control_error = 0.0f;
        }
        // 再做小误差降增益：还没到需要强修正的时候，就先温和拉回，提升直道稳定性。
        else if (std::fabs(filtered_error_px) < pid_tuning::line_follow::kErrorLowGainLimitPx)
        {
            control_error *= pid_tuning::line_follow::kErrorLowGain;
        }

        // 位置式 PID 输出“差速量”而不是“绝对速度”。
        // 输出越大，说明需要更激烈地拉开左右轮速度差来完成回正。
        float steering_output = position_pid1.compute_by_error(control_error);
        steering_output = std::clamp(steering_output,
                                     -pid_tuning::line_follow::kPidMaxOutput,
                                     pid_tuning::line_follow::kPidMaxOutput);

        // 基准速度规划策略切换：
        // mode=0 使用当前误差降速机制；
        // mode=1 使用视觉层提供的曲率/前瞻加权速度建议。
        float desired_base_speed = current_base_speed;
        bool force_full_speed = false;
        if (pid_tuning::line_follow::kBaseSpeedPlanMode == 0)
        {
            const float mean_abs_path_error = vision_image_processor_ipm_mean_abs_offset_error();
            // 若整条路径绝对偏差均值很小，认为处于稳定直道，直接给满基础速度。
            if (mean_abs_path_error < 2.0f)
            {
                desired_base_speed = current_base_speed;
                force_full_speed = true;
            }

            // 大弯主动降基础速度：
            // 如果还按直道速度冲，内外轮目标会跳得很大，速度环很容易跟不上，车体就会发飘。
            if (!force_full_speed)
            {
                float speed_scale = 1.0f;
                if (abs_filtered_error_px > pid_tuning::line_follow::kTurnSlowdownStartPx)
                {
                    const float slowdown_ratio = std::clamp(
                        (abs_filtered_error_px - pid_tuning::line_follow::kTurnSlowdownStartPx) /
                        (pid_tuning::line_follow::kTurnSlowdownFullPx - pid_tuning::line_follow::kTurnSlowdownStartPx),
                        0.0f,
                        1.0f);
                    speed_scale = 1.0f - slowdown_ratio * (1.0f - pid_tuning::line_follow::kTurnMinSpeedScale);
                }
                desired_base_speed = current_base_speed * speed_scale;
            }
        }
        else
        {
            float kappa_max = 0.0f;
            float delta_kappa_max = 0.0f;
            float curve_base_speed = 0.0f;
            float v_curve_raw = 0.0f;
            float v_curve_after_dkappa = 0.0f;
            float v_error_limit = 0.0f;
            float v_target = 0.0f;
            vision_image_processor_get_ipm_curvature_speed_limit_debug(&kappa_max,
                                                                       &delta_kappa_max,
                                                                       &curve_base_speed,
                                                                       &v_curve_raw,
                                                                       &v_curve_after_dkappa,
                                                                       &v_error_limit,
                                                                       &v_target);
            if (v_target > 0.0f)
            {
                desired_base_speed = v_target;
            }
        }

        if (g_adjusted_base_speed_state < 0.0f)
        {
            g_adjusted_base_speed_state = desired_base_speed;
        }
        else if (force_full_speed)
        {
            // 满速直通分支：本周期直接拉到基础速度上限，不再走缓升限制。
            g_adjusted_base_speed_state = current_base_speed;
        }
        else
        {
            const float max_drop = std::max(0.0f,
                                            g_adjusted_base_speed_state *
                                            pid_tuning::line_follow::kTurnSlowdownMaxDropRatioPerCycle);
            const float max_rise = std::max(0.0f,
                                            g_adjusted_base_speed_state *
                                            pid_tuning::line_follow::kTurnSlowdownMaxRiseRatioPerCycle);
            const float min_allowed = g_adjusted_base_speed_state - max_drop;
            const float max_allowed = g_adjusted_base_speed_state + max_rise;
            g_adjusted_base_speed_state = std::clamp(desired_base_speed, min_allowed, max_allowed);
        }
        const float adjusted_base_speed = std::clamp(g_adjusted_base_speed_state,
                                                     pid_tuning::line_follow::kTargetCountMin,
                                                     pid_tuning::line_follow::kTargetCountMax);

        // 差速映射：左轮=base-out，右轮=base+out。
        // steering_output 为正时，右轮更快、左轮更慢，车体会朝左修正；
        // steering_output 为负时，左轮更快、右轮更慢，车体会朝右修正。
        //
        // 这里不再把内轮硬限制在 0，而是允许轻微反转，缩短大误差区的滞留时间，
        // 也就是允许车辆在很偏的时候更“果断”地拧回来。
        const float left_target = std::clamp(adjusted_base_speed - steering_output,
                                             pid_tuning::line_follow::kTargetCountMin,
                                             pid_tuning::line_follow::kTargetCountMax);
        const float right_target = std::clamp(adjusted_base_speed + steering_output,
                                              pid_tuning::line_follow::kTargetCountMin,
                                              pid_tuning::line_follow::kTargetCountMax);
        // 重新用限幅后的左右轮目标反推实际差速，保证对外上报值和真正下发给电机的一致。
        const float applied_steering_output = (right_target - left_target) * 0.5f;
        motor_thread_set_target_count(left_target, right_target);

        // 对外发布关键调试量，供屏显/上位机读取。
        // 这样你在调试时能同时看到：看到了多大误差、实际打了多少差速、左右轮目标是多少。
        g_line_error_px.store(raw_error_px);
        g_turn_output.store(applied_steering_output);

        system_delay_ms(LINE_FOLLOW_PERIOD_MS);
    }
}
}

bool line_follow_thread_init()
{
    if (g_line_follow_running.load())
    {
        return true;
    }

    g_thread_tid = 0;
    g_thread_policy = 0;
    g_thread_priority = 0;
    g_line_error_px.store(0.0f);
    g_turn_output.store(0.0f);
    g_filtered_error = 0.0f;
    g_adjusted_base_speed_state = -1.0f;

    // 目标固定为 0：希望赛道中线最终回到图像中心，也就是“小车正对赛道”。
    position_pid1.init(pid_tuning::line_follow::kPidKp,
                       pid_tuning::line_follow::kPidKi,
                       pid_tuning::line_follow::kPidKd,
                       0.0f,
                       pid_tuning::line_follow::kPidMaxOutput);
    position_pid1.set_target(0.0f);

    g_line_follow_running = true;
    g_line_follow_thread = std::thread(line_follow_loop);
    return true;
}

void line_follow_thread_cleanup()
{
    if (!g_line_follow_running.load())
    {
        return;
    }

    // 先停循环，再join，保证线程退出与资源回收时序确定。
    g_line_follow_running = false;

    if (g_line_follow_thread.joinable())
    {
        g_line_follow_thread.join();
    }

    // 清掉 PID 和滤波残留，避免下次启动时把上一次的“方向记忆”带进来。
    position_pid1.reset();
    g_line_error_px.store(0.0f);
    g_turn_output.store(0.0f);
    g_filtered_error = 0.0f;
    g_adjusted_base_speed_state = -1.0f;
}

void line_follow_thread_print_info()
{
    const int32 tid = g_thread_tid.load();

    if (0 < tid)
    {
        printf("thread=%s tid=%d policy=%s priority=%d\r\n",
               "line_follow",
               tid,
               sched_policy_name(g_thread_policy.load()),
               g_thread_priority.load());
    }
    else
    {
        printf("thread=%s tid=unknown policy=unknown priority=unknown\r\n",
               "line_follow");
    }
}

void line_follow_thread_set_base_speed(float speed)
{
    g_base_speed.store(speed);
}

float line_follow_thread_error()
{
    return g_line_error_px.load();
}

float line_follow_thread_turn_output()
{
    return g_turn_output.load();
}

float line_follow_thread_base_speed()
{
    return g_base_speed.load();
}

float line_follow_thread_adjusted_base_speed()
{
    if (g_adjusted_base_speed_state >= 0.0f)
    {
        return g_adjusted_base_speed_state;
    }
    return g_base_speed.load();
}

#include "line_follow_thread.h"

#include "imu_thread.h"
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
// 控制周期：1ms(1000Hz)。巡线环频率高于电机速度环(5ms)的整数倍，能持续给出平滑转向目标。
constexpr int32 LINE_FOLLOW_PERIOD_MS = 1;
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
// 滤波后的横摆角速度状态：把 IMU 的瞬时抖动再收一层，减轻差速来回抽动。
float g_filtered_yaw_rate_dps = 0.0f;
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
    int policy = 0; // 获取到的当前线程调度策略
    struct sched_param param; // 获取到的当前线程调度参数

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
    struct sched_param sp; // 用于设置当前线程调度的参数结构体
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
        const float current_base_speed = g_base_speed.load(); // 当前设定的基础巡航速度
        // 约定正号与 steering_output 同向：正值表示车体正朝“左修正”方向旋转。
        const float measured_yaw_rate_dps = // 测量得到的当前横摆角速度(度/每秒)
            imu_thread_gyro_z_dps() * pid_tuning::line_follow::kGyroYawRateSign;
        g_filtered_yaw_rate_dps =
            g_filtered_yaw_rate_dps * (1.0f - pid_tuning::line_follow::kGyroYawRateFilterAlpha) +
            measured_yaw_rate_dps * pid_tuning::line_follow::kGyroYawRateFilterAlpha;
        const float abs_yaw_rate_dps = std::fabs(g_filtered_yaw_rate_dps); // 滤波后横摆角速度的绝对值

        // 偏差来源固定使用 line_error（方案A）。
        const float selected_offset_error = static_cast<float>(line_error); // 选定的赛道偏置误差(像素)

        // 方向约定：正偏差表示中线偏右（x_ref右侧），与控制器内部正方向相反，统一取负。
        const float raw_error_px = -selected_offset_error; // 统一方向后的原始偏差像素值(正表示偏右)

        // 以半幅宽度归一化到近似[-1,1]范围，并对异常值做保护限幅。
        // 这样 PID 参数可以脱离具体分辨率，后续改摄像头宽度时更容易复用。
        const float normalized_error = std::clamp( // 归一化后的中心偏差，范围近似[-1, 1]
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
        const float filtered_error_px = g_filtered_error * ((float)VISION_DOWNSAMPLED_WIDTH * 0.5f); // 滤波后的误差转换为像素值，用于直观判断
        const float abs_filtered_error_px = std::fabs(filtered_error_px); // 滤波后误差的绝对值像素大小
        float control_error = g_filtered_error; // 最终用于PID计算的控制误差量

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

        // 动态 Kp：误差越大，比例增益越强。
        const float error_sq = control_error * control_error; // 控制误差的平方，用于计算动态Kp
        const float dynamic_kp = std::clamp( // 计算得到的本次动态比例增益系数
            pid_tuning::line_follow::kDynamicKpBase +
            pid_tuning::line_follow::kDynamicKpQuadA * error_sq,
            pid_tuning::line_follow::kDynamicKpMin,
            pid_tuning::line_follow::kDynamicKpMax);
        position_pid1.set_params(dynamic_kp,
                                 pid_tuning::line_follow::kPidKi,
                                 pid_tuning::line_follow::kPidKd);

        // 位置式 PID 输出“差速量”而不是“绝对速度”。
        // 输出越大，说明需要更激烈地拉开左右轮速度差来完成回正。
        float steering_output = position_pid1.compute_by_error(control_error); // PID计算得出的基础转向差速输出量
        steering_output = std::clamp(steering_output,
                                     -pid_tuning::line_follow::kPidMaxOutput,
                                     pid_tuning::line_follow::kPidMaxOutput);

        // 视觉位置环先决定“该往哪边修、修多猛”，再把它映射成期望横摆角速度。
        // 如果实际横摆率已经超过视觉希望的水平，就主动收舵/反打，抑制甩尾和过摆。
        const float yaw_rate_ref_dps = std::clamp( // 期望的横摆角速度(由转向输出量按增益映射)
            steering_output * pid_tuning::line_follow::kGyroYawRateRefGain,
            -pid_tuning::line_follow::kGyroYawRateRefLimitDps,
            pid_tuning::line_follow::kGyroYawRateRefLimitDps);
        const float yaw_rate_error_dps = yaw_rate_ref_dps - g_filtered_yaw_rate_dps; // 横摆角速度偏差(期望值与实际滤波值之差)
        const float gyro_tracking_output = std::clamp( // 陀螺仪横摆角速度跟踪补偿计算得出的转向附加量
            yaw_rate_error_dps * pid_tuning::line_follow::kGyroYawRateErrorGain,
            -pid_tuning::line_follow::kGyroCorrectionLimit,
            pid_tuning::line_follow::kGyroCorrectionLimit);

        float gyro_oversteer_output = 0.0f; // 陀螺仪检测到过摆时的反打收舵补偿量
        const bool yaw_same_direction_as_command = // 判断当前横摆方向是否与控制指令方向一致
            ((0.0f < g_filtered_yaw_rate_dps) && (0.0f < steering_output)) ||
            ((0.0f > g_filtered_yaw_rate_dps) && (0.0f > steering_output));
        if ((abs_yaw_rate_dps > pid_tuning::line_follow::kGyroOversteerStartDps) &&
            yaw_same_direction_as_command)
        {
            const float oversteer_excess_dps = // 超过过摆横摆角速度阈值的多余部分
                abs_yaw_rate_dps - pid_tuning::line_follow::kGyroOversteerStartDps;
            gyro_oversteer_output = -std::copysign(
                std::min(oversteer_excess_dps * pid_tuning::line_follow::kGyroOversteerGain,
                         pid_tuning::line_follow::kGyroOversteerLimit),
                g_filtered_yaw_rate_dps);
        }

        steering_output = std::clamp(steering_output + gyro_tracking_output + gyro_oversteer_output,
                                     -pid_tuning::line_follow::kPidMaxOutput,
                                     pid_tuning::line_follow::kPidMaxOutput);

        float desired_base_speed = current_base_speed; // 期望的目标基础速度(可能会经过弯道降速处理)
        bool force_full_speed = false; // 是否强制满速(即车身状态良好处于直道)的标志位
        const float mean_abs_path_error = vision_image_processor_ipm_mean_abs_offset_error(); // 视觉给出的整条分析路径平均绝对偏差像素
        // 若整条路径绝对偏差均值很小且车身横摆也稳定，认为处于直道，可直接给满基础速度。
        if ((mean_abs_path_error < 2.0f) &&
            (abs_yaw_rate_dps < pid_tuning::line_follow::kGyroStraightStableMaxDps))
        {
            desired_base_speed = current_base_speed;
            force_full_speed = true;
        }
        // 大弯主动降基础速度：
        // 如果还按直道速度冲，内外轮目标会跳得很大，速度环很容易跟不上，车体就会发飘。
        // 正常情况下只由视觉误差负责弯道降速；陀螺仪降速仅在“明显过摆”时兜底触发。
        if (!force_full_speed)
        {
            float vision_speed_scale = 1.0f; // 基于视觉误差计算出的弯道降速缩放比例(0~1)
            if (abs_filtered_error_px > pid_tuning::line_follow::kTurnSlowdownStartPx)
            {
                const float slowdown_ratio = std::clamp( // 视觉大误差降速进度的系数比率(0表明刚开始触发，1表明达最大降速门限)
                    (abs_filtered_error_px - pid_tuning::line_follow::kTurnSlowdownStartPx) /
                    (pid_tuning::line_follow::kTurnSlowdownFullPx - pid_tuning::line_follow::kTurnSlowdownStartPx),
                    0.0f,
                    1.0f);
                vision_speed_scale =
                    1.0f - slowdown_ratio * (1.0f - pid_tuning::line_follow::kTurnMinSpeedScale);
            }

            float speed_scale = vision_speed_scale; // 综合降速比例(包含了视觉和陀螺仪的限制)
            const float emergency_slowdown_start_dps = std::max( // 紧急降速的陀螺仪角速度有效触发阈值
                pid_tuning::line_follow::kGyroOversteerStartDps,
                std::fabs(yaw_rate_ref_dps) + pid_tuning::line_follow::kGyroEmergencySlowdownMarginDps);
            if (yaw_same_direction_as_command &&
                (abs_yaw_rate_dps > emergency_slowdown_start_dps))
            {
                const float emergency_slowdown_ratio = std::clamp( // 陀螺仪过大引起的紧急降速进度系数比率(0~1)
                    (abs_yaw_rate_dps - emergency_slowdown_start_dps) /
                    pid_tuning::line_follow::kGyroEmergencySlowdownRangeDps,
                    0.0f,
                    1.0f);
                const float gyro_emergency_speed_scale = // 陀螺仪严重过摆时要求的基础速度降速缩放比例
                    1.0f - emergency_slowdown_ratio *
                    (1.0f - pid_tuning::line_follow::kGyroEmergencySlowdownMinScale);
                speed_scale = std::min(speed_scale, gyro_emergency_speed_scale);
            }

            desired_base_speed = current_base_speed * speed_scale;
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
            const float max_drop = std::max(0.0f, // 单次控制周期基础速度允许的最大下降量
                                            g_adjusted_base_speed_state *
                                            pid_tuning::line_follow::kTurnSlowdownMaxDropRatioPerCycle);
            const float max_rise = std::max(0.0f, // 单次控制周期基础速度允许的最大上升量
                                            g_adjusted_base_speed_state *
                                            pid_tuning::line_follow::kTurnSlowdownMaxRiseRatioPerCycle);
            const float min_allowed = g_adjusted_base_speed_state - max_drop; // 经缓冲限制后本周期允许的基础速度下限
            const float max_allowed = g_adjusted_base_speed_state + max_rise; // 经缓冲限制后本周期允许的基础速度上限
            g_adjusted_base_speed_state = std::clamp(desired_base_speed, min_allowed, max_allowed);
        }
        const float adjusted_base_speed = std::clamp(g_adjusted_base_speed_state, // 经过所有降速和缓冲限幅后的最终下发基础速度
                                                     pid_tuning::line_follow::kTargetCountMin,
                                                     pid_tuning::line_follow::kTargetCountMax);

        // 差速映射：左轮=base-out，右轮=base+out。
        // steering_output 为正时，右轮更快、左轮更慢，车体会朝左修正；
        // steering_output 为负时，左轮更快、右轮更慢，车体会朝右修正。
        //
        // 这里不再把内轮硬限制在 0，而是允许轻微反转，缩短大误差区的滞留时间，
        // 也就是允许车辆在很偏的时候更“果断”地拧回来。
        const float left_target = std::clamp(adjusted_base_speed - steering_output, // 计算得到的电机左轮最终目标速度
                                             pid_tuning::line_follow::kTargetCountMin,
                                             pid_tuning::line_follow::kTargetCountMax);
        const float right_target = std::clamp(adjusted_base_speed + steering_output, // 计算得到的电机右轮最终目标速度
                                              pid_tuning::line_follow::kTargetCountMin,
                                              pid_tuning::line_follow::kTargetCountMax);
        // 重新用限幅后的左右轮目标反推实际差速，保证对外上报值和真正下发给电机的一致。
        const float applied_steering_output = (right_target - left_target) * 0.5f; // 反推出来并实际下放给电机的转向差速调整量
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
    g_filtered_yaw_rate_dps = 0.0f;
    g_adjusted_base_speed_state = -1.0f;

    // 目标固定为 0：希望赛道中线最终回到图像中心，也就是“小车正对赛道”。
    position_pid1.init(pid_tuning::line_follow::kDynamicKpBase,
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
    g_filtered_yaw_rate_dps = 0.0f;
    g_adjusted_base_speed_state = -1.0f;
}

void line_follow_thread_print_info()
{
    const int32 tid = g_thread_tid.load(); // 缓存加载的当前巡线线程系统ID

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

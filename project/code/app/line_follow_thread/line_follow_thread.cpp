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
// 注意：状态只在“拿到新视觉帧”时推进一次，不会在 1ms 空循环里重复吃旧帧。
float g_filtered_error = 0.0f;
// 滤波后的横摆角速度状态：把 IMU 的瞬时抖动再收一层，减轻差速来回抽动。
// 注意：状态只在“拿到新 IMU 样本”时推进一次。
float g_filtered_yaw_rate_dps = 0.0f;
// 滤波后的赛道曲率：由视觉路径给出，用来生成“该转多快”的目标横摆角速度。
// 注意：状态只在“拿到新视觉帧”时推进一次。
float g_filtered_path_curvature = 0.0f;
// 滤波后的目标点夹角：由跟踪点相对图像中垂线的偏转角得到。
// 注意：状态只在“拿到新视觉帧”时推进一次。
float g_filtered_track_point_angle_deg = 0.0f;
// 误差降速后的基础速度状态：把“理想基础速度”做成缓增缓降，避免一帧一跳。
float g_adjusted_base_speed_state = -1.0f;
// 最近一次已消费的 IMU / 视觉样本序号。
uint32 g_last_imu_sample_seq = 0;
uint32 g_last_vision_frame_seq = 0;

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

float compute_signed_track_point_angle_deg(bool track_valid, int track_x, int track_y)
{
    if (!track_valid)
    {
        return 0.0f;
    }

    const float dx = static_cast<float>(track_x - (VISION_IPM_WIDTH / 2));
    const float dy = static_cast<float>((VISION_IPM_HEIGHT - 1) - track_y);
    const float clamped_dy = std::max(dy, 1.0f);
    const float angle_rad = atan2f(-dx, clamped_dy);
    return angle_rad * (180.0f / 3.1415926f);
}

void update_filtered_yaw_rate_if_new_sample()
{
    const uint32 imu_sample_seq = imu_thread_gyro_z_sample_seq();
    if (imu_sample_seq == g_last_imu_sample_seq)
    {
        return;
    }
    g_last_imu_sample_seq = imu_sample_seq;

    const float measured_yaw_rate_dps =
        imu_thread_gyro_z_dps() * pid_tuning::imu::kGyroYawRateSign;
    g_filtered_yaw_rate_dps =
        g_filtered_yaw_rate_dps * (1.0f - pid_tuning::imu::kGyroYawRateFilterAlpha) +
        measured_yaw_rate_dps * pid_tuning::imu::kGyroYawRateFilterAlpha;
}

void update_filtered_vision_inputs_if_new_frame()
{
    const uint32 vision_frame_seq = vision_image_processor_processed_frame_seq();
    if (vision_frame_seq == g_last_vision_frame_seq)
    {
        return;
    }
    g_last_vision_frame_seq = vision_frame_seq;

    // 偏差来源固定使用 line_error（方案A）。
    const float selected_offset_error = static_cast<float>(line_error);
    // 方向约定：正偏差表示中线偏右（x_ref右侧），与控制器内部正方向相反，统一取负。
    const float raw_error_px = -selected_offset_error;
    g_line_error_px.store(raw_error_px);

    // 以半幅宽度归一化到近似[-1,1]范围，并对异常值做保护限幅。
    // 这样 PID 参数可以脱离具体分辨率，后续改摄像头宽度时更容易复用。
    const float normalized_error = std::clamp(
        raw_error_px / ((float)VISION_DOWNSAMPLED_WIDTH * 0.5f),
        -pid_tuning::line_follow::kNormalizedErrorLimit,
        pid_tuning::line_follow::kNormalizedErrorLimit);

    // 一阶 IIR 滤波只在“新视觉帧到来”时推进一次，避免在 1ms 循环里对旧帧重复滤波。
    g_filtered_error =
        g_filtered_error * (1.0f - pid_tuning::line_follow::kErrorFilterAlpha) +
        normalized_error * pid_tuning::line_follow::kErrorFilterAlpha;

    // 额外从视觉处理中读取“当前被 line_error 选中的那条中线曲率”。
    // 这样目标角速度就不再从位置环输出反推，而是直接由视觉路径本身给出。
    const float *selected_centerline_curvature = nullptr;
    int selected_curvature_count = 0;
    vision_image_processor_get_ipm_selected_centerline_curvature(&selected_centerline_curvature,
                                                                 &selected_curvature_count);
    const int track_index = vision_image_processor_ipm_line_error_track_index();
    float current_path_curvature = 0.0f;
    if ((nullptr != selected_centerline_curvature) &&
        (0 <= track_index) &&
        (track_index < selected_curvature_count))
    {
        current_path_curvature = selected_centerline_curvature[track_index];
    }
    g_filtered_path_curvature =
        g_filtered_path_curvature * (1.0f - pid_tuning::yaw_rate_loop::kVisualCurvatureFilterAlpha) +
        current_path_curvature * pid_tuning::yaw_rate_loop::kVisualCurvatureFilterAlpha;

    bool track_point_valid = false;
    int track_point_x = 0;
    int track_point_y = 0;
    vision_image_processor_get_ipm_line_error_track_point(&track_point_valid, &track_point_x, &track_point_y);
    const float current_track_point_angle_deg =
        compute_signed_track_point_angle_deg(track_point_valid, track_point_x, track_point_y);
    g_filtered_track_point_angle_deg =
        g_filtered_track_point_angle_deg * (1.0f - pid_tuning::yaw_rate_loop::kTrackPointAngleFilterAlpha) +
        current_track_point_angle_deg * pid_tuning::yaw_rate_loop::kTrackPointAngleFilterAlpha;
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
 * 2) 巡线线程只在“拿到新视觉帧 / 新 IMU 样本”时推进各自滤波状态；
 * 3) 再统一方向约定，并通过归一化、死区、小误差降增益把视觉噪声整形成“可控误差”；
 * 4) 位置环 PID 负责“回中线”，角速度环 PID 负责“按视觉期望横摆率转过去”；
 * 5) 两个环并级叠加成最终差速，再叠加基础速度与视觉弯道降速后下发左右轮目标。
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
        // 这些滤波器只在“数据源真的更新了”时推进一次；
        // 若当前 1ms 周期只是重复读到旧样本，就沿用上一份滤波状态，避免 alpha 被空转放大。
        update_filtered_yaw_rate_if_new_sample();
        update_filtered_vision_inputs_if_new_frame();

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
            pid_tuning::position_loop::kDynamicKpBase +
            pid_tuning::position_loop::kDynamicKpQuadA * error_sq,
            pid_tuning::position_loop::kDynamicKpMin,
            pid_tuning::position_loop::kDynamicKpMax);
        position_pid1.set_params(dynamic_kp,
                                 pid_tuning::position_loop::kKi,
                                 pid_tuning::position_loop::kKd);

        // ---------------- 并级控制核心 ----------------
        // 第一条支路：位置环 PID，只负责“把车拉回中线”。
        // 它盯的是横向误差 e，输出一份差速量。
        const float position_output = std::clamp(
            position_pid1.compute_by_error(control_error),
            -pid_tuning::position_loop::kMaxOutput,
            pid_tuning::position_loop::kMaxOutput);

        // 第二条支路：角速度环 PID。
        // 目标横摆角速度 r_ref 直接由视觉给出：
        // 1) 视觉误差越大，说明应该更快地把车头拧回去；
        // 2) 视觉曲率越大，说明前方弯更急，需要更早建立横摆速度。
        // 这里继续采用“位置式 PID”，因为角速度环输出的是一份独立差速量，
        // 直接限幅、直接和位置环并加，调试和观察都会比增量式更直接。
        float yaw_rate_ref_dps = 0.0f;
        const vision_yaw_rate_ref_mode_enum yaw_rate_ref_mode =
            static_cast<vision_yaw_rate_ref_mode_enum>(g_vision_runtime_config.yaw_rate_ref_mode);
        if (yaw_rate_ref_mode == VISION_YAW_RATE_REF_FROM_ERROR_AND_CURVATURE)
        {
            yaw_rate_ref_dps = std::clamp(
                control_error * pid_tuning::yaw_rate_loop::kRefFromErrorGainDps +
                g_filtered_path_curvature * pid_tuning::yaw_rate_loop::kRefFromCurvatureGainDps,
                -pid_tuning::yaw_rate_loop::kRefLimitDps,
                pid_tuning::yaw_rate_loop::kRefLimitDps);
        }
        else
        {
            yaw_rate_ref_dps = std::clamp(
                g_filtered_track_point_angle_deg * pid_tuning::yaw_rate_loop::kRefFromTrackPointAngleGainDps,
                -pid_tuning::yaw_rate_loop::kRefLimitDps,
                pid_tuning::yaw_rate_loop::kRefLimitDps);
        }
        const float yaw_rate_error_dps = yaw_rate_ref_dps - g_filtered_yaw_rate_dps; // 目标角速度与实际角速度之差
        position_pid2.set_params(pid_tuning::yaw_rate_loop::kKp,
                                 pid_tuning::yaw_rate_loop::kKi,
                                 pid_tuning::yaw_rate_loop::kKd);
        const float yaw_rate_output = std::clamp(
            position_pid2.compute_by_error(yaw_rate_error_dps),
            -pid_tuning::yaw_rate_loop::kMaxOutput,
            pid_tuning::yaw_rate_loop::kMaxOutput);

        // 并级的意思，就是两条支路独立算完后再直接相加：
        // - position_output 管“位置偏了多少”；
        // - yaw_rate_output 管“车身现在转得对不对”。
        float steering_output = std::clamp(position_output + yaw_rate_output,
                                           -pid_tuning::position_loop::kMaxOutput,
                                           pid_tuning::position_loop::kMaxOutput);

        float desired_base_speed = current_base_speed; // 期望的目标基础速度(可能会经过弯道降速处理)
        bool force_full_speed = false; // 是否强制满速(即车身状态良好处于直道)的标志位
        const float mean_abs_path_error = vision_image_processor_ipm_mean_abs_offset_error(); // 视觉给出的整条分析路径平均绝对偏差像素
        // 当前版本把“陀螺仪降速”整体拿掉，只保留视觉负责速度规划。
        // 若整条路径绝对偏差均值很小，认为处于稳定直道，可直接给满基础速度。
        if (mean_abs_path_error < 2.0f)
        {
            desired_base_speed = current_base_speed;
            force_full_speed = true;
        }
        // 大弯主动降基础速度：
        // 如果还按直道速度冲，内外轮目标会跳得很大，速度环很容易跟不上，车体就会发飘。
        // 这里刻意只保留视觉降速：速度慢一点由视觉决定，姿态稳不稳交给角速度环去控。
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

            desired_base_speed = current_base_speed * vision_speed_scale;
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
    g_filtered_path_curvature = 0.0f;
    g_filtered_track_point_angle_deg = 0.0f;
    g_adjusted_base_speed_state = -1.0f;
    g_last_imu_sample_seq = 0;
    g_last_vision_frame_seq = 0;

    // 位置环：目标固定为 0，表示希望赛道中线最终回到图像中心。
    position_pid1.init(pid_tuning::position_loop::kDynamicKpBase,
                       pid_tuning::position_loop::kKi,
                       pid_tuning::position_loop::kKd,
                       0.0f,
                       pid_tuning::position_loop::kMaxOutput);
    position_pid1.set_target(0.0f);
    // 角速度环：同样目标为 0，但真正计算时使用的是 compute_by_error(r_ref - r)。
    // 这里明确采用位置式 PID，把“目标角速度和实际角速度的差”直接变成一份差速补偿量。
    // 这样更符合它作为并级支路的角色，也更方便你后面直接看输出大小来调参。
    position_pid2.init(pid_tuning::yaw_rate_loop::kKp,
                       pid_tuning::yaw_rate_loop::kKi,
                       pid_tuning::yaw_rate_loop::kKd,
                       pid_tuning::yaw_rate_loop::kMaxIntegral,
                       pid_tuning::yaw_rate_loop::kMaxOutput);
    position_pid2.set_target(0.0f);

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
    position_pid2.reset();
    g_line_error_px.store(0.0f);
    g_turn_output.store(0.0f);
    g_filtered_error = 0.0f;
    g_filtered_yaw_rate_dps = 0.0f;
    g_filtered_path_curvature = 0.0f;
    g_filtered_track_point_angle_deg = 0.0f;
    g_adjusted_base_speed_state = -1.0f;
    g_last_imu_sample_seq = 0;
    g_last_vision_frame_seq = 0;
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

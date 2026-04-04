#include "zf_common_headfile.h"
#include "battery.h"
#include "uart_thread.h"
#include "motor_thread.h"
#include "imu_thread.h"
#include "line_follow_thread.h"
#include "vision_thread.h"
#include "screen_display_thread.h"
#include "driver/pid/pid_tuning.h"
#include "driver/vision/vision_assistant_udp.h"
#include "driver/vision/vision_config.h"
#include "driver/vision/vision_infer_async.h"
#include "driver/vision/vision_transport.h"
#include "driver/vision/vision_image_processor.h"

#include "brushless.h"
#include "motor.h"

volatile sig_atomic_t g_should_exit = 0;

namespace
{
bool g_cleanup_done = false;

void cleanup_once()
{
    if (g_cleanup_done)
    {
        return;
    }

    screen_display_thread_cleanup();
    line_follow_thread_cleanup();
    vision_thread_cleanup();
    vision_assistant_udp_cleanup();
    vision_transport_udp_cleanup();
    motor_thread_cleanup();
    imu_thread_cleanup();
    uart_thread_cleanup();
    g_cleanup_done = true;
}

void stop_all_motion_immediately()
{
    // 先把目标清零，避免清线程前还有旧的速度目标继续下发。
    motor_thread_set_target_count(0.0f, 0.0f);
    line_follow_thread_set_base_speed(0.0f);
    brushless_driver.stop_all();
}

void handle_low_battery_protection()
{
    printf("[BATTERY] low voltage triggered=%.2fV threshold=%.2fV -> stopping all control loops\r\n",
           static_cast<double>(battery_low_voltage_protection_filtered_voltage_v()),
           static_cast<double>(battery_low_voltage_protection_threshold_v()));

    stop_all_motion_immediately();
    cleanup_once();
    battery_low_voltage_protection_run_alarm_loop(&g_should_exit);
}
} // namespace

void sigint_handler(int signum) 
{
    (void)signum;
    g_should_exit = 1;
}

void cleanup()
{
    cleanup_once();
}

int main(int, char**) 
{
    // 注册SIGINT信号处理函数
    signal(SIGINT, sigint_handler);
    
    battery_low_voltage_protection_init();

    if (battery_low_voltage_protection_update())
    {
        handle_low_battery_protection();
        cleanup_once();
        return 0;
    }

    if (!imu_thread_init())
    {
        cleanup();
        return -1;
    }

    // 初始化逐飞助手独立 UDP 通道（不与网页端共用 IP/端口）。
    if (g_vision_runtime_config.assistant_udp_enabled)
    {
        if (!vision_assistant_udp_init(g_vision_runtime_config.assistant_server_ip,
                                       g_vision_runtime_config.assistant_server_port))
        {
            printf("[ASSISTANT_UDP] init failed ip=%s port=%u\r\n",
                   g_vision_runtime_config.assistant_server_ip,
                   static_cast<unsigned int>(g_vision_runtime_config.assistant_server_port));
        }
        else
        {
            printf("[ASSISTANT_UDP] ready=1 ip=%s port=%u\r\n",
                   g_vision_runtime_config.assistant_server_ip,
                   static_cast<unsigned int>(g_vision_runtime_config.assistant_server_port));
        }
    }
    else
    {
        printf("[ASSISTANT_UDP] disabled\r\n");
    }

    // 初始化 UDP/TCP 到电脑端的数据通道（UDP=视频，TCP=状态）。
    if (!vision_transport_udp_init(g_vision_runtime_config.udp_web_server_ip,
                                   g_vision_runtime_config.udp_web_video_port,
                                   g_vision_runtime_config.udp_web_meta_port))
    {
        printf("[UDP_WEB] init failed\r\n");
    }
    vision_transport_udp_set_enabled(g_vision_runtime_config.udp_web_enabled);      // UDP 视频发送开关
    vision_transport_udp_set_max_fps(g_vision_runtime_config.udp_web_max_fps);       // UDP 视频发送限频
    vision_transport_udp_set_tcp_enabled(g_vision_runtime_config.udp_web_tcp_enabled); // TCP 状态发送开关
    printf("[UDP_WEB] enabled=%d server=%s video=%u meta=%u fps=%u gray=%d rgb=%d profile=%d\r\n",
           vision_transport_udp_is_enabled() ? 1 : 0,
           g_vision_runtime_config.udp_web_server_ip,
           static_cast<unsigned int>(g_vision_runtime_config.udp_web_video_port),
           static_cast<unsigned int>(g_vision_runtime_config.udp_web_meta_port),
           static_cast<unsigned int>(vision_transport_udp_get_max_fps()),
           g_vision_runtime_config.udp_web_send_gray_jpeg ? 1 : 0,
           g_vision_runtime_config.udp_web_send_rgb_jpeg ? 1 : 0,
           g_vision_runtime_config.udp_web_data_profile);

    // 初始化 ncnn 模型；后续是否真正执行推理由配置中的 infer_enabled 控制。
    LQ_NCNN ncnn;
    bool ncnn_ready = vision_infer_init_default_model(ncnn);
    // 视觉线程初始化：统一主链（采集/处理/检测/推理/发送）。
    if (!vision_thread_init("/dev/video0", &ncnn, ncnn_ready))
    {
        cleanup();
        return -1;
    }

    // 利用这 2s 视觉预热窗口同步完成 IMU 静止零偏标定：
    // 1) 不额外增加总启动时间；
    // 2) 标定结束后再启动 IMU 后台采集线程，后续按 5ms 周期持续更新并发布 gyro_z。
    if (!imu_thread_calibrate_and_start(pid_tuning::imu::kStartupCalibrateDurationMs))
    {
        cleanup();
        return -1;
    }
    if (!line_follow_thread_init())
    {
        cleanup();
        return -1;
    }

    if (!motor_thread_init())
    {
        cleanup();
        return -1;
    }
    brushless_driver.set_left_duty(0);
    brushless_driver.set_right_duty(0);
    if (g_vision_runtime_config.screen_display_enabled && !screen_display_thread_init())
    {
        cleanup();
        return -1;
    }

    // 下发视觉配置参数（按“网页发送 / 视觉处理 / 偏差计算”分组）。
    // [网页发送]
    vision_thread_set_send_mode(static_cast<vision_thread_send_mode_enum>(g_vision_runtime_config.send_mode)); // 图像模式
    vision_thread_set_send_max_fps(g_vision_runtime_config.send_max_fps); // 客户端发送限频
    vision_thread_set_infer_enabled(g_vision_runtime_config.infer_enabled); // 推理开关
    vision_thread_set_client_sender_enabled(g_vision_runtime_config.client_sender_enabled); // 逐飞发送开关
    vision_thread_set_roi_capture_mode(g_vision_runtime_config.roi_capture_mode); // ROI 抓图开关

    // [视觉处理]
    vision_image_processor_set_maze_start_row(g_vision_runtime_config.maze_start_row); // 迷宫法起始搜索行
    vision_image_processor_set_undistort_enabled(g_vision_runtime_config.undistort_enabled); // 去畸变开关
    vision_image_processor_set_ipm_triangle_filter_enabled(g_vision_runtime_config.ipm_triangle_filter_enabled); // IPM三角滤波
    vision_image_processor_set_ipm_resample_enabled(g_vision_runtime_config.ipm_resample_enabled); // IPM等距采样开关
    vision_image_processor_set_ipm_resample_step_px(g_vision_runtime_config.ipm_resample_step_px); // IPM等距采样步长
    vision_image_processor_set_ipm_boundary_curvature_enabled(g_vision_runtime_config.ipm_boundary_curvature_enabled); // 边界曲率开关
    vision_image_processor_set_ipm_boundary_kappa_sample_spacing_cm(g_vision_runtime_config.ipm_boundary_kappa_sample_spacing_cm); // 边界kappa采样间距h(cm)
    vision_image_processor_set_ipm_boundary_angle_step(g_vision_runtime_config.ipm_boundary_angle_step); // 边界三点法夹角步长
    vision_image_processor_set_ipm_boundary_shift_distance_px(g_vision_runtime_config.ipm_boundary_shift_distance_px); // IPM边界法向平移距离
    vision_image_processor_set_ipm_centerline_postprocess_enabled(g_vision_runtime_config.ipm_centerline_postprocess_enabled); // 中线后处理总开关
    vision_image_processor_set_ipm_centerline_triangle_filter_enabled(g_vision_runtime_config.ipm_centerline_triangle_filter_enabled); // 中线三角滤波
    vision_image_processor_set_ipm_centerline_resample_enabled(g_vision_runtime_config.ipm_centerline_resample_enabled); // 中线等距采样
    vision_image_processor_set_ipm_centerline_resample_step_px(g_vision_runtime_config.ipm_centerline_resample_step_px); // 中线采样步长
    vision_image_processor_set_ipm_centerline_curvature_enabled(g_vision_runtime_config.ipm_centerline_curvature_enabled); // 中线曲率开关
    vision_image_processor_set_ipm_centerline_curvature_step(g_vision_runtime_config.ipm_centerline_curvature_step); // 中线曲率计算步长

    // [偏差计算]
    vision_image_processor_set_ipm_line_error_source(static_cast<vision_ipm_line_error_source_enum>(g_vision_runtime_config.ipm_line_error_source)); // line_error 中线来源
    vision_image_processor_set_ipm_line_error_method(static_cast<vision_ipm_line_error_method_enum>(g_vision_runtime_config.ipm_line_error_method)); // line_error 计算方法
    vision_image_processor_set_ipm_line_error_fixed_index(g_vision_runtime_config.ipm_line_error_fixed_index); // line_error 固定索引
    vision_image_processor_set_ipm_line_error_weighted_points(g_vision_runtime_config.ipm_line_error_point_indices,
                                                              g_vision_runtime_config.ipm_line_error_weights,
                                                              g_vision_runtime_config.ipm_line_error_weighted_point_count); // line_error 加权索引点
    vision_image_processor_set_ipm_line_error_speed_formula(g_vision_runtime_config.ipm_line_error_speed_k,
                                                            g_vision_runtime_config.ipm_line_error_speed_b); // line_error 随速度索引公式
    vision_image_processor_set_ipm_line_error_index_range(g_vision_runtime_config.ipm_line_error_index_min,
                                                          g_vision_runtime_config.ipm_line_error_index_max); // line_error 随速度索引区间

    printf("[VISION CFG] mode=%d max_fps=%u infer=%d client_send=%d screen=%d roi_capture=%d maze_row=%d undistort=%d ipm_tri=%d ipm_resample=%d ipm_boundary_kappa_en=%d ipm_step=%.2f ipm_kappa_h_cm=%.3f ipm_angle_step=%d ipm_shift=%.2f center_post=%d center_tri=%d center_resample=%d center_kappa_en=%d center_step=%.2f line_src=%d line_method=%d line_fixed_idx=%d line_weighted_cnt=%u line_speed_k=%.4f line_speed_b=%.2f line_idx_min=%d line_idx_max=%d\r\n",
           static_cast<int>(vision_thread_get_send_mode()),
           static_cast<unsigned int>(vision_thread_get_send_max_fps()),
           vision_thread_infer_enabled() ? 1 : 0,
           vision_thread_client_sender_enabled() ? 1 : 0,
           g_vision_runtime_config.screen_display_enabled ? 1 : 0,
           vision_thread_roi_capture_mode_enabled() ? 1 : 0,
           vision_image_processor_get_maze_start_row(),
           vision_image_processor_undistort_enabled() ? 1 : 0,
           vision_image_processor_ipm_triangle_filter_enabled() ? 1 : 0,
           vision_image_processor_ipm_resample_enabled() ? 1 : 0,
           vision_image_processor_ipm_boundary_curvature_enabled() ? 1 : 0,
           static_cast<double>(vision_image_processor_ipm_resample_step_px()),
           static_cast<double>(vision_image_processor_ipm_boundary_kappa_sample_spacing_cm()),
           vision_image_processor_ipm_boundary_angle_step(),
           static_cast<double>(vision_image_processor_ipm_boundary_shift_distance_px()),
           vision_image_processor_ipm_centerline_postprocess_enabled() ? 1 : 0,
           vision_image_processor_ipm_centerline_triangle_filter_enabled() ? 1 : 0,
           vision_image_processor_ipm_centerline_resample_enabled() ? 1 : 0,
           vision_image_processor_ipm_centerline_curvature_enabled() ? 1 : 0,
           static_cast<double>(vision_image_processor_ipm_centerline_resample_step_px()),
           static_cast<int>(vision_image_processor_ipm_line_error_source()),
           static_cast<int>(vision_image_processor_ipm_line_error_method()),
           vision_image_processor_ipm_line_error_fixed_index(),
           static_cast<unsigned int>(vision_image_processor_ipm_line_error_weighted_point_count()),
           static_cast<double>(g_vision_runtime_config.ipm_line_error_speed_k),
           static_cast<double>(g_vision_runtime_config.ipm_line_error_speed_b),
           g_vision_runtime_config.ipm_line_error_index_min,
           g_vision_runtime_config.ipm_line_error_index_max);

    uart_thread_init();

    // 巡线基础速度配置（控制线程会在此基础上叠加转向差速）。
    line_follow_thread_set_base_speed(700.0f);
    
    // 为motor_thread设置目标计数，单位 counts/5ms。
    // 固定左右轮目标值为 800，便于速度环调参。
    //motor_thread_set_target_count(600.0f, 600.0f);
    
    system_delay_ms(50);
    motor_thread_print_info();
    imu_thread_print_info();
    line_follow_thread_print_info();

    while(!g_should_exit)
    {   
        if (battery_low_voltage_protection_update())
        {
            handle_low_battery_protection();
            break;
        }

        system_delay_ms(battery_low_voltage_protection_check_period_ms());
    }

    battery_low_voltage_protection_silence_buzzer();
    if (g_should_exit)
    {
        printf("收到Ctrl+C,程序即将退出\n");
    }
    cleanup_once();
    return 0;
}

#include "zf_common_headfile.h"
#include "battery.h"
#include "uart_thread.h"
#include "motor_thread.h"
#include "imu_thread.h"
#include "line_follow_thread.h"
#include "vision_thread.h"

#if defined(ENABLE_E99_VISION_STACK)
#include "driver/vision/lq_ncnn.hpp"
#include "driver/vision/vision_ncnn.h"
#endif

#if defined(ENABLE_CAR_SCREEN_DISPLAY)
#include "screen_display_thread.h"
#endif

#include "brushless.h"
#include "motor.h"

#if defined(ENABLE_SEEKFREE_ASSISTANT_CAMERA_STREAM)
// 参考 E99_my_car：图传走 TCP 客户端发送到上位机。
static constexpr const char *kAssistantServerIp = "192.168.1.74";
static constexpr uint32 kAssistantServerPort = 8888;
#endif

// 图传调试配置：集中在这里修改。
// 可选模式：
// VISION_THREAD_SEND_BINARY         : 二值图（1bit打包）
// VISION_THREAD_SEND_GRAY           : 原始灰度图
// VISION_THREAD_SEND_RGB565         : 彩色纯图（不叠加）
// VISION_THREAD_SEND_IPM_RGB565     : 彩色逆透视图
// VISION_THREAD_SEND_IPM_EDGE_GRAY  : 逆透视边线图（黑底白线）
// VISION_THREAD_SEND_RGB565_OVERLAY : 彩色叠加调试图（中心线/边线）
static constexpr vision_thread_send_mode_enum kVisionSendMode = VISION_THREAD_SEND_RGB565;
// 图传发送上限帧率，0 表示不限速。
static constexpr uint32 kVisionSendMaxFps = 60;
#if defined(ENABLE_SEEKFREE_ASSISTANT_CAMERA_STREAM)
static constexpr bool kVisionAssistantStreamEnabled = true;
#endif

volatile sig_atomic_t g_should_exit = 0;

void sigint_handler(int signum) 
{
    (void)signum;
    g_should_exit = 1;
}

void cleanup()
{
#if defined(ENABLE_CAR_SCREEN_DISPLAY)
    screen_display_thread_cleanup();
#endif
    line_follow_thread_cleanup();
    vision_thread_cleanup();
    motor_thread_cleanup();
    imu_thread_cleanup();
    uart_thread_cleanup();
}

int main(int, char**) 
{
    // 注册SIGINT信号处理函数
    signal(SIGINT, sigint_handler);
    
    battery_monitor.init();

#if defined(ENABLE_E99_VISION_STACK)
    LQ_NCNN ncnn;
    bool ncnn_ready = vision_ncnn_init_default_model(ncnn);
#endif

#if defined(ENABLE_SEEKFREE_ASSISTANT_CAMERA_STREAM)
    if (tcp_client_init(kAssistantServerIp, kAssistantServerPort) == 0)
    {
        printf("tcp_client ok\r\n");
    }
    else
    {
        printf("tcp_client error\r\n");
        return -1;
    }

    seekfree_assistant_interface_init(tcp_client_send_data, tcp_client_read_data);
#endif

    // 初始化各线程
    // if (!motor_thread_init())
    // {
    //     cleanup();
    //     return -1;
    // }

    // if (!imu_thread_init())
    // {
    //     cleanup();
    //     return -1;
    // }

#if defined(ENABLE_E99_VISION_STACK)
    if (!vision_thread_init("/dev/video0", &ncnn, ncnn_ready))
    {
        cleanup();
        return -1;
    }
#else
    if (!vision_thread_init("/dev/video0"))
    {
        cleanup();
        return -1;
    }
#endif

    // if (!line_follow_thread_init())
    // {
    //     cleanup();
    //     return -1;
    // }

#if defined(ENABLE_CAR_SCREEN_DISPLAY)
    if (!screen_display_thread_init())
    {
        cleanup();
        return -1;
    }
#endif

    vision_thread_set_send_mode(kVisionSendMode);
    vision_thread_set_send_max_fps(kVisionSendMaxFps);

#if defined(ENABLE_SEEKFREE_ASSISTANT_CAMERA_STREAM)
    vision_thread_set_assistant_camera_stream(kVisionAssistantStreamEnabled);
    printf("[VISION SEND CFG] mode=%d max_fps=%u stream=%d\r\n",
           static_cast<int>(vision_thread_get_send_mode()),
           static_cast<unsigned int>(vision_thread_get_send_max_fps()),
           vision_thread_assistant_camera_stream_enabled() ? 1 : 0);
#else
    printf("[VISION SEND CFG] mode=%d max_fps=%u\r\n",
           static_cast<int>(vision_thread_get_send_mode()),
           static_cast<unsigned int>(vision_thread_get_send_max_fps()));
#endif

    // uart_thread_init();

    // line_follow_thread_set_base_speed(150.0f);
    
    // 为motor_thread设置目标计数，单位 counts/5ms。
    // 方便其他线程调用
    //motor_thread_set_target_count(1000.0f, 1000.0f);
    // brushless_driver.set_left_duty(0);
    // brushless_driver.set_right_duty(0);
    // system_delay_ms(50);
    // motor_thread_print_info();
    // imu_thread_print_info();
    // line_follow_thread_print_info();

    while(!g_should_exit)
    {   
        battery_monitor.update();

        // printf("\r\n");
        // printf("roll=%.2f  pitch=%.2f  yaw=%.2f\r\n",
        //        att.roll, att.pitch, att.yaw);
        // printf("battery=%.2f V\r\n", battery_monitor.voltage_v());
        // printf("target=%.1f count/5ms  left=%.1f count/5ms (duty=%d)  right=%.1f count/5ms (duty=%d)\r\n",
        //        motor_thread_left_target_count(),
        //        motor_thread_left_count(), motor_thread_left_duty(),
        //        motor_thread_right_count(), motor_thread_right_duty());
        // printf("\r\n");
        system_delay_ms(100);
    }

    printf("收到Ctrl+C,程序即将退出\n");
    cleanup();
    return 0;
}

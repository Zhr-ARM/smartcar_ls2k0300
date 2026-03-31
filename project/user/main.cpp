#include "zf_common_headfile.h"
#include "battery.h"
#include "uart_thread.h"
#include "motor_thread.h"
#include "imu_thread.h"
#include "line_follow_thread.h"
#include "vision_thread.h"
#include "screen_display_thread.h"
#include "driver/vision/vision_infer_async.h"
#include "driver/vision/vision_transport.h"
#include "driver/vision/vision_image_processor.h"

#include "brushless.h"
#include "motor.h"

// 图传调试配置：集中在这里修改。
// 可选模式（常用）：
// VISION_THREAD_SEND_BINARY : 二值图（黑白）
// VISION_THREAD_SEND_RGB565 : 彩色纯图（不画线）
// VISION_THREAD_SEND_GRAY   : 灰度图（画边线+中线）
// 视觉输出模式（会影响客户端发送、UDP网页图像源、屏显图像源）。
static constexpr vision_thread_send_mode_enum kVisionSendMode = VISION_THREAD_SEND_GRAY;
// 图传发送上限帧率，0 表示不限速。
static constexpr uint32 kVisionSendMaxFps = 60;
// 推理总开关：false 时关闭红色识别与 ncnn 推理链路。
static constexpr bool kVisionInferEnabled = false;
// 客户端发送开关：控制逐飞助手发送链路是否启用。
static constexpr bool kVisionClientSenderEnabled = false;
// 车载屏显示开关：true 时启动 screen_display_thread。
static constexpr bool kVisionScreenDisplayEnabled = false;
// UDP 网页图传开关：控制是否向电脑端发送视频帧。
static constexpr bool kVisionUdpWebEnabled = true;
// UDP 网页图传最大发送帧率，0 表示不限速。
static constexpr uint32 kVisionUdpWebMaxFps = 30;
// TCP 状态上报开关：控制是否向电脑端发送 JSON 状态数据。
static constexpr bool kVisionUdpWebTcpEnabled = true;
// 电脑端接收服务 IP（运行 vision_pc_receiver.py 的主机地址）。
static constexpr const char *kVisionUdpWebServerIp = "172.21.79.179";
// 电脑端 UDP 视频端口（需与 PC 接收端 --udp-port 一致）。
static constexpr uint16 kVisionUdpWebVideoPort = 10000;
// 电脑端 TCP 状态端口（需与 PC 接收端 --tcp-port 一致）。
static constexpr uint16 kVisionUdpWebMetaPort = 10001;
// 采图模式：检测到红色矩形后，每 1s 保存一次推理 ROI 彩图，共 20 张。
static constexpr bool kVisionRoiCaptureMode = false;
// 迷宫法左右起点搜索行，固定单行搜索。
// 建议范围 [1, 118]（160x120），值越大越靠近图像底部。
static constexpr int kVisionMazeStartRow = 100;

volatile sig_atomic_t g_should_exit = 0;

void sigint_handler(int signum) 
{
    (void)signum;
    g_should_exit = 1;
}

void cleanup()
{
    screen_display_thread_cleanup();
    line_follow_thread_cleanup();
    vision_thread_cleanup();
    vision_transport_udp_cleanup();
    motor_thread_cleanup();
    imu_thread_cleanup();
    uart_thread_cleanup();
}

int main(int, char**) 
{
    // 注册SIGINT信号处理函数
    signal(SIGINT, sigint_handler);
    
    battery_monitor.init();

    // 初始化 UDP/TCP 到电脑端的数据通道（UDP=视频，TCP=状态）。
    if (!vision_transport_udp_init(kVisionUdpWebServerIp, kVisionUdpWebVideoPort, kVisionUdpWebMetaPort))
    {
        printf("[UDP_WEB] init failed\r\n");
    }
    vision_transport_udp_set_enabled(kVisionUdpWebEnabled);      // UDP 视频发送开关
    vision_transport_udp_set_max_fps(kVisionUdpWebMaxFps);       // UDP 视频发送限频
    vision_transport_udp_set_tcp_enabled(kVisionUdpWebTcpEnabled); // TCP 状态发送开关
    printf("[UDP_WEB] enabled=%d server=%s video=%u meta=%u fps=%u\r\n",
           vision_transport_udp_is_enabled() ? 1 : 0,
           kVisionUdpWebServerIp,
           static_cast<unsigned int>(kVisionUdpWebVideoPort),
           static_cast<unsigned int>(kVisionUdpWebMetaPort),
           static_cast<unsigned int>(vision_transport_udp_get_max_fps()));

    // 初始化 ncnn 模型；后续是否真正执行推理由 kVisionInferEnabled 控制。
    LQ_NCNN ncnn;
    bool ncnn_ready = vision_infer_init_default_model(ncnn);
    // 视觉线程初始化：统一主链（采集/处理/检测/推理/发送）。
    if (!vision_thread_init("/dev/video0", &ncnn, ncnn_ready))
    {
        cleanup();
        return -1;
    }

    // 视觉线程预热等待，给相机与首帧处理留稳定时间。
    system_delay_ms(2000);
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

    if (kVisionScreenDisplayEnabled && !screen_display_thread_init())
    {
        cleanup();
        return -1;
    }

    // 下发视觉配置参数。
    vision_thread_set_send_mode(kVisionSendMode);                       // 图像模式
    vision_thread_set_send_max_fps(kVisionSendMaxFps);                 // 客户端发送限频
    vision_thread_set_infer_enabled(kVisionInferEnabled);              // 推理开关
    vision_thread_set_client_sender_enabled(kVisionClientSenderEnabled); // 逐飞发送开关
    vision_thread_set_roi_capture_mode(kVisionRoiCaptureMode);         // ROI 抓图开关
    vision_image_processor_set_maze_start_row(kVisionMazeStartRow);    // 迷宫法起始搜索行

    printf("[VISION CFG] mode=%d max_fps=%u infer=%d client_send=%d screen=%d roi_capture=%d maze_row=%d\r\n",
           static_cast<int>(vision_thread_get_send_mode()),
           static_cast<unsigned int>(vision_thread_get_send_max_fps()),
           vision_thread_infer_enabled() ? 1 : 0,
           vision_thread_client_sender_enabled() ? 1 : 0,
           kVisionScreenDisplayEnabled ? 1 : 0,
           vision_thread_roi_capture_mode_enabled() ? 1 : 0,
           vision_image_processor_get_maze_start_row());

    uart_thread_init();

    // 巡线基础速度配置（控制线程会在此基础上叠加转向差速）。
    line_follow_thread_set_base_speed(50.0f);
    
    // 为motor_thread设置目标计数，单位 counts/5ms。
    // 固定左右轮目标值为 800，便于速度环调参。
    //motor_thread_set_target_count(600.0f, 600.0f);
    brushless_driver.set_left_duty(0);
    brushless_driver.set_right_duty(0);
    system_delay_ms(50);
    motor_thread_print_info();
    //imu_thread_print_info();
    line_follow_thread_print_info();

    while(!g_should_exit)
    {   
        battery_monitor.update();
        system_delay_ms(1000);
    }

    printf("收到Ctrl+C,程序即将退出\n");
    cleanup();
    return 0;
}

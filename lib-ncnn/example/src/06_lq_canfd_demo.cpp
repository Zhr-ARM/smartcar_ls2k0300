#include "lq_all_demo.hpp"
#include <sstream>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <chrono>

// 获取当前时间戳字符串
static std::string GetTimestamp()
{
    
    auto now = std::chrono::system_clock::now();
    auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    time_t t = std::chrono::system_clock::to_time_t(now);
    tm* tm = localtime(&t);

    std::stringstream ss;
    ss << std::put_time(tm, "%Y-%m-%d %H:%M:%S") << "." << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

// CANFD对象
ls_canfd g_can;

// CAN接收回调函数（在独立线程中运行）
void CAN_RxCallback(const ls_canfd_frame_t &frame)
{
    printf("[%s] 收到: CAN ID 0x%03X, 长度 %d, 数据: ", 
           GetTimestamp().c_str(), frame.can_id, frame.len);
    for (int i = 0; i < frame.len; i++) {
        printf("%02X ", frame.data[i]);
    }
    printf("\n");
}

void lq_canfd_demo(void)
{
    // 初始化CAN，使用独立线程模式，接收回调函数为CAN_RxCallback
    if (!g_can.canfd_init(CAN1, CANFD_MODE_THREAD, CAN_RxCallback)) {
        printf("CAN初始化失败!\n");
        return;
    }

    printf("主程序开始运行...\n");
    printf("CAN数据在独立线程中接收处理\n\n");

    // 主循环
    int count = 0;
    while (1) {
        printf("[%s] 主程序运行中... count = %d\n", GetTimestamp().c_str(), count++);
        
        // 发送数据
        uint8_t tx_data[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x11, 0x22, 0x33, 0x44};
        g_can.canfd_write_data(0x123, tx_data, sizeof(tx_data));
        printf("[%s] 发送数据: CAN ID 0x123, 长度 %zu\n", 
               GetTimestamp().c_str(), sizeof(tx_data));
        
        sleep(2);
    }

    return;
}

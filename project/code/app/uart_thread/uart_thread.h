#ifndef __UART_THREAD_H__
#define __UART_THREAD_H__

// ============================================================
// ★ 双板角色定义：改这一行即可切换！
//    接收机 = 巡线执行板（收策略）
//    发送机 = 模型识别板（发策略）
//    复制到另一块板的项目后，改下面这行就行
// ============================================================
#define BOARD_IS_RECEIVER
// #define BOARD_IS_SENDER

class timer_fd;

// ============================================================
// 双板策略枚举
// ============================================================
enum DualBoardStrategy
{
    STRATEGY_NONE     = 0x00,  // 无策略 / 默认巡线
    STRATEGY_LEFT     = 0x01,  // 左绕行
    STRATEGY_RIGHT    = 0x02,  // 右绕行
    STRATEGY_STRAIGHT = 0x03   // 直行
};

// ============================================================
// 协议常量
// ============================================================
#define DUAL_BOARD_FRAME_HEADER  0xAA   // 帧头
#define DUAL_BOARD_FRAME_LEN     3      // 协议帧长度 [帧头, 策略, 校验]
#define DUAL_BOARD_STRATEGY_TIMEOUT_MS  1000  // 接收机超时回退时间
#define DUAL_BOARD_SENDER_POLL_MS       30    // 发送机轮询周期

// ============================================================
// 兼容旧接口（main.cpp 调用）
// ============================================================
bool uart_thread_init();
void uart_thread_cleanup();
void uart_thread_print_threads(timer_fd *motor_timer);

// ============================================================
// 双板通信 API
// ============================================================

// 接收机用：获取当前策略（互斥量保护，线程安全）
DualBoardStrategy dual_board_get_strategy();

// 发送机用：发送策略帧到接收机
void dual_board_send_strategy(DualBoardStrategy s);

// 工具函数：策略枚举 → 调试名字
const char *dual_board_strategy_name(DualBoardStrategy s);

#endif

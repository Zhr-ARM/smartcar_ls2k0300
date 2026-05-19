#ifndef __UART_THREAD_H__
#define __UART_THREAD_H__

#include "zf_common_headfile.h"

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

// ============================================================
// 速度环调试 UART DMA 发送 API
// ============================================================

/**
 * @brief 初始化速度环调试 UART（复用双板通信的 ttyS1，DMA 非阻塞发送）
 *
 * 与 uart_thread_init() 互斥：本函数只打开 ttyS1 串口，
 * 不启动双板协议收发线程，专用于速度环调试数据上报。
 *
 * @return 初始化成功返回 true
 */
bool uart_thread_speed_debug_init();

/**
 * @brief 发送速度环调试数据（非阻塞 DMA 写，复用 ttyS1）
 * @param left_target  左轮目标速度 (counts/5ms)
 * @param right_target 右轮目标速度 (counts/5ms)
 * @param left_current 左轮当前速度 (counts/5ms)
 * @param right_current 右轮当前速度 (counts/5ms)
 *
 * 发送格式: <speed_debug>:left_target,right_target,left_current,right_current\n
 */
void uart_thread_speed_debug_send(float left_target, float right_target,
                                  float left_current, float right_current);

/**
 * @brief 关闭速度环调试 UART（关闭 ttyS1）
 */
void uart_thread_speed_debug_cleanup();

#endif

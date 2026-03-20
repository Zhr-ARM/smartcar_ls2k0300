#ifndef __UART_THREAD_H__
#define __UART_THREAD_H__

// 使能上位机调参功能的宏：1为开启，0为关闭
#define ENABLE_UART_TUNING 0

class timer_fd;

/**
 * @brief 初始化串口通信线程
 * @return 初始化成功返回 true，失败返回 false
 */
bool uart_thread_init();

/**
 * @brief 释放串口通信线程及相关资源
 */
void uart_thread_cleanup();

/**
 * @brief 打印主线程、定时器线程和串口线程信息
 * @param motor_timer 电机控制定时器对象指针，可为空
 */
void uart_thread_print_threads(timer_fd *motor_timer);

#endif

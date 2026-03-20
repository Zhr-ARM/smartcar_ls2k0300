#ifndef __UART_H__
#define __UART_H__

#include "zf_common_headfile.h"

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#define UART_DEFAULT_DEVICE "/dev/ttyS1"
#define UART_DEFAULT_BAUDRATE (115200)
#define UART_DEFAULT_RX_BUFFER_SIZE (256)

class Uart
{
public:
    /**
     * @brief 接收线程调度信息
     */
    struct ThreadInfo
    {
        int32 tid;
        int32 policy;
        int32 priority;
        bool valid;
    };

    using ReceiveCallback = std::function<void(const uint8 *data, int32 length)>;

    /**
     * @brief 构造串口对象
     * @param device 串口设备节点路径
     * @param baudrate 串口波特率
     */
    Uart(const char *device = UART_DEFAULT_DEVICE, uint32 baudrate = UART_DEFAULT_BAUDRATE);

    /**
     * @brief 析构并关闭串口资源
     */
    ~Uart();

    /**
     * @brief 按当前配置初始化串口
     * @return 初始化成功返回 true，失败返回 false
     */
    bool init();

    /**
     * @brief 使用指定配置初始化串口
     * @param device 串口设备节点路径
     * @param baudrate 串口波特率
     * @return 初始化成功返回 true，失败返回 false
     */
    bool init(const char *device, uint32 baudrate = UART_DEFAULT_BAUDRATE);

    /**
     * @brief 关闭串口并停止接收线程
     */
    void close();

    /**
     * @brief 查询串口是否已打开
     * @return 已打开返回 true，否则返回 false
     */
    bool is_open() const;

    /**
     * @brief 发送二进制数据
     * @param data 待发送数据缓冲区
     * @param length 数据长度
     * @return 实际发送字节数，失败返回负值
     */
    int32 send(const uint8 *data, size_t length);

    /**
     * @brief 发送字符串数据
     * @param str 待发送字符串
     * @return 实际发送字节数，失败返回负值
     */
    int32 send_string(const char *str);

    /**
     * @brief 接收串口数据
     * @param buffer 接收缓冲区
     * @param length 缓冲区长度
     * @param timeout_ms 超时时间，单位 ms，负值表示阻塞等待
     * @return 实际接收字节数，失败返回负值
     */
    int32 receive(uint8 *buffer, size_t length, int32 timeout_ms = -1);

    /**
     * @brief 设置接收回调函数
     * @param callback 数据接收回调
     */
    void set_receive_callback(const ReceiveCallback &callback);

    /**
     * @brief 启动接收回调线程
     * @param buffer_size 接收缓冲区大小
     * @param poll_timeout_ms 轮询等待超时时间，单位 ms
     * @return 启动成功返回 true，失败返回 false
     */
    bool start_receive_callback(size_t buffer_size = UART_DEFAULT_RX_BUFFER_SIZE, int32 poll_timeout_ms = 20);

    /**
     * @brief 停止接收回调线程
     */
    void stop_receive_callback();

    /**
     * @brief 获取接收线程调度信息
     * @return 接收线程信息结构体
     */
    ThreadInfo rx_thread_info() const;

private:
    /**
     * @brief 打开串口设备节点
     * @param device 串口设备路径
     * @param fd 用于返回文件描述符
     * @return 打开成功返回 true，失败返回 false
     */
    static bool open_device(const char *device, int32 &fd);

    /**
     * @brief 配置串口参数
     * @param fd 串口文件描述符
     * @param baudrate 串口波特率
     * @return 配置成功返回 true，失败返回 false
     */
    static bool configure_device(int32 fd, uint32 baudrate);

    /**
     * @brief 等待串口收发事件就绪
     * @param events 等待的 poll 事件
     * @param timeout_ms 超时时间，单位 ms
     * @return 就绪返回正值，超时返回 0，失败返回负值
     */
    int32 wait_ready(short events, int32 timeout_ms) const;

    /**
     * @brief 接收回调线程主循环
     * @param buffer_size 接收缓冲区大小
     * @param poll_timeout_ms 轮询等待超时时间，单位 ms
     */
    void rx_loop(size_t buffer_size, int32 poll_timeout_ms);

    std::string device_;
    uint32 baudrate_;
    int32 fd_;
    std::atomic<bool> opened_;
    std::atomic<bool> rx_running_;
    std::atomic<int32> rx_tid_;
    std::atomic<int32> rx_policy_;
    std::atomic<int32> rx_priority_;

    ReceiveCallback rx_callback_;
    mutable std::mutex callback_mutex_;
    std::mutex tx_mutex_;
    std::mutex rx_mutex_;
    std::thread rx_thread_;
};

#endif

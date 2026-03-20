#include "uart.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/syscall.h>
#include <termios.h>
#include <unistd.h>

namespace
{
/**
 * @brief 将波特率数值转换为 termios 速度枚举
 * @param baudrate 波特率
 * @return 对应的 termios 速度值
 */
speed_t uart_baudrate_to_speed(uint32 baudrate)
{
    switch (baudrate)
    {
        case 9600:      return B9600;
        case 19200:     return B19200;
        case 38400:     return B38400;
        case 57600:     return B57600;
        case 115200:    return B115200;
        case 230400:    return B230400;
        case 460800:    return B460800;
        case 921600:    return B921600;
        default:        return B115200;
    }
}
}

/**
 * @brief 构造串口对象
 * @param device 串口设备节点路径
 * @param baudrate 串口波特率
 */
Uart::Uart(const char *device, uint32 baudrate)
    : device_((NULL != device) ? device : UART_DEFAULT_DEVICE),
      baudrate_(baudrate),
      fd_(-1),
      opened_(false),
      rx_running_(false),
      rx_tid_(0),
      rx_policy_(0),
      rx_priority_(0)
{
}

/**
 * @brief 析构并关闭串口资源
 */
Uart::~Uart()
{
    close();
}

/**
 * @brief 按当前配置初始化串口
 * @return 初始化成功返回 true，失败返回 false
 */
bool Uart::init()
{
    int32 new_fd = -1;

    close();

    if (!open_device(device_.c_str(), new_fd))
    {
        return false;
    }

    if (!configure_device(new_fd, baudrate_))
    {
        ::close(new_fd);
        return false;
    }

    fd_ = new_fd;
    opened_ = true;
    return true;
}

/**
 * @brief 使用指定配置初始化串口
 * @param device 串口设备节点路径
 * @param baudrate 串口波特率
 * @return 初始化成功返回 true，失败返回 false
 */
bool Uart::init(const char *device, uint32 baudrate)
{
    if (NULL == device)
    {
        return false;
    }

    device_ = device;
    baudrate_ = baudrate;
    return init();
}

/**
 * @brief 关闭串口并停止接收线程
 */
void Uart::close()
{
    opened_ = false;
    stop_receive_callback();

    if (0 <= fd_)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

/**
 * @brief 查询串口是否已打开
 * @return 已打开返回 true，否则返回 false
 */
bool Uart::is_open() const
{
    return opened_.load();
}

/**
 * @brief 发送二进制数据
 * @param data 待发送数据缓冲区
 * @param length 数据长度
 * @return 实际发送字节数，失败返回负值
 */
int32 Uart::send(const uint8 *data, size_t length)
{
    size_t total_sent = 0;

    if (NULL == data)
    {
        return -1;
    }

    if (0 == length)
    {
        return 0;
    }

    if (!is_open())
    {
        return -1;
    }

    std::lock_guard<std::mutex> lock(tx_mutex_);

    while ((total_sent < length) && opened_.load())
    {
        ssize_t send_size = ::write(fd_, data + total_sent, length - total_sent);

        if (0 < send_size)
        {
            total_sent += (size_t)send_size;
            continue;
        }

        if ((0 > send_size) && (EINTR == errno))
        {
            continue;
        }

        if ((0 == send_size) || (EAGAIN == errno) || (EWOULDBLOCK == errno))
        {
            int32 wait_ret = wait_ready(POLLOUT, -1);
            if (0 >= wait_ret)
            {
                return (0 < total_sent) ? (int32)total_sent : wait_ret;
            }
            continue;
        }

        return (0 < total_sent) ? (int32)total_sent : -1;
    }

    return (int32)total_sent;
}

/**
 * @brief 发送字符串数据
 * @param str 待发送字符串
 * @return 实际发送字节数，失败返回负值
 */
int32 Uart::send_string(const char *str)
{
    if (NULL == str)
    {
        return -1;
    }

    return send((const uint8 *)str, strlen(str));
}

/**
 * @brief 接收串口数据
 * @param buffer 接收缓冲区
 * @param length 缓冲区长度
 * @param timeout_ms 超时时间，单位 ms
 * @return 实际接收字节数，失败返回负值
 */
int32 Uart::receive(uint8 *buffer, size_t length, int32 timeout_ms)
{
    if (NULL == buffer)
    {
        return -1;
    }

    if (0 == length)
    {
        return 0;
    }

    if (!is_open())
    {
        return -1;
    }

    std::lock_guard<std::mutex> lock(rx_mutex_);

    int32 wait_ret = wait_ready(POLLIN, timeout_ms);
    if (0 >= wait_ret)
    {
        return wait_ret;
    }

    ssize_t recv_size = ::read(fd_, buffer, length);
    if (0 <= recv_size)
    {
        return (int32)recv_size;
    }

    if ((EAGAIN == errno) || (EWOULDBLOCK == errno))
    {
        return 0;
    }

    return -1;
}

/**
 * @brief 设置接收回调函数
 * @param callback 数据接收回调
 */
void Uart::set_receive_callback(const ReceiveCallback &callback)
{
    std::lock_guard<std::mutex> lock(callback_mutex_);
    rx_callback_ = callback;
}

/**
 * @brief 启动接收回调线程
 * @param buffer_size 接收缓冲区大小
 * @param poll_timeout_ms 轮询等待超时时间，单位 ms
 * @return 启动成功返回 true，失败返回 false
 */
bool Uart::start_receive_callback(size_t buffer_size, int32 poll_timeout_ms)
{
    ReceiveCallback callback;

    if (!is_open())
    {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        callback = rx_callback_;
    }

    if (!callback)
    {
        printf("uart start callback error: receive callback is empty\r\n");
        return false;
    }

    if (rx_running_.load())
    {
        return true;
    }

    rx_running_ = true;
    rx_thread_ = std::thread(&Uart::rx_loop, this, buffer_size, poll_timeout_ms);
    return true;
}

/**
 * @brief 停止接收回调线程
 */
void Uart::stop_receive_callback()
{
    rx_running_ = false;

    if (rx_thread_.joinable())
    {
        rx_thread_.join();
    }
}

/**
 * @brief 获取接收线程调度信息
 * @return 接收线程信息结构体
 */
Uart::ThreadInfo Uart::rx_thread_info() const
{
    ThreadInfo info;
    info.tid = rx_tid_.load();
    info.policy = rx_policy_.load();
    info.priority = rx_priority_.load();
    info.valid = (0 < info.tid);
    return info;
}

/**
 * @brief 打开串口设备节点
 * @param device 串口设备路径
 * @param fd 用于返回文件描述符
 * @return 打开成功返回 true，失败返回 false
 */
bool Uart::open_device(const char *device, int32 &fd)
{
    fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (0 > fd)
    {
        printf("uart open %s error: %s\r\n", device, strerror(errno));
        return false;
    }

    return true;
}

/**
 * @brief 配置串口参数
 * @param fd 串口文件描述符
 * @param baudrate 串口波特率
 * @return 配置成功返回 true，失败返回 false
 */
bool Uart::configure_device(int32 fd, uint32 baudrate)
{
    struct termios tty;
    speed_t speed = uart_baudrate_to_speed(baudrate);

    if (0 != tcgetattr(fd, &tty))
    {
        printf("uart tcgetattr error: %s\r\n", strerror(errno));
        return false;
    }

    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if ((0 != cfsetispeed(&tty, speed)) || (0 != cfsetospeed(&tty, speed)))
    {
        printf("uart set speed error: %s\r\n", strerror(errno));
        return false;
    }

    if (0 != tcflush(fd, TCIOFLUSH))
    {
        printf("uart tcflush error: %s\r\n", strerror(errno));
        return false;
    }

    if (0 != tcsetattr(fd, TCSANOW, &tty))
    {
        printf("uart tcsetattr error: %s\r\n", strerror(errno));
        return false;
    }

    return true;
}

/**
 * @brief 等待串口收发事件就绪
 * @param events 等待的 poll 事件
 * @param timeout_ms 超时时间，单位 ms
 * @return 就绪返回正值，超时返回 0，失败返回负值
 */
int32 Uart::wait_ready(short events, int32 timeout_ms) const
{
    struct pollfd poll_fd;
    int32 ret = 0;

    if (!is_open())
    {
        return -1;
    }

    memset(&poll_fd, 0, sizeof(poll_fd));
    poll_fd.fd = fd_;
    poll_fd.events = events;

    do
    {
        ret = poll(&poll_fd, 1, timeout_ms);
    } while ((0 > ret) && (EINTR == errno));

    if (0 >= ret)
    {
        return ret;
    }

    if (poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL))
    {
        return -1;
    }

    return ret;
}

/**
 * @brief 接收回调线程主循环
 * @param buffer_size 接收缓冲区大小
 * @param poll_timeout_ms 轮询等待超时时间，单位 ms
 */
void Uart::rx_loop(size_t buffer_size, int32 poll_timeout_ms)
{
    std::vector<uint8> buffer;
    int policy = 0;
    struct sched_param param;

    if (0 == buffer_size)
    {
        buffer_size = 1;
    }

    buffer.resize(buffer_size);
    memset(&param, 0, sizeof(param));

    rx_tid_ = (int32)syscall(SYS_gettid);
    if (0 == pthread_getschedparam(pthread_self(), &policy, &param))
    {
        rx_policy_ = policy;
        rx_priority_ = param.sched_priority;
    }
    else
    {
        rx_policy_ = 0;
        rx_priority_ = 0;
    }

    while (rx_running_.load() && opened_.load())
    {
        int32 recv_len = receive(buffer.data(), buffer.size(), poll_timeout_ms);
        if (0 < recv_len)
        {
            ReceiveCallback callback;

            {
                std::lock_guard<std::mutex> lock(callback_mutex_);
                callback = rx_callback_;
            }

            if (callback)
            {
                callback(buffer.data(), recv_len);
            }
        }
        else if (0 > recv_len)
        {
            break;
        }
    }

    rx_running_ = false;
}

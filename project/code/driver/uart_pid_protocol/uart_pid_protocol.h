#ifndef __UART_PID_PROTOCOL_H__
#define __UART_PID_PROTOCOL_H__

#include <vector>
#include <mutex>
#include "zf_common_headfile.h"

/**
 * @brief 上位机 PID 调参协议处理类
 * 作用：负责协议帧的收发、解析和参数同步
 */
class UartPidProtocol
{
public:
    /**
     * @brief 协议发送回调类型
     */
    typedef bool (*SendFuncT)(const uint8* data, size_t length);

    /**
     * @brief 初始化协议处理器
     * @param send_func 底层发送回调函数
     */
    void init(SendFuncT send_func);

    /**
     * @brief 清理协议处理器状态
     */
    void cleanup();

    /**
     * @brief 接收底层串口数据
     * @param data 接收数据缓冲区
     * @param length 数据长度
     */
    void receive_data(const uint8* data, int32 length);

    /**
     * @brief 发送启动时的 PID 上报帧
     */
    void send_boot_report();

    /**
     * @brief 发送周期状态上报帧
     */
    void send_status_push();

private:
    std::vector<uint8> rx_buffer_;
    std::mutex rx_mutex_;
    SendFuncT send_func_;
    uint8 status_seq_;

    /**
     * @brief 解析接收缓冲区中的完整协议帧
     */
    void try_parse_rx_frames();

    /**
     * @brief 处理单帧协议数据
     * @param frame 完整协议帧
     */
    void handle_frame(const std::vector<uint8>& frame);

    /**
     * @brief 处理 PID 设置请求
     * @param seq 帧序号
     * @param payload 负载数据
     * @param payload_len 负载长度
     */
    void handle_pid_set_request(uint8 seq, const uint8* payload, uint8 payload_len);

    /**
     * @brief 发送协议帧
     * @param cmd 命令字
     * @param seq 帧序号
     * @param payload 负载数据
     * @return 发送成功返回 true，失败返回 false
     */
    bool send_protocol_frame(uint8 cmd, uint8 seq, const std::vector<uint8>& payload);

    /**
     * @brief 发送错误响应帧
     * @param seq 帧序号
     * @param related_cmd 关联命令字
     * @param error_code 错误码
     */
    void send_error_frame(uint8 seq, uint8 related_cmd, uint8 error_code);

    /**
     * @brief 发送 PID 数据帧
     * @param cmd 命令字
     * @param seq 帧序号
     */
    void send_pid_frame(uint8 cmd, uint8 seq);

    /**
     * @brief 发送空负载确认帧
     * @param cmd 命令字
     * @param seq 帧序号
     */
    void send_empty_ack(uint8 cmd, uint8 seq);

    /**
     * @brief 生成下一帧状态序号
     * @return 新的状态帧序号
     */
    uint8 next_status_seq();
};

extern UartPidProtocol g_pid_protocol;

#endif // __UART_PID_PROTOCOL_H__

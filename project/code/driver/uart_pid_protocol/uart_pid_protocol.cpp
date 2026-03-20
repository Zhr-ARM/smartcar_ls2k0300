#include "uart_pid_protocol.h"
#include "motor_thread.h"
#include <algorithm>
#include <cstring>

// #define left_mototr
// #define right_mototr

#if defined(left_mototr) && defined(right_mototr)
#error "left_mototr and right_mototr cannot be defined at the same time"
#endif

namespace {
    constexpr uint8 kFrameHead = 0x55;
    constexpr uint8 kFrameTail = 0xCC;
    constexpr uint8 kProtocolVersion = 0x01;
    constexpr uint8 kCmdMotorStatusPush = 0x10;
    constexpr uint8 kCmdPidSetReq = 0x20;
    constexpr uint8 kCmdPidSetAck = 0xA0;
    constexpr uint8 kCmdPidGetReq = 0x21;
    constexpr uint8 kCmdPidGetAck = 0xA1;
    constexpr uint8 kCmdErrorReport = 0xE0;
    constexpr uint8 kErrorLengthInvalid = 0x01;
    constexpr uint8 kErrorUnsupportedCmd = 0x02;
    constexpr uint8 kErrorInvalidParam = 0x03;
    constexpr uint8 kErrorUnsupportedVersion = 0x04;
    constexpr size_t kProtocolMinFrameSize = 7;

#if defined(left_mototr) || defined(right_mototr)
    constexpr size_t kStatusPayloadFloatCount = 4;
    constexpr size_t kPidPayloadFloatCount = 3;
#else
    constexpr size_t kStatusPayloadFloatCount = 8;
    constexpr size_t kPidPayloadFloatCount = 6;
#endif

    /**
     * @brief 计算协议校验和
     * @param data 数据缓冲区
     * @param length 数据长度
     * @return 异或校验结果
     */
    uint8 calc_checksum(const uint8 *data, size_t length)
    {
        uint8 checksum = 0;
        for (size_t i = 0; i < length; ++i)
        {
            checksum ^= data[i];
        }
        return checksum;
    }

    /**
     * @brief 向负载追加浮点数
     * @param payload 目标负载缓冲区
     * @param value 待追加浮点值
     */
    void append_float(std::vector<uint8> &payload, float value)
    {
        uint8 bytes[sizeof(float)];
        memcpy(bytes, &value, sizeof(value));
        payload.insert(payload.end(), bytes, bytes + sizeof(value));
    }

    /**
     * @brief 从字节流读取浮点数
     * @param data 浮点数据起始地址
     * @return 解析出的浮点值
     */
    float read_float(const uint8 *data)
    {
        float value = 0.0f;
        memcpy(&value, data, sizeof(value));
        return value;
    }

    /**
     * @brief 构建 PID 参数负载
     * @param params PID 参数对象
     * @param payload 用于返回负载数据
     */
    void build_pid_payload(const MotorPidParams &params, std::vector<uint8> &payload)
    {
        payload.clear();
        payload.reserve(kPidPayloadFloatCount * sizeof(float));
#if defined(left_mototr)
        append_float(payload, params.left_kp);
        append_float(payload, params.left_ki);
        append_float(payload, params.left_kd);
#elif defined(right_mototr)
        append_float(payload, params.right_kp);
        append_float(payload, params.right_ki);
        append_float(payload, params.right_kd);
#else
        append_float(payload, params.left_kp);
        append_float(payload, params.left_ki);
        append_float(payload, params.left_kd);
        append_float(payload, params.right_kp);
        append_float(payload, params.right_ki);
        append_float(payload, params.right_kd);
#endif
    }
}

UartPidProtocol g_pid_protocol;

/**
 * @brief 初始化协议处理器
 * @param send_func 底层发送回调函数
 */
void UartPidProtocol::init(SendFuncT send_func)
{
    send_func_ = send_func;
    status_seq_ = 0;
    std::lock_guard<std::mutex> lock(rx_mutex_);
    rx_buffer_.clear();
}

/**
 * @brief 清理协议处理器状态
 */
void UartPidProtocol::cleanup()
{
    std::lock_guard<std::mutex> lock(rx_mutex_);
    rx_buffer_.clear();
    send_func_ = nullptr;
}

/**
 * @brief 生成下一帧状态序号
 * @return 新的状态帧序号
 */
uint8 UartPidProtocol::next_status_seq()
{
    return ++status_seq_;
}

/**
 * @brief 接收底层串口数据
 * @param data 接收数据缓冲区
 * @param length 数据长度
 */
void UartPidProtocol::receive_data(const uint8* data, int32 length)
{
    if ((NULL == data) || (length <= 0))
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(rx_mutex_);
        rx_buffer_.insert(rx_buffer_.end(), data, data + length);
    }

    try_parse_rx_frames();
}

/**
 * @brief 发送协议帧
 * @param cmd 命令字
 * @param seq 帧序号
 * @param payload 负载数据
 * @return 发送成功返回 true，失败返回 false
 */
bool UartPidProtocol::send_protocol_frame(uint8 cmd, uint8 seq, const std::vector<uint8>& payload)
{
    if (!send_func_)
    {
        return false;
    }

    std::vector<uint8> frame;
    frame.reserve(kProtocolMinFrameSize + payload.size());
    frame.push_back(kFrameHead);
    frame.push_back(kProtocolVersion);
    frame.push_back(cmd);
    frame.push_back(seq);
    frame.push_back((uint8)payload.size());
    frame.insert(frame.end(), payload.begin(), payload.end());
    frame.push_back(calc_checksum(frame.data() + 1, 4 + payload.size()));
    frame.push_back(kFrameTail);

    return send_func_(frame.data(), frame.size());
}

/**
 * @brief 发送错误响应帧
 * @param seq 帧序号
 * @param related_cmd 关联命令字
 * @param error_code 错误码
 */
void UartPidProtocol::send_error_frame(uint8 seq, uint8 related_cmd, uint8 error_code)
{
    std::vector<uint8> payload;
    payload.reserve(2);
    payload.push_back(error_code);
    payload.push_back(related_cmd);
    send_protocol_frame(kCmdErrorReport, seq, payload);
}

/**
 * @brief 发送 PID 数据帧
 * @param cmd 命令字
 * @param seq 帧序号
 */
void UartPidProtocol::send_pid_frame(uint8 cmd, uint8 seq)
{
    MotorPidParams params;
    std::vector<uint8> payload;

    if (!motor_thread_get_pid_params(params))
    {
        send_error_frame(seq, cmd, kErrorInvalidParam);
        return;
    }

    build_pid_payload(params, payload);
    send_protocol_frame(cmd, seq, payload);
}

/**
 * @brief 发送启动时的 PID 上报帧
 */
void UartPidProtocol::send_boot_report()
{
    status_seq_ = 0;
    send_pid_frame(kCmdPidGetAck, 0);
}

/**
 * @brief 发送空负载确认帧
 * @param cmd 命令字
 * @param seq 帧序号
 */
void UartPidProtocol::send_empty_ack(uint8 cmd, uint8 seq)
{
    std::vector<uint8> payload;
    send_protocol_frame(cmd, seq, payload);
}

/**
 * @brief 发送周期状态上报帧
 */
void UartPidProtocol::send_status_push()
{
    MotorUartStatus status = motor_thread_uart_status();
    std::vector<uint8> payload;

    payload.reserve(kStatusPayloadFloatCount * sizeof(float));
#if defined(left_mototr)
    append_float(payload, status.left_error);
    append_float(payload, status.left_target_count);
    append_float(payload, status.left_duty);
    append_float(payload, status.left_current_count);
#elif defined(right_mototr)
    append_float(payload, status.right_error);
    append_float(payload, status.right_target_count);
    append_float(payload, status.right_duty);
    append_float(payload, status.right_current_count);
#else
    append_float(payload, status.left_error);
    append_float(payload, status.right_error);
    append_float(payload, status.left_target_count);
    append_float(payload, status.right_target_count);
    append_float(payload, status.left_duty);
    append_float(payload, status.right_duty);
    append_float(payload, status.left_current_count);
    append_float(payload, status.right_current_count);
#endif
    send_protocol_frame(kCmdMotorStatusPush, next_status_seq(), payload);
}

/**
 * @brief 处理 PID 设置请求
 * @param seq 帧序号
 * @param payload 负载数据
 * @param payload_len 负载长度
 */
void UartPidProtocol::handle_pid_set_request(uint8 seq, const uint8* payload, uint8 payload_len)
{
    if (payload_len != kPidPayloadFloatCount * sizeof(float))
    {
        send_error_frame(seq, kCmdPidSetReq, kErrorLengthInvalid);
        return;
    }

    MotorPidParams params;
    if (!motor_thread_get_pid_params(params))
    {
        send_error_frame(seq, kCmdPidSetReq, kErrorInvalidParam);
        return;
    }

#if defined(left_mototr)
    params.left_kp = read_float(payload);
    params.left_ki = read_float(payload + sizeof(float));
    params.left_kd = read_float(payload + 2 * sizeof(float));
#elif defined(right_mototr)
    params.right_kp = read_float(payload);
    params.right_ki = read_float(payload + sizeof(float));
    params.right_kd = read_float(payload + 2 * sizeof(float));
#else
    params.left_kp = read_float(payload);
    params.left_ki = read_float(payload + sizeof(float));
    params.left_kd = read_float(payload + 2 * sizeof(float));
    params.right_kp = read_float(payload + 3 * sizeof(float));
    params.right_ki = read_float(payload + 4 * sizeof(float));
    params.right_kd = read_float(payload + 5 * sizeof(float));
#endif

    if (!motor_thread_set_pid_params(params))
    {
        send_error_frame(seq, kCmdPidSetReq, kErrorInvalidParam);
        return;
    }

    send_empty_ack(kCmdPidSetAck, seq);
}

/**
 * @brief 处理单帧协议数据
 * @param frame 完整协议帧
 */
void UartPidProtocol::handle_frame(const std::vector<uint8>& frame)
{
    uint8 version = frame[1];
    uint8 cmd = frame[2];
    uint8 seq = frame[3];
    uint8 payload_len = frame[4];
    const uint8* payload = frame.data() + 5;

    if (version != kProtocolVersion)
    {
        send_error_frame(seq, cmd, kErrorUnsupportedVersion);
        return;
    }

    switch (cmd)
    {
        case kCmdPidSetReq:
            handle_pid_set_request(seq, payload, payload_len);
            break;

        case kCmdPidGetReq:
            send_error_frame(seq, cmd, kErrorUnsupportedCmd);
            break;

        default:
            send_error_frame(seq, cmd, kErrorUnsupportedCmd);
            break;
    }
}

/**
 * @brief 解析接收缓冲区中的完整协议帧
 */
void UartPidProtocol::try_parse_rx_frames()
{
    while (true)
    {
        std::vector<uint8> frame;

        {
            std::lock_guard<std::mutex> lock(rx_mutex_);

            std::vector<uint8>::iterator frame_head =
                std::find(rx_buffer_.begin(), rx_buffer_.end(), kFrameHead);
            if (frame_head != rx_buffer_.begin())
            {
                rx_buffer_.erase(rx_buffer_.begin(), frame_head);
            }

            if (rx_buffer_.size() < kProtocolMinFrameSize)
            {
                return;
            }

            size_t payload_len = rx_buffer_[4];
            size_t frame_len = kProtocolMinFrameSize + payload_len;

            if (rx_buffer_.size() < frame_len)
            {
                return;
            }

            if (rx_buffer_[frame_len - 1] != kFrameTail)
            {
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }

            uint8 checksum = calc_checksum(rx_buffer_.data() + 1, 4 + payload_len);
            if (checksum != rx_buffer_[frame_len - 2])
            {
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }

            frame.assign(rx_buffer_.begin(), rx_buffer_.begin() + frame_len);
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + frame_len);
        }

        handle_frame(frame);
    }
}

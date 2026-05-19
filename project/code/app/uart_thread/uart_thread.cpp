#include "uart_thread.h"

#include "uart.h"
#include "zf_common_headfile.h"

#ifdef BOARD_IS_SENDER
#include "driver/vision/vision_pipeline.h"
#endif

#ifdef BOARD_IS_RECEIVER
#include "driver/vision/vision_image_processor.h"
#include "driver/vision/vision_config.h"
#endif

#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>

// ============================================================
// 共享状态
// ============================================================
namespace
{

// 双板通信用 UART（ttyS1, 115200）
Uart g_dual_uart("/dev/ttyS1", 115200);

// 策略状态 + 互斥量
std::mutex g_strategy_mutex;
DualBoardStrategy g_current_strategy = STRATEGY_NONE;

// 接收机：上次收到有效帧的时间戳（用于超时回退）
std::chrono::steady_clock::time_point g_last_received_time;

#ifdef BOARD_IS_RECEIVER
// IPM 绕行偏移恢复值（策略生效前保存，退出时恢复）
float g_restore_center_offset_px = 0.0f;
// 上次已应用的策略（去重）
DualBoardStrategy g_last_applied_strategy = STRATEGY_NONE;
// 恢复值是否已保存
bool g_restore_offset_saved = false;
#endif

// 工作线程
std::thread g_worker_thread;
std::atomic<bool> g_worker_running(false);

// ============================================================
// 协议编解码
// ============================================================

uint8_t compute_checksum(uint8_t header, uint8_t strategy)
{
    return header ^ strategy;
}

void encode_frame(DualBoardStrategy s, uint8_t *frame)
{
    frame[0] = DUAL_BOARD_FRAME_HEADER;
    frame[1] = static_cast<uint8_t>(s);
    frame[2] = compute_checksum(frame[0], frame[1]);
}

/**
 * @brief 逐字节帧解析状态机
 * @param byte       新收到的字节
 * @param out_strategy  输出解析出的策略（仅在解析成功时写入）
 * @return true 表示成功解析出一帧
 */
bool try_parse_frame(uint8_t byte, DualBoardStrategy *out_strategy)
{
    enum class ParseState : uint8_t
    {
        WAIT_HEADER = 0,
        WAIT_STRATEGY,
        WAIT_CHECKSUM
    };

    static ParseState s_state = ParseState::WAIT_HEADER;
    static uint8_t s_header = 0;
    static uint8_t s_strategy_byte = 0;

    switch (s_state)
    {
        case ParseState::WAIT_HEADER:
            if (byte == DUAL_BOARD_FRAME_HEADER)
            {
                s_header = byte;
                s_state = ParseState::WAIT_STRATEGY;
            }
            return false;

        case ParseState::WAIT_STRATEGY:
            s_strategy_byte = byte;
            s_state = ParseState::WAIT_CHECKSUM;
            return false;

        case ParseState::WAIT_CHECKSUM:
        {
            s_state = ParseState::WAIT_HEADER;

            // 校验
            uint8_t expected = compute_checksum(s_header, s_strategy_byte);
            if (byte != expected)
            {
                return false;
            }

            // 策略值范围检查
            if (s_strategy_byte > static_cast<uint8_t>(STRATEGY_STRAIGHT))
            {
                return false;
            }

            *out_strategy = static_cast<DualBoardStrategy>(s_strategy_byte);
            return true;
        }

        default:
            s_state = ParseState::WAIT_HEADER;
            return false;
    }
}

// ============================================================
// 调试输出
// ============================================================

void debug_print_frame(const char *direction, const uint8_t *frame)
{
#ifdef BOARD_IS_SENDER
    const char *role = "SENDER";
#else
    const char *role = "RECEIVER";
#endif
    printf("[%s] %s frame: [%02X %02X %02X] strategy=%s\r\n",
           role,
           direction,
           frame[0], frame[1], frame[2],
           dual_board_strategy_name(static_cast<DualBoardStrategy>(frame[1])));
}

}  // namespace

// ============================================================
// 接收机实现
// ============================================================
#ifdef BOARD_IS_RECEIVER

namespace
{

/**
 * @brief 将策略应用到 IPM 中线偏移（复用单板绕行机制）
 * @param s 当前策略
 */
void apply_strategy_to_ipm(DualBoardStrategy s)
{
    if (s == g_last_applied_strategy)
    {
        return;  // 去重，相同策略不重复设置
    }

    const float delta = g_vision_runtime_config.infer_avoid_offset_delta_px;

    // 首次从 NONE 进入有效策略时，保存当前偏移作为恢复值
    if (g_last_applied_strategy == STRATEGY_NONE && s != STRATEGY_NONE)
    {
        g_restore_center_offset_px = vision_image_processor_ipm_center_target_offset_from_left_px();
        g_restore_offset_saved = true;
    }

    // 计算新偏移（复用 vision_pipeline 中 resolve_target_board_offset 的逻辑）
    float new_offset = g_restore_center_offset_px;
    switch (s)
    {
        case STRATEGY_LEFT:
            new_offset = g_restore_center_offset_px - delta;  // weapon → 左偏 → 左绕行
            break;
        case STRATEGY_RIGHT:
            new_offset = g_restore_center_offset_px + delta;  // supply → 右偏 → 右绕行
            break;
        case STRATEGY_STRAIGHT:
            new_offset = g_restore_center_offset_px;           // vehicle → 不变 → 直行
            break;
        case STRATEGY_NONE:
        default:
            // 恢复原始偏移
            g_restore_offset_saved = false;
            break;
    }

    // 限幅到赛道宽度内
    const float track_w = g_vision_runtime_config.ipm_track_width_px;
    if (track_w > 0.0f)
    {
        if (new_offset < 0.0f)
        {
            new_offset = 0.0f;
        }
        if (new_offset > track_w)
        {
            new_offset = track_w;
        }
    }

    vision_image_processor_set_ipm_center_target_offset_from_left_px(new_offset);
    g_last_applied_strategy = s;

    printf("[RECEIVER] IPM offset: strategy=%s offset %.1f -> %.1f (delta=%.1f)\r\n",
           dual_board_strategy_name(s),
           static_cast<double>(g_restore_center_offset_px),
           static_cast<double>(new_offset),
           static_cast<double>(delta));
}

/**
 * @brief UART 接收回调：逐字节喂给帧解析器
 */
void receiver_uart_callback(const uint8_t *data, int32_t length)
{
    for (int32_t i = 0; i < length; i++)
    {
        DualBoardStrategy parsed;
        if (try_parse_frame(data[i], &parsed))
        {
            {
                std::lock_guard<std::mutex> lock(g_strategy_mutex);
                g_current_strategy = parsed;
                g_last_received_time = std::chrono::steady_clock::now();
            }

            // 立即应用到 IPM 中线偏移，驱动车辆绕行
            apply_strategy_to_ipm(parsed);

            uint8_t frame[DUAL_BOARD_FRAME_LEN];
            encode_frame(parsed, frame);
            debug_print_frame("recv", frame);
        }
    }
}

}  // namespace
#endif  // BOARD_IS_RECEIVER

// ============================================================
// 发送机实现
// ============================================================
#ifdef BOARD_IS_SENDER

namespace
{

// 上次发送的策略（去重用）
DualBoardStrategy g_last_sent_strategy = STRATEGY_NONE;
// 目标丢失消抖计数
int g_target_lost_count = 0;
constexpr int kTargetLostThreshold = 20;  // 连续 N 次未检测到才发 NONE

/**
 * @brief 发送机监控线程：轮询视觉识别结果，发现目标即发送策略
 */
void sender_monitor_loop()
{
    while (g_worker_running.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(DUAL_BOARD_SENDER_POLL_MS));

        if (!vision_pipeline_target_board_active())
        {
            g_target_lost_count++;
            if (g_target_lost_count >= kTargetLostThreshold &&
                g_last_sent_strategy != STRATEGY_NONE)
            {
                dual_board_send_strategy(STRATEGY_NONE);
            }
            continue;
        }

        // 检测到目标
        g_target_lost_count = 0;

        const char *state_name = vision_pipeline_target_board_state_name();
        DualBoardStrategy new_strategy = STRATEGY_NONE;

        if (std::strcmp(state_name, "weapon") == 0)
        {
            new_strategy = STRATEGY_LEFT;
        }
        else if (std::strcmp(state_name, "supply") == 0)
        {
            new_strategy = STRATEGY_RIGHT;
        }
        else if (std::strcmp(state_name, "vehicle") == 0)
        {
            new_strategy = STRATEGY_STRAIGHT;
        }

        // 策略变化且有效时才发送
        if (new_strategy != g_last_sent_strategy && new_strategy != STRATEGY_NONE)
        {
            dual_board_send_strategy(new_strategy);
        }
    }
}

}  // namespace
#endif  // BOARD_IS_SENDER

// ============================================================
// 公共 API
// ============================================================

bool uart_thread_init()
{
    if (!g_dual_uart.init())
    {
#ifdef BOARD_IS_SENDER
        printf("[SENDER] dual-board UART init failed (ttyS1)\r\n");
#else
        printf("[RECEIVER] dual-board UART init failed (ttyS1)\r\n");
#endif
        return false;
    }

#ifdef BOARD_IS_RECEIVER
    g_dual_uart.set_receive_callback(receiver_uart_callback);
    if (!g_dual_uart.start_receive_callback())
    {
        printf("[RECEIVER] UART receive callback start failed\r\n");
        g_dual_uart.close();
        return false;
    }

    g_last_received_time = std::chrono::steady_clock::now();
    printf("[RECEIVER] dual-board UART ready on ttyS1, listening for strategy frames\r\n");
#endif

#ifdef BOARD_IS_SENDER
    g_worker_running = true;
    g_worker_thread = std::thread(sender_monitor_loop);
    printf("[SENDER] dual-board UART ready on ttyS1, monitoring vision for targets\r\n");
#endif

    return true;
}

void uart_thread_cleanup()
{
#ifdef BOARD_IS_SENDER
    g_worker_running = false;
    if (g_worker_thread.joinable())
    {
        g_worker_thread.join();
    }
#endif

#ifdef BOARD_IS_RECEIVER
    g_dual_uart.stop_receive_callback();
#endif

    g_dual_uart.close();

#ifdef BOARD_IS_SENDER
    printf("[SENDER] dual-board UART closed\r\n");
#else
    printf("[RECEIVER] dual-board UART closed\r\n");
#endif
}

void uart_thread_print_threads(timer_fd * /*motor_timer*/)
{
    // 兼容旧接口：双板模式下暂不打印线程信息
}

DualBoardStrategy dual_board_get_strategy()
{
    std::lock_guard<std::mutex> lock(g_strategy_mutex);

    // 超时回退检查
    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now - g_last_received_time)
                          .count();
    if (elapsed_ms > DUAL_BOARD_STRATEGY_TIMEOUT_MS)
    {
        if (g_current_strategy != STRATEGY_NONE)
        {
            printf("[RECEIVER] strategy timeout (%ldms), fallback to NONE\r\n",
                   static_cast<long>(elapsed_ms));
            g_current_strategy = STRATEGY_NONE;

#ifdef BOARD_IS_RECEIVER
            // 超时恢复原始 IPM 偏移
            if (g_restore_offset_saved)
            {
                vision_image_processor_set_ipm_center_target_offset_from_left_px(
                    g_restore_center_offset_px);
                g_last_applied_strategy = STRATEGY_NONE;
                g_restore_offset_saved = false;
                printf("[RECEIVER] IPM offset restored to %.1f\r\n",
                       static_cast<double>(g_restore_center_offset_px));
            }
#endif
        }
    }

    return g_current_strategy;
}

void dual_board_send_strategy(DualBoardStrategy s)
{
    uint8_t frame[DUAL_BOARD_FRAME_LEN];
    encode_frame(s, frame);

    {
        std::lock_guard<std::mutex> lock(g_strategy_mutex);
#ifdef BOARD_IS_SENDER
        g_last_sent_strategy = s;
#endif
    }

    int32_t sent = g_dual_uart.send(frame, DUAL_BOARD_FRAME_LEN);
    if (sent == DUAL_BOARD_FRAME_LEN)
    {
        debug_print_frame("sent", frame);
    }
    else
    {
#ifdef BOARD_IS_SENDER
        printf("[SENDER] UART send failed, sent=%d expected=%d\r\n",
               sent, DUAL_BOARD_FRAME_LEN);
#else
        printf("[RECEIVER] UART send failed, sent=%d expected=%d\r\n",
               sent, DUAL_BOARD_FRAME_LEN);
#endif
    }
}

const char *dual_board_strategy_name(DualBoardStrategy s)
{
    switch (s)
    {
        case STRATEGY_NONE:     return "NONE";
        case STRATEGY_LEFT:     return "LEFT";
        case STRATEGY_RIGHT:    return "RIGHT";
        case STRATEGY_STRAIGHT: return "STRAIGHT";
        default:                return "UNKNOWN";
    }
}

// ============================================================
// 速度环调试 UART DMA 发送实现
// 复用双板通信的 g_dual_uart（ttyS1），不启动双板协议线程。
// ============================================================

namespace
{

bool g_speed_debug_active = false;

} // namespace

bool uart_thread_speed_debug_init()
{
    if (g_speed_debug_active)
    {
        return true;
    }

    // 复用双板通信的 ttyS1，仅打开串口，不启动协议收发
    if (!g_dual_uart.init())
    {
        printf("[SPEED_DEBUG] UART init failed on ttyS1\r\n");
        return false;
    }

    g_speed_debug_active = true;
    printf("[SPEED_DEBUG] UART ready on ttyS1 @ 115200 baud (DMA nonblock write, dual-board protocol disabled)\r\n");
    return true;
}

void uart_thread_speed_debug_send(float left_target, float right_target,
                                  float left_current, float right_current)
{
    if (!g_speed_debug_active)
    {
        return;
    }

    // 格式化: <speed_debug>:ch0,ch1,ch2,ch3\n
    // ch0=左轮目标速, ch1=右轮目标速, ch2=左轮当前速, ch3=右轮当前速
    char buf[128];
    int len = snprintf(buf, sizeof(buf),
                       "<speed_debug>:%.1f,%.1f,%.1f,%.1f\n",
                       static_cast<double>(left_target),
                       static_cast<double>(right_target),
                       static_cast<double>(left_current),
                       static_cast<double>(right_current));

    if (len > 0 && len < static_cast<int>(sizeof(buf)))
    {
        // 非阻塞 DMA 写：单次 write() 不等待，内核驱动自动使用 DMA
        g_dual_uart.send_nonblock(reinterpret_cast<const uint8 *>(buf), static_cast<size_t>(len));
    }
}

void uart_thread_speed_debug_cleanup()
{
    g_dual_uart.close();
    g_speed_debug_active = false;
    printf("[SPEED_DEBUG] UART closed (ttyS1)\r\n");
}

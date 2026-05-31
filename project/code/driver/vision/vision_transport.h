#ifndef VISION_TRANSPORT_H_
#define VISION_TRANSPORT_H_

#include "zf_common_headfile.h"

// 作用：初始化 transport 模块内部状态。
// 意义：统一发送（UDP/TCP发送）的初始化入口。
// 如何修改：如需新增发送后端，优先在 transport 内部扩展，不改上层线程接口。
// 是否调用：是，vision_pipeline_init 中调用。
void vision_transport_init();

// 作用：执行一次发送步进（UDP/TCP发送）。
// 意义：让主循环只保留一次 send_step 调用，降低上层耦合。
// 如何修改：新增发送策略（限频、格式）应在实现内改。
// 是否调用：是，vision_pipeline_send_step 每帧调用。
void vision_transport_send_step();

// 作用：读取最近一次发送耗时（us）。
// 意义：用于性能统计与日志输出。
// 如何修改：仅用于观测，不建议业务逻辑依赖其绝对值。
// 是否调用：是，vision_thread 性能统计路径调用。
uint32 vision_transport_get_last_send_time_us();

// 作用：初始化 UDP 视频 + TCP 状态上传通道。
// 意义：统一网页端调试链路入口。
// 如何修改：
// - server_ip: 接收端IP；
// - video_port: UDP视频端口；
// - meta_port: TCP状态端口（0 可禁用）。
// 是否调用：是，main.cpp 启动时调用。
bool vision_transport_udp_init(const char *server_ip, uint16 video_port, uint16 meta_port);

// 作用：UDP/TCP 发送模块清理接口。
// 意义：与初始化形成对称，便于后续补充底层 close。
// 如何修改：若底层驱动增加 close，应在实现中补上。
// 是否调用：是，程序退出 cleanup 调用。
void vision_transport_udp_cleanup();

// 作用：开关 UDP 图像发送。
// 意义：可只保留 TCP 状态或彻底关闭网页链路。
// 如何修改：true=发送，false=不发送。
// 是否调用：是，main.cpp 配置下发。
void vision_transport_udp_set_enabled(bool enabled);
bool vision_transport_udp_is_enabled();

// 作用：设置 UDP 图像限频（0 表示不限）。
// 意义：控制网络带宽和CPU负载。
// 如何修改：推荐 15~30 起步，按网络情况调优。
// 是否调用：是，main.cpp 配置下发。
void vision_transport_udp_set_max_fps(uint32 max_fps);
uint32 vision_transport_udp_get_max_fps();

// 作用：开关 TCP 状态发送。
// 意义：可只发视频不发状态，减少解析负担。
// 如何修改：true=发送状态，false=不发送状态。
// 是否调用：是，main.cpp 配置下发。
void vision_transport_udp_set_tcp_enabled(bool enabled);
bool vision_transport_udp_tcp_enabled();

#endif

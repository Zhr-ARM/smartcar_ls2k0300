#ifndef VISION_ASSISTANT_UDP_H_
#define VISION_ASSISTANT_UDP_H_

#include "zf_common_headfile.h"

// 初始化逐飞助手 UDP 发送通道，并绑定 seekfree_assistant 回调到该通道。
// 成功后，seekfree_assistant_camera_send() 将发送到 ip:port。
bool vision_assistant_udp_init(const char *ip, uint16 port);

// 关闭逐飞助手 UDP 通道。
void vision_assistant_udp_cleanup();

// 查询逐飞助手 UDP 通道是否已初始化成功。
bool vision_assistant_udp_is_ready();

#endif

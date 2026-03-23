#ifndef VISION_CLIENT_SENDER_H_
#define VISION_CLIENT_SENDER_H_

#include "driver/vision/vision_pipeline.h"

// 初始化发送模块
void vision_client_sender_init();

// 发送一帧图像（根据模式），发送开关关闭时直接返回
void vision_client_sender_send_step();
// 最近一次send_step耗时（单位：us）
uint32 vision_client_sender_get_last_send_time_us();

// 设置发送模式：二值/灰度/彩色
void vision_client_sender_set_mode(vision_send_mode_enum mode);
vision_send_mode_enum vision_client_sender_get_mode();

// 设置发送最大帧率，0 表示不限速。
void vision_client_sender_set_max_fps(uint32 max_fps);
uint32 vision_client_sender_get_max_fps();

// 设置发送使能
void vision_client_sender_set_enabled(bool enabled);
bool vision_client_sender_is_enabled();

#endif

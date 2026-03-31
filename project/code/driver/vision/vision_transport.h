#ifndef VISION_TRANSPORT_H_
#define VISION_TRANSPORT_H_

#include "zf_common_headfile.h"

typedef enum
{
    VISION_SEND_BINARY = 0,
    VISION_SEND_GRAY = 1,
    VISION_SEND_RGB565 = 2
} vision_send_mode_enum;

void vision_transport_init();
void vision_transport_send_step();

uint32 vision_transport_get_last_send_time_us();
void vision_transport_set_send_mode(vision_send_mode_enum mode);
vision_send_mode_enum vision_transport_get_send_mode();
void vision_transport_set_send_max_fps(uint32 max_fps);
uint32 vision_transport_get_send_max_fps();
void vision_transport_set_send_enabled(bool enabled);
bool vision_transport_is_send_enabled();

bool vision_transport_udp_init(const char *server_ip, uint16 video_port, uint16 meta_port);
void vision_transport_udp_cleanup();
void vision_transport_udp_set_enabled(bool enabled);
bool vision_transport_udp_is_enabled();
void vision_transport_udp_set_max_fps(uint32 max_fps);
uint32 vision_transport_udp_get_max_fps();
void vision_transport_udp_set_tcp_enabled(bool enabled);
bool vision_transport_udp_tcp_enabled();

#endif

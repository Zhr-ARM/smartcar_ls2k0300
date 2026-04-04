#ifndef LINE_FOLLOW_THREAD_H_
#define LINE_FOLLOW_THREAD_H_

#include "zf_common_headfile.h"

bool line_follow_thread_init();
void line_follow_thread_cleanup();
void line_follow_thread_print_info();
void line_follow_thread_set_base_speed(float speed);
float line_follow_thread_error();
float line_follow_thread_turn_output();
float line_follow_thread_base_speed();
float line_follow_thread_adjusted_base_speed();

// WASM bridge 专用：由桥接层把网页收到的原始速度写进 compat 状态。
void wasm_compat_line_follow_thread_set_adjusted_base_speed(float speed);

#endif

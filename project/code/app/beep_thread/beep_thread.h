#ifndef BEEP_THREAD_H_
#define BEEP_THREAD_H_

#include "zf_common_headfile.h"

bool beep_thread_init();
void beep_thread_cleanup();
void beep_thread_request_beep(int duration_ms);
void beep_thread_set_alarm_enabled(bool enabled);
bool beep_thread_alarm_enabled();
void beep_thread_print_info();

#endif

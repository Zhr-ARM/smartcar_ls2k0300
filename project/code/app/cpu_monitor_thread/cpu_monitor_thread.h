#ifndef CPU_MONITOR_THREAD_H_
#define CPU_MONITOR_THREAD_H_

#include "zf_common_headfile.h"

bool cpu_monitor_thread_init();
void cpu_monitor_thread_cleanup();
void cpu_monitor_thread_print_info();

#endif

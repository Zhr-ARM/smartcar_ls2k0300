#ifndef CONFIG_HTTP_THREAD_H_
#define CONFIG_HTTP_THREAD_H_

#include "zf_common_headfile.h"

bool config_http_thread_init();
void config_http_thread_cleanup();
bool config_http_thread_is_running();

#endif

#include "line_follow_thread.h"

namespace
{
float g_base_speed = 0.0f;
float g_adjusted_base_speed = 0.0f;
}

bool line_follow_thread_init()
{
    return true;
}

void line_follow_thread_cleanup()
{
    g_base_speed = 0.0f;
    g_adjusted_base_speed = 0.0f;
}

void line_follow_thread_print_info()
{
}

void line_follow_thread_set_base_speed(float speed)
{
    g_base_speed = speed;
}

float line_follow_thread_error()
{
    return 0.0f;
}

float line_follow_thread_turn_output()
{
    return 0.0f;
}

float line_follow_thread_base_speed()
{
    return g_base_speed;
}

float line_follow_thread_adjusted_base_speed()
{
    return g_adjusted_base_speed;
}

void wasm_compat_line_follow_thread_set_adjusted_base_speed(float speed)
{
    g_adjusted_base_speed = speed;
}

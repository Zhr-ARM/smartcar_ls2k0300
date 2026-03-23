#include "lq_all_demo.hpp"
#include "lq_display_ips20.hpp"

void lq_ips20_demo()
{
    lq_ips20_drv_init(1);
    lq_ips20_drv_fill_area(10, 20, 30, 40, U16YELLOW);
    lq_ips20_drv_draw_line(10, 20, 30, 40, U16RED);
    lq_ips20_drv_draw_circle(50, 50, 30, U16BLACK);
}
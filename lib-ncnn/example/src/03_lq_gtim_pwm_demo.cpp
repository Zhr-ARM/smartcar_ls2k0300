#include "lq_all_demo.hpp"

/********************************************************************************
 * @brief   GTIM PWM 输出模式测试.
 * @param   none.
 * @return  none.
 * @note    初始构造方法有两种, 多文件赋值使用时也有两种方法.
 ********************************************************************************/
void lq_gtim_pwm_demo(void)
{
    // 默认极性的构造方式
    ls_gtim_pwm pwm1(GTIM_PWM0_PIN87, 100, 2000);
    // 自定义极性的构造方式
    ls_gtim_pwm pwm2(GTIM_PWM1_PIN88, 100, 2000, GTIM_PWM_POL_NORMAL);
    // 拷贝构造使用方法, 调用 pwm3 与调用 pwm1 同效
    ls_gtim_pwm pwm3(pwm1);
    // 拷贝赋值使用方法, 调用 pwm4 与调用 pwm2 同效
    ls_gtim_pwm pwm4 = pwm2;

    while (1)
    {
        pwm1.gtim_pwm_set_duty(1000);
        pwm2.gtim_pwm_set_duty(2000);
        sleep(1);
        pwm1.gtim_pwm_set_duty(3000);
        pwm2.gtim_pwm_set_duty(4000);
        sleep(1);
        pwm1.gtim_pwm_set_duty(5000);
        pwm2.gtim_pwm_set_duty(6000);
        sleep(1);
        pwm1.gtim_pwm_set_duty(7000);
        pwm2.gtim_pwm_set_duty(8000);
        sleep(1);
    }
}

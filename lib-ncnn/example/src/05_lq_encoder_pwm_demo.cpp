#include "lq_all_demo.hpp"

/********************************************************************************
 * @brief   编码器 PWM 输出模式测试.
 * @param   none.
 * @return  none.
 * @note    初始构造方法有两种, 多文件赋值使用时也有两种方法.
 ********************************************************************************/
void lq_encoder_pwm_demo(void)
{
    ls_encoder_pwm enc1(ENC_PWM0_PIN64, PIN_72);
    ls_encoder_pwm enc2(ENC_PWM1_PIN65, PIN_73);
    ls_encoder_pwm enc3(ENC_PWM2_PIN66, PIN_74);
    ls_encoder_pwm enc4(ENC_PWM3_PIN67, PIN_75);

    while(1)
    {
        printf("encoder count1: %-6.2f\n", enc1.encoder_get_count());
        printf("encoder count2: %-6.2f\n", enc2.encoder_get_count());
        printf("encoder count3: %-6.2f\n", enc3.encoder_get_count());
        printf("encoder count4: %-6.2f\n\n", enc4.encoder_get_count());
        usleep(50000);
    }
}

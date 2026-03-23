#include "lq_all_demo.hpp"

/********************************************************************************
 * @brief   GPIO 输出模式测试.
 * @param   none.
 * @return  none.
 * @note    GPIO 输出测试, 使用引脚 74 作为输出引脚, 交替输出高电平和低电平.
            该引脚位于母板编码器 3 所用引脚上.
 ********************************************************************************/
void lq_gpio_output_demo(void)
{
    // 初始化 GPIO 引脚 74 为输出模式
    ls_gpio gpio(PIN_74, GPIO_MODE_OUT);

    while (1)
    {
        gpio.gpio_level_set(GPIO_HIGH);
        usleep(5000);
        gpio.gpio_level_set(GPIO_LOW);
        usleep(5000);
    }
}

/********************************************************************************
 * @brief   GPIO 输入模式测试.
 * @param   none.
 * @return  none.
 * @note    GPIO 输入测试, 使用引脚 74 作为输入引脚, 读取当前电平.
            该引脚位于母板编码器 3 所用引脚上.
 ********************************************************************************/
void lq_gpio_input_demo(void)
{
    // 初始化 GPIO 引脚 74 为输入模式
    ls_gpio gpio(PIN_74, GPIO_MODE_IN);

    while (1)
    {
        printf("gpio 77 value = %d\n", gpio.gpio_level_get());  // 获取并打印当前电平值
        usleep(5000);
    }
}

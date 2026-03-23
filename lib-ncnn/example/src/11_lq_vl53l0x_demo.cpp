#include "lq_all_demo.hpp"

/********************************************************************************
 * @brief   VL53L0X 设备驱动测试.
 * @param   none.
 * @return  none.
 * @note    VL53L0X 设备驱动测试.
 * @note    使用前需要加载 driver 目录下的 VL53L0X 设备驱动.
 ********************************************************************************/
void lq_vl53l0x_demo(void)
{
    uint16_t dis;

    lq_i2c_vl53l0x vl53l0x;

    while (1)
    {
        printf("VL53L0X distance = %05u\n\n", vl53l0x.get_vl53l0x_dis());
        usleep(100*100);
    }
}

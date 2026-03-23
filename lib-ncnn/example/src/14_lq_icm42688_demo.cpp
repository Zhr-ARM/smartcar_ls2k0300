#include "lq_all_demo.hpp"

/********************************************************************************
 * @brief   ICM42688 设备驱动测试.
 * @param   none.
 * @return  none.
 * @note    ICM42688 设备驱动测试.
 * @note    使用前需要加载 driver 目录下的 ICM42688 设备驱动.
 ********************************************************************************/
void lq_icm42688_demo(void)
{
    uint8_t id;
    int16_t ax, ay, az, gx, gy, gz;

    lq_i2c_icm42688 icm42688;

    while (1)
    {
        icm42688.get_icm42688_gyro(&ax, &ay, &az, &gx, &gy, &gz);
        printf("ID = 0x%02x, ax=%08d, ay=%08d, az=%08d, gx=%08d, gy=%08d, gz=%08d, tem=%7.2f\n\n",
                icm42688.get_icm42688_id(), ax, ay, az, gx, gy, gz, icm42688.get_icm42688_tem());
        usleep(100*100);
    }
}
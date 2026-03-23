#include "lq_all_demo.hpp"

/********************************************************************************
 * @brief   LSM6DSR 设备驱动测试.
 * @param   none.
 * @return  none.
 * @note    LSM6DSR 设备驱动测试.
 * @note    使用前需要加载 driver 目录下的 LSM6DSR 设备驱动.
 ********************************************************************************/
void lq_lsm6dsr_demo(void)
{
    uint8_t id;
    int16_t ax, ay, az, gx, gy, gz;

    lq_i2c_lsm6dsr lsm6dsr;

    while (1)
    {
        lsm6dsr.get_lsm6dsr_gyro(&ax, &ay, &az, &gx, &gy, &gz);
        printf("ID = 0x%02x, ax=%08d, ay=%08d, az=%08d, gx=%08d, gy=%08d, gz=%08d\n\n", lsm6dsr.get_lsm6dsr_id(), ax, ay, az, gx, gy, gz);
        usleep(100*100);
    }
}
#include "lq_all_demo.hpp"

/********************************************************************************
 * @brief   MPU6050 设备驱动测试.
 * @param   none.
 * @return  none.
 * @note    MPU6050 设备驱动测试.
 * @note    使用前需要加载 driver 目录下的 MPU6050 设备驱动.
 ********************************************************************************/
void lq_mpu6050_demo(void)
{
    uint8_t id;
    int16_t ax, ay, az, gx, gy, gz;

    lq_i2c_mpu6050 mpu6050;

    while (1)
    {
        mpu6050.get_mpu6050_gyro(&ax, &ay, &az, &gx, &gy, &gz);
        printf("ID = 0x%02x, ax=%05d, ay=%05d, az=%05d, gx=%05d, gy=%05d, gz=%05d\n\n", mpu6050.get_mpu6050_id(), ax, ay, az, gx, gy, gz);
        usleep(100*100);
    }
}

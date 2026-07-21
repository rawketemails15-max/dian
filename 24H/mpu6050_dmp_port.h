#ifndef MPU6050_DMP_PORT_H_
#define MPU6050_DMP_PORT_H_

#include <stdint.h>

int mpu6050_dmp_i2c_write(uint8_t slaveAddress, uint8_t registerAddress,
    uint8_t length, const uint8_t *data);
int mpu6050_dmp_i2c_read(uint8_t slaveAddress, uint8_t registerAddress,
    uint8_t length, uint8_t *data);
void mpu6050_dmp_delay_ms(unsigned long durationMs);
void mpu6050_dmp_get_ms(unsigned long *timeMs);

#endif

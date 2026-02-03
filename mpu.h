#ifndef MPU_H
#define MPU_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define MPU6050_ADDR 0x68
#define I2C_PORT i2c0
#define SDA_PIN 4
#define SCL_PIN 5

void mpu_init();
void mpu_read_raw(int16_t accel[3], int16_t gyro[3]);
float kalman_filter(float new_angle, float new_rate, float dt);

#endif

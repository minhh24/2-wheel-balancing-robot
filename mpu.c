#include "mpu.h"
#include <math.h>

// Các biến trạng thái của bộ lọc Kalman
static float Q_angle = 0.001f;   // Nhiễu quá trình cho góc
static float Q_bias  = 0.003f;   // Nhiễu quá trình cho bias (độ lệch)
static float R_measure = 0.03f;  // Nhiễu đo lường từ Accelerometer

static float angle = 0.0f; 
static float bias = 0.0f;
static float P[2][2] = {{0, 0}, {0, 0}};

void mpu_init() {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN); gpio_pull_up(SCL_PIN);

    uint8_t wake_up[] = {0x6B, 0x00};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, wake_up, 2, false);
    
    uint8_t dlpf_config[] = {0x1A, 0x05}; // DLPF ~10Hz
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, dlpf_config, 2, false);
}

void mpu_read_raw(int16_t accel[3], int16_t gyro[3]) {
    uint8_t buffer[6], reg = 0x3B;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 6, false);
    accel[0] = (buffer[0] << 8) | buffer[1];
    accel[1] = (buffer[2] << 8) | buffer[3];
    accel[2] = (buffer[4] << 8) | buffer[5];

    reg = 0x43;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 6, false);
    gyro[0] = (buffer[0] << 8) | buffer[1];
    gyro[1] = (buffer[2] << 8) | buffer[3];
    gyro[2] = (buffer[4] << 8) | buffer[5];
}

// Thuật toán Kalman để ước lượng góc nghiêng
float kalman_filter(float new_angle, float new_rate, float dt) {
    // Bước 1: Dự đoán trạng thái tiếp theo
    float rate = new_rate - bias;
    angle += dt * rate;

    // Bước 2: Cập nhật hiệp phương sai sai số dự đoán
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Bước 3: Tính toán hệ số Kalman Gain
    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Bước 4: Hiệu chỉnh góc và bias dựa trên giá trị đo thực tế
    float y = new_angle - angle;
    angle += K[0] * y;
    bias  += K[1] * y;

    // Bước 5: Cập nhật hiệp phương sai sai số sau khi hiệu chỉnh
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}

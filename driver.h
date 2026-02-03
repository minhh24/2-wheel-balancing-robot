#ifndef DRIVER_H
#define DRIVER_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"

// Cấu hình chân (như cũ)
#define DIR_L 16
#define STEP_L 17
#define EN_L 18
#define DIR_R 19
#define STEP_R 20
#define EN_R 21

// --- CẤU HÌNH CHO 1/8 BƯỚC ---
#define MICROSTEPS 8 
// Tần số tối đa cần tăng lên để bù cho việc chia nhỏ bước
// 1/8 bước cần 1600 xung/vòng. Max 20 vòng/giây -> 32000Hz là đẹp
#define MAX_STEP_FREQ 32000.0f 

// Xung nhịp sau khi qua bộ chia (Clock Div)
// Pico 125MHz / 12.5 (Div) = 10MHz
#define PWM_DIV_INTEGER 12
#define PWM_DIV_FRAC 8 // 8/16 = 0.5 -> Tổng div = 12.5
#define CLOCKED_FREQ 10000000.0f 

void motors_init();
void step_set_speed(float speed_l, float speed_r);

#endif

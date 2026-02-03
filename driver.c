#include "driver.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <math.h>

void motors_init() {
    uint pins[] = {DIR_L, EN_L, DIR_R, EN_R};
    for(int i = 0; i < 4; i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_OUT);
    }

    // Kéo EN xuống LOW để bật Driver
    gpio_put(EN_L, 0); 
    gpio_put(EN_R, 0);

    // Cấu hình chân STEP chạy PWM
    gpio_set_function(STEP_L, GPIO_FUNC_PWM);
    gpio_set_function(STEP_R, GPIO_FUNC_PWM);

    // --- FIX LỖI GIẬT: Cài đặt bộ chia tần số (Clock Divider) ---
    uint slice_l = pwm_gpio_to_slice_num(STEP_L);
    uint slice_r = pwm_gpio_to_slice_num(STEP_R);

    // Chia tần số 125MHz xuống còn 10MHz để đếm được nhịp chậm hơn
    // Giúp motor quay mượt ở tốc độ thấp (cân bằng)
    pwm_set_clkdiv_int_frac(slice_l, PWM_DIV_INTEGER, PWM_DIV_FRAC);
    pwm_set_clkdiv_int_frac(slice_r, PWM_DIV_INTEGER, PWM_DIV_FRAC);

    pwm_set_enabled(slice_l, true);
    pwm_set_enabled(slice_r, true);
}

static void hardware_drive(uint step_pin, uint dir_pin, float speed, bool reverse) {
    float abs_speed = fabsf(speed);
    uint slice = pwm_gpio_to_slice_num(step_pin);
    
    // Giảm vùng chết (Deadzone) xuống cực nhỏ vì 1/8 bước rất mịn
    // Giúp xe phản ứng ngay cả với góc nghiêng nhỏ xíu
    if (abs_speed < 0.001f) { 
        pwm_set_gpio_level(step_pin, 0); // Duty = 0 (Tắt xung)
        return; 
    }

    // Xác định chiều quay
    gpio_put(dir_pin, (speed > 0) ^ reverse);

    // --- TÍNH TOÁN TẦN SỐ CHO 1/8 BƯỚC ---
    // target_freq = %Tốc độ * Max_Freq (32kHz)
    float target_freq = abs_speed * MAX_STEP_FREQ;
    
    // Đảm bảo tần số tối thiểu để không bị lỗi chia cho 0
    // 100Hz ở 1/8 bước là rất chậm (~0.06 vòng/giây) -> Siêu mượt
    if (target_freq < 50.0f) target_freq = 50.0f;

    // Tính giá trị Wrap (Chu kỳ) dựa trên Clock đã chia (10MHz)
    // Wrap = 10,000,000 / Freq
    uint32_t wrap = (uint32_t)(CLOCKED_FREQ / target_freq) - 1;
    
    // Kẹp giá trị trong giới hạn 16-bit (65535)
    if (wrap > 65535) wrap = 65535;

    // Cập nhật PWM
    pwm_set_wrap(slice, wrap);
    pwm_set_gpio_level(step_pin, wrap / 2); // Duty Cycle 50%
}

void step_set_speed(float speed_l, float speed_r) {
    hardware_drive(STEP_L, DIR_L, speed_l, false);
    hardware_drive(STEP_R, DIR_R, speed_r, true);
}

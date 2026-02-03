#include "lqr.h"
#include <math.h>

float LQR_Compute(LQR_State *s, const LQR_Params *p, float tilt, float gyro, float dt) {
    if (fabsf(tilt) > 45.0f || !p->system_enabled) {
        s->x_wheel_pos = 0; s->x_wheel_vel = 0; s->current_speed_cmd = 0;
        return 0.0f;
    }

    // 1. Cập nhật biến trạng thái
    s->x_tilt = (tilt - p->param_offset) * (M_PI / 180.0f);
    s->x_tilt_vel = gyro * (M_PI / 180.0f);
    s->x_wheel_pos += s->x_wheel_vel * dt;

    // 2. Tính toán LQR
    // Lưu ý: Với 1/8 bước, hệ thống sẽ phản ứng "mềm" hơn.
    // Nếu thấy xe vẫn yếu, bạn có thể tăng Max Power trên GUI lên.
    float balance_term = (p->K[0] * s->x_tilt) + (p->K[1] * s->x_tilt_vel);
    float pos_error = s->x_wheel_pos - p->target_position;
    float pos_term = (p->K[2] * s->x_wheel_vel) + (p->K[3] * pos_error);
    
    float target_output = (balance_term + pos_term) * p->param_max_power;

    // 3. Bão hòa đầu ra
    if (target_output > 1.0f) target_output = 1.0f;
    if (target_output < -1.0f) target_output = -1.0f;

    s->current_speed_cmd = target_output;
    s->x_wheel_vel = s->current_speed_cmd;
    
    return s->current_speed_cmd;
}

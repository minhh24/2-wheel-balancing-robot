#ifndef LQR_H
#define LQR_H
#include <stdbool.h>

typedef struct { float K[4], param_offset, target_position, param_max_power; bool system_enabled; } LQR_Params;
typedef struct { float x_tilt, x_tilt_vel, x_wheel_pos, x_wheel_vel, current_speed_cmd; } LQR_State;

float LQR_Compute(LQR_State *s, const LQR_Params *p, float tilt, float gyro, float dt);
#endif

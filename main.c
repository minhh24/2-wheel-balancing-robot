#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "pico/critical_section.h"
#include "mpu.h"
#include "driver.h"
#include "lqr.h"

// --- micro-ROS Headers ---
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <rmw_microros/rmw_microros.h>
#include "pico_uart_transports.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// --- Biến toàn cục ---
critical_section_t g_crit_sec;
// Params mặc định: K_Tilt=1.5, K_Gyro=0.05, K_Speed=0.04, Offset=0
LQR_Params g_params = {{1.5, 0.05, 0.04, 0.0}, 0.0, 0.0, 0.4, false};
LQR_State g_state = {0};
float current_tilt = 0;

// ROS Objects
rcl_publisher_t pub;
rcl_subscription_t sub;
std_msgs__msg__Float32MultiArray msg_p; // Message nhận (Params)
std_msgs__msg__Float32MultiArray msg_d; // Message gửi (Debug)

// Bộ đệm dữ liệu
float buf_p[8]; // Buffer nhận 8 tham số
float buf_d[3]; // Buffer gửi 3 giá trị [Angle, Pos, PWM]

// --- Callback nhận tham số từ Python ---
void param_cb(const void *msin) {
    const std_msgs__msg__Float32MultiArray *m = (const std_msgs__msg__Float32MultiArray *)msin;
    if (m->data.size >= 8) {
        critical_section_enter_blocking(&g_crit_sec);
        
        // Cập nhật các hệ số K
        for(int i=0; i<4; i++) g_params.K[i] = m->data.data[i];
        
        g_params.param_offset = m->data.data[4];
        g_params.system_enabled = (m->data.data[5] > 0.5f);
        g_params.param_max_power = m->data.data[6];
        g_params.target_position = m->data.data[7];
        
        // Reset trạng thái nếu tắt hệ thống
        if (!g_params.system_enabled) {
            g_state.x_wheel_pos = 0;
            g_state.x_wheel_vel = 0;
            g_state.current_speed_cmd = 0;
        }
        critical_section_exit(&g_crit_sec);
        
        // Nháy LED báo hiệu nhận lệnh thành công
        gpio_put(25, !gpio_get(25));
    }
}

// --- Callback Timer 10ms (Vòng lặp điều khiển) ---
bool timer_cb(struct repeating_timer *t) {
    int16_t a[3], g[3]; 
    mpu_read_raw(a, g);
    
    // Tính góc nghiêng
    float ang = atan2f(a[1], a[2]) * 180.0f / M_PI;
    float rate = g[0] / 131.0f;
    current_tilt = kalman_filter(ang, rate, 0.01f);
    
    // Tính toán LQR
    critical_section_enter_blocking(&g_crit_sec);
    float u = LQR_Compute(&g_state, &g_params, current_tilt, rate, 0.01f);
    critical_section_exit(&g_crit_sec);
    
    // Điều khiển động cơ
    step_set_speed(u, u);
    
    return true;
}

// --- Main Program ---
int main() {
    // 1. Cấu hình Transport
    rmw_uros_set_custom_transport(
        true, NULL, 
        pico_serial_transport_open, 
        pico_serial_transport_close, 
        pico_serial_transport_write, 
        pico_serial_transport_read
    );

    // 2. Khởi tạo ngoại vi
    gpio_init(25); gpio_set_dir(25, GPIO_OUT); // LED
    mpu_init();
    motors_init(); // <--- Lỗi undefined reference là do hàm này chưa có trong driver.c
    critical_section_init(&g_crit_sec);
    
    // 3. Khởi tạo Timer
    struct repeating_timer rt;
    add_repeating_timer_ms(-10, timer_cb, NULL, &rt);

    // 4. Khởi tạo micro-ROS
    rcl_allocator_t alc = rcl_get_default_allocator();
    rclc_support_t sup; 
    rclc_support_init(&sup, 0, NULL, &alc);
    
    rcl_node_t node; 
    rclc_node_init_default(&node, "pico_node", "", &sup);

    // Init Pub/Sub
    rclc_publisher_init_default(&pub, &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/lqr_debug");
    rclc_subscription_init_default(&sub, &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/lqr_params");

    // Gán bộ nhớ đệm
    msg_d.data.capacity = 3; msg_d.data.size = 3; msg_d.data.data = buf_d;
    msg_p.data.capacity = 8; msg_p.data.size = 0; msg_p.data.data = buf_p;

    rclc_executor_t exe; 
    rclc_executor_init(&exe, &sup.context, 1, &alc);
    rclc_executor_add_subscription(&exe, &sub, &msg_p, &param_cb, ON_NEW_DATA);

    // 5. Loop chính
    while(1) {
        // Cập nhật dữ liệu debug
        buf_d[0] = current_tilt; 
        buf_d[1] = g_state.x_wheel_pos; 
        buf_d[2] = g_state.current_speed_cmd;
        
        rcl_publish(&pub, &msg_d, NULL);
        rclc_executor_spin_some(&exe, RCL_MS_TO_NS(10));
    }
}

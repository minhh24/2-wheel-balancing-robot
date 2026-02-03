cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# --- CẤU HÌNH ĐƯỜNG DẪN (Giữ nguyên của bạn) ---
set(PICO_SDK_PATH "/home/pc34/micro_ros_ws/src/pico-sdk")
set(MICROROS_PATH "/home/pc34/micro_ros_ws/src/micro_ros_raspberrypi_pico_sdk/libmicroros")

set(PICO_BOARD pico CACHE STRING "Board type")
include(pico_sdk_import.cmake)

project(lab_mpu6050 C CXX ASM)
pico_sdk_init()

# Cấu hình Micro-ROS
include_directories(${MICROROS_PATH}/include)
link_directories(${MICROROS_PATH})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# --- QUAN TRỌNG: Thêm đầy đủ các file .c vào đây ---
add_executable(lab_mpu6050 
    main.c 
    driver.c 
    mpu.c 
    lqr.c 
    pico_uart_transport.c
)

# --- CẤU HÌNH NGÕ RA ---
pico_enable_stdio_usb(lab_mpu6050 0)
pico_enable_stdio_uart(lab_mpu6050 1)

# --- KHẮC PHỤC LỖI TẠI ĐÂY ---
target_link_libraries(lab_mpu6050
        pico_stdlib
        microros          
        hardware_pio
        hardware_i2c      
        hardware_uart     
        hardware_pwm      
        hardware_adc    
        pico_sync 
        )

pico_add_extra_outputs(lab_mpu6050)

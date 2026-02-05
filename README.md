# THIẾT KẾ XE CÂN BẰNG HAI BÁNH BẰNG THUẬT TOÁN LQR VÀ MICRO-ROS KẾT HỢP ROS2

Dự án này nghiên cứu và chế tạo mô hình xe tự cân bằng hai bánh sử dụng động cơ bước (Stepper Motor), tích hợp thuật toán điều khiển tối ưu LQR và bộ lọc Kalman trên vi điều khiển Raspberry Pi Pico. Hệ thống sử dụng kiến trúc phân tán với micro-ROS trên robot và ROS 2 trên máy tính để giám sát và tinh chỉnh tham số thời gian thực.

## Tính năng chính

* **Thuật toán LQR (Linear Quadratic Regulator):** Điều khiển đa biến (Góc nghiêng, Vận tốc góc, Vị trí, Vận tốc xe) giúp xe đứng vững và chống nhiễu tốt hơn PID.
* **Giao tiếp thời gian thực:** Sử dụng micro-ROS qua UART để truyền nhận dữ liệu giữa Pico và PC với độ trễ thấp.
* **Giám sát và Tinh chỉnh (Live Tuning):** Giao diện GUI Python (Tkinter + Matplotlib) cho phép vẽ đồ thị đáp ứng và thay đổi hệ số K trực tiếp mà không cần nạp lại firmware.
* **Bộ lọc Kalman:** Lọc nhiễu tín hiệu từ cảm biến quán tính MPU6050.
* **Điều khiển vi bước (Microstepping):** Sử dụng chế độ 1/8 bước với Driver TMC2209 để giảm rung động và tiếng ồn động cơ.

## Phần cứng (Hardware)

* **Vi điều khiển:** Raspberry Pi Pico (RP2040)
* **Cảm biến:** GY-521 (MPU6050 - Accelerometer + Gyroscope)
* **Động cơ:** 2x Động cơ bước Nema 17 (Góc bước 1.8 độ)
* **Driver động cơ:** 2x TMC2209 (Hỗ trợ UART/STEP/DIR)
* **Nguồn:** Pin Li-ion 4S (14.8V - 16.8V) + Mạch hạ áp LM7805 (5V cho Pico)
* **Giao tiếp:** Module chuyển đổi UART-to-USB (CP2102)
* **Khung xe:** Nhôm cắt CNC và in 3D

### Sơ đồ nối dây (Pinout)

| Linh kiện      | Chân Linh Kiện | Chân Pico (GPIO) | Ghi chú                          |
| :---           | :---           | :---             | :---                             |
| **MPU6050** | SDA            | GP4 (I2C0 SDA)   | Cần điện trở kéo lên (Pull-up)   |
|                | SCL            | GP5 (I2C0 SCL)   | Cần điện trở kéo lên (Pull-up)   |
|                | VCC            | 3V3 (OUT)        |                                  |
|                | GND            | GND              |                                  |
| **Motor Trái** | DIR            | GP16             |                                  |
|                | STEP           | GP17             | Chân PWM                         |
|                | EN             | GP18             | Kéo LOW để bật Driver            |
| **Motor Phải** | DIR            | GP19             |                                  |
|                | STEP           | GP20             | Chân PWM                         |
|                | EN             | GP21             | Kéo LOW để bật Driver            |
| **UART (ROS)** | TX             | GP0 (UART0 TX)   | Nối vào RX của CP2102            |
|                | RX             | GP1 (UART0 RX)   | Nối vào TX của CP2102            |
| **Debug** | LED            | GP25             | Nháy khi nhận lệnh từ PC         |

**Lưu ý về phần cứng:**
1. Driver TMC2209 cần cấp nguồn động lực (VM) trực tiếp từ Pin (12V-24V).
2. Phải nối chung Mass (GND) giữa Pin, Pico và Driver.
3. Cấu hình TMC2209 ở chế độ 1/8 bước (sử dụng Jump MS1/MS2 nếu cần).

## Cài đặt và Biên dịch

### Yêu cầu phần mềm
* Hệ điều hành: Ubuntu 22.04 (Khuyến nghị) hoặc Windows (WSL2).
* ROS 2 Distribution: Humble Hawksbill.
* Raspberry Pi Pico SDK (C/C++).
* Python 3 và các thư viện: `tkinter`, `matplotlib`, `rclpy`.
  
### link video demo
https://www.youtube.com/watch?v=C-vb4eZdHiY

### link báo cáo
https://drive.google.com/file/d/1e-q8qSSbdxxI8e2oFaYWDY7Kf4qRvYYJ/view

### Cấu hình Project và Biên dịch Firmware
Tại thư mục dự án (trên Linux/Ubuntu):

```bash
# Tạo thư mục build
mkdir build
cd build

# Cấu hình CMake
cmake ..

# Biên dịch
make



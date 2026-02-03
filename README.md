# DO AN 2: THIET KE XE CAN BANG HAI BANH BANG THUAT TOAN LQR VA MICRO-ROS

Du an nay nghien cuu va che tao mo hinh xe tu can bang hai banh su dung dong co buoc (Stepper Motor), tich hop thuat toan dieu khien toi uu LQR va bo loc Kalman tren vi dieu khien Raspberry Pi Pico. He thong su dung kien truc phan tan voi micro-ROS tren robot va ROS 2 tren may tinh de giam sat va tinh chinh tham so thoi gian thuc.

## Tinh nang chinh

* **Thuat toan LQR (Linear Quadratic Regulator):** Dieu khien da bien (Goc nghieng, Van toc goc, Vi tri, Van toc xe) giup xe dung vung va chong nhieu tot hon PID.
* **Giao tiep thoi gian thuc:** Su dung micro-ROS qua UART de truyen nhan du lieu giua Pico va PC voi do tre thap.
* **Giam sat va Tinh chinh (Live Tuning):** Giao dien GUI Python (Tkinter + Matplotlib) cho phep ve do thi dap ung va thay doi he so K truc tiep ma khong can nap lai firmware.
* **Bo loc Kalman:** Loc nhieu tin hieu tu cam bien quan tinh MPU6050.
* **Dieu khien vi buoc (Microstepping):** Su dung che do 1/8 buoc voi Driver TMC2209 de giam rung dong va tieng on dong co.

## Phan cung (Hardware)

* **Vi dieu khien:** Raspberry Pi Pico (RP2040)
* **Cam bien:** GY-521 (MPU6050 - Accelerometer + Gyroscope)
* **Dong co:** 2x Dong co buoc Nema 17 (Goc buoc 1.8 do)
* **Driver dong co:** 2x TMC2209 (Ho tro UART/STEP/DIR)
* **Nguon:** Pin Li-ion 4S (14.8V - 16.8V) + Mach ha ap LM7805 (5V cho Pico)
* **Giao tiep:** Module chuyen doi UART-to-USB (CP2102)
* **Khung xe:** Nhom cat CNC va in 3D

### So do noi day (Pinout)

| Linh kien      | Chan Linh Kien | Chan Pico (GPIO) | Ghi chu                          |
| :---           | :---           | :---             | :---                             |
| **MPU6050** | SDA            | GP4 (I2C0 SDA)   | Can dien tro keo len (Pull-up)   |
|                | SCL            | GP5 (I2C0 SCL)   | Can dien tro keo len (Pull-up)   |
|                | VCC            | 3V3 (OUT)        |                                  |
|                | GND            | GND              |                                  |
| **Motor Trai** | DIR            | GP16             |                                  |
|                | STEP           | GP17             | Chan PWM                         |
|                | EN             | GP18             | Keo LOW de bat Driver            |
| **Motor Phai** | DIR            | GP19             |                                  |
|                | STEP           | GP20             | Chan PWM                         |
|                | EN             | GP21             | Keo LOW de bat Driver            |
| **UART (ROS)** | TX             | GP0 (UART0 TX)   | Noi vao RX cua CP2102            |
|                | RX             | GP1 (UART0 RX)   | Noi vao TX cua CP2102            |
| **Debug** | LED            | GP25             | Nhang khi nhan lenh tu PC        |

**Luu y ve phan cung:**
1. Driver TMC2209 can cap nguon dong luc (VM) truc tiep tu Pin (12V-24V).
2. Phai noi chung Mass (GND) giua Pin, Pico va Driver.
3. Cau hinh TMC2209 o che do 1/8 buoc (su dung Jump MS1/MS2 neu can).

## Cai dat va Bien dich

### Yeu cau phan mem
* He dieu hanh: Ubuntu 22.04 (Khuyen nghi) hoac Windows (WSL2).
* ROS 2 Distribution: Humble Hawksbill.
* Raspberry Pi Pico SDK (C/C++).
* Python 3 va cac thu vien: `tkinter`, `matplotlib`, `rclpy`.

### 1. Cau hinh Project va Bien dich Firmware
Tai thu muc du an (tren Linux/Ubuntu):

```bash
# Tao thu muc build
mkdir build
cd build

# Cau hinh CMake
cmake ..

# Bien dich
make

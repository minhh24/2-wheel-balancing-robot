import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk
from tkinter import ttk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import time
from collections import deque

# --- ROS2 NODE (Xử lý giao tiếp nền) ---
class LQRTunerNode(Node):
    def __init__(self):
        super().__init__('lqr_tuner_gui')
        
        # Publisher: Gửi tham số xuống Robot
        # Cấu trúc mảng: [Kp_tilt, Kd_tilt, Kp_speed, Kp_pos, Offset, Enable, MaxPower, TargetPos]
        self.pub = self.create_publisher(Float32MultiArray, '/lqr_params', 10)
        
        # Subscriber: Nhận data vẽ đồ thị
        self.sub = self.create_subscription(Float32MultiArray, '/lqr_debug', self.debug_callback, 10)
        
        # Bộ đệm dữ liệu (Deque tự động xóa cũ khi đầy) -> Fix tràn RAM
        self.maxlen = 200
        self.angle_data = deque(maxlen=self.maxlen)
        self.pos_data = deque(maxlen=self.maxlen)
        
        self.connected = False
        self.last_msg_time = 0

    def debug_callback(self, msg):
        # Cập nhật thời gian nhận tin cuối cùng
        self.last_msg_time = time.time()
        self.connected = True
        
        # msg data từ Pico: [angle, pos, pwm_cmd]
        if len(msg.data) >= 2:
            self.angle_data.append(msg.data[0])
            self.pos_data.append(msg.data[1])

    def send_params_to_pico(self, params_list):
        msg = Float32MultiArray()
        # Chuyển đổi sang list float chuẩn Python để tránh lỗi type
        msg.data = [float(x) for x in params_list]
        self.pub.publish(msg)
        # In ra terminal để debug xem có gửi được không
        # print(f"Sent: {msg.data}") 

# --- GUI CLASS (Giao diện người dùng) ---
class App:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title("LQR CONTROL - TUNING DASHBOARD")
        self.root.geometry("1200x800")
        
        # Style
        style = ttk.Style()
        style.theme_use('clam')
        
        # Layout: Chia 2 cột (Trái: Control, Phải: Graph)
        paned = tk.PanedWindow(root, orient=tk.HORIZONTAL, sashrelief=tk.RAISED, sashwidth=5)
        paned.pack(fill=tk.BOTH, expand=True)
        
        left_frame = tk.Frame(paned, width=380, bg="#eeeeee")
        right_frame = tk.Frame(paned, bg="white")
        
        paned.add(left_frame)
        paned.add(right_frame)

        # --- CỘT TRÁI: ĐIỀU KHIỂN ---
        self.create_controls(left_frame)
        
        # --- CỘT PHẢI: ĐỒ THỊ ---
        self.setup_plots(right_frame)

        # Biến trạng thái motor
        self.is_running = False
        
        # Bắt đầu vòng lặp cập nhật GUI (Chạy mỗi 100ms = 10FPS để đỡ lag)
        self.update_gui_loop()

    def create_controls(self, parent):
        # 1. Master Control
        f1 = tk.LabelFrame(parent, text="MASTER CONTROL", font=("Arial", 10, "bold"), bg="#ddd")
        f1.pack(fill="x", padx=10, pady=5)
        
        self.lbl_status = tk.Label(f1, text="WAITING CONNECTION...", bg="gray", fg="white", font=("Arial", 10, "bold"))
        self.lbl_status.pack(fill="x", padx=5, pady=5)
        
        self.btn_action = tk.Button(f1, text="START MOTOR", bg="green", fg="white", font=("Arial", 12, "bold"), command=self.toggle_motor)
        self.btn_action.pack(fill="x", padx=5, pady=5)
        
        self.var_max_power = self.create_slider(f1, "MAX POWER (0.0 - 1.0)", 0.0, 1.0, 0.01, 0.4)

        # 2. Balance Loop
        f2 = tk.LabelFrame(parent, text="1. BALANCE (Giữ xe đứng)", font=("Arial", 10, "bold"), bg="#ddd")
        f2.pack(fill="x", padx=10, pady=5)
        
        self.var_k_tilt = self.create_slider(f2, "K Tilt (P - Góc) [Max 40]", 0, 40, 0.1, 1.5)
        self.var_k_gyro = self.create_slider(f2, "K Gyro (D - Tốc độ góc) [Max 5]", 0, 5, 0.01, 0.05)
        self.var_offset = self.create_slider(f2, "Offset (Cân bằng tĩnh)", -10, 10, 0.1, 0.0)

        # 3. Position Loop
        f3 = tk.LabelFrame(parent, text="2. POSITION (Về Home)", font=("Arial", 10, "bold"), bg="#ddd")
        f3.pack(fill="x", padx=10, pady=5)
        
        tk.Label(f3, text="Kéo K_Pos lên để xe tự về chỗ cũ", bg="#ddd", fg="red").pack(anchor="w", padx=5)
        self.var_k_speed = self.create_slider(f3, "K Speed (Damping) [Max 20]", 0, 20, 0.1, 0.04)
        self.var_k_pos = self.create_slider(f3, "K Pos (Distance) [Max 20]", 0, 20, 0.1, 0.0)

        # 4. Navigation
        f4 = tk.LabelFrame(parent, text="3. NAVIGATION", font=("Arial", 10, "bold"), bg="#ddd")
        f4.pack(fill="x", padx=10, pady=5)
        
        self.var_target = self.create_slider(f4, "Target Position (Mét)", -2.0, 2.0, 0.05, 0.0)
        tk.Button(f4, text="RESET HOME (Set 0)", command=lambda: self.var_target.set(0)).pack(fill="x", padx=5, pady=5)

    def create_slider(self, parent, label, min_v, max_v, res, default):
        f = tk.Frame(parent, bg="#ddd")
        f.pack(fill="x", padx=5, pady=2)
        tk.Label(f, text=label, bg="#ddd", anchor="w").pack(fill="x")
        var = tk.DoubleVar(value=default)
        s = tk.Scale(f, from_=min_v, to=max_v, resolution=res, orient=tk.HORIZONTAL, variable=var, bg="#ddd")
        s.pack(fill="x")
        return var

    def setup_plots(self, parent):
        self.fig = Figure(figsize=(5, 5), dpi=100)
        
        # Subplot 1: Angle
        self.ax1 = self.fig.add_subplot(211)
        self.ax1.set_title("Tilt Angle (Deg)")
        self.ax1.grid(True, linestyle='--', alpha=0.6)
        self.ax1.set_ylim(-15, 15)
        self.line_angle, = self.ax1.plot([], [], 'r-', label="Real Angle")
        self.line_setpoint, = self.ax1.plot([], [], 'g--', label="Setpoint")
        self.ax1.legend(loc="upper right")

        # Subplot 2: Position
        self.ax2 = self.fig.add_subplot(212)
        self.ax2.set_title("Position (Meters)")
        self.ax2.grid(True, linestyle='--', alpha=0.6)
        self.ax2.set_ylim(-0.2, 0.2)
        self.line_pos, = self.ax2.plot([], [], 'b-')

        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    def toggle_motor(self):
        # Đảo trạng thái Enable
        self.is_running = not self.is_running
        if self.is_running:
            self.btn_action.config(text="STOP", bg="red")
        else:
            self.btn_action.config(text="START MOTOR", bg="green")
        
        # Gửi lệnh NGAY LẬP TỨC khi bấm nút
        self.send_current_params()

    def send_current_params(self):
        # Thu thập toàn bộ giá trị slider
        # Thứ tự phải khớp với code Pico: [K_tilt, K_gyro, K_speed, K_pos, Offset, Enable, MaxPower, Target]
        params = [
            self.var_k_tilt.get(),
            self.var_k_gyro.get(),
            self.var_k_speed.get(),
            self.var_k_pos.get(),
            self.var_offset.get(),
            1.0 if self.is_running else 0.0, # Enable Flag (1.0 = ON)
            self.var_max_power.get(),
            self.var_target.get()
        ]
        self.node.send_params_to_pico(params)

    def update_gui_loop(self):
        # 1. Kiểm tra kết nối
        if time.time() - self.node.last_msg_time > 1.0:
            self.lbl_status.config(text="DISCONNECTED", bg="gray")
            self.node.connected = False
        else:
            self.lbl_status.config(text="CONNECTED", bg="#007acc")

        # 2. Gửi tham số định kỳ (Sync Slider)
        # Gửi liên tục để Pico cập nhật giá trị Slider realtime
        self.send_current_params()

        # 3. Vẽ đồ thị (QUAN TRỌNG: FIX LỖI CRASH)
        if self.node.connected and len(self.node.angle_data) > 1:
            # FIX LỖI: Tạo bản sao (snapshot) của dữ liệu trước khi vẽ
            # Để tránh việc luồng ROS thêm dữ liệu vào giữa chừng làm lệch kích thước trục X, Y
            current_angle = list(self.node.angle_data)
            current_pos = list(self.node.pos_data)
            
            x_axis = range(len(current_angle))
            
            # Cập nhật Line Angle
            self.line_angle.set_data(x_axis, current_angle)
            self.line_setpoint.set_data(x_axis, [0]*len(current_angle))
            self.ax1.set_xlim(0, len(current_angle))
            
            # Auto scale Y cho Angle để không bị mất nét vẽ
            min_y = min(min(current_angle), -5)
            max_y = max(max(current_angle), 5)
            self.ax1.set_ylim(min_y - 2, max_y + 2)

            # Cập nhật Line Position (Kiểm tra độ dài cho chắc)
            if len(current_pos) == len(current_angle):
                self.line_pos.set_data(x_axis, current_pos)
                self.ax2.set_xlim(0, len(current_pos))
                self.ax2.set_ylim(min(current_pos)-0.05, max(current_pos)+0.05)

            # Vẽ lại
            self.canvas.draw()

        # Gọi lại hàm này sau 100ms (10 FPS) -> Giảm lag so với code cũ (thường để 1ms hoặc 10ms)
        self.root.after(100, self.update_gui_loop)

# --- MAIN ENTRY ---
def main():
    rclpy.init()
    node = LQRTunerNode()
    
    # Chạy ROS Spin ở luồng riêng (Daemon Thread)
    # Để GUI không bị treo khi chờ ROS nhận tin
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()

    root = tk.Tk()
    app = App(root, node)
    
    # Xử lý khi tắt cửa sổ
    def on_closing():
        root.destroy()
        rclpy.shutdown()
        
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == '__main__':
    main()

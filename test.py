import serial
import serial.tools.list_ports
import threading
import queue
import time
import tkinter as tk
from tkinter import ttk, messagebox
from typing import Optional, Tuple
import sys

# -------------------------- 全局配置 --------------------------
DEFAULT_BAUDRATE = 115200
SUPPORTED_BAUDRATES = [9600, 19200, 38400, 57600, 115200, 230400]

# 回零判定阈值
HOMING_ANGLE_TOLERANCE = 0.01  # 角度误差范围
HOMING_STABLE_TIME = 0.5       # 稳定时间(秒)

# -------------------------- 全局变量与锁 --------------------------
data_queue = queue.Queue(maxsize=1)
serial_lock = threading.Lock() 

# 状态标志
is_listening = False
is_connected = False
is_homing = False  

# 全局最新状态
global_latest_status = {"status": "0", "angle": 0.0, "updated": 0}

ser = None
listen_thread = None
homing_thread = None

# -------------------------- 串口底层函数 --------------------------
def get_available_com_ports() -> list:
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def connect_serial(com_port: str, baudrate: int) -> bool:
    global ser, is_connected, is_listening
    try:
        ser = serial.Serial(
            port=com_port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.05
        )
        if ser.is_open:
            is_connected = True
            is_listening = True
            start_listen_thread()
            return True
        return False
    except Exception as e:
        print(f"连接失败：{e}")
        return False

def disconnect_serial(force: bool = False) -> None:
    global ser, is_connected, is_listening, is_homing
    is_listening = False
    is_homing = False 
    time.sleep(0.1)
    
    if is_connected and ser and ser.is_open:
        try:
            with serial_lock:
                ser.write(b"$1st\r\n") 
        except:
            pass
    
    if ser and ser.is_open:
        try:
            ser.close()
        except:
            pass
    is_connected = False

def send_command(cmd_content: str) -> str:
    global ser
    if not is_connected or not ser:
        return "错误：未连接"
    full_cmd = f"$1{cmd_content}\r\n"
    try:
        with serial_lock:
            ser.write(full_cmd.encode("ascii"))
        return f"发送: {full_cmd.strip()}"
    except Exception as e:
        return f"发送失败: {e}"

def parse_status(data: str) -> Tuple[Optional[str], Optional[str], Optional[str], Optional[float]]:
    if len(data) < 14 or not data.startswith("$1"):
        return None, None, None, None
    try:
        content = data[2:].strip()
        alarm = content[0]
        status = content[1]
        seq = content[2:4]
        angle_val = float(content[4:12])
        if angle_val > 360.0: 
            angle_val -= 720.0
        return alarm, status, seq, angle_val
    except:
        return None, None, None, None

# -------------------------- 监听线程 --------------------------
def listen_serial():
    global ser
    buffer = ""
    while is_listening and ser and ser.is_open:
        try:
            if ser.in_waiting:
                raw = ser.read(ser.in_waiting).decode('ascii', errors='replace')
                buffer += raw
                if '\n' in buffer:
                    lines = buffer.split('\n')
                    buffer = lines[-1] 
                    for line in reversed(lines[:-1]):
                        line = line.strip()
                        if line.startswith('$1') and len(line) >= 14:
                            alarm, status, seq, angle = parse_status(line)
                            if angle is not None:
                                global_latest_status["status"] = status
                                global_latest_status["angle"] = angle
                                global_latest_status["updated"] = time.time()
                                
                                if data_queue.full():
                                    try: data_queue.get_nowait()
                                    except: pass
                                data_queue.put((alarm, status, seq, angle))
                            break 
            else:
                time.sleep(0.002)
        except Exception:
            time.sleep(0.1)

def start_listen_thread():
    global listen_thread
    listen_thread = threading.Thread(target=listen_serial, daemon=True)
    listen_thread.start()

# -------------------------- 回零逻辑 --------------------------
def homing_task(status_callback, finish_callback):
    global is_homing
    send_command("1") 
    status_callback(">>> 正在回零... (点击停车可取消)")
    stable_start_time = None
    
    while is_homing:
        curr_angle = global_latest_status["angle"]
        if abs(curr_angle) <= HOMING_ANGLE_TOLERANCE:
            if stable_start_time is None:
                stable_start_time = time.time()
                status_callback(f"接近零位 ({curr_angle:.4f}°)，确认中...")
            elif time.time() - stable_start_time >= HOMING_STABLE_TIME:
                status_callback("✅ 回零完成！")
                is_homing = False
                finish_callback() 
                return
        else:
            stable_start_time = None 
        time.sleep(0.1)
        
    status_callback("⚠️ 回零已中断")
    finish_callback()

def start_homing(status_cb, finish_cb):
    global is_homing, homing_thread
    if is_homing: return 
    is_homing = True
    homing_thread = threading.Thread(target=homing_task, args=(status_cb, finish_cb), daemon=True)
    homing_thread.start()

# -------------------------- GUI 界面 --------------------------
class TurntableGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("单轴转台控制系统 V1.5 (修复波特率)")
        self.root.geometry("900x650")
        
        self.com_var = tk.StringVar()
        self.baud_var = tk.IntVar(value=DEFAULT_BAUDRATE)
        self.status_msg = tk.StringVar(value="准备就绪")
        
        self.var_dir = tk.IntVar(value=0)     
        self.var_acc = tk.StringVar(value="10")
        self.var_spd = tk.StringVar(value="20.0")
        self.var_ang = tk.StringVar(value="90.0")
        self.var_loop = tk.StringVar(value="1") 
        self.var_swing_amp = tk.StringVar(value="10.0")
        self.var_swing_freq = tk.StringVar(value="0.5")

        self.setup_ui()
        self.root.after(1000, self.refresh_ports)
        self.update_ui_loop()

    def setup_ui(self):
        # 1. 顶部连接栏
        top_frame = ttk.Frame(self.root, padding=10)
        top_frame.pack(fill=tk.X)
        
        # 端口
        ttk.Label(top_frame, text="端口:").pack(side=tk.LEFT)
        self.cb_port = ttk.Combobox(top_frame, textvariable=self.com_var, width=12)
        self.cb_port.pack(side=tk.LEFT, padx=5)
        
        # 【修复】波特率选择框加回来了
        ttk.Label(top_frame, text="波特率:").pack(side=tk.LEFT, padx=(10, 0))
        self.cb_baud = ttk.Combobox(top_frame, textvariable=self.baud_var, values=SUPPORTED_BAUDRATES, width=8)
        self.cb_baud.pack(side=tk.LEFT, padx=5)

        # 连接按钮
        ttk.Button(top_frame, text="连接 / 断开", command=self.toggle_connect).pack(side=tk.LEFT, padx=10)
        
        # 2. 状态显示区
        status_frame = ttk.LabelFrame(self.root, text="实时状态", padding=15)
        status_frame.pack(fill=tk.X, padx=10, pady=5)
        self.lbl_angle = ttk.Label(status_frame, text="0.0000°", font=("Helvetica", 40, "bold"), foreground="blue")
        self.lbl_angle.pack(side=tk.LEFT, padx=20)
        self.lbl_detail = ttk.Label(status_frame, text="状态: -- | 报警: --", font=("Arial", 12))
        self.lbl_detail.pack(side=tk.LEFT, padx=20)

        # 3. 参数设置区
        param_frame = ttk.LabelFrame(self.root, text="运行参数设置", padding=10)
        param_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(param_frame, text="旋转方向:").grid(row=0, column=0, sticky=tk.W, pady=5)
        ttk.Radiobutton(param_frame, text="顺时针 (CW)", variable=self.var_dir, value=0).grid(row=0, column=1, sticky=tk.W)
        ttk.Radiobutton(param_frame, text="逆时针 (CCW)", variable=self.var_dir, value=1).grid(row=0, column=2, sticky=tk.W)

        ttk.Label(param_frame, text="加速度 (°/s²):").grid(row=1, column=0, sticky=tk.W, pady=5)
        ttk.Entry(param_frame, textvariable=self.var_acc, width=10).grid(row=1, column=1, sticky=tk.W)
        
        ttk.Label(param_frame, text="运行速度 (°/s):").grid(row=1, column=2, sticky=tk.W, pady=5)
        ttk.Entry(param_frame, textvariable=self.var_spd, width=10).grid(row=1, column=3, sticky=tk.W)

        ttk.Label(param_frame, text="目标角度 (°):").grid(row=2, column=0, sticky=tk.W, pady=5)
        ttk.Entry(param_frame, textvariable=self.var_ang, width=10).grid(row=2, column=1, sticky=tk.W)
        
        ttk.Label(param_frame, text="多圈圈数:").grid(row=2, column=2, sticky=tk.W, pady=5)
        ttk.Entry(param_frame, textvariable=self.var_loop, width=10).grid(row=2, column=3, sticky=tk.W)

        ttk.Label(param_frame, text="[摇摆] 幅度(°):").grid(row=3, column=0, sticky=tk.W, pady=5)
        ttk.Entry(param_frame, textvariable=self.var_swing_amp, width=10).grid(row=3, column=1, sticky=tk.W)
        ttk.Label(param_frame, text="[摇摆] 频率(Hz):").grid(row=3, column=2, sticky=tk.W, pady=5)
        ttk.Entry(param_frame, textvariable=self.var_swing_freq, width=10).grid(row=3, column=3, sticky=tk.W)

        # 4. 操作按钮区
        btn_frame = ttk.LabelFrame(self.root, text="操作指令", padding=10)
        btn_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        f_base = ttk.Frame(btn_frame)
        f_base.pack(fill=tk.X, pady=5)
        self.btn_en = ttk.Button(f_base, text="伺服使能 (ON)", command=lambda: self.safe_send("mo=1"))
        self.btn_en.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        
        self.btn_dis = ttk.Button(f_base, text="伺服释放 (OFF)", command=lambda: self.safe_send("mo=0"))
        self.btn_dis.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        
        self.btn_stop = ttk.Button(f_base, text="🛑 立即停车", command=self.do_stop_all) 
        self.btn_stop.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        f_move = ttk.Frame(btn_frame)
        f_move.pack(fill=tk.X, pady=5)
        
        self.btn_pos = ttk.Button(f_move, text="执行位置模式", command=self.do_pos_mode)
        self.btn_pos.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        
        self.btn_spd = ttk.Button(f_move, text="执行速率模式", command=self.do_spd_mode)
        self.btn_spd.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        
        self.btn_mc = ttk.Button(f_move, text="多圈模式", command=self.do_multi_mode)
        self.btn_mc.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        
        self.btn_swing = ttk.Button(f_move, text="摇摆模式", command=self.do_swing_mode)
        self.btn_swing.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        
        self.btn_home = ttk.Button(f_move, text="🏠 自动回零", command=self.do_homing)
        self.btn_home.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        ttk.Label(self.root, textvariable=self.status_msg, relief=tk.SUNKEN, anchor=tk.W).pack(side=tk.BOTTOM, fill=tk.X)

        self.lockable_buttons = [
            self.btn_en, self.btn_dis, 
            self.btn_pos, self.btn_spd, self.btn_mc, self.btn_swing, self.btn_home
        ]

    # --- 核心逻辑 ---

    def safe_send(self, cmd):
        if is_homing:
            messagebox.showwarning("禁止操作", "正在回零中！\n请等待回零完成，或点击“立即停车”终止。")
            return
        msg = send_command(cmd)
        self.status_msg.set(msg)

    def do_stop_all(self):
        global is_homing
        if is_homing:
            is_homing = False 
            self.status_msg.set("正在终止回零...")
        
        send_command("st")
        self.status_msg.set("已发送停车指令")
        self.toggle_buttons(True) 

    def do_homing(self):
        if not is_connected: 
            messagebox.showerror("错误", "未连接串口")
            return
        if is_homing: return

        self.toggle_buttons(False)
        start_homing(
            status_cb=lambda msg: self.status_msg.set(msg),
            finish_cb=lambda: self.toggle_buttons(True)
        )

    def toggle_buttons(self, enable: bool):
        state = tk.NORMAL if enable else tk.DISABLED
        for btn in self.lockable_buttons:
            btn.config(state=state)

    # --- 参数读取 ---
    def get_params(self):
        try:
            d = self.var_dir.get()
            a = int(self.var_acc.get())
            v = float(self.var_spd.get())
            ang = float(self.var_ang.get())
            loop = int(self.var_loop.get())
            a = max(1, min(1000, a))
            v = max(0.0001, min(1000.0, v))
            return d, a, v, ang, loop
        except ValueError:
            messagebox.showerror("格式错误", "请输入有效的数字参数！")
            return None

    def do_pos_mode(self):
        p = self.get_params()
        if p:
            cmd = f"2{p[0]}{p[1]:04d}{p[2]:09.4f}{p[3]:08.4f}"
            self.safe_send(cmd)

    def do_spd_mode(self):
        p = self.get_params()
        if p:
            cmd = f"3{p[0]}{p[1]:04d}{p[2]:09.4f}"
            self.safe_send(cmd)

    def do_multi_mode(self):
        p = self.get_params()
        if p:
            cmd = f"5{p[0]}{p[1]:04d}{p[2]:09.4f}{p[3]:08.4f}{p[4]:02d}"
            self.safe_send(cmd)

    def do_swing_mode(self):
        try:
            amp = float(self.var_swing_amp.get())
            freq = float(self.var_swing_freq.get())
            cmd = f"4{amp:08.4f}{freq:06.3f}"
            self.safe_send(cmd)
        except ValueError:
            messagebox.showerror("错误", "摇摆参数无效")

    # --- 系统功能 ---
    def refresh_ports(self):
        ports = get_available_com_ports()
        self.cb_port['values'] = ports
        if ports and not self.com_var.get():
            self.com_var.set(ports[0])
        self.root.after(3000, self.refresh_ports)

    def toggle_connect(self):
        if not is_connected:
            if connect_serial(self.com_var.get(), self.baud_var.get()):
                self.status_msg.set("连接成功")
            else:
                messagebox.showerror("错误", "无法打开串口")
        else:
            self.do_stop_all() 
            disconnect_serial(force=True)
            self.status_msg.set("已断开")

    def update_ui_loop(self):
        try:
            while not data_queue.empty():
                alarm, status, seq, angle = data_queue.get_nowait()
                self.lbl_angle.config(text=f"{angle:.4f}°")
                self.lbl_detail.config(text=f"状态码: {status} | 报警码: {alarm} | 帧计数: {seq}")
                if alarm != '0':
                    self.lbl_angle.config(foreground="red")
                else:
                    self.lbl_angle.config(foreground="blue")
        except:
            pass
        self.root.after(20, self.update_ui_loop)

    def on_close(self):
        self.do_stop_all()
        disconnect_serial(force=True)
        self.root.destroy()
        sys.exit(0)

if __name__ == "__main__":
    root = tk.Tk()
    app = TurntableGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
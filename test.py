import serial
import serial.tools.list_ports
import threading
import queue
import time
import tkinter as tk
from tkinter import ttk, messagebox
import sys

# -------------------------- 全局配置 --------------------------
DEFAULT_BAUDRATE = 115200
SUPPORTED_BAUDRATES = [9600, 19200, 38400, 57600, 115200, 230400]

# -------------------------- 全局变量 --------------------------
ui_queue = queue.Queue(maxsize=1)
cmd_execution_lock = threading.Lock() # 互斥锁，防止指令冲突

is_listening = False
is_connected = False
is_homing = False

# 全局最新状态 (原子操作更新)
global_state = {
    "status": "0",  # 初始默认为释放
    "alarm": "0",
    "angle": 0.0,
    "seq": "00",
    "last_update_time": 0
}

ser = None
listen_thread = None

# -------------------------- 串口底层 --------------------------
def get_available_com_ports() -> list:
    return [p.device for p in serial.tools.list_ports.comports()]

def connect_serial(com_port: str, baudrate: int) -> bool:
    global ser, is_connected, is_listening
    try:
        ser = serial.Serial(
            port=com_port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.02
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
    
    if is_connected and ser and ser.is_open:
        try:
            ser.write(b"$1st\r\n") # 尝试停车
            time.sleep(0.05)
            ser.write(b"$1mo=0\r\n") # 尝试释放
        except: pass
        try: ser.close()
        except: pass
    is_connected = False

def send_raw_bytes(cmd_str: str):
    global ser
    if is_connected and ser:
        try:
            full_cmd = f"$1{cmd_str}\r\n"
            ser.write(full_cmd.encode("ascii"))
            return True
        except:
            return False
    return False

# -------------------------- 监听线程 (高频刷新) --------------------------
def parse_status(data: str):
    if len(data) < 14 or not data.startswith("$1"): return None
    try:
        content = data[2:].strip()
        alarm = content[0]
        status = content[1]
        seq = content[2:4]
        angle_val = float(content[4:12])
        if angle_val > 360.0: angle_val -= 720.0
        return {"alarm": alarm, "status": status, "seq": seq, "angle": angle_val}
    except:
        return None

def listen_serial_loop():
    global ser, global_state
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
                            res = parse_status(line)
                            if res:
                                # 更新全局状态
                                global_state.update(res)
                                global_state["last_update_time"] = time.time()
                                
                                # 推送UI
                                if ui_queue.full():
                                    try: ui_queue.get_nowait()
                                    except: pass
                                ui_queue.put(res)
                            break
            else:
                time.sleep(0.001) # 1ms 微休眠，极速响应
        except:
            time.sleep(0.1)

def start_listen_thread():
    global listen_thread
    listen_thread = threading.Thread(target=listen_serial_loop, daemon=True)
    listen_thread.start()

# -------------------------- 核心：V1.9 极速状态流转 --------------------------

def execute_command_sequence(cmd_str, status_msg_updater):
    """
    V1.9 极速切换逻辑：
    1. 判断当前状态。
    2. 若需停车，发送指令。
    3. 进入 10ms 高频检测循环：
       - 一旦检测到状态变 '1'，立即 break 并发送指令 (零延迟)。
       - 若状态长时间未变，自动补发停车指令 (防丢包)。
    """
    def task():
        with cmd_execution_lock: # 锁住，确保指令按顺序执行
            current = global_state["status"]
            
            # 1. 检查是否释放
            if current == '0' and "mo=1" not in cmd_str:
                status_msg_updater("⚠️ 错误：请先使能电机")
                return

            # 2. 如果已经在伺服状态(1)，直接秒发
            if current == '1':
                if send_raw_bytes(cmd_str):
                    # status_msg_updater(f"✅ 发送: {cmd_str}") # 可选：不弹这个，直接发
                    pass
                return

            # 3. 需要切换模式：先发停车
            status_msg_updater("⏳ 正在停止转台...")
            send_raw_bytes("st")
            
            # --- 主动轮询循环 ---
            start_t = time.time()
            last_resend_t = time.time()
            success = False
            
            while time.time() - start_t < 8.0: # 最长允许8秒刹车时间
                s = global_state["status"]
                
                # [核心判定]：只要状态变成 1 (伺服) 或 0 (释放)，立刻跳出
                if s == '1':
                    success = True
                    break
                if s == '0':
                    status_msg_updater("⚠️ 异常：电机被释放")
                    return

                # [智能补发]：如果还在运行状态(不是1也不是8)，每0.5秒补发一次st
                # 这能有效解决“显示超时”的问题
                if s not in ['1', '8', '0']:
                    if time.time() - last_resend_t > 0.5:
                        send_raw_bytes("st") # 补发
                        last_resend_t = time.time()
                
                # 极速轮询：只睡 10ms
                time.sleep(0.01)

            if not success:
                status_msg_updater(f"❌ 切换失败 (卡在状态 {global_state['status']})")
                return

            # 4. 成功停稳，立即发送新指令
            if send_raw_bytes(cmd_str):
                status_msg_updater(f"✅ 切换完成，指令执行")
            else:
                status_msg_updater("❌ 发送失败")

    threading.Thread(target=task, daemon=True).start()

# -------------------------- 回零任务 --------------------------
def homing_logic(status_updater, finish_callback):
    global is_homing
    
    # 暴力停车
    send_raw_bytes("st")
    time.sleep(0.2)
    
    send_raw_bytes("1")
    status_updater(">>> 正在回零... (点击停车可取消)")
    
    stable_start = None
    
    while is_homing:
        ang = global_state["angle"]
        # 判定归零 (0.01度以内)
        if abs(ang) <= 0.01:
            if stable_start is None:
                stable_start = time.time()
            elif time.time() - stable_start > 0.5:
                status_updater("✅ 回零成功！")
                is_homing = False
                break
        else:
            stable_start = None
            
        time.sleep(0.05) # 提高采样率
        if not is_connected: break
            
    finish_callback()

def start_homing_task(status_updater, finish_callback):
    global is_homing
    if is_homing: return
    is_homing = True
    threading.Thread(target=homing_logic, args=(status_updater, finish_callback), daemon=True).start()

# -------------------------- GUI 界面 --------------------------
class TurntableGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("单轴转台控制系统 V1.9 (极速主动轮询版)")
        self.root.geometry("920x700")
        
        self.com_var = tk.StringVar()
        self.baud_var = tk.IntVar(value=DEFAULT_BAUDRATE)
        self.status_msg = tk.StringVar(value="请连接串口...")
        
        # 参数
        self.var_dir = tk.IntVar(value=0)
        self.var_acc = tk.StringVar(value="10")
        self.var_spd = tk.StringVar(value="20.0")
        self.var_ang = tk.StringVar(value="90.0")
        self.var_loop = tk.StringVar(value="1")
        self.var_swing_amp = tk.StringVar(value="10.0")
        self.var_swing_freq = tk.StringVar(value="0.5")

        self.setup_ui()
        self.root.after(500, self.refresh_ports)
        self.update_ui_loop()

    def setup_ui(self):
        # 顶部
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill=tk.X)
        ttk.Label(top, text="端口:").pack(side=tk.LEFT)
        self.cb_port = ttk.Combobox(top, textvariable=self.com_var, width=15)
        self.cb_port.pack(side=tk.LEFT, padx=5)
        ttk.Label(top, text="波特率:").pack(side=tk.LEFT)
        self.cb_baud = ttk.Combobox(top, textvariable=self.baud_var, values=SUPPORTED_BAUDRATES, width=8)
        self.cb_baud.pack(side=tk.LEFT, padx=5)
        self.btn_connect = tk.Button(top, text="🔌 连接设备", bg="#e1e1e1", command=self.toggle_connect, width=12)
        self.btn_connect.pack(side=tk.LEFT, padx=15)

        # 状态
        stat_frame = ttk.LabelFrame(self.root, text="实时监控", padding=15)
        stat_frame.pack(fill=tk.X, padx=10, pady=5)
        self.lbl_angle = ttk.Label(stat_frame, text="0.0000°", font=("Helvetica", 42, "bold"), foreground="#ccc")
        self.lbl_angle.pack(side=tk.LEFT, padx=20)
        info_f = ttk.Frame(stat_frame)
        info_f.pack(side=tk.LEFT, padx=20)
        self.lbl_state_txt = ttk.Label(info_f, text="状态: 未连接", font=("Arial", 12))
        self.lbl_state_txt.pack(anchor=tk.W)
        self.lbl_mode_txt = ttk.Label(info_f, text="模式: --", font=("Arial", 12, "bold"))
        self.lbl_mode_txt.pack(anchor=tk.W)

        # 参数
        param_f = ttk.LabelFrame(self.root, text="参数设置", padding=10)
        param_f.pack(fill=tk.X, padx=10, pady=5)
        ttk.Label(param_f, text="方向:").grid(row=0, column=0, sticky=tk.W)
        ttk.Radiobutton(param_f, text="顺时针(CW)", variable=self.var_dir, value=0).grid(row=0, column=1)
        ttk.Radiobutton(param_f, text="逆时针(CCW)", variable=self.var_dir, value=1).grid(row=0, column=2)
        ttk.Label(param_f, text="加速度(°/s²):").grid(row=1, column=0, sticky=tk.W, pady=5)
        ttk.Entry(param_f, textvariable=self.var_acc, width=8).grid(row=1, column=1, sticky=tk.W)
        ttk.Label(param_f, text="速度(°/s):").grid(row=1, column=2, sticky=tk.W)
        ttk.Entry(param_f, textvariable=self.var_spd, width=8).grid(row=1, column=3, sticky=tk.W)
        ttk.Label(param_f, text="角度(°):").grid(row=2, column=0, sticky=tk.W, pady=5)
        ttk.Entry(param_f, textvariable=self.var_ang, width=8).grid(row=2, column=1, sticky=tk.W)
        ttk.Label(param_f, text="圈数:").grid(row=2, column=2, sticky=tk.W)
        ttk.Entry(param_f, textvariable=self.var_loop, width=8).grid(row=2, column=3, sticky=tk.W)
        ttk.Label(param_f, text="摇摆幅度(°):").grid(row=3, column=0, sticky=tk.W, pady=5)
        ttk.Entry(param_f, textvariable=self.var_swing_amp, width=8).grid(row=3, column=1, sticky=tk.W)
        ttk.Label(param_f, text="摇摆频率(Hz):").grid(row=3, column=2, sticky=tk.W)
        ttk.Entry(param_f, textvariable=self.var_swing_freq, width=8).grid(row=3, column=3, sticky=tk.W)

        # 按钮
        ctrl_f = ttk.LabelFrame(self.root, text="控制面板", padding=10)
        ctrl_f.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        row1 = ttk.Frame(ctrl_f)
        row1.pack(fill=tk.X, pady=5)
        self.btn_en = ttk.Button(row1, text="⚡ 伺服使能", command=lambda: send_raw_bytes("mo=1"))
        self.btn_en.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=3)
        self.btn_dis = ttk.Button(row1, text="💤 释放电机", command=lambda: send_raw_bytes("mo=0"))
        self.btn_dis.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=3)
        self.btn_stop = tk.Button(row1, text="🛑 立即停车", bg="#ffcccc", command=self.do_stop_all)
        self.btn_stop.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=3)
        row2 = ttk.Frame(ctrl_f)
        row2.pack(fill=tk.X, pady=5)
        self.btn_pos = ttk.Button(row2, text="位置模式", command=self.do_pos_mode)
        self.btn_pos.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        self.btn_spd = ttk.Button(row2, text="速率模式", command=self.do_spd_mode)
        self.btn_spd.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        self.btn_mc = ttk.Button(row2, text="多圈模式", command=self.do_multi_mode)
        self.btn_mc.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        self.btn_swing = ttk.Button(row2, text="摇摆模式", command=self.do_swing_mode)
        self.btn_swing.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        self.btn_home = ttk.Button(row2, text="🏠 回零", command=self.do_homing)
        self.btn_home.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Label(self.root, textvariable=self.status_msg, relief=tk.SUNKEN, anchor=tk.W).pack(side=tk.BOTTOM, fill=tk.X)

        self.motion_btns = [self.btn_pos, self.btn_spd, self.btn_mc, self.btn_swing, self.btn_home, self.btn_dis]

    def refresh_ports(self):
        pts = get_available_com_ports()
        self.cb_port['values'] = pts
        if pts and not self.com_var.get(): self.com_var.set(pts[0])
        self.root.after(3000, self.refresh_ports)

    def toggle_connect(self):
        if not is_connected:
            if connect_serial(self.com_var.get(), self.baud_var.get()):
                self.status_msg.set("✅ 连接成功")
                self.btn_connect.config(text="🔴 断开连接", bg="#ffcccc")
            else:
                messagebox.showerror("错误", "无法打开串口")
        else:
            self.do_stop_all()
            disconnect_serial()
            self.status_msg.set("⚠️ 已断开连接")
            self.btn_connect.config(text="🔌 连接设备", bg="#e1e1e1")
            self.lbl_angle.config(text="0.0000°", foreground="#ccc")
            self.lbl_state_txt.config(text="状态: 未连接", foreground="black")
            self.lbl_mode_txt.config(text="模式: --")
            self.set_motion_enable(False)

    def set_motion_enable(self, enable):
        state = tk.NORMAL if enable else tk.DISABLED
        for btn in self.motion_btns: btn.config(state=state)
        self.btn_en.config(state=tk.DISABLED if enable else tk.NORMAL)

    def do_stop_all(self):
        global is_homing
        is_homing = False
        send_raw_bytes("st")
        self.status_msg.set("🛑 已发送停车")

    def do_homing(self):
        if is_homing: return
        self.set_motion_enable(False)
        start_homing_task(
            status_updater=lambda m: self.status_msg.set(m),
            finish_callback=lambda: self.status_msg.set("回零结束")
        )

    def send_cmd_safe(self, cmd):
        if is_homing:
            self.status_msg.set("⚠️ 回零中，请先停车")
            return
        execute_command_sequence(cmd, lambda m: self.status_msg.set(m))

    def get_p(self):
        try:
            return (self.var_dir.get(), int(self.var_acc.get()), float(self.var_spd.get()), 
                    float(self.var_ang.get()), int(self.var_loop.get()))
        except:
            messagebox.showerror("参数错误", "请输入数字")
            return None

    def do_pos_mode(self):
        p = self.get_p()
        if p: self.send_cmd_safe(f"2{p[0]}{max(1,min(1000,p[1])):04d}{max(0.1,p[2]):09.4f}{p[3]:08.4f}")

    def do_spd_mode(self):
        p = self.get_p()
        if p: self.send_cmd_safe(f"3{p[0]}{max(1,min(1000,p[1])):04d}{max(0.1,p[2]):09.4f}")

    def do_multi_mode(self):
        p = self.get_p()
        if p: self.send_cmd_safe(f"5{p[0]}{max(1,min(1000,p[1])):04d}{max(0.1,p[2]):09.4f}{p[3]:08.4f}{p[4]:02d}")

    def do_swing_mode(self):
        try:
            amp = float(self.var_swing_amp.get())
            freq = float(self.var_swing_freq.get())
            self.send_cmd_safe(f"4{amp:08.4f}{freq:06.3f}")
        except:
            messagebox.showerror("错误", "参数无效")

    def update_ui_loop(self):
        try:
            if not ui_queue.empty():
                state = ui_queue.get_nowait()
                angle = state['angle']
                status = state['status']
                alarm = state['alarm']
                
                self.lbl_angle.config(text=f"{angle:.4f}°")
                if alarm != '0':
                    self.lbl_angle.config(foreground="red")
                    self.lbl_state_txt.config(text=f"报警: {alarm}", foreground="red")
                else:
                    self.lbl_angle.config(foreground="#0055ff")
                    self.lbl_state_txt.config(text="状态: 正常", foreground="green")

                s_map = {'0':'释放', '1':'伺服保持', '2':'回零中', '3':'位置运行', '4':'速率运行', 
                         '5':'速率稳定', '6':'摇摆运行', '7':'摇摆稳定', '8':'停车中', '9':'多圈运行'}
                self.lbl_mode_txt.config(text=f"模式: {s_map.get(status, status)}")

                if not is_homing:
                    if status == '0':
                        self.set_motion_enable(False)
                        self.lbl_state_txt.config(text="提示: 请点击使能", foreground="orange")
                    else:
                        self.set_motion_enable(True)
        except: pass
        self.root.after(20, self.update_ui_loop)

    def on_close(self):
        self.do_stop_all()
        disconnect_serial()
        self.root.destroy()
        sys.exit(0)

if __name__ == "__main__":
    root = tk.Tk()
    app = TurntableGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
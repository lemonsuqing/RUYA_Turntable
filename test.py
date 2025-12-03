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
HOMING_ANGLE_TOLERANCE = 0.01  
HOMING_STABLE_TIME = 0.5       

# -------------------------- 全局变量 --------------------------
data_queue = queue.Queue(maxsize=1) # 仅保留最新帧，保证UI响应速度
serial_lock = threading.Lock() 

# 状态标志
is_listening = False
is_connected = False
is_homing = False  

# 核心状态监控
current_device_status = "0"  # 默认为0（空闲/释放）
current_device_alarm = "0"

ser = None
listen_thread = None
homing_thread = None

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
            timeout=0.02 # 极短超时，提高读写响应
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
    
    # 1. 停止监听和回零
    is_listening = False
    is_homing = False
    
    if is_connected and ser and ser.is_open:
        try:
            with serial_lock:
                # 2. 退出逻辑：先停车，再释放电机
                ser.write(b"$1st\r\n") 
                time.sleep(0.1) # 给硬件一点反应时间
                ser.write(b"$1mo=0\r\n") 
                time.sleep(0.05)
        except:
            pass
        try:
            ser.close()
        except:
            pass
            
    is_connected = False

def send_raw(cmd_str: str):
    """最底层的发送，不带任何逻辑"""
    global ser
    if is_connected and ser:
        try:
            full_cmd = f"$1{cmd_str}\r\n"
            with serial_lock:
                ser.write(full_cmd.encode("ascii"))
            return True
        except:
            return False
    return False

def smart_send_movement(cmd_str: str) -> str:
    """
    智能发送：
    如果当前不是伺服状态(1#)，先发送停车(st)，
    确保硬件进入可接收指令的状态，再发送运动指令。
    解决“点击无效”的问题。
    """
    if not is_connected: return "未连接"
    
    # 如果正在回零，直接拒绝
    if is_homing:
        return "回零中，禁止其他操作"

    # 协议要求：位置模式等只有在 1# (伺服) 状态下响应
    # 如果当前是 4#(速率运行) 或 5#(速率稳定) 或 8#(停车中)，
    # 直接发位置指令会被忽略。
    # 所以我们强制先发一个停车，再发指令。
    
    # 只有当已经是 1# 状态时，才不需要发停车？
    # 为了保险起见（以及响应用户"立即切换"的需求），
    # 只要不是 mo=0 释放状态，我们都先尝试打断
    
    try:
        if current_device_status != '0': 
            # 先发停车，打断上一个动作
            send_raw("st")
            # 关键：给硬件 50ms 状态切换时间 (人眼感觉不到延迟，但对MCU很重要)
            time.sleep(0.05) 
        
        # 发送实际指令
        send_raw(cmd_str)
        return f"已发送: {cmd_str}"
    except Exception as e:
        return f"发送异常: {e}"

def parse_status(data: str) -> Tuple[Optional[str], Optional[str], Optional[str], Optional[float]]:
    if len(data) < 14 or not data.startswith("$1"):
        return None, None, None, None
    try:
        content = data[2:].strip()
        alarm = content[0]
        status = content[1]
        seq = content[2:4]
        angle_val = float(content[4:12])
        if angle_val > 360.0: angle_val -= 720.0
        return alarm, status, seq, angle_val
    except:
        return None, None, None, None

# -------------------------- 监听线程 --------------------------
def listen_serial():
    global ser, current_device_status, current_device_alarm
    buffer = ""
    while is_listening and ser and ser.is_open:
        try:
            if ser.in_waiting:
                raw = ser.read(ser.in_waiting).decode('ascii', errors='replace')
                buffer += raw
                if '\n' in buffer:
                    lines = buffer.split('\n')
                    buffer = lines[-1] 
                    # 倒序寻找最新帧
                    for line in reversed(lines[:-1]):
                        line = line.strip()
                        if line.startswith('$1') and len(line) >= 14:
                            alarm, status, seq, angle = parse_status(line)
                            if angle is not None:
                                # 更新全局状态
                                current_device_status = status
                                current_device_alarm = alarm
                                
                                # 推送UI
                                if data_queue.full():
                                    try: data_queue.get_nowait()
                                    except: pass
                                data_queue.put((alarm, status, seq, angle))
                            break 
            else:
                time.sleep(0.002) 
        except:
            time.sleep(0.1)

def start_listen_thread():
    global listen_thread
    listen_thread = threading.Thread(target=listen_serial, daemon=True)
    listen_thread.start()

# -------------------------- 回零逻辑 --------------------------
def homing_task(status_callback, finish_callback):
    global is_homing
    # 回零也需要先停车确保能接收指令
    send_raw("st")
    time.sleep(0.05)
    send_raw("1") 
    
    status_callback(">>> 回零中... (点击红色停车按钮可取消)")
    stable_start_time = None
    last_angle = 999.0
    
    while is_homing:
        # 这里从全局变量读，不消耗队列
        # 实际开发中应该加锁，但Python基本类型读写原子性在GUI展示场景下可接受
        # 为了更准确，我们可以在这里 parse 队列，但为了不跟UI抢，
        # 我们直接假设 listen_thread 在更新 current_device_status 即可
        # 更好的方式：监听线程写入一个 shared_state 对象
        
        # 简易实现：直接读取UI队列里的最新值（如果有）或全局变量不方便
        # 我们依赖全局变量 update
        
        # 从队列里"偷窥"一下最新的角度
        # 由于我们在监听线程里已经更新了 global current_device_status 
        # 但没有 global angle。我们在 parse_status 里也没写 global angle
        # 让我们修正 parse_status 逻辑中的全局变量更新
        pass 
        # (下方的逻辑依赖UI线程更新的 angle，这里只做延时逻辑其实不太准)
        # 修正：回零判定放在UI线程或增加全局angle变量
        # 鉴于代码复杂度，我们在 UI update_ui_loop 里做回零判定更合理？
        # 不，还是保持独立线程，我们在 listen_thread 增加一个 global_angle
        
        time.sleep(0.1)
        
    # 由于逻辑调整，这里仅作为占位，实际判定逻辑我们移到 homing_logic_with_feedback
    finish_callback()

# 为了简化，我们使用一个全局字典来共享状态
machine_state = {"angle": 0.0, "status": "0", "updated": 0}

def listen_serial_v2():
    global ser, current_device_status
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
                                current_device_status = status # 核心状态更新
                                
                                # 更新共享状态
                                machine_state["angle"] = angle
                                machine_state["status"] = status
                                machine_state["alarm"] = alarm
                                machine_state["seq"] = seq
                                
                                if data_queue.full():
                                    try: data_queue.get_nowait()
                                    except: pass
                                data_queue.put(machine_state.copy())
                            break
            else:
                time.sleep(0.002)
        except:
            time.sleep(0.1)

def homing_logic(status_callback, finish_callback):
    global is_homing
    send_raw("st")
    time.sleep(0.05)
    send_raw("1")
    status_callback(">>> 正在回零... ")
    
    stable_start = None
    
    while is_homing:
        ang = machine_state["angle"]
        if abs(ang) <= HOMING_ANGLE_TOLERANCE:
            if stable_start is None:
                stable_start = time.time()
            elif time.time() - stable_start > HOMING_STABLE_TIME:
                status_callback("✅ 回零完成")
                is_homing = False
                break
        else:
            stable_start = None
        time.sleep(0.1)
    
    finish_callback()

def start_homing(status_cb, finish_cb):
    global is_homing, homing_thread
    if is_homing: return
    is_homing = True
    homing_thread = threading.Thread(target=homing_logic, args=(status_cb, finish_cb), daemon=True)
    homing_thread.start()

# -------------------------- GUI 界面 --------------------------
class TurntableGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("单轴转台控制系统 V1.6 (极速响应版)")
        self.root.geometry("900x680")
        
        self.com_var = tk.StringVar()
        self.baud_var = tk.IntVar(value=DEFAULT_BAUDRATE)
        self.status_msg = tk.StringVar(value="请连接串口...")
        
        # 参数变量
        self.var_dir = tk.IntVar(value=0)     
        self.var_acc = tk.StringVar(value="10")
        self.var_spd = tk.StringVar(value="20.0")
        self.var_ang = tk.StringVar(value="90.0")
        self.var_loop = tk.StringVar(value="1") 
        self.var_swing_amp = tk.StringVar(value="10.0")
        self.var_swing_freq = tk.StringVar(value="0.5")

        self.setup_ui()
        
        # 替换监听函数为 v2 版本
        global listen_serial
        listen_serial = listen_serial_v2
        
        self.root.after(1000, self.refresh_ports)
        self.update_ui_loop()

    def setup_ui(self):
        # 1. 顶部连接栏
        top_frame = ttk.Frame(self.root, padding=10)
        top_frame.pack(fill=tk.X)
        
        ttk.Label(top_frame, text="端口:").pack(side=tk.LEFT)
        self.cb_port = ttk.Combobox(top_frame, textvariable=self.com_var, width=12)
        self.cb_port.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(top_frame, text="波特率:").pack(side=tk.LEFT, padx=(10, 0))
        self.cb_baud = ttk.Combobox(top_frame, textvariable=self.baud_var, values=SUPPORTED_BAUDRATES, width=8)
        self.cb_baud.pack(side=tk.LEFT, padx=5)

        # 连接按钮 (使用特定样式)
        self.btn_connect = tk.Button(top_frame, text="🔌 连接串口", bg="#ddd", command=self.toggle_connect, width=15)
        self.btn_connect.pack(side=tk.LEFT, padx=15)
        
        # 2. 状态显示区
        status_frame = ttk.LabelFrame(self.root, text="实时状态", padding=15)
        status_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # 角度显示
        self.lbl_angle = ttk.Label(status_frame, text="0.0000°", font=("Helvetica", 40, "bold"), foreground="#aaa")
        self.lbl_angle.pack(side=tk.LEFT, padx=20)
        
        # 详细信息
        info_frame = ttk.Frame(status_frame)
        info_frame.pack(side=tk.LEFT, padx=20)
        self.lbl_status_txt = ttk.Label(info_frame, text="通信状态: 未连接", font=("Arial", 11))
        self.lbl_status_txt.pack(anchor=tk.W)
        self.lbl_mode_txt = ttk.Label(info_frame, text="工作模式: --", font=("Arial", 11, "bold"))
        self.lbl_mode_txt.pack(anchor=tk.W)

        # 3. 参数设置区
        param_frame = ttk.LabelFrame(self.root, text="运行参数设置", padding=10)
        param_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Grid 布局参数
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

        ttk.Label(param_frame, text="摇摆 幅度(°):").grid(row=3, column=0, sticky=tk.W, pady=5)
        ttk.Entry(param_frame, textvariable=self.var_swing_amp, width=10).grid(row=3, column=1, sticky=tk.W)
        ttk.Label(param_frame, text="摇摆 频率(Hz):").grid(row=3, column=2, sticky=tk.W, pady=5)
        ttk.Entry(param_frame, textvariable=self.var_swing_freq, width=10).grid(row=3, column=3, sticky=tk.W)

        # 4. 操作按钮区
        btn_frame = ttk.LabelFrame(self.root, text="操作指令", padding=10)
        btn_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        # 基础控制
        f_base = ttk.Frame(btn_frame)
        f_base.pack(fill=tk.X, pady=5)
        self.btn_en = ttk.Button(f_base, text="⚡ 伺服使能 (ON)", command=lambda: send_raw("mo=1"))
        self.btn_en.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)
        
        self.btn_dis = ttk.Button(f_base, text="💤 释放电机 (OFF)", command=lambda: send_raw("mo=0"))
        self.btn_dis.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)
        
        # 停车是最高优先级，用红色
        self.btn_stop = tk.Button(f_base, text="🛑 立即停车", bg="#ffcccc", command=self.do_stop_all) 
        self.btn_stop.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)

        # 运动模式
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

        # 需要根据“是否释放”来互斥的按钮
        self.motion_buttons = [
            self.btn_pos, self.btn_spd, self.btn_mc, self.btn_swing, self.btn_home,
            self.btn_dis # 释放按钮在释放状态下也没必要点
        ]

    # --- 逻辑处理 ---

    def do_stop_all(self):
        global is_homing
        is_homing = False 
        send_raw("st")
        self.status_msg.set("已发送停车指令")

    def do_homing(self):
        if not is_connected: return
        if is_homing: return
        
        # 禁用按钮
        self.set_motion_buttons_state(False)
        start_homing(
            status_cb=lambda msg: self.status_msg.set(msg),
            finish_cb=lambda: self.status_msg.set("回零结束") 
            # 按钮恢复由 update_ui_loop 的状态监控自动处理
        )

    def set_motion_buttons_state(self, enable: bool):
        state = tk.NORMAL if enable else tk.DISABLED
        for btn in self.motion_buttons:
            # 如果是 tk.Button (Stop/Connect) 和 ttk.Button 属性设置方式略有不同
            # 这里除了 Connect/Stop 都是 ttk
            try:
                btn.config(state=state)
            except:
                pass

    def get_params(self):
        try:
            d = self.var_dir.get()
            a = int(self.var_acc.get())
            v = float(self.var_spd.get())
            ang = float(self.var_ang.get())
            loop = int(self.var_loop.get())
            # 限幅
            a = max(1, min(1000, a))
            v = max(0.0001, min(1000.0, v))
            return d, a, v, ang, loop
        except ValueError:
            messagebox.showerror("参数错误", "请输入有效的数字")
            return None

    def do_pos_mode(self):
        p = self.get_params()
        if p:
            cmd = f"2{p[0]}{p[1]:04d}{p[2]:09.4f}{p[3]:08.4f}"
            msg = smart_send_movement(cmd)
            self.status_msg.set(msg)

    def do_spd_mode(self):
        p = self.get_params()
        if p:
            cmd = f"3{p[0]}{p[1]:04d}{p[2]:09.4f}"
            msg = smart_send_movement(cmd)
            self.status_msg.set(msg)

    def do_multi_mode(self):
        p = self.get_params()
        if p:
            cmd = f"5{p[0]}{p[1]:04d}{p[2]:09.4f}{p[3]:08.4f}{p[4]:02d}"
            msg = smart_send_movement(cmd)
            self.status_msg.set(msg)

    def do_swing_mode(self):
        try:
            amp = float(self.var_swing_amp.get())
            freq = float(self.var_swing_freq.get())
            cmd = f"4{amp:08.4f}{freq:06.3f}"
            msg = smart_send_movement(cmd)
            self.status_msg.set(msg)
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
                self.btn_connect.config(text="❌ 断开连接", bg="#ffcccc", fg="red")
            else:
                messagebox.showerror("错误", "无法打开串口")
        else:
            self.do_stop_all()
            disconnect_serial(force=True)
            self.status_msg.set("已断开")
            self.btn_connect.config(text="🔌 连接串口", bg="#ddd", fg="black")
            
            # 断开后重置显示
            self.lbl_angle.config(text="0.0000°", foreground="#aaa")
            self.lbl_status_txt.config(text="通信状态: 未连接", foreground="black")
            self.lbl_mode_txt.config(text="工作模式: --")

    def update_ui_loop(self):
        try:
            # 1. 读取串口数据刷新界面
            if not data_queue.empty():
                state = data_queue.get_nowait()
                angle = state['angle']
                status = state['status']
                alarm = state['alarm']
                
                # 角度显示
                self.lbl_angle.config(text=f"{angle:.4f}°")
                if alarm != '0':
                    self.lbl_angle.config(foreground="red")
                    self.lbl_status_txt.config(text=f"报警: 代码 {alarm}", foreground="red")
                else:
                    self.lbl_angle.config(foreground="#0033cc") # 正常蓝色
                    
                # 状态文本映射
                status_map = {
                    '0': '电机释放 (空闲)', '1': '伺服保持', '2': '回零中',
                    '3': '位置模式运行', '4': '速率模式运行', '5': '速率稳定',
                    '6': '摇摆运行', '7': '摇摆稳定', '8': '停车中', '9': '多圈运行'
                }
                s_text = status_map.get(status, f"未知({status})")
                self.lbl_mode_txt.config(text=f"工作模式: {s_text}")
                self.lbl_status_txt.config(text="通信状态: 正常", foreground="green")

                # 2. 核心互斥逻辑：根据状态控制按钮可用性
                # 如果正在回零，全部禁用（除了停车）
                if is_homing:
                    self.set_motion_buttons_state(False)
                    self.btn_en.config(state=tk.DISABLED)
                else:
                    # 如果状态是 0 (释放)，禁用运动指令，启用使能
                    if status == '0':
                        self.set_motion_buttons_state(False)
                        self.btn_en.config(state=tk.NORMAL)
                        self.lbl_status_txt.config(text="提示: 请先点击'伺服使能'", foreground="#cc6600")
                    else:
                        # 状态非0 (已使能/运行中)，启用运动指令，禁用使能按钮(防止重复点)
                        self.set_motion_buttons_state(True)
                        self.btn_en.config(state=tk.DISABLED)
                        
                        # 特殊：如果在运行中，释放按钮应该可用
                        self.btn_dis.config(state=tk.NORMAL)

        except:
            pass
        self.root.after(20, self.update_ui_loop)

    def on_close(self):
        # 退出前彻底清理
        self.do_stop_all()
        disconnect_serial(force=True) # 内部包含 mo=0
        self.root.destroy()
        sys.exit(0)

if __name__ == "__main__":
    root = tk.Tk()
    app = TurntableGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
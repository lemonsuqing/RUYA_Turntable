import serial
import serial.tools.list_ports
import threading
import queue
import time
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
from typing import Optional, Tuple
import sys

# -------------------------- 全局配置与变量 --------------------------
DEFAULT_BAUDRATE = 115200
SUPPORTED_BAUDRATES = [9600, 19200, 38400, 57600, 115200, 230400]

# 回零判定配置
HOMING_TARGET_STATUS = "1"
HOMING_ANGLE_TOLERANCE = 0.01
HOMING_STABLE_TIME = 1.0
HOMING_ANGLE_CHANGE_THRESHOLD = 0.005
HOMING_MAX_TIMEOUT = 15.0

# 全局队列与标志
data_queue = queue.Queue(maxsize=1)  # 优化：队列大小改为1，只保留最新数据，自动丢弃旧数据
is_listening = False
is_homing = False
listen_thread = None
homing_thread = None
ser = None
is_connected = False

# 线程锁：防止多个线程同时写入串口导致指令冲突
serial_lock = threading.Lock() 

# -------------------------- 串口工具函数 --------------------------
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
            timeout=0.05 # 缩短超时时间
        )
        if ser.is_open:
            is_connected = True
            is_listening = True
            start_listen_thread()
            return True
        return False
    except Exception as e:
        print(f"串口连接失败：{e}")
        return False

def disconnect_serial(force: bool = False) -> None:
    global ser, is_connected, is_listening, is_homing
    is_listening = False
    is_homing = False
    time.sleep(0.1)
    
    if is_connected and ser and ser.is_open:
        try:
            with serial_lock: # 使用锁发送停车指令
                ser.write(b"$1st\r\n")
            time.sleep(0.05)
        except:
            if not force: print("强制停车指令发送失败")
    
    if ser and ser.is_open:
        try:
            ser.close()
        except Exception as e:
            if not force: print(f"串口关闭异常：{e}")
    is_connected = False

def send_command(cmd_content: str) -> str:
    global ser
    if not is_connected or not ser:
        return "错误：串口未连接"
    full_cmd = f"$1{cmd_content}\r\n"
    try:
        with serial_lock: # 关键：加锁，防止多线程写入冲突
            ser.write(full_cmd.encode("ascii"))
        return f"成功：发送指令 → {full_cmd.strip()}"
    except Exception as e:
        return f"错误：指令发送失败 → {str(e)}"

def parse_status(data: str) -> Tuple[Optional[str], Optional[str], Optional[str], Optional[float]]:
    # 简单的格式校验
    if len(data) < 14 or not data.startswith("$1"):
        return None, None, None, None
    
    # 协议 V1.7: $1 + 报警(1) + 状态(1) + 序号(2) + 角度(8)
    # 示例: $10150180.0000
    try:
        content = data[2:].strip()
        alarm_code = content[0]
        status_code = content[1]
        seq_num = content[2:4]
        angle_str = content[4:12]
        
        angle_val = float(angle_str)
        # 处理可能的负角度逻辑 (根据协议: >360 表示负值)
        if angle_val > 360.0:
            angle_val -= 720.0
            
        return alarm_code, status_code, seq_num, angle_val
    except:
        return None, None, None, None

def listen_serial() -> None:
    """
    监听线程优化版：
    协议规定转台每5ms发送一次数据(200Hz)。
    为了防止缓冲区积压导致界面延迟，我们需要尽可能快地读取，
    且只将最新的一帧数据放入队列更新UI。
    """
    global ser
    print("✅ 监听线程启动")
    
    buffer = ""
    
    while is_listening and ser and ser.is_open:
        try:
            if ser.in_waiting:
                # 读取所有缓冲区数据，避免积压
                raw_data = ser.read(ser.in_waiting).decode('ascii', errors='replace')
                buffer += raw_data
                
                if '\n' in buffer:
                    lines = buffer.split('\n')
                    # 保留最后一部分作为下一次的buffer
                    buffer = lines[-1]
                    
                    # 倒序遍历，找到由于网络粘包可能存在的最后一个完整帧
                    for line in reversed(lines[:-1]):
                        line = line.strip()
                        if line.startswith('$1') and len(line) >= 14:
                            alarm, status, seq, angle = parse_status(line)
                            if angle is not None:
                                # 使用 queue.LifoQueue 或者先清空再 put 保证实时性
                                # 这里简单的做法：如果队列满，先取走旧的，再放新的
                                if data_queue.full():
                                    try: data_queue.get_nowait()
                                    except: pass
                                data_queue.put((alarm, status, seq, angle, ""))
                            break # 找到最新的一个就够了，前面的丢弃
                            
            else:
                time.sleep(0.002) # 极短睡眠，释放CPU但保持高响应
                
        except Exception as e:
            if is_listening:
                print(f"监听异常: {e}")
                time.sleep(0.1)
                
    print("🛑 监听线程已退出")

def start_listen_thread() -> None:
    global listen_thread
    listen_thread = threading.Thread(target=listen_serial, daemon=True)
    listen_thread.start()

# -------------------------- 回零功能 --------------------------
def homing_with_callback(status_callback) -> None:
    global is_connected, is_homing
    is_homing = True
    if not is_connected:
        status_callback("错误：串口未连接")
        is_homing = False
        return

    # 发送回零指令
    send_command("1")
    status_callback("回零指令已发送，等待判定...")
    
    start_time = time.time()
    stable_start_time = None
    stable_start_angle = None

    try:
        while (time.time() - start_time < HOMING_MAX_TIMEOUT and is_connected and is_homing):
            # 获取最新数据 (不从队列取，避免取空，这里直接看队列里最后一次的数据即可)
            # 但由于我们要判定稳定性，最好还是从UI更新的变量或者专门的变量获取
            # 这里简化逻辑：直接读取一次队列（虽然可能被UI线程抢走，但概率较低）
            
            # 更稳妥的方式：直接利用UI线程更新的 real_time_data，
            # 但这里为了解耦，我们还是从队列里窥探或者在监听线程做分发。
            # 鉴于Python队列线程安全，我们这里简单的轮询队列
            
            current_data = None
            try:
                # 稍微等待一下新数据
                current_data = data_queue.get(timeout=0.1)
                # 取出来后为了让UI也能显示，最好再放回去或者通过回调更新UI
                # 这种架构下，建议由UI层驱动逻辑，或者监听线程分发。
                # 简易修正：我们只做判定，UI层通过自己的循环去get。
                # **修正方案**：回零线程不应该消费data_queue，否则UI就看不到了。
                # 我们改为读取 app.real_time_data (虽然跨线程读取变量，但基本类型只读没大问题)
                pass 
            except queue.Empty:
                pass
            
            # 使用 app 实例中的数据 (需要在 GUI 类中把 app 设为全局或传入)
            # 这里为了代码独立性，我们假设外部传入了获取最新状态的函数
            # 在此脚本结构下，比较难优雅实现。
            # 回退方案：回零线程只负责发指令和延时？不行，需要闭环判定。
            
            # **最佳实践修正**：让监听线程把数据写到一个全局变量 `latest_state`，
            # 队列仅用于UI刷新。
            time.sleep(0.1)
            
            # (由于代码结构限制，这里保留原有的逻辑，但注意队列抢占问题)
            # 实际运行中，UI线程消耗队列非常快，这里的逻辑可能拿不到数据。
            # 建议：在生产环境中，listen_thread 应该更新一个全局 thread-safe 变量供逻辑判断。
            
    except Exception as e:
        status_callback(f"回零异常：{e}")
    finally:
        is_homing = False
        status_callback("回零过程结束")

# 为了解决上述回零线程读取数据的问题，引入全局状态变量
global_latest_status = {"status": "0", "angle": 0.0, "updated": time.time()}

def homing_logic_v2(status_callback):
    """
    优化的回零逻辑：读取全局最新状态，而不是和UI争抢队列
    """
    global is_homing
    is_homing = True
    send_command("1")
    status_callback("开始回零...")
    
    start_t = time.time()
    stable_t = None
    
    while is_homing and (time.time() - start_t < HOMING_MAX_TIMEOUT):
        # 读取全局状态
        curr_status = global_latest_status["status"]
        curr_angle = global_latest_status["angle"]
        
        # 1. 状态码判定 (状态1=伺服，0=空闲，回零完成后通常会切回伺服或特定状态)
        # 根据协议：回零时状态是2，完成后可能是1
        # 但最准的是看角度
        
        if abs(curr_angle) < HOMING_ANGLE_TOLERANCE:
            if stable_t is None:
                stable_t = time.time()
            elif time.time() - stable_t > HOMING_STABLE_TIME:
                status_callback(f"✅ 回零成功 (角度 {curr_angle:.4f})")
                is_homing = False
                return
        else:
            stable_t = None
            
        time.sleep(0.1)
        
    if is_homing: # 超时
        status_callback("❌ 回零超时")
        is_homing = False

def start_homing_thread(status_callback) -> None:
    global homing_thread
    homing_thread = threading.Thread(target=homing_logic_v2, args=(status_callback,), daemon=True)
    homing_thread.start()

# -------------------------- GUI界面类 --------------------------
class TurntableGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("单轴转台控制系统 V1.3 (优化版)")
        self.root.geometry("820x620")
        
        # 变量初始化
        self.com_var = tk.StringVar()
        self.baud_var = tk.IntVar(value=DEFAULT_BAUDRATE)
        self.status_var = tk.StringVar(value="准备就绪")
        
        # 界面布局
        self.create_widgets()
        
        # 启动定时任务
        self.refresh_ports()
        self.update_ui_loop()

    def create_widgets(self):
        # 顶部：连接设置
        top_frame = ttk.LabelFrame(self.root, text="通讯设置", padding=10)
        top_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(top_frame, text="端口:").pack(side=tk.LEFT)
        self.cb_port = ttk.Combobox(top_frame, textvariable=self.com_var, width=10)
        self.cb_port.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(top_frame, text="波特率:").pack(side=tk.LEFT)
        self.cb_baud = ttk.Combobox(top_frame, textvariable=self.baud_var, values=SUPPORTED_BAUDRATES, width=8)
        self.cb_baud.pack(side=tk.LEFT, padx=5)
        
        self.btn_connect = ttk.Button(top_frame, text="连接设备", command=self.toggle_connect)
        self.btn_connect.pack(side=tk.LEFT, padx=10)

        # 中部：数据显示 (大字体)
        info_frame = ttk.Frame(self.root, padding=10, relief=tk.RIDGE)
        info_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        self.lbl_angle = ttk.Label(info_frame, text="0.0000°", font=("Consolas", 36, "bold"), foreground="#0055aa")
        self.lbl_angle.pack(pady=10)
        
        self.lbl_status = ttk.Label(info_frame, text="状态: 未连接", font=("Arial", 12))
        self.lbl_status.pack()

        # 底部：控制按钮
        ctrl_frame = ttk.LabelFrame(self.root, text="运动控制", padding=10)
        ctrl_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # 第一排：基础
        f1 = ttk.Frame(ctrl_frame)
        f1.pack(fill=tk.X, pady=5)
        ttk.Button(f1, text="使能开启 (mo=1)", command=lambda: self.send("mo=1")).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Button(f1, text="使能关闭 (mo=0)", command=lambda: self.send("mo=0")).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Button(f1, text="❌ 紧急停车 (st)", command=self.stop_machine).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Button(f1, text="🏠 自动回零", command=lambda: start_homing_thread(self.update_status_msg)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        
        # 第二排：模式
        f2 = ttk.Frame(ctrl_frame)
        f2.pack(fill=tk.X, pady=5)
        ttk.Button(f2, text="位置模式旋转", command=self.cmd_position).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Button(f2, text="速率模式旋转", command=self.cmd_speed).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        
        # 底部状态栏
        ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W).pack(side=tk.BOTTOM, fill=tk.X)

    def refresh_ports(self):
        ports = get_available_com_ports()
        self.cb_port['values'] = ports
        if ports and not self.com_var.get():
            self.com_var.set(ports[0])
        self.root.after(3000, self.refresh_ports)

    def update_ui_loop(self):
        # 从队列取数据更新UI
        try:
            while not data_queue.empty():
                alarm, status, seq, angle, err = data_queue.get_nowait()
                
                # 更新全局状态供回零线程使用
                global_latest_status["status"] = status
                global_latest_status["angle"] = angle
                global_latest_status["updated"] = time.time()
                
                # 更新界面
                self.lbl_angle.config(text=f"{angle:.4f}°")
                status_text = f"状态码: {status} | 报警: {alarm} | 计数: {seq}"
                if alarm != '0':
                    self.lbl_status.config(text=status_text + " (⚠️报警)", foreground="red")
                else:
                    self.lbl_status.config(text=status_text, foreground="black")
        except:
            pass
        
        self.root.after(20, self.update_ui_loop) # 50Hz刷新率足够人眼

    def toggle_connect(self):
        if not is_connected:
            if connect_serial(self.com_var.get(), self.baud_var.get()):
                self.btn_connect.config(text="断开连接")
                self.update_status_msg("已连接")
            else:
                messagebox.showerror("错误", "连接失败")
        else:
            self.force_stop()
            self.btn_connect.config(text="连接设备")
            self.update_status_msg("已断开")

    def send(self, cmd):
        msg = send_command(cmd)
        self.update_status_msg(msg)

    def stop_machine(self):
        global is_homing
        is_homing = False # 终止回零逻辑
        self.send("st")
        
    def force_stop(self):
        disconnect_serial(force=True)

    def update_status_msg(self, msg):
        self.status_var.set(msg)

    # --- 指令弹窗逻辑封装 ---
    def cmd_position(self):
        d = simpledialog.askinteger("位置模式", "方向 (0顺/1逆):", initialvalue=0, minvalue=0, maxvalue=1)
        if d is None: return
        a = simpledialog.askinteger("位置模式", "加速度 (1-1000):", initialvalue=10, minvalue=1, maxvalue=1000)
        if a is None: return
        v = simpledialog.askfloat("位置模式", "速度 (0.1-1000):", initialvalue=10.0, minvalue=0.1, maxvalue=1000.0)
        if v is None: return
        ang = simpledialog.askfloat("位置模式", "角度 (0-360):", initialvalue=90.0)
        if ang is None: return
        
        # 格式化: 2 + 方向(1) + 加速度(4) + 速度(9) + 角度(8)
        cmd = f"2{d}{a:04d}{v:09.4f}{ang:08.4f}"
        self.send(cmd)

    def cmd_speed(self):
        d = simpledialog.askinteger("速率模式", "方向 (0顺/1逆):", initialvalue=0, minvalue=0, maxvalue=1)
        if d is None: return
        a = simpledialog.askinteger("速率模式", "加速度 (1-1000):", initialvalue=10, minvalue=1, maxvalue=1000)
        if a is None: return
        v = simpledialog.askfloat("速率模式", "速度 (0.1-1000):", initialvalue=10.0, minvalue=0.1, maxvalue=1000.0)
        if v is None: return
        
        cmd = f"3{d}{a:04d}{v:09.4f}"
        self.send(cmd)

    def on_close(self):
        self.force_stop()
        self.root.destroy()
        sys.exit(0)

if __name__ == "__main__":
    root = tk.Tk()
    app = TurntableGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
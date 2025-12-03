import serial
import serial.tools.list_ports
import threading
import queue
import time
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
from typing import Optional, Tuple
import sys

# -------------------------- 全局配置与变量（新增is_homing标志）--------------------------
DEFAULT_BAUDRATE = 115200
SUPPORTED_BAUDRATES = [9600, 19200, 38400, 57600, 115200, 230400]

# 回零判定配置
HOMING_TARGET_STATUS = "1"
HOMING_ANGLE_TOLERANCE = 0.01
HOMING_STABLE_TIME = 1.0
HOMING_ANGLE_CHANGE_THRESHOLD = 0.005
HOMING_MAX_TIMEOUT = 15.0

# 全局队列与标志（关键改动：新增is_homing、homing_thread）
data_queue = queue.Queue(maxsize=10)
is_listening = False  # 监听线程开关
is_homing = False     # 回零线程开关（新增）
listen_thread = None  # 监听线程对象
homing_thread = None  # 回零线程对象（新增）
ser = None            # 串口对象
is_connected = False  # 连接状态

# -------------------------- 串口工具函数（强化强制关闭）--------------------------
def get_available_com_ports() -> list:
    """获取当前可用的COM口列表"""
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def connect_serial(com_port: str, baudrate: int) -> bool:
    """连接串口"""
    global ser, is_connected, is_listening
    try:
        ser = serial.Serial(
            port=com_port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.1
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
    """断开串口（关键改动：新增force参数，强制关闭忽略异常）"""
    global ser, is_connected, is_listening, is_homing
    # 强制停止所有线程标志
    is_listening = False
    is_homing = False
    time.sleep(0.1)  # 给线程100ms响应停止信号
    
    # 关键改动：强制发送停车指令（确保转台立即停止）
    if is_connected and ser and ser.is_open:
        try:
            ser.write(b"$1st\r\n")  # 紧急停车指令
            time.sleep(0.05)
        except:
            if not force:
                print("强制停车指令发送失败")
    
    # 关闭串口（强制模式忽略异常）
    if ser and ser.is_open:
        try:
            ser.close()
        except Exception as e:
            if not force:
                print(f"串口关闭异常：{e}")
    is_connected = False
    
    # 清空队列，释放资源
    while not data_queue.empty():
        try:
            data_queue.get_nowait()
        except:
            pass

def send_command(cmd_content: str) -> str:
    """发送指令（返回执行结果）"""
    global ser
    if not is_connected or not ser:
        return "错误：串口未连接"
    full_cmd = f"$1{cmd_content}\r\n"
    try:
        ser.write(full_cmd.encode("ascii"))
        return f"成功：发送指令 → {full_cmd.strip()}"
    except Exception as e:
        return f"错误：指令发送失败 → {str(e)}"

def parse_status(data: str) -> Tuple[Optional[str], Optional[str], Optional[str], Optional[float]]:
    """解析转台状态数据"""
    if not data.startswith("$1") or len(data) != 14:
        return None, None, None, None
    status_data = data[2:]
    alarm_code = status_data[0]
    status_code = status_data[1]
    seq_num = status_data[2:4]
    angle_str = status_data[4:12]
    try:
        angle_val = float(angle_str)
        if angle_val > 359.9999:
            angle_val -= 720
        return alarm_code, status_code, seq_num, angle_val
    except:
        return None, None, None, None

def listen_serial() -> None:
    """监听串口线程（关键改动：响应is_listening标志，立即退出）"""
    global ser
    while is_listening:
        try:
            if ser and ser.in_waiting > 0:
                data = ser.read_until(b"\r\n").decode("ascii").strip()
                if data and is_listening:  # 关键改动：再次检查，避免线程残留
                    alarm_code, status_code, seq_num, angle = parse_status(data)
                    if all([alarm_code, status_code, seq_num, angle is not None]):
                        try:
                            data_queue.put((alarm_code, status_code, seq_num, angle, ""), timeout=0.01)
                        except:
                            pass
        except Exception as e:
            if is_listening:  # 仅在正常监听时报告错误
                try:
                    data_queue.put(("0", "0", "00", 0.0, f"监听错误：{str(e)}"), timeout=0.01)
                except:
                    pass
        time.sleep(0.01)
    print("✅ 监听线程已强制终止")

def start_listen_thread() -> None:
    """启动监听线程"""
    global listen_thread
    listen_thread = threading.Thread(target=listen_serial, daemon=True)
    listen_thread.start()

# -------------------------- 回零功能（关键改动：支持强制停止）--------------------------
def homing_with_callback(status_callback) -> None:
    """回零功能（关键改动：响应is_homing标志，强制停止）"""
    global is_connected, is_homing
    is_homing = True  # 标记回零线程运行中
    if not is_connected:
        status_callback("错误：串口未连接，无法回零")
        is_homing = False
        return

    status_callback("正在检查转台状态...")
    time.sleep(1)
    result = send_command("1")
    status_callback(f"回零指令已发送 → {result}")
    if "错误" in result:
        is_homing = False
        return

    status_callback("转台开始回零，正在判定...")
    start_time = time.time()
    stable_start_angle = None
    stable_start_time = None

    try:
        # 关键改动：循环条件新增is_homing，强制关闭时立即退出
        while (time.time() - start_time < HOMING_MAX_TIMEOUT 
               and is_connected 
               and is_homing):
            # 读取最新角度
            latest_alarm = "0"
            latest_status = "0"
            latest_angle = 0.0
            while not data_queue.empty():
                try:
                    alarm, status, seq, angle, err = data_queue.get_nowait()
                    if not err:
                        latest_alarm = alarm
                        latest_status = status
                        latest_angle = angle
                except:
                    pass

            # 判定条件1：状态码+角度
            if latest_status == HOMING_TARGET_STATUS and abs(latest_angle) <= HOMING_ANGLE_TOLERANCE:
                status_callback(f"✅ 回零成功！状态码：{latest_status} | 角度：{latest_angle:.4f}°")
                break

            # 判定条件2：角度稳定
            if abs(latest_angle) <= HOMING_ANGLE_TOLERANCE:
                if stable_start_angle is None:
                    stable_start_angle = latest_angle
                    stable_start_time = time.time()
                    status_callback(f"🔍 接近零位（{latest_angle:.4f}°），1秒稳定判定中...")
                else:
                    stable_duration = time.time() - stable_start_time
                    angle_change = abs(latest_angle - stable_start_angle)
                    status_callback(f"🔍 稳定判定中：{stable_duration:.1f}秒 | 变化：{angle_change:.6f}°")
                    if stable_duration >= HOMING_STABLE_TIME and angle_change <= HOMING_ANGLE_CHANGE_THRESHOLD:
                        status_callback(f"✅ 回零成功！1秒稳定 | 角度：{latest_angle:.4f}°")
                        break
            else:
                stable_start_angle = None
                stable_start_time = None
                status_callback(f"🔍 回零中：当前角度 → {latest_angle:.4f}°")

            time.sleep(0.1)

        # 关键改动：区分超时和强制停止
        if not is_homing:
            status_callback("❌ 回零已被强制停止")
        elif time.time() - start_time >= HOMING_MAX_TIMEOUT:
            status_callback(f"❌ 回零超时（{HOMING_MAX_TIMEOUT}秒）！当前角度：{latest_angle:.4f}°")
    except Exception as e:
        status_callback(f"❌ 回零异常：{str(e)}")
    finally:
        is_homing = False  # 重置标志，确保线程退出
        print("✅ 回零线程已终止")

def start_homing_thread(status_callback) -> None:
    """启动回零线程（关键改动：记录homing_thread对象）"""
    global homing_thread
    homing_thread = threading.Thread(target=homing_with_callback, args=(status_callback,), daemon=True)
    homing_thread.start()

# -------------------------- GUI界面类（版本升级+强制关闭逻辑）--------------------------
class TurntableGUI:
    def __init__(self, root):
        self.root = root
        # 关键改动：版本号升级到v1.2，git可检测到
        self.root.title("国产转台控制程序 v1.2（强制关闭优化版）")
        self.root.geometry("800x600")
        self.root.resizable(False, False)

        # 初始化变量
        self.com_var = tk.StringVar()
        self.baud_var = tk.IntVar(value=DEFAULT_BAUDRATE)
        self.real_time_data = {
            "alarm": "0",
            "status": "0",
            "seq": "00",
            "angle": 0.0,
            "error": ""
        }

        # 构建界面
        self.setup_ui()

        # 启动实时数据更新循环
        self.update_real_time_data()

        # 定期刷新COM口列表
        self.refresh_com_ports()
        self.root.after(5000, self.refresh_com_ports)

    def setup_ui(self):
        """构建界面布局"""
        # 1. 顶部配置区
        config_frame = ttk.Frame(self.root, padding="10")
        config_frame.pack(fill=tk.X, side=tk.TOP)

        ttk.Label(config_frame, text="COM口：").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.com_combobox = ttk.Combobox(config_frame, textvariable=self.com_var, width=10)
        self.com_combobox.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(config_frame, text="波特率：").grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
        self.baud_combobox = ttk.Combobox(config_frame, textvariable=self.baud_var, width=10)
        self.baud_combobox["values"] = SUPPORTED_BAUDRATES
        self.baud_combobox.current(SUPPORTED_BAUDRATES.index(DEFAULT_BAUDRATE))
        self.baud_combobox.grid(row=0, column=3, padx=5, pady=5)

        self.connect_btn = ttk.Button(config_frame, text="连接", command=self.toggle_connect)
        self.connect_btn.grid(row=0, column=4, padx=5, pady=5)

        # 2. 实时数据显示区（覆盖式）
        data_frame = ttk.Frame(self.root, padding="10", relief=tk.SUNKEN)
        data_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        ttk.Label(data_frame, text="转台实时数据（覆盖式显示）", font=("Arial", 12, "bold")).pack(anchor=tk.W, pady=5)
        self.data_label = ttk.Label(
            data_frame,
            text="等待连接...（强制关闭将立即停机）",  # 关键改动：提示文字新增
            font=("Arial", 14),
            foreground="blue",
            justify=tk.LEFT,
            wraplength=750
        )
        self.data_label.pack(anchor=tk.W, pady=20)

        # 3. 基础控制区
        base_frame = ttk.Frame(self.root, padding="10")
        base_frame.pack(fill=tk.X, padx=10, pady=5)

        ttk.Label(base_frame, text="基础控制", font=("Arial", 10, "bold")).grid(row=0, column=0, padx=5, pady=5, columnspan=4)
        self.power_on_btn = ttk.Button(base_frame, text="电机上电", command=self.motor_power_on, state=tk.DISABLED)
        self.power_on_btn.grid(row=1, column=0, padx=5, pady=5, ipadx=10)
        self.power_off_btn = ttk.Button(base_frame, text="电机释放", command=self.motor_release, state=tk.DISABLED)
        self.power_off_btn.grid(row=1, column=1, padx=5, pady=5, ipadx=10)
        self.stop_btn = ttk.Button(base_frame, text="停车", command=self.stop, state=tk.DISABLED)
        self.stop_btn.grid(row=1, column=2, padx=5, pady=5, ipadx=10)
        self.homing_btn = ttk.Button(base_frame, text="回零", command=self.homing, state=tk.DISABLED)
        self.homing_btn.grid(row=1, column=3, padx=5, pady=5, ipadx=10)

        # 4. 运动模式区
        motion_frame = ttk.Frame(self.root, padding="10")
        motion_frame.pack(fill=tk.X, padx=10, pady=5)

        ttk.Label(motion_frame, text="运动模式", font=("Arial", 10, "bold")).grid(row=0, column=0, padx=5, pady=5, columnspan=4)
        self.position_btn = ttk.Button(motion_frame, text="位置模式", command=self.position_mode, state=tk.DISABLED)
        self.position_btn.grid(row=1, column=0, padx=5, pady=5, ipadx=10)
        self.speed_btn = ttk.Button(motion_frame, text="速率模式", command=self.speed_mode, state=tk.DISABLED)
        self.speed_btn.grid(row=1, column=1, padx=5, pady=5, ipadx=10)
        self.swing_btn = ttk.Button(motion_frame, text="摇摆模式", command=self.swing_mode, state=tk.DISABLED)
        self.swing_btn.grid(row=1, column=2, padx=5, pady=5, ipadx=10)
        self.multi_circle_btn = ttk.Button(motion_frame, text="多圈模式", command=self.multi_circle_mode, state=tk.DISABLED)
        self.multi_circle_btn.grid(row=1, column=3, padx=5, pady=5, ipadx=10)

        # 5. 状态栏
        self.status_var = tk.StringVar(value="就绪：未连接串口 | 强制关闭=立即停机（v1.2）")  # 关键改动：版本号+提示
        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(fill=tk.X, side=tk.BOTTOM)

    def refresh_com_ports(self):
        """刷新COM口列表"""
        available_ports = get_available_com_ports()
        self.com_combobox["values"] = available_ports
        if available_ports and not self.com_var.get():
            self.com_var.set(available_ports[0])
        self.root.after(5000, self.refresh_com_ports)

    def toggle_connect(self):
        """连接/断开串口切换"""
        if not is_connected:
            com_port = self.com_var.get()
            baudrate = self.baud_var.get()
            if not com_port:
                messagebox.showwarning("警告", "请选择COM口！")
                return
            self.status_var.set(f"正在连接 {com_port}（{baudrate}）... | 强制关闭=立即停机")
            self.root.update()
            if connect_serial(com_port, baudrate):
                self.connect_btn.config(text="断开")
                self.status_var.set(f"已连接：{com_port} | 波特率：{baudrate} | 强制关闭=立即停机")
                self.power_on_btn.config(state=tk.NORMAL)
                self.power_off_btn.config(state=tk.NORMAL)
                self.stop_btn.config(state=tk.NORMAL)
                self.homing_btn.config(state=tk.NORMAL)
                self.position_btn.config(state=tk.NORMAL)
                self.speed_btn.config(state=tk.NORMAL)
                self.swing_btn.config(state=tk.NORMAL)
                self.multi_circle_btn.config(state=tk.NORMAL)
            else:
                self.status_var.set(f"连接失败：{com_port} | 强制关闭=立即停机")
                messagebox.showerror("错误", "串口连接失败，请检查端口和权限！")
        else:
            self.connect_btn.config(text="连接")
            self.force_stop_all()  # 断开时也强制停止所有
            self.status_var.set("已断开连接 | 强制关闭=立即停机")
            self.power_on_btn.config(state=tk.DISABLED)
            self.power_off_btn.config(state=tk.DISABLED)
            self.stop_btn.config(state=tk.DISABLED)
            self.homing_btn.config(state=tk.DISABLED)
            self.position_btn.config(state=tk.DISABLED)
            self.speed_btn.config(state=tk.DISABLED)
            self.swing_btn.config(state=tk.DISABLED)
            self.multi_circle_btn.config(state=tk.DISABLED)

    def update_real_time_data(self):
        """实时更新数据显示（覆盖式）"""
        try:
            while not data_queue.empty():
                alarm, status, seq, angle, error = data_queue.get_nowait()
                self.real_time_data["alarm"] = alarm
                self.real_time_data["status"] = status
                self.real_time_data["seq"] = seq
                self.real_time_data["angle"] = angle
                self.real_time_data["error"] = error
        except:
            pass

        if self.real_time_data["error"]:
            display_text = f"❌ 异常：{self.real_time_data['error']}\n（强制关闭将立即停机）"  # 关键改动：新增提示
        else:
            display_text = (
                f"📊 最新状态：\n"
                f"   报警码：{self.real_time_data['alarm']}（0=正常）\n"
                f"   状态码：{self.real_time_data['status']}（1=伺服状态）\n"
                f"   发送周期：{self.real_time_data['seq']}\n"
                f"   当前角度：{self.real_time_data['angle']:.4f}°\n"
                f"（强制关闭将立即停机）"  # 关键改动：新增提示
            )
        self.data_label.config(text=display_text)

        self.root.after(100, self.update_real_time_data)

    # -------------------------- 基础控制功能 --------------------------
    def motor_power_on(self):
        result = send_command("mo=1")
        self.status_var.set(f"电机上电 → {result} | 强制关闭=立即停机")

    def motor_release(self):
        result = send_command("mo=0")
        self.status_var.set(f"电机释放 → {result} | 强制关闭=立即停机")

    def stop(self):
        result = send_command("st")
        self.status_var.set(f"紧急停车 → {result} | 强制关闭=立即停机")

    def homing(self):
        start_homing_thread(self.status_var.set)

    # -------------------------- 运动模式功能 --------------------------
    def get_int_input(self, title, prompt, default, min_val, max_val) -> Optional[int]:
        try:
            value = simpledialog.askinteger(title, prompt, initialvalue=default, minvalue=min_val, maxvalue=max_val)
            return value if value is not None else None
        except:
            messagebox.showerror("错误", "输入无效，请输入整数！")
            return None

    def get_float_input(self, title, prompt, default, min_val, max_val) -> Optional[float]:
        try:
            value = simpledialog.askfloat(title, prompt, initialvalue=default, minvalue=min_val, maxvalue=max_val)
            return value if value is not None else None
        except:
            messagebox.showerror("错误", "输入无效，请输入浮点数！")
            return None

    def position_mode(self):
        direction = self.get_int_input("位置模式", "方向（0=顺时针/1=逆时针）", 0, 0, 1)
        if direction is None:
            return
        acc = self.get_int_input("位置模式", "加速度（1~1000 度/秒²）", 10, 1, 1000)
        if acc is None:
            return
        speed = self.get_float_input("位置模式", "速度（0.0001~1000.0 度/秒）", 10.0, 0.0001, 1000.0)
        if speed is None:
            return
        angle = self.get_float_input("位置模式", "目标角度（0~359.9999 度）", 180.0, 0, 359.9999)
        if angle is None:
            return

        dir_str = str(direction)
        acc_str = f"{acc:04d}"
        speed_str = f"{speed:09.4f}"
        angle_str = f"{angle:08.4f}"
        cmd = f"2{dir_str}{acc_str}{speed_str}{angle_str}"
        result = send_command(cmd)
        self.status_var.set(f"位置模式 → {result} | 方向：{direction} | 目标角度：{angle}° | 强制关闭=立即停机")

    def speed_mode(self):
        direction = self.get_int_input("速率模式", "方向（0=顺时针/1=逆时针）", 1, 0, 1)
        if direction is None:
            return
        acc = self.get_int_input("速率模式", "加速度（1~1000 度/秒²）", 10, 1, 1000)
        if acc is None:
            return
        speed = self.get_float_input("速率模式", "速度（0.0001~1000.0 度/秒）", 10.0, 0.0001, 1000.0)
        if speed is None:
            return

        dir_str = str(direction)
        acc_str = f"{acc:04d}"
        speed_str = f"{speed:09.4f}"
        cmd = f"3{dir_str}{acc_str}{speed_str}"
        result = send_command(cmd)
        self.status_var.set(f"速率模式 → {result} | 方向：{direction} | 速度：{speed}°/s | 强制关闭=立即停机")

    def swing_mode(self):
        amp = self.get_float_input("摇摆模式", "摇摆幅度（0~359.9999 度）", 10.0, 0, 359.9999)
        if amp is None:
            return
        freq = self.get_float_input("摇摆模式", "摇摆频率（0.001~10.0 HZ）", 0.1, 0.001, 10.0)
        if freq is None:
            return

        amp_str = f"{amp:08.4f}"
        freq_str = f"{freq:06.3f}"
        cmd = f"4{amp_str}{freq_str}"
        result = send_command(cmd)
        self.status_var.set(f"摇摆模式 → {result} | 幅度：{amp}° | 频率：{freq}HZ | 强制关闭=立即停机")

    def multi_circle_mode(self):
        direction = self.get_int_input("多圈模式", "方向（0=顺时针/1=逆时针）", 0, 0, 1)
        if direction is None:
            return
        acc = self.get_int_input("多圈模式", "加速度（1~1000 度/秒²）", 10, 1, 1000)
        if acc is None:
            return
        speed = self.get_float_input("多圈模式", "速度（0.0001~1000.0 度/秒）", 10.0, 0.0001, 1000.0)
        if speed is None:
            return
        angle = self.get_float_input("多圈模式", "目标角度（0~359.9999 度）", 180.0, 0, 359.9999)
        if angle is None:
            return
        circles = self.get_int_input("多圈模式", "旋转圈数（1~99）", 2, 1, 99)
        if circles is None:
            return

        dir_str = str(direction)
        acc_str = f"{acc:04d}"
        speed_str = f"{speed:09.4f}"
        angle_str = f"{angle:08.4f}"
        circle_str = f"{circles:02d}"
        cmd = f"5{dir_str}{acc_str}{speed_str}{angle_str}{circle_str}"
        result = send_command(cmd)
        self.status_var.set(f"多圈模式 → {result} | 圈数：{circles} | 目标角度：{angle}° | 强制关闭=立即停机")

    def force_stop_all(self):
        """强制停止所有操作（关键新增函数：git可检测到）"""
        print("⚠️  执行强制停止：停车+断开串口+终止线程（v1.2）")
        # 1. 立即发送停车指令（优先级最高）
        if is_connected and ser and ser.is_open:
            try:
                ser.write(b"$1st\r\n")
                time.sleep(0.05)
            except:
                print("强制停车指令发送失败（可能已断开）")
        # 2. 断开串口（强制模式）
        disconnect_serial(force=True)
        # 3. 等待线程终止（最多1秒）
        if listen_thread and listen_thread.is_alive():
            listen_thread.join(timeout=1.0)
        if homing_thread and homing_thread.is_alive():
            homing_thread.join(timeout=1.0)
        print("✅ 强制停止完成（v1.2）")

    def on_close(self):
        """关闭窗口时的强制停止逻辑（关键改动）"""
        self.force_stop_all()
        self.root.destroy()
        # 强制退出Python进程（避免残留）
        sys.exit(0)

# -------------------------- 程序入口（新增异常捕获）--------------------------
if __name__ == "__main__":
    root = tk.Tk()
    app = TurntableGUI(root)
    # 绑定窗口关闭事件
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    # 关键改动：处理所有强制关闭场景（Alt+F4、任务管理器结束等）
    try:
        root.mainloop()
    except Exception as e:
        print(f"程序异常，执行强制停止：{e}")
        app.force_stop_all()
        sys.exit(0)
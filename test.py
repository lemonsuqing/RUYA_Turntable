import serial
import time
import threading
from typing import Optional, Tuple

# -------------------------- 配置参数（根据实际串口修改）--------------------------
SERIAL_PORT = "COM5"  # 转台实际串口
BAUD_RATE = 115200    # 固定波特率
TIMEOUT = 0.1         # 串口读取超时
FRAME_START = "$1"    # 指令起始标识
FRAME_END = "\r\n"    # 指令结束标识

# 全局变量：存储实时状态（供主线程和监听线程共享）
current_status = {
    "alarm_code": "0",    # 报警码
    "status_code": "0",   # 状态码
    "seq_num": "00",      # 发送周期序号
    "angle": 0.0,         # 当前角度
    "is_listening": True  # 监听线程开关
}
status_lock = threading.Lock()  # 线程安全锁


class TurntableController:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.1):
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.is_connected = False
        self.status_thread = None  # 状态监听线程

    def connect(self) -> bool:
        """连接串口+启动实时状态监听线程"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=self.timeout
            )
            if self.ser.is_open:
                self.is_connected = True
                print(f"✅ 串口连接成功！端口：{self.port}")
                # 启动实时状态监听线程
                self._start_status_listener()
                return True
            return False
        except Exception as e:
            print(f"❌ 串口连接失败：{str(e)}")
            return False

    def disconnect(self) -> None:
        """断开串口+停止监听线程"""
        # 停止监听线程
        with status_lock:
            current_status["is_listening"] = False
        if self.status_thread and self.status_thread.is_alive():
            self.status_thread.join(timeout=2)
            print("✅ 状态监听线程已停止")
        
        # 关闭串口
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.is_connected = False
            print("✅ 串口已断开连接")

    def _send_command(self, cmd_content: str) -> None:
        """发送指令（线程安全，避免与状态读取冲突）"""
        if not self.is_connected:
            print("❌ 串口未连接，无法发送指令")
            return
        full_cmd = f"{FRAME_START}{cmd_content}{FRAME_END}"
        try:
            # 发送指令时暂时锁定串口，避免与状态读取冲突
            with status_lock:
                self.ser.write(full_cmd.encode("ascii"))
            print(f"📤 发送指令：{full_cmd.strip()}（与指令汇总表完全一致）")
        except Exception as e:
            print(f"❌ 指令发送失败：{str(e)}")

    def _parse_status(self, data: str) -> None:
        """解析转台主动发送的状态帧（更新到全局变量）"""
        if not data.startswith(FRAME_START) or len(data) != len(FRAME_START) + 12:
            return
        status_data = data[len(FRAME_START):]
        alarm_code = status_data[0]
        status_code = status_data[1]
        seq_num = status_data[2:4]
        angle_str = status_data[4:12]
        
        # 转换角度（处理负角度：>359.9999 视为负值）
        angle_val = float(angle_str)
        if angle_val > 359.9999:
            angle_val -= 720
        
        # 线程安全更新全局状态
        with status_lock:
            current_status["alarm_code"] = alarm_code
            current_status["status_code"] = status_code
            current_status["seq_num"] = seq_num
            current_status["angle"] = angle_val

    def _status_listener(self) -> None:
        """实时状态监听线程（独立运行，持续接收转台主动发送的数据）"""
        print("📡 状态监听线程已启动，实时接收转台数据...")
        while True:
            # 检查是否需要停止监听
            with status_lock:
                if not current_status["is_listening"]:
                    break
            
            try:
                # 持续读取串口数据（转台主动发送，无需触发）
                if self.ser.in_waiting > 0:
                    data = self.ser.read_until(FRAME_END.encode("ascii")).decode("ascii").strip()
                    if data:
                        self._parse_status(data)
                        # 每10个周期打印一次状态（避免冗余，也可改为实时打印）
                        # with status_lock:
                        #     if int(current_status["seq_num"]) % 10 == 0:
                        #         print(f"📥 实时状态：报警[{current_status['alarm_code']}] | 状态[{current_status['status_code']}] | 周期[{current_status['seq_num']}] | 角度[{current_status['angle']:.4f}°]")
            except Exception as e:
                continue
            
            time.sleep(0.01)  # 降低CPU占用

    def _start_status_listener(self) -> None:
        """启动状态监听线程"""
        self.status_thread = threading.Thread(target=self._status_listener, daemon=True)
        self.status_thread.start()

    def get_real_time_angle(self) -> float:
        """获取实时角度（线程安全）"""
        with status_lock:
            return current_status["angle"]

    # -------------------------- 基础控制指令（不变）--------------------------
    def motor_release(self) -> None:
        print("\n=== 执行【电机释放】===")
        self._send_command("mo=0")
        time.sleep(1)
        print("✅ 电机释放指令已发送")

    def motor_power_on(self) -> None:
        print("\n=== 执行【电机上电】===")
        self._send_command("mo=1")
        time.sleep(1)
        print("✅ 电机上电指令已发送")

    def stop(self) -> None:
        print("\n=== 执行【停车】===")
        self._send_command("st")
        time.sleep(1)
        print("✅ 停车指令已发送")

    # -------------------------- 优化后的回零功能（核心改进）--------------------------
    def homing(self) -> None:
        """回零功能：循环监听角度，稳定零位即判定成功（无固定超时）"""
        print("\n=== 执行【回零】===")
        # 前置检查：是否上电
        with status_lock:
            if current_status["status_code"] != "1":
                print("⚠️  转台未上电（伺服状态），建议先执行「电机上电」")
                confirm = input("是否继续回零？（y/n，默认n）：").strip().lower()
                if confirm != "y":
                    print("❌ 回零取消")
                    return

        # 发送回零指令
        self._send_command("1")
        print("✅ 回零指令已发送，转台开始回零（速率较慢，请耐心等待）...")
        print("📌 判定逻辑：角度稳定在±0.01°以内持续3秒，即视为回零成功")
        print("⚠️  按Ctrl+C可中断回零")

        # 循环监听角度，判定回零成功
        stable_count = 0  # 稳定零位的计数（每0.1秒计数一次）
        target_stable_count = 30  # 3秒 = 30 * 0.1秒
        last_angle = 0.0

        try:
            while True:
                current_angle = self.get_real_time_angle()
                angle_diff = abs(current_angle - last_angle)
                last_angle = current_angle

                # 条件1：角度接近零位（±0.01°）
                if abs(current_angle) <= 0.01:
                    # 条件2：角度稳定（0.1秒内变化≤0.001°）
                    if angle_diff <= 0.001:
                        stable_count += 1
                        print(f"🔍 零位稳定中：{stable_count}/{target_stable_count}（当前角度：{current_angle:.4f}°）", end="\r")
                    else:
                        stable_count = 0  # 角度抖动，重置计数
                else:
                    stable_count = 0  # 未到零位，重置计数
                    print(f"🔍 回零中：当前角度→{current_angle:.4f}°（目标：0.0000°）", end="\r")

                # 稳定3秒，判定回零成功
                if stable_count >= target_stable_count:
                    print(f"\n✅ 回零成功！最终角度：{current_angle:.4f}°（稳定3秒）")
                    return

                time.sleep(0.1)  # 每0.1秒检查一次

        except KeyboardInterrupt:
            print("\n⚠️  检测到手动中断，执行停车...")
            self.stop()
            print("❌ 回零已中断")

    # -------------------------- 其他功能（不变，仅复用实时角度监听）--------------------------
    def set_status_freq(self, freq_index: int) -> None:
        print("\n=== 执行【设置状态频率】===")
        if 0 <= freq_index <= 7:
            self._send_command(f"rs={freq_index}")
            freq_map = {0:200, 1:100, 2:50, 3:20, 4:10, 5:5, 6:2, 7:1}
            print(f"✅ 状态频率已设置为：{freq_map[freq_index]}HZ（序号{freq_index}）")
        else:
            print("❌ 频率序号错误！仅支持0~7")

    def rotate_position(self, direction: int, acceleration: int, speed: float, target_angle: float) -> None:
        print("\n=== 执行【位置模式旋转】===")
        if direction not in [0, 1]:
            print("❌ 方向错误！仅支持0（顺时针）/1（逆时针）")
            return
        if not (1 <= acceleration <= 1000):
            print("❌ 加速度错误！范围1~1000（度/秒²）")
            return
        if not (0.0001 <= speed <= 1000.0):
            print("❌ 速度错误！范围0.0001~1000.0（度/秒）")
            return
        if not (0 <= target_angle <= 359.9999):
            print("❌ 目标角度错误！范围0~359.9999（度）")
            return

        dir_str = str(direction)
        acc_str = f"{acceleration:04d}"
        speed_str = f"{speed:09.4f}"
        angle_str = f"{target_angle:08.4f}"
        cmd_content = f"2{dir_str}{acc_str}{speed_str}{angle_str}"

        self._send_command(cmd_content)
        print(f"✅ 位置模式指令已发送：方向{direction} | 加速度{acceleration}°/s² | 速度{speed}°/s | 目标角度{target_angle}°")
        # 可选：实时显示旋转进度
        time.sleep(1)
        print("📊 旋转进度（实时更新）：")
        for _ in range(10):
            current_angle = self.get_real_time_angle()
            angle_diff = abs(current_angle - target_angle)
            print(f"   当前角度：{current_angle:.4f}° | 距离目标：{angle_diff:.4f}°", end="\r")
            time.sleep(0.5)
        print()

    def rotate_speed(self, direction: int, acceleration: int, speed: float) -> None:
        print("\n=== 执行【速率模式旋转】===")
        if direction not in [0, 1]:
            print("❌ 方向错误！仅支持0（顺时针）/1（逆时针）")
            return
        if not (1 <= acceleration <= 1000):
            print("❌ 加速度错误！范围1~1000（度/秒²）")
            return
        if not (0.0001 <= speed <= 1000.0):
            print("❌ 速度错误！范围0.0001~1000.0（度/秒）")
            return

        dir_str = str(direction)
        acc_str = f"{acceleration:04d}"
        speed_str = f"{speed:09.4f}"
        cmd_content = f"3{dir_str}{acc_str}{speed_str}"

        self._send_command(cmd_content)
        print(f"✅ 速率模式指令已发送：方向{direction} | 加速度{acceleration}°/s² | 速度{speed}°/s")

    def rotate_swing(self, amplitude: float, frequency: float) -> None:
        print("\n=== 执行【摇摆模式旋转】===")
        if not (0 <= amplitude <= 359.9999):
            print("❌ 摇摆幅度错误！范围0~359.9999（度）")
            return
        if not (0.001 <= frequency <= 10.0):
            print("❌ 摇摆频率错误！范围0.001~10.0（HZ）")
            return

        amp_str = f"{amplitude:08.4f}"
        freq_str = f"{frequency:06.3f}"
        cmd_content = f"4{amp_str}{freq_str}"

        self._send_command(cmd_content)
        print(f"✅ 摇摆模式指令已发送：幅度{amplitude}° | 频率{frequency}HZ")

    def rotate_multi_circle(self, direction: int, acceleration: int, speed: float, target_angle: float, circles: int) -> None:
        print("\n=== 执行【多圈位置模式旋转】===")
        if direction not in [0, 1]:
            print("❌ 方向错误！仅支持0（顺时针）/1（逆时针）")
            return
        if not (1 <= acceleration <= 1000):
            print("❌ 加速度错误！范围1~1000（度/秒²）")
            return
        if not (0.0001 <= speed <= 1000.0):
            print("❌ 速度错误！范围0.0001~1000.0（度/秒）")
            return
        if not (0 <= target_angle <= 359.9999):
            print("❌ 目标角度错误！范围0~359.9999（度）")
            return
        if not (1 <= circles <= 99):
            print("❌ 圈数错误！范围1~99")
            return

        dir_str = str(direction)
        acc_str = f"{acceleration:04d}"
        speed_str = f"{speed:09.4f}"
        angle_str = f"{target_angle:08.4f}"
        circle_str = f"{circles:02d}"
        cmd_content = f"5{dir_str}{acc_str}{speed_str}{angle_str}{circle_str}"

        self._send_command(cmd_content)
        print(f"✅ 多圈位置模式指令已发送：方向{direction} | 加速度{acceleration}°/s² | 速度{speed}°/s | 目标角度{target_angle}° | 圈数{circles}")


# -------------------------- 菜单交互（不变）--------------------------
def print_menu():
    print("\n" + "="*60)
    print("📋 国产转盘控制菜单（优化版：实时状态+智能回零）")
    print("="*60)
    print("1. 电机释放（mo=0）          2. 电机上电（mo=1）")
    print("3. 停车（st）                4. 回零（1）→ 智能判定成功")
    print("5. 设置状态发送频率（rs=N）  6. 位置模式旋转")
    print("7. 速率模式旋转              8. 摇摆模式旋转")
    print("9. 多圈位置模式旋转          0. 退出程序")
    print("="*60)


def input_int(prompt: str, min_val: int, max_val: int, default: int = None) -> int:
    while True:
        user_input = input(prompt).strip()
        if not user_input and default is not None:
            return default
        try:
            val = int(user_input)
            if min_val <= val <= max_val:
                return val
            print(f"❌ 输入错误！请输入{min_val}~{max_val}之间的整数")
        except ValueError:
            print("❌ 输入错误！请输入有效整数")


def input_float(prompt: str, min_val: float, max_val: float, default: float = None) -> float:
    while True:
        user_input = input(prompt).strip()
        if not user_input and default is not None:
            return default
        try:
            val = float(user_input)
            if min_val <= val <= max_val:
                return val
            print(f"❌ 输入错误！请输入{min_val}~{max_val}之间的浮点数")
        except ValueError:
            print("❌ 输入错误！请输入有效浮点数")


# -------------------------- 主程序 --------------------------
if __name__ == "__main__":
    controller = TurntableController(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=TIMEOUT)
    if not controller.connect():
        exit(1)

    try:
        while True:
            print_menu()
            choice = input_int("请选择功能序号：", 0, 9)

            if choice == 1:
                controller.motor_release()
            elif choice == 2:
                controller.motor_power_on()
            elif choice == 3:
                controller.stop()
            elif choice == 4:
                controller.homing()  # 优化后的智能回零
            elif choice == 5:
                freq_idx = input_int("请输入频率序号（0~7，默认0）：", 0, 7, default=0)
                controller.set_status_freq(freq_idx)
            elif choice == 6:
                dir_ = input_int("方向（0=顺时针/1=逆时针，默认0）：", 0, 1, default=0)
                acc = input_int("加速度（1~1000，默认10）：", 1, 1000, default=10)
                spd = input_float("速度（0.0001~1000.0，默认10.0）：", 0.0001, 1000.0, default=10.0)
                angle = input_float("目标角度（0~359.9999，默认180.0）：", 0, 359.9999, default=180.0)
                controller.rotate_position(dir_, acc, spd, angle)
            elif choice == 7:
                dir_ = input_int("方向（0=顺时针/1=逆时针，默认1）：", 0, 1, default=1)
                acc = input_int("加速度（1~1000，默认10）：", 1, 1000, default=10)
                spd = input_float("速度（0.0001~1000.0，默认10.0）：", 0.0001, 1000.0, default=10.0)
                controller.rotate_speed(dir_, acc, spd)
            elif choice == 8:
                amp = input_float("摇摆幅度（0~359.9999，默认10.0）：", 0, 359.9999, default=10.0)
                freq = input_float("摇摆频率（0.001~10.0，默认0.1）：", 0.001, 10.0, default=0.1)
                controller.rotate_swing(amp, freq)
            elif choice == 9:
                dir_ = input_int("方向（0=顺时针/1=逆时针，默认0）：", 0, 1, default=0)
                acc = input_int("加速度（1~1000，默认10）：", 1, 1000, default=10)
                spd = input_float("速度（0.0001~1000.0，默认10.0）：", 0.0001, 1000.0, default=10.0)
                angle = input_float("目标角度（0~359.9999，默认180.0）：", 0, 359.9999, default=180.0)
                circles = input_int("旋转圈数（1~99，默认2）：", 1, 99, default=2)
                controller.rotate_multi_circle(dir_, acc, spd, angle, circles)
            elif choice == 0:
                print("\n⚠️  准备退出程序，执行停车+电机释放...")
                controller.stop()
                controller.motor_release()
                break

            time.sleep(2)

    except KeyboardInterrupt:
        print("\n⚠️  检测到手动中断，执行紧急停车+电机释放...")
        controller.stop()
        controller.motor_release()
    finally:
        controller.disconnect()
        print("\n=== 程序结束 ===")
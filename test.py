import serial
import time
from typing import Optional, Tuple

# -------------------------- 配置参数（必须根据实际情况修改）--------------------------
SERIAL_PORT = "COM5"  # 转台连接的串口（如COM5、/dev/ttyUSB0）
BAUD_RATE = 115200    # 固定波特率（与指令汇总表一致）
TIMEOUT = 0.1         # 串口读取超时时间
FRAME_START = "$1"    # 指令起始标识（固定）
FRAME_END = "\r\n"    # 指令结束标识（固定）


class TurntableController:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.1):
        """初始化转台控制器（串口连接）"""
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.is_connected = False

    def connect(self) -> bool:
        """连接串口（严格匹配指令汇总表的通讯参数）"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                parity=serial.PARITY_NONE,  # 无奇偶校验
                stopbits=serial.STOPBITS_ONE,  # 1个停止位
                bytesize=serial.EIGHTBITS,  # 8位数据位
                timeout=self.timeout
            )
            if self.ser.is_open:
                self.is_connected = True
                print(f"✅ 串口连接成功！端口：{self.port}")
                return True
            return False
        except Exception as e:
            print(f"❌ 串口连接失败：{str(e)}")
            return False

    def disconnect(self) -> None:
        """断开串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.is_connected = False
            print("✅ 串口已断开连接")

    def _send_command(self, cmd_content: str) -> None:
        """发送指令（自动补全FRAME_START和FRAME_END）"""
        if not self.is_connected:
            print("❌ 串口未连接，无法发送指令")
            return
        full_cmd = f"{FRAME_START}{cmd_content}{FRAME_END}"
        try:
            self.ser.write(full_cmd.encode("ascii"))
            print(f"📤 发送指令：{full_cmd.strip()}（与指令汇总表完全一致）")
        except Exception as e:
            print(f"❌ 指令发送失败：{str(e)}")

    def _read_status(self) -> Optional[Tuple[str, str, str, float]]:
        """读取转台状态（解析报警码、状态码、序号、角度）"""
        if not self.is_connected:
            return None
        try:
            data = self.ser.read_until(FRAME_END.encode("ascii")).decode("ascii").strip()
            # 验证状态帧格式：以$1开头，长度为"$1"+12位状态数据
            if not data.startswith(FRAME_START) or len(data) != len(FRAME_START) + 12:
                return None
            status_data = data[len(FRAME_START):]
            alarm_code = status_data[0]       # 1位：报警码
            status_code = status_data[1]      # 1位：状态码
            seq_num = status_data[2:4]        # 2位：序号
            angle_str = status_data[4:12]     # 8位：角度（如180.0000）
            # 转换负角度（限位轴>360表示负值）
            angle_val = float(angle_str)
            if angle_val > 359.9999:
                angle_val -= 720
            # 每5帧打印一次状态，避免冗余
            if int(seq_num) % 5 == 0:
                print(f"📥 状态：报警[{alarm_code}] 状态[{status_code}] 角度[{angle_val:.4f}°]")
            return alarm_code, status_code, seq_num, angle_val
        except Exception as e:
            return None

    def get_current_angle(self) -> Optional[float]:
        """获取当前角度（简化状态读取）"""
        status = self._read_status()
        return status[3] if status else None

    # -------------------------- 基础控制指令（与汇总表完全匹配）--------------------------
    def motor_release(self) -> None:
        """电机释放（指令：$1mo=0）"""
        print("\n=== 执行【电机释放】===")
        self._send_command("mo=0")
        time.sleep(1)
        print("✅ 电机释放指令已发送")

    def motor_power_on(self) -> None:
        """电机上电（指令：$1mo=1）"""
        print("\n=== 执行【电机上电】===")
        self._send_command("mo=1")
        time.sleep(1)
        print("✅ 电机上电指令已发送")

    def stop(self) -> None:
        """停车（指令：$1st）"""
        print("\n=== 执行【停车】===")
        self._send_command("st")
        time.sleep(1)
        print("✅ 停车指令已发送")

    def homing(self) -> None:
        """回零（指令：$11）"""
        print("\n=== 执行【回零】===")
        self._send_command("1")
        print("✅ 回零指令已发送，转台正在回零...")
        # 等待回零完成（角度接近0°）
        start_time = time.time()
        while time.time() - start_time < 30:
            angle = self.get_current_angle()
            if angle is not None and abs(angle) < 0.01:
                print(f"✅ 回零完成！当前角度：{angle:.4f}°")
                return
            time.sleep(0.5)
        print("⚠️  回零超时，请手动确认角度是否归零")

    def set_status_freq(self, freq_index: int) -> None:
        """设置状态信息发送频率（指令：$1rs=N）"""
        print("\n=== 执行【设置状态频率】===")
        if 0 <= freq_index <= 7:
            self._send_command(f"rs={freq_index}")
            freq_map = {0:200, 1:100, 2:50, 3:20, 4:10, 5:5, 6:2, 7:1}
            print(f"✅ 状态频率已设置为：{freq_map[freq_index]}HZ（序号{freq_index}）")
        else:
            print("❌ 频率序号错误！仅支持0~7")

    # -------------------------- 运动模式指令（严格匹配汇总表格式）--------------------------
    def rotate_position(self, direction: int, acceleration: int, speed: float, target_angle: float) -> None:
        """
        位置模式旋转（指令格式：$12+方向+加速度+速度+角度）
        :param direction: 0=顺时针 / 1=逆时针
        :param acceleration: 1~1000（4位补零，如10→0010）
        :param speed: 0.0001~1000.0（9位格式，如10→0010.0000）
        :param target_angle: 0~359.9999（8位格式，如180→180.0000）
        """
        print("\n=== 执行【位置模式旋转】===")
        # 参数校验
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

        # 格式化参数（严格匹配指令汇总表的位数要求）
        dir_str = str(direction)
        acc_str = f"{acceleration:04d}"          # 加速度：4位补零
        speed_str = f"{speed:09.4f}"             # 速度：9位（4整+小数点+4小）
        angle_str = f"{target_angle:08.4f}"       # 角度：8位（3整+小数点+4小）
        cmd_content = f"2{dir_str}{acc_str}{speed_str}{angle_str}"

        # 发送指令
        self._send_command(cmd_content)
        print(f"✅ 位置模式指令已发送：方向{direction} | 加速度{acceleration}°/s² | 速度{speed}°/s | 目标角度{target_angle}°")

    def rotate_speed(self, direction: int, acceleration: int, speed: float) -> None:
        """
        速率模式旋转（指令格式：$13+方向+加速度+速度）
        :param direction: 0=顺时针 / 1=逆时针
        :param acceleration: 1~1000（4位补零）
        :param speed: 0.0001~1000.0（9位格式）
        """
        print("\n=== 执行【速率模式旋转】===")
        # 参数校验
        if direction not in [0, 1]:
            print("❌ 方向错误！仅支持0（顺时针）/1（逆时针）")
            return
        if not (1 <= acceleration <= 1000):
            print("❌ 加速度错误！范围1~1000（度/秒²）")
            return
        if not (0.0001 <= speed <= 1000.0):
            print("❌ 速度错误！范围0.0001~1000.0（度/秒）")
            return

        # 格式化参数
        dir_str = str(direction)
        acc_str = f"{acceleration:04d}"
        speed_str = f"{speed:09.4f}"
        cmd_content = f"3{dir_str}{acc_str}{speed_str}"

        # 发送指令
        self._send_command(cmd_content)
        print(f"✅ 速率模式指令已发送：方向{direction} | 加速度{acceleration}°/s² | 速度{speed}°/s")

    def rotate_swing(self, amplitude: float, frequency: float) -> None:
        """
        摇摆模式旋转（指令格式：$14+幅度+频率）
        :param amplitude: 0~359.9999（8位格式，如10→010.0000）
        :param frequency: 0.001~10.0（6位格式，如0.1→00.100）
        """
        print("\n=== 执行【摇摆模式旋转】===")
        # 参数校验
        if not (0 <= amplitude <= 359.9999):
            print("❌ 摇摆幅度错误！范围0~359.9999（度）")
            return
        if not (0.001 <= frequency <= 10.0):
            print("❌ 摇摆频率错误！范围0.001~10.0（HZ）")
            return

        # 格式化参数
        amp_str = f"{amplitude:08.4f}"   # 幅度：8位（3整+小数点+4小）
        freq_str = f"{frequency:06.3f}"  # 频率：6位（2整+小数点+3小）
        cmd_content = f"4{amp_str}{freq_str}"

        # 发送指令
        self._send_command(cmd_content)
        print(f"✅ 摇摆模式指令已发送：幅度{amplitude}° | 频率{frequency}HZ")

    def rotate_multi_circle(self, direction: int, acceleration: int, speed: float, target_angle: float, circles: int) -> None:
        """
        多圈位置模式旋转（指令格式：$15+方向+加速度+速度+角度+圈数）
        :param direction: 0=顺时针 / 1=逆时针
        :param acceleration: 1~1000（4位补零）
        :param speed: 0.0001~1000.0（9位格式）
        :param target_angle: 0~359.9999（8位格式）
        :param circles: 1~99（2位补零，如2→02）
        """
        print("\n=== 执行【多圈位置模式旋转】===")
        # 参数校验
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

        # 格式化参数
        dir_str = str(direction)
        acc_str = f"{acceleration:04d}"
        speed_str = f"{speed:09.4f}"
        angle_str = f"{target_angle:08.4f}"
        circle_str = f"{circles:02d}"  # 圈数：2位补零
        cmd_content = f"5{dir_str}{acc_str}{speed_str}{angle_str}{circle_str}"

        # 发送指令
        self._send_command(cmd_content)
        print(f"✅ 多圈位置模式指令已发送：方向{direction} | 加速度{acceleration}°/s² | 速度{speed}°/s | 目标角度{target_angle}° | 圈数{circles}")


# -------------------------- 菜单交互功能（方便单独控制）--------------------------
def print_menu():
    print("\n" + "="*60)
    print("📋 国产转盘控制菜单（指令已验证）")
    print("="*60)
    print("1. 电机释放（mo=0）          2. 电机上电（mo=1）")
    print("3. 停车（st）                4. 回零（1）")
    print("5. 设置状态发送频率（rs=N）  6. 位置模式旋转")
    print("7. 速率模式旋转              8. 摇摆模式旋转")
    print("9. 多圈位置模式旋转          0. 退出程序")
    print("="*60)


def input_int(prompt: str, min_val: int, max_val: int, default: int = None) -> int:
    """输入整数（带范围校验+默认值）"""
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
    """输入浮点数（带范围校验+默认值）"""
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
    # 初始化控制器
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
                controller.homing()
            elif choice == 5:
                freq_idx = input_int("请输入频率序号（0~7，默认0）：", 0, 7, default=0)
                controller.set_status_freq(freq_idx)
            elif choice == 6:
                # 位置模式：默认参数与指令汇总表示例一致
                dir_ = input_int("方向（0=顺时针/1=逆时针，默认0）：", 0, 1, default=0)
                acc = input_int("加速度（1~1000，默认10）：", 1, 1000, default=10)
                spd = input_float("速度（0.0001~1000.0，默认10.0）：", 0.0001, 1000.0, default=10.0)
                angle = input_float("目标角度（0~359.9999，默认180.0）：", 0, 359.9999, default=180.0)
                controller.rotate_position(dir_, acc, spd, angle)
            elif choice == 7:
                # 速率模式：默认参数与指令汇总表示例一致
                dir_ = input_int("方向（0=顺时针/1=逆时针，默认1）：", 0, 1, default=1)
                acc = input_int("加速度（1~1000，默认10）：", 1, 1000, default=10)
                spd = input_float("速度（0.0001~1000.0，默认10.0）：", 0.0001, 1000.0, default=10.0)
                controller.rotate_speed(dir_, acc, spd)
            elif choice == 8:
                # 摇摆模式：默认参数与指令汇总表示例一致
                amp = input_float("摇摆幅度（0~359.9999，默认10.0）：", 0, 359.9999, default=10.0)
                freq = input_float("摇摆频率（0.001~10.0，默认0.1）：", 0.001, 10.0, default=0.1)
                controller.rotate_swing(amp, freq)
            elif choice == 9:
                # 多圈模式：默认参数与指令汇总表示例一致
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

            # 执行完功能后等待状态稳定
            time.sleep(2)

    except KeyboardInterrupt:
        print("\n⚠️  检测到手动中断，执行紧急停车+电机释放...")
        controller.stop()
        controller.motor_release()
    finally:
        controller.disconnect()
        print("\n=== 程序结束 ===")
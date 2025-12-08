import argparse
import json
import time
import sys
import serial
import threading
import csv
from abc import ABC, abstractmethod
from datetime import datetime

# ==========================================
# 辅助打印
# ==========================================
def print_resp(msg):
    print(f"> {msg}")

# ==========================================
# 1. 抽象基类
# ==========================================
class TurntableDriver(ABC):
    def __init__(self, config):
        self.config = config
        self.port = config.get("COMPort")
        self.baud = config.get("Baudrate")
        self.is_connected = False
        # 初始状态设为 None，强制必须读到数据才算有效
        self.latest_state = {"status": None, "angle": 0.0, "alarm": "0"}
        self.lock = threading.Lock()
        
        self.save_csv_path = None
        self.print_screen = False
        self.csv_file = None
        self.csv_writer = None
        self.running = False

    @abstractmethod
    def connect(self) -> bool: pass

    @abstractmethod
    def disconnect_only(self): pass

    @abstractmethod
    def emergency_stop_and_close(self): pass

    @abstractmethod
    def cmd_init(self) -> bool: pass

    @abstractmethod
    def cmd_free(self) -> bool: pass

    @abstractmethod
    def cmd_stop(self) -> bool: pass

    @abstractmethod
    def cmd_speed_run(self, acc, speed) -> bool: pass

    @abstractmethod
    def cmd_position_run(self, dir_code, acc, speed, target_angle) -> bool:
        """
        Mode 2: 位置模式
        dir_code: 0=顺时针, 1=逆时针
        target_angle: 绝对目标角度 (0-360)
        """
        pass

    @abstractmethod
    def cmd_multi_run(self, dir_code, acc, speed, target_angle, loops) -> bool:
        """
        Mode 5: 多圈模式
        loops: 圈数 (0-99)
        """
        pass

    @abstractmethod
    def get_current_state(self): pass

    def set_output_options(self, print_screen: bool, csv_path: str):
        self.print_screen = print_screen
        self.save_csv_path = csv_path

# ==========================================
# 2. RUYA (如洋) 协议实现
# ==========================================
class RuyaDriver(TurntableDriver):
    def __init__(self, config):
        super().__init__(config)
        self.ser = None
        self.listen_thread = None

    def connect(self) -> bool:
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05
            )
            if self.ser.is_open:
                self.is_connected = True
                self.running = True
                
                if self.save_csv_path:
                    try:
                        self.csv_file = open(self.save_csv_path, 'w', newline='')
                        self.csv_writer = csv.writer(self.csv_file)
                        self.csv_writer.writerow(["Timestamp", "Status", "Alarm", "Angle"])
                    except Exception as e:
                        print_resp(f"Error opening CSV: {e}")

                self.listen_thread = threading.Thread(target=self._listen_loop, daemon=True)
                self.listen_thread.start()
                
                # 等待直到收到第一帧有效数据
                retry = 0
                while self.latest_state["status"] is None and retry < 20:
                    time.sleep(0.1)
                    retry += 1
                
                return True
            return False
        except Exception as e:
            print_resp(f"Connection Error: {e}")
            return False

    def disconnect_only(self):
        self.running = False
        time.sleep(0.1) 
        if self.ser and self.ser.is_open:
            try: self.ser.close()
            except: pass
        if self.csv_file:
            try: self.csv_file.close()
            except: pass
        self.is_connected = False

    def emergency_stop_and_close(self):
        self.running = False 
        if self.ser and self.ser.is_open:
            try:
                self._send_raw("st")
                time.sleep(0.1)
                self._send_raw("mo=0")
                time.sleep(0.1)
            except: pass
            try: self.ser.close()
            except: pass
        if self.csv_file:
            try: self.csv_file.close()
            except: pass
        self.is_connected = False

    def _listen_loop(self):
        buffer = ""
        while self.running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    raw = self.ser.read(self.ser.in_waiting).decode('ascii', errors='replace')
                    buffer += raw
                    if '\n' in buffer:
                        lines = buffer.split('\n')
                        buffer = lines[-1]
                        for line in reversed(lines[:-1]):
                            line = line.strip()
                            if line.startswith('$1') and len(line) >= 14:
                                self._parse_frame(line)
                                break
                else:
                    time.sleep(0.005)
            except Exception:
                pass

    def _parse_frame(self, data):
        try:
            content = data[2:].strip()
            alarm = content[0]
            status = content[1]
            angle_val = float(content[4:12])
            if angle_val > 360.0: angle_val -= 720.0
            
            with self.lock:
                self.latest_state["status"] = status
                self.latest_state["alarm"] = alarm
                self.latest_state["angle"] = angle_val

            if self.print_screen:
                print(f"Status: {status} | Alarm: {alarm} | Angle: {angle_val:.4f}")

            if self.csv_writer:
                self.csv_writer.writerow([
                    datetime.now().strftime("%H:%M:%S.%f"),
                    status, alarm, angle_val
                ])
        except:
            pass

    def _send_raw(self, cmd_str):
        if not self.is_connected: return False
        try:
            full_cmd = f"$1{cmd_str}\r\n"
            self.ser.write(full_cmd.encode('ascii'))
            return True
        except:
            return False

    def _ensure_ready_to_move(self):
        timeout = 5.0
        start_t = time.time()

        while time.time() - start_t < timeout:
            s = self.latest_state["status"]
            if s == '1': return True
            if s == '0':
                print_resp("Auto-Initializing (mo=1)...")
                self._send_raw("mo=1")
                time.sleep(0.5) 
                continue
            if s in ['2', '3', '4', '5', '6', '7', '9']:
                print_resp("Stopping previous motion to change mode...")
                self._send_raw("st")
                time.sleep(0.2)
                continue
            if s == '8':
                time.sleep(0.1)
                continue
            time.sleep(0.1)
        
        print_resp("Error: Failed to ready turntable (Timeout)")
        return False

    # --- 接口实现 ---
    def cmd_init(self) -> bool: return self._send_raw("mo=1")
    def cmd_free(self) -> bool: return self._send_raw("mo=0")
    def cmd_stop(self) -> bool: return self._send_raw("st")

    def cmd_speed_run(self, acc, speed) -> bool:
        if not self._ensure_ready_to_move(): return False
        direction = 0 
        acc_clamped = max(1, min(1000, int(acc)))
        spd_clamped = max(0.0001, min(1000.0, float(speed)))
        cmd = f"3{direction}{acc_clamped:04d}{spd_clamped:09.4f}"
        return self._send_raw(cmd)

    def cmd_position_run(self, dir_code, acc, speed, target_angle) -> bool:
        """Mode 2: 标准位置模式"""
        if not self._ensure_ready_to_move(): return False
        
        acc_clamped = max(1, min(1000, int(acc)))
        spd_clamped = max(0.0001, min(1000.0, float(speed)))
        ang_clamped = float(target_angle)
        
        # 格式: 2{dir}{acc}{spd}{angle}
        cmd = f"2{dir_code}{acc_clamped:04d}{spd_clamped:09.4f}{ang_clamped:08.4f}"
        return self._send_raw(cmd)

    def cmd_multi_run(self, dir_code, acc, speed, target_angle, loops) -> bool:
        """Mode 5: 多圈模式 (New!)"""
        if not self._ensure_ready_to_move(): return False
        
        acc_clamped = max(1, min(1000, int(acc)))
        spd_clamped = max(0.0001, min(1000.0, float(speed)))
        ang_clamped = float(target_angle)
        loops_clamped = max(0, min(99, int(loops))) # 协议限制2位数字 (00-99)
        
        # 格式: 5{dir}{acc}{spd}{angle}{loops}
        cmd = f"5{dir_code}{acc_clamped:04d}{spd_clamped:09.4f}{ang_clamped:08.4f}{loops_clamped:02d}"
        return self._send_raw(cmd)

    def get_current_state(self):
        with self.lock:
            return self.latest_state.copy()

# ==========================================
# 3. 业务逻辑核心 (Calculation)
# ==========================================
def calculate_move_params(current_angle, input_delta):
    """
    核心算法：根据当前角度和增量，计算方向、圈数、绝对目标角度
    """
    # 1. 确定方向
    # 增量 > 0: 顺时针(0); 增量 < 0: 逆时针(1)
    direction = 0 if input_delta >= 0 else 1
    
    # 2. 计算绝对增量值
    abs_delta = abs(input_delta)
    
    # 3. 计算圈数 (整数部分)
    loops = int(abs_delta // 360)
    
    # 4. 计算剩余角度 (小数部分)
    remainder = abs_delta % 360
    
    # 5. 计算目标绝对角度
    # 如果是顺时针，目标 = 当前 + 余数
    # 如果是逆时针，目标 = 当前 - 余数
    if direction == 0:
        target_abs = current_angle + remainder
    else:
        target_abs = current_angle - remainder
        
    # 6. 归一化目标角度到 [0, 360)
    # 比如算出来是 370，其实是 10度
    # 比如算出来是 -30，其实是 330度
    if target_abs >= 360.0:
        target_abs -= 360.0
    elif target_abs < 0.0:
        target_abs += 360.0
        
    return direction, loops, target_abs

# ==========================================
# 4. 主程序
# ==========================================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--command", required=True)
    parser.add_argument("--acc", type=float)
    parser.add_argument("--speed", type=float)
    parser.add_argument("--angle", type=float) # 注意：这里现在代表“增量”
    parser.add_argument("--printScreen", type=str, default="False")
    parser.add_argument("--SaveCSVFile", type=str)

    args = parser.parse_args()

    # Load Config
    try:
        with open(args.config, 'r', encoding='utf-8') as f: config = json.load(f)
    except Exception as e:
        print_resp(f"Error loading config: {e}")
        return

    # Init Driver
    driver = RuyaDriver(config)
    is_print = args.printScreen.lower() == "true"
    driver.set_output_options(is_print, args.SaveCSVFile)

    if not driver.connect():
        print_resp("Error: Connection Failed")
        return

    force_cleanup = False
    cmd = args.command
    
    try:
        if cmd == "Init":
            if driver.cmd_init(): print_resp("OK")
            else: print_resp("Error")

        elif cmd == "Free Mode":
            if driver.cmd_free(): print_resp("OK")
            else: print_resp("Error")

        elif cmd == "Stop":
            if driver.cmd_stop(): print_resp("OK")
            else: print_resp("Error")

        elif cmd == "Speed Run":
            if args.acc is None or args.speed is None:
                print_resp("Error: Missing params")
            else:
                if driver.cmd_speed_run(args.acc, args.speed):
                    print_resp("OK")
                    if is_print:
                        while True: time.sleep(0.5)
                    else:
                        time.sleep(0.5)
                else:
                    print_resp("Error")

        # === 核心修改：Position Run 智能逻辑 ===
        elif cmd == "Position Run":
            force_cleanup = True 
            
            if args.acc is None or args.speed is None or args.angle is None:
                print_resp("Error: Missing params")
            else:
                # 1. 获取当前状态
                state = driver.get_current_state()
                current_angle = state['angle']
                input_delta = args.angle
                
                print_resp(f"Current: {current_angle:.4f} | Input Delta: {input_delta}")
                
                # 2. 计算目标参数
                # dir_code: 0=CW, 1=CCW
                # loops: 圈数
                # target_abs: 0-360的绝对目标位置
                dir_code, loops, target_abs = calculate_move_params(current_angle, input_delta)
                
                print_resp(f"Calc Result -> Dir: {dir_code} (0=CW/1=CCW) | Loops: {loops} | Target Abs: {target_abs:.4f}")
                
                # 3. 智能选择指令
                success = False
                if loops == 0:
                    # 圈数为0，使用普通位置模式 (Mode 2)
                    print_resp("Action: Single Turn Mode (Mode 2)")
                    success = driver.cmd_position_run(dir_code, args.acc, args.speed, target_abs)
                else:
                    # 圈数>0，使用多圈模式 (Mode 5)
                    # 注意：协议限制最大99圈，如果这里算出100圈，可能需要做限幅或分段
                    if loops > 99:
                        print_resp("Warning: Loops > 99, capped at 99 by protocol limit.")
                    print_resp(f"Action: Multi Turn Mode (Mode 5) - {loops} loops")
                    success = driver.cmd_multi_run(dir_code, args.acc, args.speed, target_abs, loops)

                # 4. 等待到位逻辑 (通用)
                if success:
                    print_resp("OK")
                    print_resp(f"POSHEAD {current_angle:.4f}") # 打印起始角度
                    
                    time.sleep(0.2) 
                    wait_start = time.time()
                    # 多圈可能时间很久，超时时间根据圈数动态增加
                    # 假设最慢 1度/秒，转1圈360秒。这得给够时间。
                    # 简单估算：每圈给60秒余量，或者直接设个很大的值
                    timeout = 120 + (loops * 60) 
                    
                    completed = False
                    
                    while time.time() - wait_start < timeout:
                        s = driver.get_current_state()
                        status = s['status']
                        # 检查状态是否回到 1 (伺服) 或 0
                        if status in ['1', '0']:
                            time.sleep(0.5)
                            s2 = driver.get_current_state()
                            if s2['status'] in ['1', '0']:
                                completed = True
                                break
                        time.sleep(0.1)

                    if completed:
                        print_resp("Complete")
                        final = driver.get_current_state()
                        print_resp(f"POSTAIL {final['angle']:.4f}")
                        force_cleanup = False
                    else:
                        print_resp("Error: Timeout")
                else:
                    print_resp("Error: Send command failed")

        else:
            print_resp(f"Error: Unknown command")

    except KeyboardInterrupt:
        force_cleanup = True
    except Exception as e:
        print_resp(f"Error: {e}")
        force_cleanup = True
    finally:
        if force_cleanup:
            driver.emergency_stop_and_close()
        else:
            driver.disconnect_only()

if __name__ == "__main__":
    main()
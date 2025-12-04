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
        self.latest_state = {"status": None, "angle": 0.0, "alarm": "0"}
        self.lock = threading.Lock()
        
        self.save_csv_path = None
        self.print_screen = False
        self.csv_file = None
        self.csv_writer = None
        self.running = False

    @abstractmethod
    def connect(self) -> bool:
        pass

    @abstractmethod
    def disconnect_only(self):
        """正常断开：只关串口，保持电机状态（关键！）"""
        pass

    @abstractmethod
    def emergency_stop_and_close(self):
        """紧急断开：停车+释放+关串口（用于Ctrl+C）"""
        pass

    @abstractmethod
    def cmd_init(self) -> bool:
        pass

    @abstractmethod
    def cmd_free(self) -> bool:
        pass

    @abstractmethod
    def cmd_stop(self) -> bool:
        pass

    @abstractmethod
    def cmd_speed_run(self, acc, speed) -> bool:
        pass

    @abstractmethod
    def cmd_position_run(self, acc, speed, angle) -> bool:
        pass

    @abstractmethod
    def get_current_state(self):
        pass

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
        """ 正常退出：只关串口，不发停车指令 """
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
        """ 异常退出：强制停车释放 """
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
    def cmd_init(self) -> bool:
        return self._send_raw("mo=1")

    def cmd_free(self) -> bool:
        return self._send_raw("mo=0")

    def cmd_stop(self) -> bool:
        return self._send_raw("st")

    def cmd_speed_run(self, acc, speed) -> bool:
        if not self._ensure_ready_to_move(): return False
        
        direction = 0 
        acc_clamped = max(1, min(1000, int(acc)))
        spd_clamped = max(0.0001, min(1000.0, float(speed)))
        cmd = f"3{direction}{acc_clamped:04d}{spd_clamped:09.4f}"
        return self._send_raw(cmd)

    def cmd_position_run(self, acc, speed, angle) -> bool:
        if not self._ensure_ready_to_move(): return False

        direction = 0 
        acc_clamped = max(1, min(1000, int(acc)))
        spd_clamped = max(0.0001, min(1000.0, float(speed)))
        ang_clamped = float(angle)
        cmd = f"2{direction}{acc_clamped:04d}{spd_clamped:09.4f}{ang_clamped:08.4f}"
        return self._send_raw(cmd)

    def get_current_state(self):
        with self.lock:
            return self.latest_state.copy()

# ==========================================
# 3. 主程序逻辑
# ==========================================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--command", required=True)
    parser.add_argument("--acc", type=float)
    parser.add_argument("--speed", type=float)
    parser.add_argument("--angle", type=float)
    parser.add_argument("--printScreen", type=str, default="False")
    parser.add_argument("--SaveCSVFile", type=str)

    args = parser.parse_args()

    # 1. 加载配置
    try:
        with open(args.config, 'r', encoding='utf-8') as f:
            config = json.load(f)
    except Exception as e:
        print_resp(f"Error loading config: {e}")
        return

    # 2. 实例化
    driver = RuyaDriver(config)
    is_print = args.printScreen.lower() == "true"
    driver.set_output_options(is_print, args.SaveCSVFile)

    # 3. 连接
    if not driver.connect():
        print_resp("Error: Connection Failed")
        return

    # 标记：是否因为异常退出而需要强制清理
    force_cleanup = False

    # 4. 执行
    cmd = args.command
    
    try:
        # === Init ===
        if cmd == "Init":
            if driver.cmd_init(): print_resp("OK")
            else: print_resp("Error")

        # === Free Mode ===
        elif cmd == "Free Mode":
            if driver.cmd_free(): print_resp("OK")
            else: print_resp("Error")

        # === Stop ===
        elif cmd == "Stop":
            if driver.cmd_stop(): print_resp("OK")
            else: print_resp("Error")

        # === Speed Run (核心修改) ===
        elif cmd == "Speed Run":
            if args.acc is None or args.speed is None:
                print_resp("Error: Missing params")
            else:
                if driver.cmd_speed_run(args.acc, args.speed):
                    print_resp("OK")
                    
                    # 关键逻辑：
                    # 如果用户要求 printScreen，我们必须死循环来保持打印，此时退出只能靠 Ctrl+C
                    if is_print:
                        while True:
                            time.sleep(0.5)
                    else:
                        # 如果不打印，发完指令直接退出！
                        # 并且退出时 force_cleanup 为 False，所以不会停止电机
                        time.sleep(0.5) # 稍微等一下确保指令发完
                        
                else:
                    print_resp("Error")

        # === Position Run ===
        elif cmd == "Position Run":
            force_cleanup = True # 位置模式如果被打断，通常建议停止
            
            if args.acc is None or args.speed is None or args.angle is None:
                print_resp("Error: Missing params")
            else:
                state = driver.get_current_state()
                print_resp(f"POSHEAD {state['angle']:.4f}")
                
                if driver.cmd_position_run(args.acc, args.speed, args.angle):
                    print_resp("OK")
                    
                    time.sleep(0.2) 
                    wait_start = time.time()
                    timeout = 120
                    completed = False
                    
                    while time.time() - wait_start < timeout:
                        s = driver.get_current_state()
                        status = s['status']
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
                        force_cleanup = False # 正常到位，保持状态退出
                    else:
                        print_resp("Error: Timeout")
                else:
                    print_resp("Error")

        else:
            print_resp(f"Error: Unknown command")

    except KeyboardInterrupt:
        force_cleanup = True
        # print_resp("Interrupted")
    except Exception as e:
        print_resp(f"Error: {e}")
        force_cleanup = True
    finally:
        # 5. 退出策略
        if force_cleanup:
            # 只有在 Ctrl+C 或 出错时，才停车+释放
            driver.emergency_stop_and_close()
        else:
            # 正常运行结束 (Init, Free, Stop, SpeedRun, PositionRun到位)
            # 只断开串口，不改变电机状态
            driver.disconnect_only()

if __name__ == "__main__":
    main()
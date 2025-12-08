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
    def cmd_reset(self) -> bool: 
        """回零指令"""
        pass

    @abstractmethod
    def cmd_speed_run(self, acc, speed) -> bool: pass

    @abstractmethod
    def cmd_position_run(self, dir_code, acc, speed, target_angle) -> bool: pass

    @abstractmethod
    def cmd_multi_run(self, dir_code, acc, speed, target_angle, loops) -> bool: pass

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
                print_resp("Stopping previous motion...")
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

    def cmd_reset(self) -> bool:
        """Mode: Reset/Homing"""
        # 回零也需要转台准备好（例如不能正在高速旋转）
        if not self._ensure_ready_to_move(): return False
        # 协议指令: 1
        return self._send_raw("1")

    def cmd_speed_run(self, acc, speed) -> bool:
        if not self._ensure_ready_to_move(): return False
        direction = 0 
        acc_clamped = max(1, min(1000, int(acc)))
        spd_clamped = max(0.0001, min(1000.0, float(speed)))
        cmd = f"3{direction}{acc_clamped:04d}{spd_clamped:09.4f}"
        return self._send_raw(cmd)

    def cmd_position_run(self, dir_code, acc, speed, target_angle) -> bool:
        if not self._ensure_ready_to_move(): return False
        acc_clamped = max(1, min(1000, int(acc)))
        spd_clamped = max(0.0001, min(1000.0, float(speed)))
        ang_clamped = float(target_angle)
        cmd = f"2{dir_code}{acc_clamped:04d}{spd_clamped:09.4f}{ang_clamped:08.4f}"
        return self._send_raw(cmd)

    def cmd_multi_run(self, dir_code, acc, speed, target_angle, loops) -> bool:
        if not self._ensure_ready_to_move(): return False
        acc_clamped = max(1, min(1000, int(acc)))
        spd_clamped = max(0.0001, min(1000.0, float(speed)))
        ang_clamped = float(target_angle)
        loops_clamped = max(0, min(99, int(loops)))
        cmd = f"5{dir_code}{acc_clamped:04d}{spd_clamped:09.4f}{ang_clamped:08.4f}{loops_clamped:02d}"
        return self._send_raw(cmd)

    def get_current_state(self):
        with self.lock:
            return self.latest_state.copy()

# ==========================================
# 3. 业务逻辑核心 (Calculation)
# ==========================================
def calculate_move_params(current_angle, input_delta):
    direction = 0 if input_delta >= 0 else 1
    abs_delta = abs(input_delta)
    loops = int(abs_delta // 360)
    remainder = abs_delta % 360
    
    if direction == 0:
        target_abs = current_angle + remainder
    else:
        target_abs = current_angle - remainder
        
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
    parser.add_argument("--angle", type=float)
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
            
        # === 新增：Reset (回零) ===
        elif cmd == "Reset":
            force_cleanup = True # 回零是阻塞操作，如果被打断建议清理
            if driver.cmd_reset():
                print_resp("OK")
                
                # 回零可能需要很长时间，逻辑类似 Position Run
                time.sleep(1.0) # 等待开始转动
                wait_start = time.time()
                timeout = 180 # 假设回零最慢需要3分钟
                completed = False
                
                while time.time() - wait_start < timeout:
                    s = driver.get_current_state()
                    status = s['status']
                    current_angle = s['angle']
                    
                    # 判定回零成功条件：
                    # 1. 状态必须是 静止(1) 或 空闲(0)
                    # 2. 角度必须极其接近 0.0 (误差 < 0.1度)
                    if status in ['1', '0'] and abs(current_angle) < 0.1:
                        # 二次确认
                        time.sleep(0.5)
                        s2 = driver.get_current_state()
                        if s2['status'] in ['1', '0'] and abs(s2['angle']) < 0.1:
                            completed = True
                            break
                    
                    time.sleep(0.2)

                if completed:
                    print_resp("Complete")
                    final = driver.get_current_state()
                    print_resp(f"POSTAIL {final['angle']:.4f}")
                    force_cleanup = False
                else:
                    print_resp("Error: Timeout or Not at Zero")
            else:
                print_resp("Error: Send Reset failed")

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

        elif cmd == "Position Run":
            force_cleanup = True 
            
            if args.acc is None or args.speed is None or args.angle is None:
                print_resp("Error: Missing params")
            else:
                state = driver.get_current_state()
                current_angle = state['angle']
                input_delta = args.angle
                
                print_resp(f"Current: {current_angle:.4f} | Input Delta: {input_delta}")
                
                dir_code, loops, target_abs = calculate_move_params(current_angle, input_delta)
                
                print_resp(f"Calc Result -> Dir: {dir_code} (0=CW/1=CCW) | Loops: {loops} | Target Abs: {target_abs:.4f}")
                
                success = False
                if loops == 0:
                    print_resp("Action: Single Turn Mode (Mode 2)")
                    success = driver.cmd_position_run(dir_code, args.acc, args.speed, target_abs)
                else:
                    if loops > 99: print_resp("Warning: Loops > 99, capped at 99.")
                    print_resp(f"Action: Multi Turn Mode (Mode 5) - {loops} loops")
                    success = driver.cmd_multi_run(dir_code, args.acc, args.speed, target_abs, loops)

                if success:
                    print_resp("OK")
                    print_resp(f"POSHEAD {current_angle:.4f}")
                    
                    time.sleep(0.2) 
                    wait_start = time.time()
                    timeout = 120 + (loops * 60) 
                    
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
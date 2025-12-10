# RUYA单轴转台命令行控制工具使用手册

## 1. 简介

本工具是一个基于命令行的转台控制程序，支持 **RUYA（如洋）** 协议。它设计用于自动化测试脚本集成或手动调试。

* 支持相对角度控制、自动多圈计算、智能状态切换、回零控制、CSV数据记录及异常安全保护。

---

## 2. 配置文件

在运行程序前，必须在同目录下创建一个 JSON 配置文件（例如 `RUYAconfig.json`）。

**文件内容示例：**

```json
{
  "COMPort": "COM5",     
  "Baudrate": 115200,    
  "UartAsciiStart": "$1", 
  "UartAsciiStop": "\r\n",
  "Protocol": "RUYA"    
}
```

* **COMPort**: 转台连接的串口号（如 Windows 下的 `COM3`, `COM5`）。
* **Baudrate**: 波特率，通常为 `115200`。
* **Protocol**: 协议类型，目前固定为 `"RUYA"`。

---

## 3. 命令行参数总览

**调用格式：**

```bash
python main.py --config [配置文件] --command [指令名称] [可选参数]
# 或者 (如果是编译后的exe)
main.exe --config [配置文件] --command [指令名称] [可选参数]
```


| 参数            | 类型   | 必填   | 说明                    | 示例                  |
| :-------------- | :----- | :----- | :---------------------- | :-------------------- |
| `--config`      | String | **是** | 配置文件路径            | `"RUYAconfig.json"`   |
| `--command`     | String | **是** | 执行的动作指令          | `"Position Run"`      |
| `--acc`         | Float  | 视指令 | 加速度 (度/秒²)         | `100`                 |
| `--speed`       | Float  | 视指令 | 运行速度 (度/秒)        | `20`                  |
| `--angle`       | Float  | 视指令 | **增量角度** (度)       | `90` 或 `-180`        |
| `--printScreen` | String | 否     | 是否刷屏打印实时数据    | `"True"` 或 `"False"` |
| `--SaveCSVFile` | String | 否     | 保存日志到CSV文件的路径 | `"log.csv"`           |

---

## 4. 指令详解 (Command)

### 4.1 基础控制

#### `Init` (初始化/使能)

* **功能**：给电机上电，进入伺服保持状态（Mode 1）。

* **示例**：

  ```bash
  main.exe --config="RUYAconfig.json" --command "Init"
  ```

* **输出**：`> OK`

#### `Free Mode` (释放/空闲)

* **功能**：给电机断电，转台可手推（Mode 0）。

* **示例**：

  ```bash
  main.exe --config="RUYAconfig.json" --command "Free Mode"
  ```

* **输出**：`> OK`

#### `Stop` (停止)

* **功能**：使转台停止转动，但保持伺服力矩（Mode 1）。

* **示例**：

  ```bash
  main.exe --config="RUYAconfig.json" --command "Stop"
  ```

* **输出**：`> OK`

#### `Reset` (回零)

* **功能**：转台自动寻找零位。程序会**阻塞等待**直到转台完全停在 0.0 度附近。

* **逻辑**：自动停车 -> 发送回零指令 -> 轮询等待到位。

* **示例**：

  ```bash
  main.exe --config="RUYAconfig.json" --command "Reset"
  ```

* **输出**：

  ```text
  > OK
  > Complete
  > POSTAIL 0.0000
  ```

---

### 4.2 运动控制

#### `Speed Run` (速率模式)

* **功能**：按指定速度持续旋转。

* **行为特点**：**“发射后不管”**。发送指令成功后，脚本立即退出，转台**保持旋转**。

* **参数**：

  * `--acc`: 加速度
  * `--speed`: 速度

* **示例**：

  ```bash
  main.exe --config="RUYAconfig.json" --command "Speed Run" --acc 10 --speed 50
  ```

* **如何停止**：需再次运行 `Stop` 命令，或在运行时开启监控并按 Ctrl+C。

#### `Position Run` (位置模式 - 相对运动)

* **功能**：从当前位置转动指定的**增量角度**。支持自动多圈计算。

* **行为特点**：**“阻塞等待”**。程序会一直运行，直到转台到达目标位置停稳后才退出。

* **参数**：

  * `--acc`: 加速度
  * `--speed`: 速度
  * `--angle`: **增量角度** (正数=顺时针，负数=逆时针，支持大于360度)

* **智能逻辑**：

  * 自动计算：根据增量自动判断是单圈模式(Mode 2)还是多圈模式(Mode 5)。
  * 自动使能：如果当前处于释放状态，会自动先上电。

* **示例 1 (顺时针转 90 度)**：

  ```bash
  main.exe --config="RUYAconfig.json" --command "Position Run" --acc 20 --speed 20 --angle 90
  ```

* **示例 2 (逆时针转 10 圈)**：

  ```bash
  main.exe --config="RUYAconfig.json" --command "Position Run" --acc 20 --speed 50 --angle -3600
  ```

* **输出流**：

  ```text
  > Current: 30.0000 | Input Delta: 90.0   (计算过程)
  > Action: Single Turn Mode ...
  > OK                                     (指令发送成功)
  > POSHEAD 30.0000                        (起始位置)
  > Complete                               (到位停稳)
  > POSTAIL 120.0000                       (结束位置)
  ```

---

## 5. 高级功能

### 5.1 实时监控 (`--printScreen`)

如果你希望在执行 `Speed Run` 时脚本不要退出，而是像示波器一样打印实时数据：

```bash
main.exe --config="RUYAconfig.json" --command "Speed Run" ... --printScreen True
```

* **注意**：此时脚本会死循环，直到按下 `Ctrl+C` 停止。

### 5.2 数据保存 (`--SaveCSVFile`)

将运行过程中的所有状态记录到 Excel 可读的 CSV 文件中：

```bash
main.exe --config="RUYAconfig.json" --command "Position Run" ... --SaveCSVFile "my_test_data.csv"
```

* 文件包含列：`Timestamp` (时间戳), `Status` (状态码), `Alarm` (报警码), `Angle` (角度)。

---

## 6. 安全与异常机制

1. **自动状态修复 (Auto-Ready)**：

   * 在执行任何运动指令前，程序会自动检查转台状态。
   * 如果电机未使能，会自动发送 `Init`。
   * 如果电机正在转，会自动发送 `Stop` 并等待停稳。
   * 用户无需手动干预状态。
2. **强制中断保护 (Ctrl+C)**：

   * 在运行时的任何时候按下 `Ctrl+C` 强行终止脚本，程序都会触发 **紧急制动 (Emergency Stop)**。
   * 动作：发送停车指令 -> 发送释放指令 -> 关闭串口。
   * 确保无人值守时的安全性。
3. **正常退出保持**：

   * 如果 `Position Run` 正常跑完，或者 `Speed Run` 指令发送完毕，脚本退出时**只断开串口连接**，**不释放电机**，保持伺服力矩（避免负载掉落）。
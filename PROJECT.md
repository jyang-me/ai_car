# ESP32 语音控制 STM32 小车项目档案

本文档记录当前项目架构、关键代码、硬件接线、通信协议、构建烧录方式和已验证的调试结论。根目录 `README.md` 作为快速入口，本文档作为更偏技术记录的项目说明。

## 1. 项目目标

使用 ESP32-S3 运行小智语音固件，通过语音/MCP 工具生成小车控制指令，再经 JDY34/JDY31 蓝牙透传链路发送给 STM32F103 小车，实现前进、后退、左右转和停止。

当前目标链路：

```text
用户语音
  -> ESP32-S3 xiaozhi-esp32
  -> MCP 工具 self.car.*
  -> CarController 生成 ASCII 指令
  -> ESP32 UART1
  -> JDY34 BLE 主机
  -> BLE 透传
  -> JDY31 BLE 从机
  -> STM32 USART1 PA10
  -> bt_remote 解析
  -> OLED 显示 Cmd/Raw/左右 PWM
  -> 电机控制
```

## 2. 仓库结构

```text
E:\Prj\ai_car
├── README.md                 # 项目快速说明和排查手册
├── PROJECT.md                # 本项目技术档案
├── xiaozhi-esp32\            # ESP32 小智固件工程
└── f103_test\                # STM32F103 小车固件工程
```

主要工程：

| 工程 | 路径 | 作用 |
| --- | --- | --- |
| ESP32 固件 | `xiaozhi-esp32/` | 语音接入、MCP 工具注册、JDY34 透传发送 |
| ESP32 改造归档 | `esp32_ai_car/` | 父仓库中直接可见的 ESP32 小车控制相关源码 |
| STM32 固件 | `f103_test/` | 蓝牙串口接收、OLED 状态显示、电机控制 |

说明：`xiaozhi-esp32/` 是独立上游 Git 仓库，父仓库不直接提交整个上游源码目录。当前 AI 小车相关 ESP32 改动已归档到：

```text
esp32_ai_car/
```

同一批改动也已导出到：

```text
docs/esp32-xiaozhi-ai-car.patch
```

可以直接复制 `esp32_ai_car/` 下的同名路径文件到 `xiaozhi-esp32`，也可以在干净的 `xiaozhi-esp32` 仓库中用 `git apply` 恢复这些改动。

系统架构图文件：

| 文件 | 说明 |
| --- | --- |
| `docs/system_architecture_ai_car.drawio` | 可编辑 draw.io 系统架构图 |
| `docs/system_architecture_ai_car_preview.html` | 浏览器快速预览页 |
| `docs/module_flow_diagrams_ai_car.drawio` | 分模块流程图，包含 ESP32、蓝牙透传、STM32、调试验证 4 页 |
| `docs/module_flow_diagrams_ai_car_preview.html` | 分模块流程图浏览器预览页 |

## 3. 当前实测结论

截至当前调试，已经确认：

- ESP32 端语音/MCP 可以生成小车控制指令。
- ESP32 日志可看到 `MCP/voice car command: forward speed=5 level=5 -> F5`。
- JDY34 发送日志可看到 `Jdy34Ble: TX: F5`。
- JDY31 接 USB-TTL 时串口输出准确。
- 电脑 USB-TTL 直接发送到 STM32 `PA10` 时，OLED 可以显示接收指令。
- 因此 STM32 的 `USART1 / PA10 / OLED` 接收显示链路是正常的。
- 如果手机蓝牙发送后 OLED 无变化，问题不在 STM32 解析代码，重点检查手机是否真正连接 JDY31 的 SPP 串口服务、是否文本发送，以及 JDY31 是否已经被 JDY34 占用。

## 4. 硬件连接

### 4.1 ESP32 与 JDY34

板级配置文件：

```text
xiaozhi-esp32/main/boards/bread-compact-wifi-s3cam/config.h
```

当前配置：

```c
#define JDY34_UART_NUM UART_NUM_1
#define JDY34_UART_TX_PIN GPIO_NUM_14
#define JDY34_UART_RX_PIN GPIO_NUM_3
#define JDY34_STATE_PIN GPIO_NUM_NC
#define JDY34_PEER_MAC "220C180B3AB0"
```

接线：

| JDY34 | ESP32-S3 | 说明 |
| --- | --- | --- |
| VCC | 3.3V | 按模块要求供电 |
| GND | GND | 必须共地 |
| RXD | GPIO14 / UART1 TX | ESP32 发给 JDY34 |
| TXD | GPIO3 / UART1 RX | JDY34 回 ESP32 |

注意：当前板级配置中 `GPIO14` 也被定义为 `LAMP_GPIO`。如果后续出现灯光或 JDY34 串口异常，需要重新分配引脚，避免复用冲突。

### 4.2 JDY31 与 STM32

STM32 使用：

```text
USART1
PA9  = TX
PA10 = RX
9600 8N1
```

接线：

| JDY31 | STM32F103 | 说明 |
| --- | --- | --- |
| VCC | 3.3V | 按模块要求供电 |
| GND | GND | 必须共地 |
| TXD | PA10 / USART1 RX | JDY31 输出到 STM32，最关键 |
| RXD | PA9 / USART1 TX | 可接，用于双向调试 |

若 JDY31 接 USB-TTL 有输出，但 STM32 OLED 不变，优先检查：

- `JDY31 TXD -> STM32 PA10` 是否接对；
- JDY31 与 STM32 是否共地；
- STM32 是否烧录了最新固件；
- STM32 是否进入 `Phone BT` 或 `Voice BT` 页面。

## 5. 通信协议

STM32 当前解析文本 ASCII 指令。推荐 ESP32 语音链路发送两字节格式：

| 指令 | 含义 |
| --- | --- |
| `F5` | 前进，速度档位 5 |
| `B5` | 后退，速度档位 5 |
| `L5` | 左转，速度档位 5 |
| `R5` | 右转，速度档位 5 |
| `S0` | 停止 |

也支持单字母测试：

| 指令 | 含义 |
| --- | --- |
| `F` / `f` / `W` / `w` / `1` | 前进 |
| `B` / `b` / `X` / `x` / `2` | 后退 |
| `L` / `l` / `A` / `a` / `3` | 左转 |
| `R` / `r` / `D` / `d` / `4` | 右转 |
| `S` / `s` / `0` | 停止 |
| `+` / `6` | 速度档位加 1 |
| `-` / `7` | 速度档位减 1 |

手机串口助手注意：

- 使用文本模式发送 `F`、`F5`、`S0`。
- 不要发送中文“前进”“停止”。
- 如果使用 HEX 模式，文本 `F5` 对应 `46 35`，文本 `S0` 对应 `53 30`。

## 6. ESP32 端实现

### 6.1 关键文件

| 文件 | 作用 |
| --- | --- |
| `xiaozhi-esp32/main/boards/common/car_controller.cpp` | 注册 MCP 小车工具，生成 `F5/B5/L5/R5/S0` |
| `xiaozhi-esp32/main/boards/common/car_controller.h` | 小车控制接口 |
| `xiaozhi-esp32/main/boards/common/jdy34_ble.cpp` | JDY34 UART、AT 指令、绑定和透传 |
| `xiaozhi-esp32/main/boards/common/jdy34_ble.h` | JDY34 驱动接口 |
| `xiaozhi-esp32/main/boards/bread-compact-wifi-s3cam/config.h` | JDY34 UART 引脚和 JDY31 MAC |

父仓库中的 ESP32 改动源码归档：

```text
esp32_ai_car/
```

补丁归档：

```text
docs/esp32-xiaozhi-ai-car.patch
```

### 6.2 MCP 工具

当前 ESP32 注册工具：

| 工具 | 发送指令 |
| --- | --- |
| `self.car.forward` | `F<level>` |
| `self.car.backward` | `B<level>` |
| `self.car.left` | `L<level>` |
| `self.car.right` | `R<level>` |
| `self.car.stop` | `S0` |
| `self.car.get_status` | 查询 ESP32/JDY34 状态 |

速度参数支持 `0-9` 档，也支持 `0-100` 百分比。超过 9 的值会归一化为 `0-9` 档。

### 6.3 JDY34 绑定方式

当前代码使用：

```text
AT+BAND<JDY31_MAC>
```

不是旧文档里的 `AT+CONN=...`。

当前绑定 MAC：

```text
220C180B3AB0
```

### 6.4 安全停车

ESP32 端：

```text
CAR_COMMAND_TIMEOUT_MS = 10000 ms
```

当 ESP32 认为小车处于移动状态，且超过 10 秒没有新指令，会自动发送 `S0`。

STM32 端：

```text
BT_REMOTE_MOVE_MS = 2000 ms
```

当 STM32 遥控状态超过 2 秒没有新接收数据，会停车保护。

## 7. STM32 端实现

### 7.1 关键文件

| 文件 | 作用 |
| --- | --- |
| `f103_test/HARDWARE/BT_REMOTE/bt_remote.c` | 指令解析、OLED 显示、遥控控制 |
| `f103_test/HARDWARE/BT_REMOTE/bt_remote.h` | 遥控状态结构和参数 |
| `f103_test/Core/Src/usart.c` | USART 初始化 |
| `f103_test/Core/Src/stm32f1xx_it.c` | USART 中断回调 |
| `f103_test/HARDWARE/MENU/menu.c` | 菜单模式 |

### 7.2 USART1 接收流程

```text
JDY31 TXD
  -> STM32 PA10 / USART1 RX
  -> USART1_IRQHandler()
  -> HAL_UART_IRQHandler(&huart1)
  -> HAL_UART_RxCpltCallback()
  -> bt_remote_on_rx_byte(g_bt_receivedata)
  -> bt_remote_parse_data(data)
  -> bt_remote_start_rx()
```

### 7.3 OLED 显示

当前蓝牙遥控页面只显示核心调试信息：

```text
Phone BT 或 Voice BT
Cmd:<指令><档位> Raw:<原始字符>
Left :<左电机 PWM>
Right:<右电机 PWM>
```

如果 `Cmd` 或 `Raw` 有变化，说明 STM32 收到了串口数据。
如果电脑 USB-TTL 直接发送到 `PA10` 时 OLED 有变化，而 JDY31 接入后没变化，问题在 JDY31 输出到 PA10 这一段。

## 8. 构建与烧录

### 8.1 ESP32

进入工程：

```powershell
cd E:\Prj\ai_car\xiaozhi-esp32
```

编译：

```powershell
idf.py build
```

烧录并监视日志：

```powershell
idf.py -p COM13 flash monitor
```

如果 ESP32 串口不是 `COM13`，替换为设备管理器中的实际端口。

判断 ESP32 到 JDY34 发送是否正常：

```text
CarController: MCP/voice car command: forward speed=5 level=5 -> F5
Jdy34Ble: TX: F5
```

### 8.2 STM32

Keil 工程：

```text
f103_test/MDK-ARM/f103c8t6_test.uvprojx
```

命令行编译：

```powershell
& 'D:\Keil5\KEIL_MDK\UV4\UV4.exe' -b 'E:\Prj\ai_car\f103_test\MDK-ARM\f103c8t6_test.uvprojx' -j0 -o 'E:\Prj\ai_car\f103_test\MDK-ARM\build_from_codex.log'
```

当前已验证编译结果：

```text
0 Error(s), 0 Warning(s)
```

HEX 输出：

```text
f103_test/MDK-ARM/f103c8t6_test/f103c8t6_test.hex
```

推荐用 Keil 图形界面烧录：

1. 打开 `f103_test/MDK-ARM/f103c8t6_test.uvprojx`
2. 连接 ST-Link
3. Build
4. Download

若命令行下载出现：

```text
Flash Download failed - Target DLL has been cancelled
```

说明 ST-Link 下载链路未建立成功，需要检查 ST-Link 接线、驱动、目标板供电和 Keil Flash 配置。

## 9. 标准排查流程

### 9.1 先验 STM32

接线：

```text
USB-TTL TXD -> STM32 PA10
USB-TTL GND -> STM32 GND
```

电脑串口助手：

```text
9600 8N1
文本发送 F 或 F5
```

预期：

```text
OLED 显示 Cmd/Raw 变化
```

如果这一步正常，STM32 代码、PA10 和 OLED 都是好的。

### 9.2 再验 JDY31

接线：

```text
JDY31 TXD -> USB-TTL RXD
JDY31 GND -> USB-TTL GND
```

手机连接 JDY31 后发送：

```text
F
F5
S0
```

预期：

```text
电脑串口助手能看到 F、F5 或 S0
```

如果手机配对成功但电脑串口看不到数据，说明手机没有真正通过 JDY31 的 SPP 串口发送，或 JDY31 已经被 JDY34 占用。

### 9.3 验 JDY34 到 JDY31

接线：

```text
ESP32 -> JDY34
JDY31 TXD -> USB-TTL RXD
共地
```

触发 ESP32 语音前进指令。

预期：

```text
ESP32 日志：Jdy34Ble: TX: F5
电脑串口助手：收到 F5
```

### 9.4 最后验整链路

接线：

```text
ESP32 -> JDY34 -> BLE -> JDY31 TXD -> STM32 PA10
JDY31 GND -> STM32 GND
```

预期：

```text
OLED 显示 Cmd:F5 或 Raw 变化
左右电机 PWM 根据模式和 KEY0 使能状态变化
```

## 10. 常见问题

### 电脑发 STM32 正常，手机蓝牙发 OLED 不变

结论：STM32 没问题，查手机到 JDY31。

检查：

- 手机是否连接 JDY31，而不是 JDY34；
- 是否使用经典蓝牙 SPP 串口助手；
- 是否只是系统配对成功，但串口助手未连接；
- 是否使用文本模式发送；
- JDY31 是否被 JDY34/ESP32 占用；
- JDY31 TXD 接 USB-TTL 时，电脑能否看到手机发送的数据。

### JDY31 有输出，STM32 OLED 不变

检查：

- `JDY31 TXD -> STM32 PA10`；
- `JDY31 GND -> STM32 GND`；
- STM32 是否烧录最新固件；
- 当前是否进入 `Phone BT` 或 `Voice BT`。

### ESP32 一直显示或发送 `S0`

`S0` 是停止指令，不一定是错误。

常见原因：

- 没有新的运动指令；
- ESP32 10 秒安全停车触发；
- STM32 2 秒接收超时停车；
- LLM/语音识别没有调用运动工具；
- 串口助手或手机发送的是停止指令。

### OLED 有 Cmd，但电机不转

检查：

- 是否按 `KEY0` 使能运行；
- 当前是否在 `Phone BT` 或 `Voice BT`；
- 电机驱动板供电是否正常；
- 电机 PWM/方向线是否接对；
- 安全超时是否很快停车。

## 11. 当前关键参数汇总

| 参数 | 当前值 |
| --- | --- |
| STM32 蓝牙串口 | USART1 |
| STM32 蓝牙 RX | PA10 |
| STM32 蓝牙 TX | PA9 |
| STM32 蓝牙波特率 | 9600 |
| STM32 遥控失联停车 | 2000 ms |
| STM32 速度档位 | 1-5，默认 3 |
| ESP32 JDY34 UART | UART1 |
| ESP32 JDY34 TX | GPIO14 |
| ESP32 JDY34 RX | GPIO3 |
| JDY34 绑定 JDY31 MAC | `220C180B3AB0` |
| ESP32 安全停车 | 10000 ms |

## 12. 后续建议

1. 若继续用手机测试，先断开 ESP32/JDY34，只让手机单独连接 JDY31，确认 JDY31 TXD 能输出。
2. 若继续用 ESP32 语音控制，优先用 `JDY31 TXD -> USB-TTL RXD` 抓包确认是否收到 `F5/B5/L5/R5/S0`。
3. 若 OLED 显示正常但电机动作不符合预期，再进入电机方向、PWM、KEY0 使能和供电排查。
4. 后续可以给 STM32 增加临时 RX 计数或蜂鸣提示，用于无 OLED 时判断 USART1 是否收到字节。

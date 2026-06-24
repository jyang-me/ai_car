# AI Car Project

本项目用于实现 ESP32 小智语音控制 STM32F103 小车。当前通信链路为：

```text
语音指令
  -> ESP32-S3 xiaozhi-esp32
  -> UART
  -> JDY34 BLE 主机
  -> BLE 透传
  -> JDY31 BLE 从机
  -> USART1
  -> STM32F103 小车
  -> OLED 显示指令和左右电机 PWM
```

仓库包含两个主要工程：

| 目录 | 说明 |
| --- | --- |
| `xiaozhi-esp32/` | ESP32-S3 小智固件，负责语音识别后的 MCP 工具调用和 JDY34 发送 |
| `esp32_ai_car/` | ESP32 端 AI 小车相关改造源码归档，可直接复制回 `xiaozhi-esp32` |
| `f103_test/` | STM32F103 小车固件，负责蓝牙串口接收、OLED 状态显示和电机控制 |

注意：`xiaozhi-esp32/` 本地目录是独立的上游 Git 仓库，父仓库不会直接提交整个上游工程。当前 AI 小车相关 ESP32 改动已归档到 `esp32_ai_car/`，并额外导出补丁 `docs/esp32-xiaozhi-ai-car.patch`。

系统架构图：

| 文件 | 说明 |
| --- | --- |
| `docs/functional_architecture_ai_car.drawio` | 推荐主图：按小车功能划分为循迹和蓝牙遥控两条主线 |
| `docs/functional_architecture_ai_car_preview.html` | 功能架构图浏览器快速预览页 |
| `docs/system_architecture_ai_car.drawio` | 可编辑 draw.io 系统架构图 |
| `docs/system_architecture_ai_car_preview.html` | 浏览器快速预览页 |
| `docs/module_flow_diagrams_ai_car.drawio` | 分模块流程图，包含 ESP32、蓝牙透传、STM32、调试验证 4 页 |
| `docs/module_flow_diagrams_ai_car_preview.html` | 分模块流程图浏览器预览页 |
| `docs/module_flows/` | 分模块流程图的 4 个独立单页 draw.io 文件，兼容只显示第一页的打开方式 |

## 当前验证状态

- ESP32 端可以通过 MCP/语音工具生成小车指令。
- ESP32 日志可看到类似 `MCP/voice car command: forward speed=5 level=5 -> F5`。
- JDY34 发送日志可看到类似 `Jdy34Ble: TX: F5`。
- JDY31 接 USB-TTL 时，串口输出准确。
- 电脑 USB-TTL 直接发到 STM32 `PA10` 时，OLED 可以显示指令。
- 因此 STM32 的 `USART1/PA10/OLED` 接收显示链路已验证正常。
- 如果手机蓝牙发送 OLED 无变化，优先检查手机是否真正连接 JDY31 的 SPP 串口服务，以及 JDY31 是否被 JDY34 占用。

## 硬件连接

### ESP32 与 JDY34

当前板级配置位于：

`xiaozhi-esp32/main/boards/bread-compact-wifi-s3cam/config.h`

| JDY34 | ESP32-S3 | 说明 |
| --- | --- | --- |
| VCC | 3.3V | 不建议接 5V |
| GND | GND | 必须共地 |
| RXD | GPIO14 / UART1 TX | ESP32 发给 JDY34 |
| TXD | GPIO3 / UART1 RX | JDY34 回 ESP32 |

当前配置：

```c
#define JDY34_UART_NUM UART_NUM_1
#define JDY34_UART_TX_PIN GPIO_NUM_14
#define JDY34_UART_RX_PIN GPIO_NUM_3
#define JDY34_STATE_PIN GPIO_NUM_NC
#define JDY34_PEER_MAC "220C180B3AB0"
```

注意：GPIO14 同时在该板配置中定义为 `LAMP_GPIO`，若后续灯光和 JDY34 同时异常，需要重新分配其中一个引脚。

### JDY31 与 STM32

STM32 端使用 `USART1`，参数为 `9600 8N1`。

| JDY31 | STM32F103 | 说明 |
| --- | --- | --- |
| VCC | 3.3V | 按模块要求供电 |
| GND | GND | 必须共地 |
| TXD | PA10 / USART1 RX | JDY31 输出给 STM32，这是最关键的一根线 |
| RXD | PA9 / USART1 TX | 可接，用于双向调试 |

如果 JDY31 接 USB-TTL 有输出，但 STM32 OLED 不变，优先检查 `JDY31 TXD -> PA10` 和共地。

## 指令协议

STM32 支持文本 ASCII 指令。推荐语音链路发送两字节格式：

| 指令 | 动作 |
| --- | --- |
| `F5` | 前进，速度档位 5 |
| `B5` | 后退，速度档位 5 |
| `L5` | 左转，速度档位 5 |
| `R5` | 右转，速度档位 5 |
| `S0` | 停止 |

也支持单字母测试：

| 指令 | 动作 |
| --- | --- |
| `F` / `f` / `W` / `w` / `1` | 前进 |
| `B` / `b` / `X` / `x` / `2` | 后退 |
| `L` / `l` / `A` / `a` / `3` | 左转 |
| `R` / `r` / `D` / `d` / `4` | 右转 |
| `S` / `s` / `0` | 停止 |
| `+` / `6` | 速度档位加 1 |
| `-` / `7` | 速度档位减 1 |

手机串口助手测试时要使用文本模式。若使用 HEX 模式，文本 `F5` 应发送：

```text
46 35
```

文本 `S0` 应发送：

```text
53 30
```

不要发送中文“前进”“停止”，STM32 当前只解析上述 ASCII 指令。

## ESP32 固件

### 关键代码

| 文件 | 作用 |
| --- | --- |
| `xiaozhi-esp32/main/boards/common/car_controller.cpp` | 注册 MCP 小车工具，生成 `F5/B5/L5/R5/S0` |
| `xiaozhi-esp32/main/boards/common/car_controller.h` | 小车控制接口 |
| `xiaozhi-esp32/main/boards/common/jdy34_ble.cpp` | JDY34 UART/AT 指令/BLE 绑定/透传 |
| `xiaozhi-esp32/main/boards/common/jdy34_ble.h` | JDY34 驱动接口 |
| `xiaozhi-esp32/main/boards/bread-compact-wifi-s3cam/config.h` | JDY34 UART 引脚和 JDY31 MAC 配置 |

父仓库同步 GitHub 时，ESP32 相关源码归档在：

```text
esp32_ai_car/
```

同一批改动的补丁备份在：

```text
docs/esp32-xiaozhi-ai-car.patch
```

### MCP 工具

ESP32 当前注册的小车工具包括：

| MCP 工具 | 发送到 STM32 的指令 |
| --- | --- |
| `self.car.forward` | `F<level>` |
| `self.car.backward` | `B<level>` |
| `self.car.left` | `L<level>` |
| `self.car.right` | `R<level>` |
| `self.car.stop` | `S0` |
| `self.car.get_status` | 查询 ESP32/JDY34 状态 |

速度参数支持 `0-9` 档，也支持 `0-100` 百分比，代码会归一化为 `0-9`。

### 安全停止

ESP32 端 `CAR_COMMAND_TIMEOUT_MS` 当前为 `10000` ms。移动状态下超过该时间未收到新指令，ESP32 会自动发送 `S0`。

### 编译和烧录

进入 ESP32 工程：

```powershell
cd E:\Prj\ai_car\xiaozhi-esp32
```

常用命令：

```powershell
idf.py build
idf.py -p COM13 flash monitor
```

如果串口不是 `COM13`，替换为设备管理器中 ESP32 的实际端口。

### 日志判断

ESP32 端看到以下日志，说明语音/MCP 到 JDY34 发送是正常的：

```text
CarController: MCP/voice car command: forward speed=5 level=5 -> F5
Jdy34Ble: TX: F5
```

若随后看到：

```text
Jdy34Ble: TX: S0
```

通常是安全超时停车，属于正常保护逻辑。

## STM32 固件

### 关键代码

| 文件 | 作用 |
| --- | --- |
| `f103_test/HARDWARE/BT_REMOTE/bt_remote.c` | 蓝牙指令解析、OLED 显示、遥控运动逻辑 |
| `f103_test/HARDWARE/BT_REMOTE/bt_remote.h` | 蓝牙遥控状态结构体和参数 |
| `f103_test/Core/Src/usart.c` | USART1/USART3 初始化 |
| `f103_test/Core/Src/stm32f1xx_it.c` | USART1 中断回调入口 |
| `f103_test/HARDWARE/MENU/menu.c` | OLED 菜单模式 |

### USART1 配置

STM32 使用：

```text
USART1
PA9  = TX
PA10 = RX
9600 8N1
```

接收流程：

```text
USART1_IRQHandler
  -> HAL_UART_IRQHandler(&huart1)
  -> HAL_UART_RxCpltCallback
  -> bt_remote_on_rx_byte(g_bt_receivedata)
  -> bt_remote_parse_data(...)
  -> bt_remote_start_rx()
```

### OLED 显示

蓝牙遥控页面已简化为：

```text
Phone BT 或 Voice BT
Cmd:<指令><档位> Raw:<原始字符>
Left :<左电机 PWM>
Right:<右电机 PWM>
```

如果 `Raw` 和 `Cmd` 有变化，说明 STM32 收到了串口数据。
如果电脑 USB-TTL 发 `F` 或 `F5` 能显示，而 JDY31 接上不显示，问题在 JDY31 到 STM32 的接线或 JDY31 是否真正输出。

### 编译

Keil 工程：

```text
f103_test/MDK-ARM/f103c8t6_test.uvprojx
```

命令行编译示例：

```powershell
& 'D:\Keil5\KEIL_MDK\UV4\UV4.exe' -b 'E:\Prj\ai_car\f103_test\MDK-ARM\f103c8t6_test.uvprojx' -j0 -o 'E:\Prj\ai_car\f103_test\MDK-ARM\build_from_codex.log'
```

当前已验证编译结果：

```text
0 Error(s), 0 Warning(s)
```

生成的 HEX：

```text
f103_test/MDK-ARM/f103c8t6_test/f103c8t6_test.hex
```

### 烧录

推荐直接用 Keil：

1. 打开 `f103_test/MDK-ARM/f103c8t6_test.uvprojx`
2. 连接 ST-Link
3. 点击 Build
4. 点击 Download

如果命令行下载失败并出现：

```text
Flash Download failed - Target DLL has been cancelled
```

说明本机 ST-Link 下载链路没有成功建立，优先检查 ST-Link 接线、驱动、目标板供电和 Keil Debug/Flash 配置。

## 手机蓝牙测试注意事项

手机测试 JDY31 时，建议先断开 ESP32/JDY34，只让手机单独连接 JDY31。

关键点：

- 手机系统蓝牙“配对成功”不等于 SPP 串口已经连通。
- 使用支持经典蓝牙 SPP 的串口助手。
- 连接目标应为 JDY31，而不是 JDY34。
- 发送模式应为文本模式。
- 先发送单字母 `F` 测试，再发送 `F5`。
- 如果手机发 `F` 没变化，把 JDY31 TXD 接 USB-TTL RXD，看电脑串口是否能收到 `F`。

推荐验证顺序：

```text
手机 -> JDY31 -> USB-TTL -> 电脑串口助手
```

能看到 `F` 或 `F5` 后，再接回：

```text
手机 -> JDY31 -> STM32 PA10 -> OLED
```

## 分段排查流程

### 1. 验证 STM32

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

预期：OLED 的 `Cmd` 和 `Raw` 变化。

### 2. 验证 JDY31

接线：

```text
JDY31 TXD -> USB-TTL RXD
JDY31 GND -> USB-TTL GND
```

手机连接 JDY31 后发送 `F` 或 `F5`。

预期：电脑串口助手能看到 `F` 或 `F5`。

### 3. 验证 JDY34 到 JDY31

接线：

```text
ESP32 -> JDY34
JDY31 TXD -> USB-TTL RXD
共地
```

ESP32 语音触发前进。

预期：

```text
ESP32 日志：Jdy34Ble: TX: F5
电脑串口助手：收到 F5
```

### 4. 验证整链路

接线：

```text
ESP32 -> JDY34 -> BLE -> JDY31 TXD -> STM32 PA10
JDY31 GND -> STM32 GND
```

预期：OLED 显示 `Cmd:F5`，左右电机 PWM 根据当前模式和启动状态变化。

## 常见问题

### 电脑发 STM32 正常，手机蓝牙发 OLED 不变

STM32 没问题，优先查手机到 JDY31：

- 手机是否连的是 JDY31；
- 是否使用 SPP 串口助手；
- 是否文本发送；
- JDY31 是否已经被 JDY34 连接占用；
- JDY31 TXD 接 USB-TTL 时，电脑是否能看到手机发出的数据。

### JDY31 有输出，但 STM32 OLED 不变

优先查：

- `JDY31 TXD -> STM32 PA10` 是否接反；
- JDY31 和 STM32 是否共地；
- STM32 是否烧录了最新固件；
- STM32 是否进入 `Phone BT` 或 `Voice BT` 页面。

### ESP32 日志一直发 `S0`

`S0` 是停止指令。常见原因：

- 没有新的运动指令；
- 安全超时触发；
- LLM/语音识别没有调用前进/后退/转向工具；
- 手机或串口助手只发送了停止命令。

### OLED 有 `Cmd`，但电机不转

优先查：

- 是否按 `KEY0` 使能运行；
- 当前是否在 `Phone BT` 或 `Voice BT`；
- 电机驱动供电是否正常；
- PWM 和方向线是否接对；
- 安全超时是否很快停车。

## 当前关键参数

| 参数 | 当前值 |
| --- | --- |
| STM32 蓝牙串口 | USART1 |
| STM32 蓝牙 RX | PA10 |
| STM32 蓝牙波特率 | 9600 |
| ESP32 JDY34 UART | UART1 |
| ESP32 JDY34 TX | GPIO14 |
| ESP32 JDY34 RX | GPIO3 |
| JDY34 绑定 MAC | `220C180B3AB0` |
| ESP32 安全停车超时 | 10000 ms |
| STM32 遥控失联停车超时 | 2000 ms |
| STM32 速度档位 | 1-5，默认 3 |

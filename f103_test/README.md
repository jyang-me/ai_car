# STM32F103 智能小车控制工程

这是一个基于 `STM32F103C8T6` 的智能小车固件工程，使用 STM32 HAL 库和 Keil MDK-ARM 开发。当前版本集成了 OLED 菜单、双路直流电机 PWM 控制、8 路灰度/红外巡线、JY61P 姿态角解析、蓝牙/语音串口遥控、蜂鸣器和状态灯反馈。

工程主要面向巡线圆环、指定角点计数、手机蓝牙遥控和语音蓝牙遥控等任务测试。

## 功能概览

- OLED 开机菜单选择运行模式
- 双路直流电机方向控制和 TIM3 PWM 调速
- 8 路巡线传感器读取、黑线识别和加权偏差计算
- JY61P 串口姿态角解析，使用 yaw 角做方向保持和转向闭环
- 顺时针/逆时针巡线圆环任务，支持 4 个角点完成判定
- 指定角点巡线任务，支持按键设置目标计数
- 手机蓝牙/语音蓝牙遥控，支持前进、后退、左右 90 度转向、停止和速度档位
- 按键启停、返回菜单、安全停车
- 蜂鸣器和任务状态灯反馈

## 运行模式

开机后通过 OLED 二级菜单选择模式。一级菜单包含 `Line Mode` 和 `BT Mode`，分别对应循迹模式和蓝牙控制模式；进入二级菜单后再选择具体任务。

| 菜单显示 | 枚举值 | 说明 |
| --- | --- | --- |
| `Line CW` | `MENU_MODE_LINE_CW` | 顺时针巡线圆环，默认计 4 个角点后完成 |
| `Line CCW` | `MENU_MODE_LINE_CCW` | 逆时针巡线圆环，默认计 4 个角点后完成 |
| `Line Target` | `MENU_MODE_LINE_TARGET_CORNERS` | 指定角点数量巡线，未启动前可用手动计数键增加目标数 |
| `Phone BT` | `MENU_MODE_BT_REMOTE` | 手机蓝牙串口遥控 |
| `Voice BT` | `MENU_MODE_VOICE_REMOTE` | 语音蓝牙串口遥控，协议与手机蓝牙共用 |

## 按键操作

### 菜单界面

| 按键 | 动作 |
| --- | --- |
| `KEY0` | 在当前菜单层切换下一个选项 |
| `KEY1` | 一级菜单中进入二级菜单；二级菜单中锁定具体模式并进入状态界面 |

### 运行界面

| 按键 | 动作 |
| --- | --- |
| `KEY0` | 启动/停止当前模式；普通直行调试状态下可切换直行和角度保持 |
| `KEY1` | 安全停车，返回 OLED 模式菜单 |
| `MANUAL_COUNT_KEY` | `Line Target` 未启动时增加目标角点数；运行中触发一次蜂鸣反馈 |

遥控模式下，`KEY0` 用于使能或关闭电机输出。关闭后会清零遥控状态并停止电机。

## 蓝牙/语音遥控协议

USART1 默认配置为 `9600 8N1`，接线为 `PA9/TX`、`PA10/RX`。支持单字节 ASCII 指令和部分数值指令。

| 指令 | 动作 |
| --- | --- |
| `F` / `f` / `W` / `w` / `1` / `0x01` | 前进，带 yaw 方向锁定修正 |
| `B` / `b` / `X` / `x` / `2` / `0x05` | 后退 |
| `L` / `l` / `A` / `a` / `3` / `0x03` | 左转 90 度 |
| `R` / `r` / `D` / `d` / `4` / `0x07` | 右转 90 度 |
| `S` / `s` / `0` / `0x00` | 停止 |
| `+` / `6` / `0x09` | 速度档位加 1 |
| `-` / `7` / `0x0A` | 速度档位减 1 |
| `M` / `m` / `8` / `0x10` | 切换到遥控模式 |
| `O` / `o` / `9` / `0x11` | 切换到避障模式占位，当前不输出运动 |
| `T` / `t` / `0x12` | 切换到循迹模式占位，当前不输出运动 |

速度档位范围为 `1` 到 `5`，默认 `3`。遥控指令超过 `BT_REMOTE_MOVE_MS` 未刷新时会自动停车，当前值为 `2000 ms`。

## 硬件连接

主要引脚定义来自 [Core/Inc/main.h](Core/Inc/main.h)。

| 模块 | 引脚 | 说明 |
| --- | --- | --- |
| LED0 | `PC13` | 板载/运行心跳灯，低电平点亮 |
| TASK_LED | `PB1` | 任务状态灯，低电平点亮 |
| BUZZER | `PB0` | 蜂鸣器，低电平触发 |
| KEY0 | `PB5` | 按键 0，上拉输入 |
| KEY1 | `PC14` | 按键 1，上拉输入 |
| MANUAL_COUNT_KEY | `PC15` | 手动计数按键，上拉输入 |
| 左电机方向 | `PA4` / `PA5` | `AIN1` / `AIN2` |
| 右电机方向 | `PA3` / `PA0` | `BIN1` / `BIN2` |
| 左电机 PWM | `PA6` | TIM3_CH1 |
| 右电机 PWM | `PA7` | TIM3_CH2 |
| 巡线 L1-L4 | `PB15` / `PB14` / `PB13` / `PB12` | 左侧 4 路传感器 |
| 巡线 R1-R4 | `PB6` / `PA15` / `PB3` / `PB4` | 右侧 4 路传感器 |
| OLED SCL/SDA | `PB8` / `PB9` | 软件 I2C OLED |
| JY61P UART | `PB10` / `PB11` | USART3 TX/RX，默认 `9600` |
| 蓝牙 UART | `PA9` / `PA10` | USART1 TX/RX，默认 `9600` |
| 编码器 A | `PA2` / `PA1` | A1/A2 EXTI 输入 |
| 编码器 B | `PA11` / `PA8` | B1/B2 EXTI 输入 |

巡线模块读取逻辑：GPIO 为 `RESET` 时认为检测到黑线。`line_read_state()` 的 bit 顺序为 `L4, L3, L2, L1, R1, R2, R3, R4`。

## 工程结构

```text
.
├── Core/                 # STM32CubeMX 生成的主程序、外设初始化和中断入口
│   ├── Inc/
│   └── Src/
├── Drivers/              # CMSIS 与 STM32F1 HAL 驱动
├── HARDWARE/             # 项目自定义硬件驱动和控制模块
│   ├── BT_REMOTE/        # 蓝牙/语音遥控协议和状态显示
│   ├── CONTROL/          # 车体参数、速度/位置控制接口
│   ├── ENCODER/          # 编码器
│   ├── JY61P/            # JY61P 姿态模块串口解析
│   ├── KEY/              # 按键扫描
│   ├── LINE/             # 巡线传感器读取和偏差计算
│   ├── LINE_TASK/        # 巡线任务状态机
│   ├── MENU/             # OLED 菜单与状态显示
│   ├── MOTOR/            # 电机方向和 PWM 装载
│   ├── MPU6050/          # MPU6050/DMP 相关代码，当前主流程未使用
│   ├── OLED/             # OLED 驱动
│   ├── PID/              # PID 控制
│   └── PROTOCOL/         # 通信协议预留模块
├── MDK-ARM/              # Keil 工程文件和构建输出
└── f103c8t6_test.ioc     # STM32CubeMX 配置文件
```

## 构建与下载

### Keil MDK-ARM

1. 打开 `MDK-ARM/f103c8t6_test.uvprojx`
2. 确认目标芯片为 `STM32F103C8T6` 或兼容型号
3. 编译工程
4. 使用 ST-Link/J-Link 下载到开发板

当前仓库以 Keil 工程为主要构建入口，未提供 Makefile/CMake 构建脚本。

### STM32CubeMX

如需修改外设配置，可打开 `f103c8t6_test.ioc` 后重新生成代码。重新生成后注意保留 `USER CODE BEGIN/END` 区域内的用户代码。

## 上电运行流程

1. 上电初始化 HAL、GPIO、TIM3 PWM、USART1、USART3、PID、蓝牙接收和 OLED
2. OLED 显示 `Main Menu` 一级菜单
3. 使用 `KEY0` 选择一级菜单，`KEY1` 进入二级菜单
4. 在二级菜单中使用 `KEY0` 选择具体模式，`KEY1` 锁定模式
5. 进入状态界面后按 `KEY0` 启动
6. OLED 实时显示运行状态、角点计数、yaw、巡线偏差、传感器状态和左右 PWM
7. 任务完成或超时后电机停止，蜂鸣器提示

## 主要控制参数

| 参数 | 位置 | 作用 |
| --- | --- | --- |
| `BASE_DRIVE_PWM` | `Core/Src/main.c` | 普通直行基础 PWM |
| `MAX_TURN_CORRECTION` | `Core/Src/main.c` | yaw 修正最大值 |
| `SOFTSTART_STEP` | `Core/Src/main.c` | 直行软启动步进 |
| `LINE_BASE_PWM` | `HARDWARE/LINE_TASK/line_task.c` | 巡线基础 PWM |
| `LINE_KP` | `HARDWARE/LINE_TASK/line_task.c` | 巡线偏差比例系数 |
| `LINE_MAX_CORR` | `HARDWARE/LINE_TASK/line_task.c` | 巡线修正限幅 |
| `LINE_CIRCLE_TIMEOUT_MS` | `HARDWARE/LINE_TASK/line_task.c` | 圆环任务超时 |
| `BT_REMOTE_BASE_PWM` | `HARDWARE/BT_REMOTE/bt_remote.c` | 蓝牙遥控基础速度 |
| `BT_REMOTE_SPEED_STEP` | `HARDWARE/BT_REMOTE/bt_remote.c` | 蓝牙速度档位步进 |
| `BT_REMOTE_MOVE_MS` | `HARDWARE/BT_REMOTE/bt_remote.h` | 遥控指令失联停车时间 |

调试速度、转向、计数、超时和方向稳定性时，优先检查以上参数。

## 调试提示

- OLED 的 `Line` 字段显示巡线偏差，`S` 字段显示 8 位传感器状态，可用于检查传感器接线和黑线识别方向。
- OLED 的 `Yaw` 字段用于检查 JY61P 串口数据是否正常刷新。
- 如果小车电机方向相反，优先检查 `HARDWARE/MOTOR/motor.c` 中的方向控制逻辑和电机接线。
- 如果 JY61P 无数据，检查 USART3 的 `PB10/PB11` 接线、波特率和模块输出帧类型。
- 如果蓝牙无响应，检查 USART1 的 `PA9/PA10` 接线、波特率，以及 OLED 上的 `RX` 和 `Raw` 字段是否变化。
- 上电调试建议先架空车轮，确认方向、PWM、按键和急停逻辑正常后再落地运行。

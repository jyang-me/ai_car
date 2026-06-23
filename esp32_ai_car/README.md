# ESP32 AI Car Integration Files

这里归档的是 `xiaozhi-esp32` 中和 AI 小车控制直接相关的改造文件。`xiaozhi-esp32/` 本地目录本身是独立 Git 仓库，因此父仓库 `ai_car` 不直接提交整个 ESP32 上游工程；本目录用于让 GitHub 上可以直接看到和恢复本项目的 ESP32 端代码。

## 文件对应关系

把本目录中的文件复制到一份 `xiaozhi-esp32` 工程的相同相对路径即可。

| 本目录文件 | 目标路径 |
| --- | --- |
| `main/boards/common/car_controller.cpp` | `xiaozhi-esp32/main/boards/common/car_controller.cpp` |
| `main/boards/common/car_controller.h` | `xiaozhi-esp32/main/boards/common/car_controller.h` |
| `main/boards/common/jdy34_ble.cpp` | `xiaozhi-esp32/main/boards/common/jdy34_ble.cpp` |
| `main/boards/common/jdy34_ble.h` | `xiaozhi-esp32/main/boards/common/jdy34_ble.h` |
| `main/boards/bread-compact-wifi-s3cam/config.h` | `xiaozhi-esp32/main/boards/bread-compact-wifi-s3cam/config.h` |
| `main/boards/bread-compact-wifi-s3cam/compact_wifi_board_s3cam.cc` | `xiaozhi-esp32/main/boards/bread-compact-wifi-s3cam/compact_wifi_board_s3cam.cc` |
| `main/CMakeLists.txt` | `xiaozhi-esp32/main/CMakeLists.txt` |
| `sdkconfig.defaults.ai_car` | `xiaozhi-esp32/sdkconfig.defaults.ai_car` |

## 主要功能

- 注册 MCP 工具：`self.car.forward`、`self.car.backward`、`self.car.left`、`self.car.right`、`self.car.stop`、`self.car.get_status`。
- 将语音/MCP 指令转换为 STM32 可解析的 ASCII 控制指令：`F5`、`B5`、`L5`、`R5`、`S0`。
- 通过 ESP32 UART1 控制 JDY34。
- 使用 `AT+BAND<JDY31_MAC>` 绑定 JDY31。
- 通过 JDY34/JDY31 BLE 透传链路把指令发到 STM32。
- 10 秒无新运动指令时自动发送 `S0` 安全停车。

## 当前硬件参数

```c
#define JDY34_UART_NUM UART_NUM_1
#define JDY34_UART_TX_PIN GPIO_NUM_14
#define JDY34_UART_RX_PIN GPIO_NUM_3
#define JDY34_STATE_PIN GPIO_NUM_NC
#define JDY34_PEER_MAC "220C180B3AB0"
```

注意：`GPIO14` 在原板级配置中也被定义为 `LAMP_GPIO`。如果灯光和 JDY34 串口同时使用，需要重新分配其中一个引脚。

## 构建示例

在 `xiaozhi-esp32` 工程中：

```powershell
idf.py build
idf.py -p COM13 flash monitor
```

如果串口不是 `COM13`，替换为设备管理器中的实际 ESP32 端口。

## 补丁文件

父仓库还保留了同一批改动的补丁：

```text
docs/esp32-xiaozhi-ai-car.patch
```

如果更喜欢补丁方式，可在 `xiaozhi-esp32` 工程根目录执行：

```powershell
git apply E:\Prj\ai_car\docs\esp32-xiaozhi-ai-car.patch
```

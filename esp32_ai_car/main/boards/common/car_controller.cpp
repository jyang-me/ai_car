#include "car_controller.h"

#include <esp_log.h>

#define TAG "CarController"
#define CAR_COMMAND_TIMEOUT_MS 10000

const char* MotionName(char motion) {
    switch (motion) {
        case 'F':
            return "forward";
        case 'B':
            return "backward";
        case 'L':
            return "left";
        case 'R':
            return "right";
        case 'S':
            return "stop";
        default:
            return "unknown";
    }
}

CarController& CarController::GetInstance() {
    static CarController instance;
    return instance;
}

bool CarController::Initialize(uart_port_t uart_num,
                               gpio_num_t tx_pin,
                               gpio_num_t rx_pin,
                               gpio_num_t state_pin,
                               const std::string& peer_mac) {
    if (initialized_) {
        return true;
    }

    ble_ = std::make_unique<Jdy34Ble>(uart_num, tx_pin, rx_pin, state_pin);
    if (!ble_->Initialize()) {
        return false;
    }

    if (!peer_mac.empty()) {
        ble_->Connect(peer_mac);
    }

    last_command_tick_ = xTaskGetTickCount();
    xTaskCreate(&CarController::SafetyTask, "car_safety", 3072, this, 4, nullptr);
    initialized_ = true;
    ESP_LOGI(TAG, "Car controller initialized");
    return true;
}

void CarController::RegisterMcpTools() {
    auto& mcp_server = McpServer::GetInstance();

    mcp_server.AddTool("self.car.forward", "Drive the car forward. Speed can be 0-9 or 0-100 percent.",
        PropertyList({Property("speed", kPropertyTypeInteger, 5, 0, 100)}),
        [this](const PropertyList& properties) -> ReturnValue {
            return Forward(properties["speed"].value<int>());
        });

    mcp_server.AddTool("self.car.backward", "Drive the car backward. Speed can be 0-9 or 0-100 percent.",
        PropertyList({Property("speed", kPropertyTypeInteger, 5, 0, 100)}),
        [this](const PropertyList& properties) -> ReturnValue {
            return Backward(properties["speed"].value<int>());
        });

    mcp_server.AddTool("self.car.left", "Turn the car left in place. Speed can be 0-9 or 0-100 percent.",
        PropertyList({Property("speed", kPropertyTypeInteger, 5, 0, 100)}),
        [this](const PropertyList& properties) -> ReturnValue {
            return Left(properties["speed"].value<int>());
        });

    mcp_server.AddTool("self.car.right", "Turn the car right in place. Speed can be 0-9 or 0-100 percent.",
        PropertyList({Property("speed", kPropertyTypeInteger, 5, 0, 100)}),
        [this](const PropertyList& properties) -> ReturnValue {
            return Right(properties["speed"].value<int>());
        });

    mcp_server.AddTool("self.car.stop", "Stop the car immediately.",
        PropertyList(),
        [this](const PropertyList& properties) -> ReturnValue {
            return Stop();
        });

    mcp_server.AddTool("self.car.get_status", "Get car controller and JDY34 BLE status.",
        PropertyList(),
        [this](const PropertyList& properties) -> ReturnValue {
            return GetStatusJson();
        });
}

bool CarController::Forward(int speed) {
    return SendMotion('F', speed);
}

bool CarController::Backward(int speed) {
    return SendMotion('B', speed);
}

bool CarController::Left(int speed) {
    return SendMotion('L', speed);
}

bool CarController::Right(int speed) {
    return SendMotion('R', speed);
}

bool CarController::Stop() {
    last_motion_ = 'S';
    moving_ = false;
    last_command_tick_ = xTaskGetTickCount();
    ESP_LOGI(TAG, "MCP/voice car command: stop -> S0");
    bool sent = ble_ && ble_->SendData("S0");
    if (!sent) {
        ESP_LOGW(TAG, "Failed to send stop command");
    }
    return sent;
}

std::string CarController::GetStatusJson() const {
    std::string ble_status = ble_ ? ble_->GetStatusJson() : "{\"initialized\":false,\"connected\":false}";
    return std::string("{\"initialized\":") + (initialized_ ? "true" : "false") +
        ",\"moving\":" + (moving_ ? "true" : "false") +
        ",\"motion\":\"" + std::string(1, last_motion_) + "\"" +
        ",\"speed\":" + std::to_string(last_speed_) +
        ",\"ble\":" + ble_status + "}";
}

bool CarController::SendMotion(char motion, int speed) {
    int level = NormalizeSpeedLevel(speed);
    last_motion_ = motion;
    last_speed_ = level;
    moving_ = motion != 'S' && level > 0;
    last_command_tick_ = xTaskGetTickCount();

    std::string command;
    command.push_back(motion);
    command.push_back(static_cast<char>('0' + level));
    ESP_LOGI(TAG, "MCP/voice car command: %s speed=%d level=%d -> %s",
        MotionName(motion), speed, level, command.c_str());
    bool sent = ble_ && ble_->SendData(command);
    if (!sent) {
        ESP_LOGW(TAG, "Failed to send car command: %s", command.c_str());
    }
    return sent;
}

int CarController::NormalizeSpeedLevel(int speed) const {
    if (speed < 0) {
        speed = 0;
    } else if (speed > 100) {
        speed = 100;
    }

    if (speed <= 9) {
        return speed;
    }

    int level = (speed + 5) / 10;
    if (level > 9) {
        level = 9;
    }
    return level;
}

void CarController::SafetyLoop() {
    while (true) {
        if (moving_) {
            TickType_t elapsed = xTaskGetTickCount() - last_command_tick_;
            if (elapsed >= pdMS_TO_TICKS(CAR_COMMAND_TIMEOUT_MS)) {
                ESP_LOGW(TAG, "Command timeout, stop car");
                Stop();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void CarController::SafetyTask(void* arg) {
    static_cast<CarController*>(arg)->SafetyLoop();
}

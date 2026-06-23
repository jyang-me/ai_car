#ifndef CAR_CONTROLLER_H
#define CAR_CONTROLLER_H

#include "jdy34_ble.h"
#include "mcp_server.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <memory>
#include <string>

class CarController {
public:
    static CarController& GetInstance();

    bool Initialize(uart_port_t uart_num,
                    gpio_num_t tx_pin,
                    gpio_num_t rx_pin,
                    gpio_num_t state_pin,
                    const std::string& peer_mac);
    void RegisterMcpTools();

    bool Forward(int speed);
    bool Backward(int speed);
    bool Left(int speed);
    bool Right(int speed);
    bool Stop();
    std::string GetStatusJson() const;

private:
    CarController() = default;

    bool SendMotion(char motion, int speed);
    int NormalizeSpeedLevel(int speed) const;
    void SafetyLoop();
    static void SafetyTask(void* arg);

    std::unique_ptr<Jdy34Ble> ble_;
    bool initialized_ = false;
    bool moving_ = false;
    int last_speed_ = 5;
    char last_motion_ = 'S';
    TickType_t last_command_tick_ = 0;
};

#endif

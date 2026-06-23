#ifndef JDY34_BLE_H
#define JDY34_BLE_H

#include <driver/gpio.h>
#include <driver/uart.h>

#include <string>

class Jdy34Ble {
public:
    Jdy34Ble(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t state_pin = GPIO_NUM_NC);

    bool Initialize();
    bool SendData(const std::string& data);
    bool SendCommand(const std::string& command, uint32_t wait_ms = 120, std::string* response = nullptr);
    bool Connect(const std::string& mac);
    bool IsConnected() const;
    std::string GetStatusJson() const;

private:
    bool ConfigureUart(uint32_t baud_rate);
    std::string ReadResponse(uint32_t wait_ms);
    bool DetectBaudRate(uint32_t* baud_rate);

    uart_port_t uart_num_;
    gpio_num_t tx_pin_;
    gpio_num_t rx_pin_;
    gpio_num_t state_pin_;
    uint32_t baud_rate_ = 9600;
    bool initialized_ = false;
    bool connected_ = false;
};

#endif

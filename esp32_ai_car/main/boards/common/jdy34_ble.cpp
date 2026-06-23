#include "jdy34_ble.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <algorithm>
#include <cctype>
#include <cstring>

#define TAG "Jdy34Ble"

Jdy34Ble::Jdy34Ble(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t state_pin)
    : uart_num_(uart_num), tx_pin_(tx_pin), rx_pin_(rx_pin), state_pin_(state_pin) {
}

bool Jdy34Ble::Initialize() {
    if (initialized_) {
        return true;
    }
    if (tx_pin_ == GPIO_NUM_NC || rx_pin_ == GPIO_NUM_NC) {
        ESP_LOGE(TAG, "UART pins are not configured");
        return false;
    }

    ESP_ERROR_CHECK(uart_driver_install(uart_num_, 1024, 1024, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_set_pin(uart_num_, tx_pin_, rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    if (state_pin_ != GPIO_NUM_NC) {
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = 1ULL << state_pin_;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        gpio_config(&io_conf);
    }

    uint32_t detected_baud = 0;
    if (!DetectBaudRate(&detected_baud)) {
        ESP_LOGE(TAG, "JDY34 did not respond to AT. Check VCC/GND/TX/RX wiring and baud rate.");
        initialized_ = true;
        return true;
    }

    initialized_ = true;
    if (detected_baud != 9600) {
        ESP_LOGW(TAG, "JDY34 detected at %lu baud, switching module to 9600", static_cast<unsigned long>(detected_baud));
        SendCommand("AT+BAUD=4", 500);
        vTaskDelay(pdMS_TO_TICKS(300));
        ConfigureUart(9600);
        SendCommand("AT", 200);
    } else {
        SendCommand("AT+BAUD=4", 300);
    }

    SendCommand("AT+ROLE=1");
    SendCommand("AT+NAME=ESP32-CAR");
    return true;
}

bool Jdy34Ble::ConfigureUart(uint32_t baud_rate) {
    uart_config_t uart_config = {};
    uart_config.baud_rate = baud_rate;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    esp_err_t err = uart_param_config(uart_num_, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART%u at %lu baud: %s",
            uart_num_, static_cast<unsigned long>(baud_rate), esp_err_to_name(err));
        return false;
    }
    baud_rate_ = baud_rate;
    uart_flush(uart_num_);
    return true;
}

std::string Jdy34Ble::ReadResponse(uint32_t wait_ms) {
    std::string response;
    int64_t deadline = esp_timer_get_time() + static_cast<int64_t>(wait_ms) * 1000;
    uint8_t buffer[128];

    while (esp_timer_get_time() < deadline) {
        int len = uart_read_bytes(uart_num_, buffer, sizeof(buffer), pdMS_TO_TICKS(20));
        if (len > 0) {
            response.append(reinterpret_cast<const char*>(buffer), len);
        }
    }

    response.erase(std::remove_if(response.begin(), response.end(),
        [](unsigned char ch) { return ch != '\r' && ch != '\n' && !std::isprint(ch); }),
        response.end());
    return response;
}

bool Jdy34Ble::DetectBaudRate(uint32_t* baud_rate) {
    const uint32_t baud_rates[] = {9600, 115200, 38400, 19200, 57600};

    for (auto candidate : baud_rates) {
        if (!ConfigureUart(candidate)) {
            continue;
        }

        std::string response;
        std::string command = "AT\r\n";
        uart_write_bytes(uart_num_, command.data(), command.size());
        uart_wait_tx_done(uart_num_, pdMS_TO_TICKS(80));
        response = ReadResponse(250);
        ESP_LOGI(TAG, "Probe baud %lu response: %s", static_cast<unsigned long>(candidate),
            response.empty() ? "<empty>" : response.c_str());

        if (response.find("OK") != std::string::npos || response.find("AT") != std::string::npos) {
            *baud_rate = candidate;
            return true;
        }
    }

    return false;
}

bool Jdy34Ble::SendCommand(const std::string& command, uint32_t wait_ms, std::string* response) {
    if (!initialized_) {
        return false;
    }

    uart_flush(uart_num_);
    std::string line = command + "\r\n";
    int written = uart_write_bytes(uart_num_, line.data(), line.size());
    uart_wait_tx_done(uart_num_, pdMS_TO_TICKS(80));
    std::string local_response = ReadResponse(wait_ms);
    if (response != nullptr) {
        *response = local_response;
    }
    ESP_LOGI(TAG, "AT command: %s, response: %s", command.c_str(),
        local_response.empty() ? "<empty>" : local_response.c_str());
    return written == static_cast<int>(line.size());
}

bool Jdy34Ble::Connect(const std::string& mac) {
    if (mac.empty()) {
        ESP_LOGW(TAG, "JDY-31 MAC is empty, skip auto connect");
        return false;
    }

    std::string normalized_mac;
    normalized_mac.reserve(mac.size());
    for (unsigned char ch : mac) {
        if (std::isxdigit(ch)) {
            normalized_mac.push_back(static_cast<char>(std::toupper(ch)));
        }
    }

    if (normalized_mac.size() != 12) {
        ESP_LOGE(TAG, "Invalid JDY slave MAC: %s", mac.c_str());
        return false;
    }

    std::string response;
    SendCommand("AT+BAND" + normalized_mac, 1200, &response);
    connected_ = response.find("OK") != std::string::npos ||
        response.find("BAND") != std::string::npos ||
        response.find("Connected") != std::string::npos;
    if (!connected_) {
        ESP_LOGW(TAG, "JDY34 bind command did not confirm connection");
    }
    return connected_;
}

bool Jdy34Ble::SendData(const std::string& data) {
    if (!initialized_) {
        return false;
    }

    int written = uart_write_bytes(uart_num_, data.data(), data.size());
    uart_wait_tx_done(uart_num_, pdMS_TO_TICKS(80));
    ESP_LOGI(TAG, "TX: %s", data.c_str());
    return written == static_cast<int>(data.size());
}

bool Jdy34Ble::IsConnected() const {
    if (state_pin_ != GPIO_NUM_NC) {
        return gpio_get_level(state_pin_) == 1;
    }
    return connected_;
}

std::string Jdy34Ble::GetStatusJson() const {
    return std::string("{\"initialized\":") + (initialized_ ? "true" : "false") +
        ",\"connected\":" + (IsConnected() ? "true" : "false") +
        ",\"baud\":" + std::to_string(baud_rate_) + "}";
}

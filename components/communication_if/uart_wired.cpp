#include <stdio.h>
#include "communication_if.h"
#include "uart_wired.h"
#include "esp_log.h"

static const int RX_BUF_SIZE = 1024;

UartWired :: UartWired(bool debugMode) {
    _debugMode = debugMode;
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(0, 1, 3, -1, -1);
}

int UartWired :: sendData(const char* data) {
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_0, data, len);
    if (_debugMode) {
        static const char *TX_TASK_TAG = "TX_TASK";
        esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
        ESP_LOGI(TX_TASK_TAG, "Wrote %d bytes", txBytes);
    }
    return txBytes;
}
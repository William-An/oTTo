#include <stdio.h>
#include "communication_if.h"
#include "uart_wired.h"
#include "esp_log.h"
#include "communication_struct.h"

UartWired :: UartWired(bool debugMode) {
    _debugMode = debugMode;
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, TX_BUF_SIZE * 2, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(0, 1, 3, -1, -1);
}

int UartWired :: sendData(const void* data, size_t size) {

    return uart_write_bytes(UART_NUM_0, data, size);
}

uint32_t UartWired :: calculateCRC() {
    uint32_t crc = 0;
    return crc;
}

int UartWired :: receiveData(void *buf, uint32_t length, TickType_t ticks_to_wait) {
    
    return uart_read_bytes(UART_NUM_0, buf, length, ticks_to_wait);
}

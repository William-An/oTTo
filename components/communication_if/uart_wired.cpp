#include <stdio.h>
#include "communication_if.h"
#include "uart_wired.h"
#include "esp_log.h"
#include "communication_struct.h"

UartWired :: UartWired(bool debugMode) {
    _debugMode = debugMode;
}

int UartWired :: sendData(const void* data, size_t size) {

    return uart_write_bytes(UART_NUM_0, data, size);
}

uint32_t UartWired :: calculateCRC() {
    uint32_t crc = 0;
    return crc;
}

int UartWired :: receiveData(void *buf, uint32_t length, TickType_t ticks_to_wait) {
    // TODO Weili: Make the uart port num customizable
    return uart_read_bytes(UART_NUM_0, buf, length, ticks_to_wait);
}

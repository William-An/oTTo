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

int UartWired :: sendData(const void* data, uint32_t size) {

    Command_Data_Packet dataPacket;
    dataPacket.header = HEADER;
    dataPacket.commandData = *(Command_Data*)data;
    // dataPacket.timestamp = esp_timer_get_time();
    dataPacket.timestamp = 0;
    dataPacket.CRC = calculateCRC();

    const int txBytes = uart_write_bytes(UART_NUM_0, (void*) &dataPacket, 32);
    if (_debugMode) {
        static const char *TX_TASK_TAG = "TX_TASK";
        esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
        ESP_LOGI(TX_TASK_TAG, "Wrote %d bytes", txBytes);
    }
    return txBytes;
}

uint32_t UartWired :: calculateCRC() {
    uint32_t crc = 0;
    return crc;
}

int UartWired :: receiveData(void* data, uint32_t size) {
    // const TickType_t xDelay = pdMS_TO_TICKS( 1000 );
    static const char *TX_TASK_TAG = "TX_TASK";
    int readBytes = 0;

    if (size == sizeof(Command_Data_Packet)) {
        readBytes = uart_read_bytes(UART_NUM_0, data, sizeof(Command_Data_Packet), portMAX_DELAY);
        // Command_Data_Packet* packet = (Command_Data_Packet*) data;
        // Command_Data packet_data = packet -> commandData;
        Command_Data packet_data = * (Command_Data*) data;
        esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
        // ESP_LOGI(TX_TASK_TAG, "The data leftAngularVelo is %f", packet_data.leftAngularVelo);
        // ESP_LOGI(TX_TASK_TAG, "The data rightAngularVelo is %f", packet_data.rightAngularVelo);
        // ESP_LOGI(TX_TASK_TAG, "The data angleRotatedLeftMotor is %f", packet_data.angleRotatedLeftMotor);
        // ESP_LOGI(TX_TASK_TAG, "The data angleRotatedRightMotor is %f", packet_data.angleRotatedRightMotor);
    }

    return readBytes;
}

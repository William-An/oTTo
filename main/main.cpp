/************************************************************
*
* @file:      main.cpp
* @author:    Weili An, Xin Du, Yuqing Fan, Ruichao Zhang
* @email:     {an107, du201, fan230, zhan3147}@purdue.edu
* @version:   v1.0.0
* @date:      09/05/2021
* @brief:     Top level entry for oTTo project
*
************************************************************/

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "imu_sensor.h"
#include "imu_icm_20948.h"
#include "test_imu.h"
#include "communication_if.h"
#include "uart_wired.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

extern "C" void app_main(void);

// static const int RX_BUF_SIZE = 1024;

// #define TXD_PIN (GPIO_NUM_4)
// #define RXD_PIN (GPIO_NUM_2)

// void init(void) {
//     const uart_config_t uart_config = {
//         .baud_rate = 115200,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .source_clk = UART_SCLK_APB,
//     };
//     // We won't use a buffer for sending data.
//     uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
//     uart_param_config(UART_NUM_0, &uart_config);
//     // uart_set_pin(UART_NUM_0, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
//     uart_set_pin(0, 1, 3, -1, -1);
// }

// int sendData(const char* logName, const char* data)
// {
//     const int len = strlen(data);
//     const int txBytes = uart_write_bytes(UART_NUM_0, data, len);
//     // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
//     return txBytes;
// }

// static void tx_task(void *arg)
// {
//     static const char *TX_TASK_TAG = "TX_TASK";
//     esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
//     CommunicationIF myIF;
//     while (1) {
//         sendData(TX_TASK_TAG, myIF.myString);
//         vTaskDelay(2000 / portTICK_PERIOD_MS);
//     }
// }

// static void rx_task(void *arg)
// {
//     static const char *RX_TASK_TAG = "RX_TASK";
//     esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
//     uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
//     while (1) {
//         const int rxBytes = uart_read_bytes(UART_NUM_0, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
//         if (rxBytes > 0) {
//             data[rxBytes] = 0;
//             // ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
//             ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
//         }
//     }
//     free(data);
// }


void app_main(void)
{
    // Test IMU Fusion
    // test_fusion();
    // init();
    // xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    // xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);

    UartWired uartWired(false);
    while (true) {
        const char* sentString = "Hello!";
        const int txBytes = uartWired.sendData(sentString);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

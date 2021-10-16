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
#include "communication_struct.h"

extern "C" void app_main(void);

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

void vTaskFunction( void *pvParameters )
{
    const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    char *pcTaskName;
    const TickType_t xDelay250ms = pdMS_TO_TICKS( 1000 );
    /* The string to print out is passed in via the parameter. Cast this to a
    character pointer. */
    pcTaskName = ( char * ) pvParameters;
    const int len = strlen(pcTaskName);
    for( ;; )
    {
        ESP_LOGI(RX_TASK_TAG, "%s", pcTaskName);
        // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, pcTaskName, len, ESP_LOG_INFO);
        /* Delay for a period. This time a call to vTaskDelay() is used which places
        the task into the Blocked state until the delay period has expired. The
        parameter takes a time specified in ‘ticks’, and the pdMS_TO_TICKS() macro
        is used (where the xDelay250ms constant is declared) to convert 250
        milliseconds into an equivalent time in ticks. */
        // vTaskDelay( xDelay250ms );
  
    }
}

static const char *pcTextForTask1 = "Task 1 is running\r\n";
static const char *pcTextForTask2 = "Task 2 is running\r\n";

void app_main(void)
{
    // const esp_timer_create_args_t periodic_timer_args = {
    //         .name = "periodic"
    // };

    // esp_timer_handle_t periodic_timer;
    // ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    // ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 500000));

    UartWired uartWired(false);
    void* data = (void*) malloc(32); // 28 bytes are the largest possible packet
    float i = 0;
    while (true) {
        Command_Data sentString;
        sentString.leftAngularVelo = 24.38 + i;
        sentString.rightAngularVelo = 55.98;
        sentString.angleRotatedLeftMotor = 999.4342;
        sentString.angleRotatedRightMotor = 10.2;
        const int txBytes = uartWired.sendData(&sentString, 32);
        // vTaskDelay(2000 / portTICK_PERIOD_MS);
        // const int rxBytes = uartWired.receiveData(data, 32);
        // if (rxBytes > 0) { // if receive anything
        //     ESP_LOGI("RX_TASK", "Read %d bytes\n", rxBytes);
        //     uartWired.sendData(data);
        // }
        // ESP_LOGI("RX_TASK", "Test");
        // ESP_LOGI("RX_TASK", "%d num of bytes read", rxBytes);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        i++;
        // ESP_LOGI("TIMER", "Started timers, time since boot: %lld us", esp_timer_get_time());
    }
}


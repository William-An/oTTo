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
#include "esp_app_trace.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "imu_sensor.h"
#include "imu_icm_20948.h"
#include "communication_if.h"
#include "uart_wired.h"
#include "esp_now_wireless.h"
#include "otto.h"
#include "motor_driver.h"
#include "motor_stepper.h"
#include "test_motor.h"
#include "lcd1602.h"
extern "C" void app_main(void);

static QueueHandle_t dataInQueue;
static QueueHandle_t dataOutQueue;

// static EspNow* espNow;
UartWired uartWired(false);

void app_main(void)
{
    // Redirect trace to JTAG app trace
    // Has some limitations, see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/app_trace.html#limitations
    // esp_log_set_vprintf(esp_apptrace_vprintf);

    // Init console output
    // ASCII Art from https://patorjk.com/software/taag/#p=display&f=Isometric1&t=oTTo%0ArOBOT
    // Shows "OTTO ROBOT"
    printf("      ___           ___           ___           ___                    \n");
    printf("     /\\  \\         /\\  \\         /\\  \\         /\\  \\                   \n");
    printf("    /::\\  \\        \\:\\  \\        \\:\\  \\       /::\\  \\                  \n");
    printf("   /:/\\:\\  \\        \\:\\  \\        \\:\\  \\     /:/\\:\\  \\                 \n");
    printf("  /:/  \\:\\  \\       /::\\  \\       /::\\  \\   /:/  \\:\\  \\                \n");
    printf(" /:/__/ \\:\\__\\     /:/\\:\\__\\     /:/\\:\\__\\ /:/__/ \\:\\__\\               \n");
    printf(" \\:\\  \\ /:/  /    /:/  \\/__/    /:/  \\/__/ \\:\\  \\ /:/  /               \n");
    printf("  \\:\\  /:/  /    /:/  /        /:/  /       \\:\\  /:/  /                \n");
    printf("   \\:\\/:/  /     \\/__/         \\/__/         \\:\\/:/  /                 \n");
    printf("    \\::/  /                                   \\::/  /                  \n");
    printf("     \\/__/                                     \\/__/                   \n");
    printf("      ___           ___           ___           ___           ___      \n");
    printf("     /\\  \\         /\\  \\         /\\  \\         /\\  \\         /\\  \\     \n");
    printf("    /::\\  \\       /::\\  \\       /::\\  \\       /::\\  \\        \\:\\  \\    \n");
    printf("   /:/\\:\\  \\     /:/\\:\\  \\     /:/\\:\\  \\     /:/\\:\\  \\        \\:\\  \\   \n");
    printf("  /::\\~\\:\\  \\   /:/  \\:\\  \\   /::\\~\\:\\__\\   /:/  \\:\\  \\       /::\\  \\  \n");
    printf(" /:/\\:\\ \\:\\__\\ /:/__/ \\:\\__\\ /:/\\:\\ \\:|__| /:/__/ \\:\\__\\     /:/\\:\\__\\ \n");
    printf(" \\/_|::\\/:/  / \\:\\  \\ /:/  / \\:\\~\\:\\/:/  / \\:\\  \\ /:/  /    /:/  \\/__/ \n");
    printf("    |:|::/  /   \\:\\  /:/  /   \\:\\ \\::/  /   \\:\\  /:/  /    /:/  /      \n");
    printf("    |:|\\/__/     \\:\\/:/  /     \\:\\/:/  /     \\:\\/:/  /     \\/__/       \n");
    printf("    |:|  |        \\::/  /       \\::/__/       \\::/  /                  \n");
    printf("     \\|__|         \\/__/         ~~            \\/__/                   \n");   
    printf("\n");
    printf("Developed by Weili An, Xin Du, Yuqing Fan, Ruichao Zhang at Purdue University\n");
    printf("Current version: v0.0.1a\n"); 
    printf("\n");           
    // Init task need to have to priority to ensure the
    // rest tasks can be properly initated
    xTaskCreate(otto_init, "OTTO Init task", 4096, NULL, OTTO_INIT_TASK_PRI, NULL);
    // test_stepper();
}

/**
 * @brief Init all the peripherals used in oTTo and
 *        launch all tasks
 *        create all queues between tasks
 * 
 */
void otto_init(void *param) {
    // General info
    // From sample application
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(__func__, "This is %s chip with %d CPU core(s), WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
    ESP_LOGI(__func__, "silicon revision %d, ", chip_info.revision);
    ESP_LOGI(__func__, "%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    ESP_LOGI(__func__, "Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
    ESP_LOGI(__func__, "FreeRTOS %d ms per tick", portTICK_RATE_MS);

    // I2C Port initialization
    ESP_LOGI(__func__, "Init I2C port %d", OTTO_I2C_PORT_NUM);
    ESP_ERROR_CHECK(i2c_init(OTTO_I2C_PORT_NUM));

    // data in queue initialization
    ESP_LOGI(__func__, "Init data in Queue");
    dataInQueue = xQueueCreate(OTTO_DATA_IN_QUEUE_LEN, sizeof(Feedback_Data));

    // UART initialization
    ESP_ERROR_CHECK(uart_init(OTTO_UART_PORT_NUM));

    // ESPNOW initialization
    ESP_ERROR_CHECK(espnow_init(dataInQueue, NULL));

    // Get Mac addr
    ESP_ERROR_CHECK(get_macAddr());
    
    // COMM Sender task
    // ESP_LOGI(__func__, "Launch Comm sender task");
    // xTaskCreate(comm_sender_task, "Comm sender Task", 4096, NULL, OTTO_COMM_SENDER_TASK_PRI, NULL);

    // COMM Receiver task
    ESP_LOGI(__func__, "Launch Comm receiver task");
    xTaskCreate(comm_receiver_task, "Comm receiver Task", 4096, NULL, OTTO_COMM_RECEIVER_TASK_PRI, NULL);

    // Clean up init task
    vTaskDelete(NULL);
}

/**
 * @brief Receive data from PC side through UART, it then sends the data out through ESPNOW
 * 
 * @param param 
 */
void comm_receiver_task(void *param) {

    ESP_LOGI(__func__, "Comm receiver Task begins");
    Command_Data_Packet_ESP_NOW commandDataPacketEspNow;
    
    EspNowWireless espNowWireless(false);
    UartWired uartWired(false);

    // Flush current fifo before processing
    ESP_ERROR_CHECK(uart_flush(OTTO_UART_PORT_NUM));

    while (1) {
        uint8_t headerByte;

        while (1) {
            int length = 0;
            ESP_ERROR_CHECK(uart_get_buffered_data_len(OTTO_UART_PORT_NUM, (size_t*)&length));
            ESP_LOGI(__func__, "Buffer size: %d", length);
            // TODO Weili: Did not check for possible restart?
            // TODO Like: H1 H2 H1 H2 H3 H4 DATA_PACKET
            // TODO Weili: Use bit shift op for these
            uartWired.receiveData(&headerByte, sizeof(headerByte), portMAX_DELAY);
            if (headerByte == HEADER_BYTE1) {
                uartWired.receiveData(&headerByte, sizeof(headerByte), portMAX_DELAY);
                if (headerByte == HEADER_BYTE2) {
                    uartWired.receiveData(&headerByte, sizeof(headerByte), portMAX_DELAY);
                    if (headerByte == HEADER_BYTE3) {
                        uartWired.receiveData(&headerByte, sizeof(headerByte), portMAX_DELAY);
                        if (headerByte == HEADER_BYTE4) {
                            headerByte = 0;
                            break;
                        } else {
                            ESP_LOGI(__func__, "Failed at header byte 4, exp: %x act: %x", HEADER_BYTE4, headerByte);
                        }
                    } else {
                        ESP_LOGI(__func__, "Failed at header byte 3, exp: %x act: %x", HEADER_BYTE3, headerByte);
                    }
                } else {
                    ESP_LOGI(__func__, "Failed at header byte 2, exp: %x act: %x", HEADER_BYTE2, headerByte);
                }
            } else {
                ESP_LOGI(__func__, "Failed at header byte 1, exp: %x act: %x", HEADER_BYTE1, headerByte);
            }  
        }
        // TODO Weili: MATLAB does not pad the struct, thus right now just hard-code the value in
        int readBytes = uartWired.receiveData(&commandDataPacketEspNow, 28, portMAX_DELAY);
        if (readBytes == 28) {
            // todo: add checking for header, CRC, ... to check the correctness of the packet
            // commandData = commandDataPacket.commandData;
            ESP_LOGI(__func__, "Comm receiver Task: received one packet: omega_left: %.2f", commandDataPacketEspNow.commandData.leftAngularVelo);
            espNowWireless.sendData(&commandDataPacketEspNow, 28);
        }
    }

    ESP_LOGE(__func__, "COMM receiver Task quit unexpectedly");
    vTaskDelete(NULL);
}

/**
 * @brief Init communication sender
 * 
 * @param param 
 */
void comm_sender_task(void *param) {
    
    ESP_LOGI(__func__, "Comm sender Task begins");
    Feedback_Data feedbackData;
    Feedback_Data_Packet_UART feedbackDataPacket;
    UartWired uartWired(false);

    while (1) {
        if( xQueueReceive( dataInQueue, &feedbackData, portMAX_DELAY ) == pdTRUE) {
            feedbackDataPacket.CRC = 0; // TODO: 
            feedbackDataPacket.header = HEADER;
            feedbackDataPacket.feedBackData = feedbackData;
            feedbackDataPacket.timestamp = 0; // TODO: 
            uartWired.sendData(&feedbackDataPacket, sizeof(Feedback_Data_Packet_UART));
        }
    }

    ESP_LOGE(__func__, "COMM sender Task quit unexpectedly");
    vTaskDelete(NULL);
}
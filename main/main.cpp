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
#include "communication_struct.h"
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
    xTaskCreate(otto_init, "OTTO Init task", 2048, NULL, OTTO_INIT_TASK_PRI, NULL);
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

    // Get Mac addr
    ESP_ERROR_CHECK(get_macAddr());

    // Communication initialization
    ESP_LOGI(__func__, "Init communication with ESP_NOW");
    static EspNowWireless espNow(false, dataInQueue, dataOutQueue);
    
    // COMM Sender task
    // ESP_LOGI(__func__, "Launch Comm sender task");
    // xTaskCreate(comm_sender_task, "Comm sender Task", 4096, NULL, OTTO_COMM_SENDER_TASK_PRI, NULL);

    // COMM Receiver task
    ESP_LOGI(__func__, "Launch Comm receiver task");
    xTaskCreate(comm_receiver_task, "Comm receiver Task", 4096, (void*) &espNow, OTTO_COMM_RECEIVER_TASK_PRI, NULL);

    // Clean up init task
    vTaskDelete(NULL);
}

// TODO Weird, IMU updates 1/2 the freq given:
// TODO If update every 5 ticks, the comm will get result every 10 ticks
// TODO But the tick updates to 1kHz, it is now working properly
/**
 * @brief IMU subsystem task, init IMU and take constant measurement
 * 
 * @param param 
 */
void imu_task(void *param) {
    // Variable used
    AngleVector3_t eulerAngles; // Tmp var to hold imu measurement
    TickType_t xLastWakeTime;   // Last time the measurement is run
    TickType_t xImuFrequency;   // Measurement update frequency
    BaseType_t xErr;            // RTOS call error status
    Feedback_Data feedbackData;

    ESP_LOGI(__func__, "IMU Task begins");

    // Assume I2C initialized
    ICM20948IMU imu(OTTO_IMU_RATE_HZ, ICM20948_I2CADDR_DEFAULT);
    ESP_ERROR_CHECK(imu.begin(OTTO_I2C_PORT_NUM));

    // Init loop vars
    xLastWakeTime = xTaskGetTickCount();
    xImuFrequency = 1000 / OTTO_IMU_RATE_HZ / portTICK_RATE_MS;
    ESP_LOGI(__func__, "IMU Update per %d ticks", xImuFrequency);

    while (1) {
        // Delay according to update rate
        vTaskDelayUntil(&xLastWakeTime, xImuFrequency);

        // Gather imu data
        ESP_ERROR_CHECK(imu.updateAll());
        imu.runFusion();
        eulerAngles = imu.getEulerAngles();
        ESP_LOGD(__func__, "r%.2frp%.2fpy%.2fy", eulerAngles.roll, eulerAngles.pitch, eulerAngles.yaw);

        feedbackData.leftAngularVelo = 0;
        feedbackData.rightAngularVelo = 0;
        feedbackData.angleRotatedLeftMotor = 0;
        feedbackData.angleRotatedRightMotor = 0;
        feedbackData.yaw = eulerAngles.yaw;
        feedbackData.pitch = eulerAngles.pitch;
        feedbackData.roll = eulerAngles.roll;

        // Send data onto queue, not waiting for slot as can just
        // use next measurement
        if (xQueueSendToBack(dataOutQueue, &feedbackData, portMAX_DELAY) == errQUEUE_FULL) {
            ESP_LOGW(__func__, "IMU Queue full, cannot send");
        }
    }

    ESP_LOGE(__func__, "IMU Task quit unexpectedly");
    vTaskDelete(NULL);
}

/**
 * @brief Init communication receiver
 * 
 * @param param 
 */
void comm_receiver_task(void *param) {

    ESP_LOGI(__func__, "Comm receiver Task begins");
    Command_Data_Packet_ESP_NOW commandDataPacketEspNow;
    EspNowWireless espNow = *((EspNowWireless*)param);

    // Flush current fifo before processing
    ESP_ERROR_CHECK(uart_flush(UART_NUM_0));
    while (1) {
        uint8_t headerByte;

        while (1) {
            int length = 0;
            ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_0, (size_t*)&length));
            ESP_LOGI(__func__, "Buffer size: %d", length);
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
                        }
                    }
                }
            }  
        }

        int readBytes = uartWired.receiveData(&commandDataPacketEspNow, sizeof(commandDataPacketEspNow), portMAX_DELAY);
        if (readBytes == sizeof(commandDataPacketEspNow)) {
            // todo: add checking for header, CRC, ... to check the correctness of the packet
            // commandData = commandDataPacket.commandData;
            ESP_LOGI(__func__, "Comm receiver Task: received one packet: omega_left: %.2f", commandDataPacketEspNow.commandData.leftAngularVelo);
            espNow.sendData(&commandDataPacketEspNow, sizeof(commandDataPacketEspNow));
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

    while (1) {
        if( xQueueReceive( dataOutQueue, &feedbackData, portMAX_DELAY ) == pdTRUE) {
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
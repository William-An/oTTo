/*
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
    // printf("      ___           ___           ___           ___                    \n");
    // printf("     /\\  \\         /\\  \\         /\\  \\         /\\  \\                   \n");
    // printf("    /::\\  \\        \\:\\  \\        \\:\\  \\       /::\\  \\                  \n");
    // printf("   /:/\\:\\  \\        \\:\\  \\        \\:\\  \\     /:/\\:\\  \\                 \n");
    // printf("  /:/  \\:\\  \\       /::\\  \\       /::\\  \\   /:/  \\:\\  \\                \n");
    // printf(" /:/__/ \\:\\__\\     /:/\\:\\__\\     /:/\\:\\__\\ /:/__/ \\:\\__\\               \n");
    // printf(" \\:\\  \\ /:/  /    /:/  \\/__/    /:/  \\/__/ \\:\\  \\ /:/  /               \n");
    // printf("  \\:\\  /:/  /    /:/  /        /:/  /       \\:\\  /:/  /                \n");
    // printf("   \\:\\/:/  /     \\/__/         \\/__/         \\:\\/:/  /                 \n");
    // printf("    \\::/  /                                   \\::/  /                  \n");
    // printf("     \\/__/                                     \\/__/                   \n");
    // printf("      ___           ___           ___           ___           ___      \n");
    // printf("     /\\  \\         /\\  \\         /\\  \\         /\\  \\         /\\  \\     \n");
    // printf("    /::\\  \\       /::\\  \\       /::\\  \\       /::\\  \\        \\:\\  \\    \n");
    // printf("   /:/\\:\\  \\     /:/\\:\\  \\     /:/\\:\\  \\     /:/\\:\\  \\        \\:\\  \\   \n");
    // printf("  /::\\~\\:\\  \\   /:/  \\:\\  \\   /::\\~\\:\\__\\   /:/  \\:\\  \\       /::\\  \\  \n");
    // printf(" /:/\\:\\ \\:\\__\\ /:/__/ \\:\\__\\ /:/\\:\\ \\:|__| /:/__/ \\:\\__\\     /:/\\:\\__\\ \n");
    // printf(" \\/_|::\\/:/  / \\:\\  \\ /:/  / \\:\\~\\:\\/:/  / \\:\\  \\ /:/  /    /:/  \\/__/ \n");
    // printf("    |:|::/  /   \\:\\  /:/  /   \\:\\ \\::/  /   \\:\\  /:/  /    /:/  /      \n");
    // printf("    |:|\\/__/     \\:\\/:/  /     \\:\\/:/  /     \\:\\/:/  /     \\/__/       \n");
    // printf("    |:|  |        \\::/  /       \\::/__/       \\::/  /                  \n");
    // printf("     \\|__|         \\/__/         ~~            \\/__/                   \n");   
    // printf("\n");
    printf("Developed by Weili An, Xin Du, Yuqing Fan, Ruichao dZhang at Purdue University\n");
    // printf("Current version: v0.0.1a\n"); 
    // printf("\n");           
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

    // Get Mac addr
    ESP_ERROR_CHECK(get_macAddr());

    // data in queue initialization
    ESP_LOGI(__func__, "Init data in Queue");
    dataInQueue = xQueueCreate(OTTO_DATA_IN_QUEUE_LEN, sizeof(Command_Data));

    // data out queue initialization
    ESP_LOGI(__func__, "Init data out Queue");
    dataOutQueue = xQueueCreate(OTTO_DATA_OUT_QUEUE_LEN, sizeof(Feedback_Data));

    // Start subsystem tasks
    // IMU task
    // Pin IMU task to APP_CPU per ESP32 Guidelines: 
    //  https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/freertos-smp.html#floating-points
    ESP_LOGI(__func__, "Launch IMU task");
    xTaskCreatePinnedToCore(imu_task, "IMU Task", 8192, NULL, OTTO_IMU_TASK_PRI, NULL, APP_CPU_NUM);

    // LCD task
    ESP_LOGI(__func__, "Launch LCD task");
    xTaskCreate(display_task, "LCD Task", 4096, NULL, OTTO_DISP_TASK_PRI, NULL);

    // Motor task
    ESP_LOGI(__func__, "Launch motor task");
    xTaskCreatePinnedToCore(motor_task, "motor Task", 4096, NULL, OTTO_MOTOR_TASK_PRI, NULL, PRO_CPU_NUM);

    // Communication initialization
    if (ESP_NOW_MODE) {
        ESP_LOGI(__func__, "Init communication with ESP_NOW");
        ESP_ERROR_CHECK(espnow_init(dataInQueue, dataOutQueue));

        // COMM Sender task
        ESP_LOGI(__func__, "Launch Comm sender task");
        xTaskCreate(comm_sender_task, "C)omm sender Task", 4096, NULL, OTTO_COMM_SENDER_TASK_PRI, NULL);
    } else {
        ESP_LOGI(__func__, "Init communication with UART");
        ESP_ERROR_CHECK(uart_init(UART_NUM_0));
        
        // COMM Sender task
        ESP_LOGI(__func__, "Launch Comm sender task");
        xTaskCreate(comm_sender_task, "Comm sender Task", 4096, NULL, OTTO_COMM_SENDER_TASK_PRI, NULL);

        // COMM Receiver task
        ESP_LOGI(__func__, "Launch Comm receiver task");
        xTaskCreate(comm_receiver_task, "Comm receiver Task", 4096, NULL, OTTO_COMM_RECEIVER_TASK_PRI, NULL);
    }
    
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
    xImuFrequency = (1000 / OTTO_IMU_RATE_HZ) / portTICK_RATE_MS;
    ESP_LOGI(__func__, "IMU Update per %d ticks", xImuFrequency);

    while (1) {
        // Delay according to update rate
        vTaskDelayUntil(&xLastWakeTime, xImuFrequency);

        // Gather imu data
        ESP_ERROR_CHECK(imu.updateAll());
        imu.runFusion();
        eulerAngles = imu.getEulerAngles();
        // ESP_LOGD(__func__, "r%.2frp%.2fpy%.2fy", eulerAngles.roll, eulerAngles.pitch, eulerAngles.yaw);

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
    Command_Data_Packet_UART commandDataPacket;
    Command_Data commandData;

    UartWired uartWired(false);

    // Flush current fifo before processing
    ESP_ERROR_CHECK(uart_flush(UART_NUM_0));

    while (1) {
        uint8_t headerByte;

        // Sync header bytes
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

        int readBytes = uartWired.receiveData(&commandData, sizeof(commandData), portMAX_DELAY);
        if (readBytes == sizeof(commandData)) {
            // todo: add checking for header, CRC, ... to check the correctness of the packet
            // commandData = commandDataPacket.commandData;
            ESP_LOGI(__func__, "UART: received one packet: omega_left: %.2f", commandData.leftAngularVelo);
            // if (xQueueSendToBack( dataInQueue, &commandData, ( TickType_t ) 0 ) != pdPASS ) {
            //     ESP_LOGI(__func__, "Comm receiver Task: queue full");
            // }
            Feedback_Data feedbackData;
            feedbackData.leftAngularVelo = commandData.leftAngularVelo;
            feedbackData.rightAngularVelo = commandData.rightAngularVelo;
            feedbackData.angleRotatedLeftMotor = commandData.angleRotatedLeftMotor;
            feedbackData.angleRotatedRightMotor = commandData.angleRotatedRightMotor;
            feedbackData.yaw = 1;
            feedbackData.pitch = 2;
            feedbackData.roll = 3;

            if (xQueueSendToBack( dataOutQueue, &feedbackData, ( TickType_t ) 0 ) != pdPASS ) {
                ESP_LOGI(__func__, "Comm receiver Task: queue full");
            }
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
    Feedback_Data_Packet_UART feedbackDataPacketUART;
    Feedback_Data_Packet_ESP_NOW feedbackDataPacketESPNOW;

    UartWired uartWired(false);
    EspNowWireless espNowWireless(false);

    while (1) {
        if( xQueueReceive( dataOutQueue, &feedbackData, portMAX_DELAY ) == pdTRUE) {

            if (ESP_NOW_MODE) {
                feedbackDataPacketESPNOW.CRC = 0; // TODO: 
                feedbackDataPacketESPNOW.feedBackData = feedbackData;
                feedbackDataPacketESPNOW.timestamp = 0; // TODO: 
                // ESP_LOGI(__func__, "About to send back data through ESP-NOW! yaw: %.2f", feedbackDataPacketESPNOW.feedBackData.yaw);
                // ESP_LOGI(__func__, "About to send back data through ESP-NOW! yaw: %.2f", feedbackDataPacketESPNOW.feedBackData.pitch);
                // ESP_LOGI(__func__, "About to send back data through ESP-NOW! yaw: %.2f", feedbackDataPacketESPNOW.feedBackData.roll);
                espNowWireless.sendData(&feedbackDataPacketESPNOW, sizeof(Feedback_Data_Packet_ESP_NOW));
            } else {
                feedbackDataPacketUART.CRC = 0; // TODO: 
                feedbackDataPacketUART.header = HEADER;
                feedbackDataPacketUART.feedBackData = feedbackData;
                feedbackDataPacketUART.timestamp = 0; // TODO: 
                uartWired.sendData(&feedbackDataPacketUART, sizeof(Feedback_Data_Packet_UART));
            }
            
        }
    }

    ESP_LOGE(__func__, "COMM sender Task quit unexpectedly");
    vTaskDelete(NULL);
}


/**
 * @brief Init lcd 
 * 
 * @param param 
 */
void display_task(void *param) {
    Command_Data commandData;
    LCD1602 lcd(0b0100000, 0b0100111);
    ESP_ERROR_CHECK(lcd.begin(OTTO_I2C_PORT_NUM));
    uint8_t sw_var;
    int page = 0;
    int i = 0;
    MenuState n_state = ROOT;

    while(1) {
        lcd.readReg(0b0100111, MCP23008_REG_GPIO, &sw_var, 1);
        //printf("sw var%d \n",~sw_var);
        uint8_t down = ~sw_var & 0b00001;      //gp 0
        uint8_t left = ~sw_var & 0b00010;    //gp 1
        uint8_t menu = ~sw_var & 0b00100;    //gp 2
        uint8_t right = ~sw_var & 0b01000;    //gp 3
        uint8_t up = ~sw_var & 0b10000;   //gp 4
        ESP_LOGI(__func__, "menu: %d up: %d down %d\n left %d right %d",menu, up, down, left, right);

// enum MenuState{ROOT, 
//                 CMD, 
//                     ANG,VELO,
//                 FDB, 
//                     YALL, PITCH, ROLL,
//                 MAC, 
//                     WIFI,ETH,BLU};
        if (menu){
            n_state = ROOT;
        }
        switch (n_state){
            case ROOT: 
                lcd.clear();
                lcd.move_cursor(0,0);
                lcd.write_string("oTTo Project"); 
                lcd.move_cursor(1,0);
                lcd.write_string("By WA, XD, YF, RZ"); 
                break;
            case CMD:
                lcd.clear();
                lcd.move_cursor(0,0);
                lcd.write_string("Command Data"); 
                lcd.move_cursor(1,0);
                lcd.write_string("Feedback Data");
                if (down){
                    n_state = FDB;
                }
                if (up){
                    n_state = MAC;
                } 
                if (left){
                    n_state = ROOT;
                }
                if (right){
                    n_state = ANG;
                }
                break;
            case VELO:
                lcd.clear();
                lcd.move_cursor(0,0);
                lcd.write_string("Angular Velocity"); 
                lcd.move_cursor(1,0);
                lcd.write_string("Rotated Angle"); 
                if (down || up){
                    n_state = VELO;
                }
                if (right){
                    n_state = ANGD;
                }
                if (left){
                    n_state = CMD;
                }
                break;
            case ANG:
                lcd.clear();
                lcd.move_cursor(1,0);
                lcd.write_string("Angular Velocity"); 
                lcd.move_cursor(0,0);
                lcd.write_string("Rotated Angle"); 
                if (down || up){
                    n_state = ANG;
                }
                if (right){
                    n_state = VELOD;
                }
                if (left){
                    n_state = CMD;
                }
                break;
            case FDB:
                lcd.clear();
                lcd.move_cursor(1,0);
                lcd.write_string("MAC address"); 
                lcd.move_cursor(0,0);
                lcd.write_string("Feedback Data"); 
                if (down){
                    n_state = MAC;
                } 
                if (up){
                    n_state = CMD;
                }
                if (left){
                    n_state = ROOT;
                }
                if (right){
                    n_state = YALL;
                }
                break;
            case YALL:
                lcd.clear();
                lcd.move_cursor(1,0);
                lcd.write_string("PITCH"); 
                lcd.move_cursor(0,0);
                lcd.write_string("YAW"); 
                if (down){
                    n_state = PITCH;
                } 
                if (up){
                    n_state = ROLL;
                }
                if (left){
                    n_state = FDB;
                }
                if (right){
                    n_state = YALLD;
                }
                break;
            case PITCH:
                lcd.clear();
                lcd.move_cursor(1,0);
                lcd.write_string("ROLL"); 
                lcd.move_cursor(0,0);
                lcd.write_string("PITCH"); 
                if (down){
                    n_state = ROLL;
                } 
                if (up){
                    n_state = YALL;
                }
                if (left){
                    n_state = FDB;
                }
                if (right){
                    n_state = PITCHD;
                }
                break;
            case ROLL:
                lcd.clear();
                lcd.move_cursor(1,0);
                lcd.write_string("YAW"); 
                lcd.move_cursor(0,0);
                lcd.write_string("ROLL"); 
                if (down){
                    n_state = YALL;
                } 
                if (up){
                    n_state = PITCH;
                }
                if (left){
                    n_state = FDB;
                }
                if (right){
                    n_state = ROLLD;
                }
                break;
            case MAC:
                lcd.clear();
                lcd.move_cursor(0,0);
                lcd.write_string("MAC address"); 
                lcd.move_cursor(1,0);
                lcd.write_string("Command Data");
                if (down){
                    n_state = CMD;
                }  
                if (up){
                    n_state = FDB;
                }
                if (left){
                    n_state = ROOT;
                }
                if (right){
                    n_state = WIFI;
                }
                break;
            case WIFI:
                lcd.clear();
                lcd.move_cursor(0,0);
                lcd.write_string("WIFI: "); 
                lcd.move_cursor(1,0);
                lcd.write_string("0c:dc:7e:89:32:5c");
                if (down){
                    n_state = ETH;
                }  
                if (up){
                    n_state = BLU;
                }
                if (left){
                    n_state = MAC;
                }

                break;
            case ETH:
                lcd.clear();
                lcd.move_cursor(0,0);
                lcd.write_string("ETHERNET: "); 
                lcd.move_cursor(1,0);
                lcd.write_string("0c:dc:7e:89:32:5c");
                if (down){
                    n_state = BLU;
                }  
                if (up){
                    n_state = WIFI;
                }
                if (left){
                    n_state = MAC;
                }
            case BLU:
                lcd.clear();
                lcd.move_cursor(0,0);
                lcd.write_string("BLU "); 
                lcd.move_cursor(1,0);
                lcd.write_string("0c:dc:7e:89:32:5c");
                if (down){
                    n_state = WIFI;
                }  
                if (up){
                    n_state = ETH;
                }
                if (left){
                    n_state = MAC;
                }
        }
       

        if( xQueuePeek( dataInQueue, (void*) &( commandData ), pdMS_TO_TICKS( 100 ) ) ) {
            // ESP_LOGE(__func__, "received. %f", commandData);
            // todo: display the received data
            char lvl[30];
            char rvl[30];
            char lal[30];
            char ral[30];
            sprintf(lvl,"LEFT: %.2f",commandData.leftAngularVelo);
            sprintf(rvl,"RIGHT: %.2f",commandData.rightAngularVelo);
            sprintf(lal,"LEFT: %.2f",commandData.angleRotatedLeftMotor);
            sprintf(ral,"RIGHT: %.2f",commandData.angleRotatedRightMotor);
            //ESP_LOGD(__func__, "%.2f",commandData.leftAngularVelo);
           // lcd.clear();
            // lcd.write_string(str);
            //lcd.write_string(lvl);

        switch (n_state){
            case ANGD:
                lcd.clear();
                lcd.move_cursor(0,0);
                lcd.write_string(lal); 
                lcd.move_cursor(1,0);
                lcd.write_string(ral); 
                if (left){
                    n_state = ANG;
                }
                break;
            case VELOD:
                lcd.clear();
                lcd.move_cursor(0,0);
                lcd.write_string(lvl); 
                lcd.move_cursor(1,0);
                lcd.write_string(rvl); 
                if (left){
                    n_state = VELO;
                }
                break;
        }

        if( xQueuePeek( dataInQueue, (void*) &( feedbackData ), pdMS_TO_TICKS( 100 ) ) ) {
            // ESP_LOGE(__func__, "received. %f", commandData);
            // todo: display the received data
            char lvl[30];
            char rvl[30];
            char lal[30];
            char ral[30];
            sprintf(yall,"%.2f",feedbackData.YALL);
            sprintf(pitch,"%.2f",feedbackData.PITCH);
            sprintf(roll,"%.2f",feedbackData.ROLL);

            //ESP_LOGD(__func__, "%.2f",commandData.leftAngularVelo);
           // lcd.clear();
            // lcd.write_string(str);
            //lcd.write_string(lvl);

        switch (n_state){
            case YALLD:
                lcd.clear();
                lcd.move_cursor(0,0);
                lcd.write_string("YALL"); 
                lcd.move_cursor(1,0);
                lcd.write_string(yall); 
                if (left){
                    n_state = YALL;
                }
                break;
            case PITCHD:
                lcd.clear();
                lcd.move_cursor(0,0);
                lcd.write_string("PITCH"); 
                lcd.move_cursor(1,0);
                lcd.write_string(pitch); 
                if (left){
                    n_state = PITCH;
                }
                break;
            case ROLLD:
                lcd.clear();
                lcd.move_cursor(0,0);
                lcd.write_string("roll"); 
                lcd.move_cursor(1,0);
                lcd.write_string(roll); 
                if (left){
                    n_state = ROLL;
                }
            break;

        }
       

        
        // todo: check button status
        }
    }
}

/**
 * @brief Motor task
 * 
 * @param param 
 */
void motor_task(void *param) {
    A4988_Driver ref_wheel;

    Nema17Config_t nema17;
    nema17.fullStep = 1.8;
    A4988_Driver ops_wheel (SIXTEENTH_STEP, OPPOSITE, nema17);

    MotorIOConfig_t motor_pin;
    motor_pin.step = GPIO_NUM_33;
    motor_pin.en = GPIO_NUM_25;
    motor_pin.dir = GPIO_NUM_32;
    // TODO Connect the following pins to VCC
    motor_pin.ms1 = GPIO_NUM_21;
    motor_pin.ms2 = GPIO_NUM_21;
    motor_pin.ms3 = GPIO_NUM_21;
    ESP_ERROR_CHECK(ref_wheel.configIO(motor_pin));

    motor_pin.step = GPIO_NUM_18;
    motor_pin.en = GPIO_NUM_4;
    motor_pin.dir = GPIO_NUM_19;
    // TODO Connect the following pins to VCC
    motor_pin.ms1 = GPIO_NUM_21;
    motor_pin.ms2 = GPIO_NUM_21;
    motor_pin.ms3 = GPIO_NUM_21;
    ESP_ERROR_CHECK(ops_wheel.configIO(motor_pin));
    Command_Data commandData;
    while(1) {
        if( xQueueReceive( dataInQueue, &commandData, portMAX_DELAY) == pdTRUE ) {
            ESP_LOGI(__func__, "motor_task received command");
            // todo: use the data to drive the motor accordingly;
            ESP_LOGI(__func__, "commandData.leftAngularVelo: %f", commandData.leftAngularVelo);
            ESP_LOGI(__func__, "commandData.rightAngularVelo: %f", commandData.rightAngularVelo);
            ESP_ERROR_CHECK(ref_wheel.setContinuous(commandData.leftAngularVelo));
            ESP_ERROR_CHECK(ops_wheel.setContinuous(commandData.rightAngularVelo));
        } else {
            ESP_LOGW(__func__, "fail to retrieve item for motor");
        }
    }
}
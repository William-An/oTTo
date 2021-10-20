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

extern "C" void app_main(void);
extern esp_err_t i2c_init(uint8_t i2c_portNum);

void app_main(void)
{
    i2c_port_t port = 0;
    ESP_LOGI("Main", "Init I2C port %d", port);
    i2c_init(port);
    // Test IMU Fusion
    test_fusion(port);
}
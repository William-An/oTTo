/************************************************************
*
* @file:      test_fusion.cpp
* @author:    Weili An
* @email:     an107@purdue.edu
* @version:   v1.1.0
* @date:      09/14/2021
* @brief:     Sample program for ICM 20948 Fusion
*
************************************************************/

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "imu_sensor.h"
#include "imu_icm_20948.h"

void test_fusion(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    // Begin IMU Fusin test
    printf("Begin ICM 20948 Fusion test");
    
    ICM20948IMU imu(100, ICM20948_I2CADDR_DEFAULT);
    ESP_ERROR_CHECK(imu.begin());

    Vector3_t accelVec;
    Vector3_t gyroVec;
    Vector3_t magnetVec;
    float temp;
    AngleVector3 eulerAngles;
    for(;;) {
        // Perform a read
        imu.updateAll();

        // Get sensor measurement
        // accelVec = imu.getAccel();
        // gyroVec = imu.getGyro();
        // magnetVec = imu.getMagnet();
        // temp = imu.getTemp();

        // ESP_LOGI("[ICM]", "Accel: x = %.5f y = %.5f z = %.5f", accelVec.x, accelVec.y, accelVec.z);
        // ESP_LOGI("[ICM]", "Gyro: x = %.5f y = %.5f z = %.5f", gyroVec.x, gyroVec.y, gyroVec.z);
        // ESP_LOGI("[ICM]", "Mag: x = %.5f y = %.5f z = %.5f", magnetVec.x, magnetVec.y, magnetVec.z);
        // ESP_LOGI("[ICM]", "temp: t = %.5f", temp);

        imu.runFusion();
        eulerAngles = imu.getEulerAngles();
        ESP_LOGI("[ICM ARHS]", "r%.2frp%.2fpy%.2fy", eulerAngles.roll, eulerAngles.pitch, eulerAngles.yaw);

        vTaskDelay(10 / portTICK_RATE_MS);
    }
}

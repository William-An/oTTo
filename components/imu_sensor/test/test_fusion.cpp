/************************************************************
*
* @file:      test_fusion.cpp
* @author:    Weili An
* @email:     an107@purdue.edu
* @version:   v1.0.0
* @date:      09/05/2021
* @brief:     Sample test function for IMU fusion algorithm
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

extern "C" void test_fusion(void);

float getRandom(uint64_t range, uint8_t decimal) {
    uint64_t bound = range * pow10(decimal);
    return (float)(((esp_random() % bound) - bound/2) / 100.0);
}

void test_fusion(void)
{
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_LOGI("IMU", "Beginning testing imu fusion");
    Vector3_t mock_accel;
    Vector3_t mock_gyro;
    Vector3_t mock_magnet;
    const uint32_t mock_updateFreq = 10;

    // Initialize IMU
    ICM20948IMU imu(mock_updateFreq);

    // Mock test loop
    while (true) {
        mock_accel = {
            .x = (mock_accel.x + getRandom(1600, 2)),
            .y = (mock_accel.y + getRandom(1600, 2)),
            .z = (mock_accel.z + getRandom(1600, 2)),
        };

        mock_gyro = {
            .x = (mock_gyro.x + getRandom(90, 2)),
            .y = (mock_gyro.y + getRandom(90, 2)),
            .z = (mock_gyro.z + getRandom(90, 2)),
        };

        mock_magnet = {
            .x = (mock_magnet.x + getRandom(10, 2)),
            .y = (mock_magnet.y + getRandom(10, 2)),
            .z = (mock_magnet.z + getRandom(10, 2)),
        };

        // Fusion algorithm
        int64_t prior = esp_timer_get_time();
        imu.runFusion();
        int64_t after = esp_timer_get_time();

        AngleVector3_t angles = imu.getEulerAngles();
        ESP_LOGI("IMU", "Roll: %.2f Pitch: %.2f Yaw: %.2f Calc time (us): %lld",
            angles.roll, angles.pitch, angles.yaw, (after - prior));
        vTaskDelay((1000 / mock_updateFreq) / portTICK_PERIOD_MS);
    }

    imu.runFusion();
}

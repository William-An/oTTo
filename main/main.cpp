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
#include "lcd1602.h"

// I2C Pin def
#define OTTO_I2C_SDA_PIN 26
#define OTTO_I2C_SCL_PIN 25
#define OTTO_I2C_FREQ 400000

// I2C Conf
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */

// Use port 0 of I2C
#define OTTO_I2C_PORT_NUM 0

#define TAG "OTTO_LCD"

/**
 * @brief Init I2C port
 * 
 * @param i2c_portNum 
 * @return esp_err_t 
 */
esp_err_t i2c_init(uint8_t i2c_portNum) {
    // Default ICM 20948 I2C config
    i2c_config_t default_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = OTTO_I2C_SDA_PIN,
        .scl_io_num = OTTO_I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };
    default_conf.clk_flags = 0; // Avoid not initialized clk flags error
    default_conf.master.clk_speed = OTTO_I2C_FREQ;

    // Configure port
    ESP_ERROR_CHECK(i2c_param_config(i2c_portNum, &default_conf));

    // Install driver
    ESP_ERROR_CHECK(i2c_driver_install(i2c_portNum, default_conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));

    return ESP_OK;
}

extern "C" void app_main(void);

void app_main(void)
{
    // Test IMU Fusion
    //test_fusion();
    ESP_LOGI(TAG, "INIT");
    i2c_init(OTTO_I2C_PORT_NUM);
    ESP_LOGI(TAG, "LCD init");
    LCD1602 lcd(0b0100000);
    ESP_LOGI(TAG, "begin");
    ESP_ERROR_CHECK(lcd.begin(OTTO_I2C_PORT_NUM));
    ESP_LOGI(TAG, "RESET");
    ESP_ERROR_CHECK(lcd.reset());
    ESP_LOGI(TAG, "HOME");
    ESP_ERROR_CHECK(lcd.home());
    ESP_LOGI(TAG, "MOVE CURSOR");
    ESP_ERROR_CHECK(lcd.move_cursor(0, 0));
    ESP_LOGI(TAG, "WRITE A");
    ESP_ERROR_CHECK(lcd.write_char('A'));

    for(;;) {
        ESP_LOGI(TAG, "WRITE A");
        ESP_ERROR_CHECK(lcd.write_char('A'));
        vTaskDelay(1000 / portTICK_RATE_MS);
    }



}

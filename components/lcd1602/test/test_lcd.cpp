/************************************************************
*
* @file:      test_lcd.cpp
* @author:    Yuqing Fan
* @email:     fan230@purdue.edu
* @version:   v1.1.0
* @date:      09/14/2021
* @brief:     Sample program for lcd display
*
************************************************************/

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "lcd1602.h"
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

void test_lcd(void)
{
   
    ESP_LOGI(TAG, "INIT");
    i2c_init(OTTO_I2C_PORT_NUM);
    ESP_LOGI(TAG, "LCD init");
    LCD1602 lcd(0b0100000,0b0100111);
    ESP_LOGI(TAG, "begin");
    ESP_ERROR_CHECK(lcd.begin(OTTO_I2C_PORT_NUM));
}
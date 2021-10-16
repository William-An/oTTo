/************************************************************
*
* @file:      utility.cpp
* @author:    Weili An
* @email:     an107@purdue.edu
* @version:   v1.0.0
* @date:      09/15/2021
* @brief:     Utility functions for oTTo
*
************************************************************/

#include <inttypes.h>
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

// I2C Pin def
#define OTTO_I2C_SDA_PIN 27
#define OTTO_I2C_SCL_PIN 26
#define OTTO_I2C_FREQ 400000

// I2C Conf
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */

// Use port 0 of I2C
#define OTTO_I2C_PORT_NUM 0

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
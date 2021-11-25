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
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "otto.h"

// I2C Pin def
#define OTTO_I2C_SDA_PIN 27
#define OTTO_I2C_SCL_PIN 26
#define OTTO_I2C_FREQ 400000

// I2C Conf
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */

#define RX_BUF_SIZE 128
#define TX_BUF_SIZE 0

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
    ESP_ERROR_CHECK(
        i2c_driver_install(
            i2c_portNum, default_conf.mode, 
            I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));

    return ESP_OK;
}

esp_err_t uart_init(uint8_t uart_portNum) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(uart_portNum, RX_BUF_SIZE * 2, TX_BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(0, 1, 3, -1, -1));
    
    return ESP_OK;
}

/**
 * @brief Get mac addr of the current ESP32 over serial
 * 
 * @return esp_err_t 
 */
esp_err_t get_macAddr() {
    uint8_t mac[6] = {0};
    esp_err_t err = ESP_OK;

    err = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (err != ESP_OK)
        return err;
    ESP_LOGI("MAC", "Station MAC addr: %x:%x:%x:%x:%x:%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    err = esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    if (err != ESP_OK)
        return err;
    ESP_LOGI("MAC", "AP MAC addr: %x:%x:%x:%x:%x:%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    err = esp_read_mac(mac, ESP_MAC_BT);
    if (err != ESP_OK)
        return err;
    ESP_LOGI("MAC", "Bluetooth MAC addr: %x:%x:%x:%x:%x:%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    err = esp_read_mac(mac, ESP_MAC_ETH);
    if (err != ESP_OK)
        return err;
    ESP_LOGI("MAC", "Ethernet MAC addr: %x:%x:%x:%x:%x:%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    return err;
}
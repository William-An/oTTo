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
#include "otto.h"

// I2C Pin def
#define OTTO_I2C_SDA_PIN 27
#define OTTO_I2C_SCL_PIN 26
#define OTTO_I2C_FREQ 400000

#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

// I2C Conf
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */

QueueHandle_t _dataInQueue;
QueueHandle_t _dataOutQueue;
static uint8_t broadcast_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// For UART
static const int RX_BUF_SIZE = 1024;
static const int TX_BUF_SIZE = 0;

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
    ESP_ERROR_CHECK(uart_param_config(uart_portNum, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(0, 1, 3, -1, -1));
    
    return ESP_OK;
}

esp_err_t espnow_init(QueueHandle_t dataInQueue, QueueHandle_t dataOutQueue) {
    _dataInQueue = dataInQueue;
    _dataOutQueue = dataOutQueue;
    
    // Print mac addr
    get_macAddr();

    // Init nvs
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    // Init WiFi
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());

    // Init ESP-NOW
    ESP_LOGI("ESP-NOW", "Init ESP-NOW");
    ESP_ERROR_CHECK(esp_now_init());
    uint32_t esp_now_version;
    ESP_ERROR_CHECK(esp_now_get_version(&esp_now_version));
    ESP_LOGI("ESP-NOW", "ESP-NOW Version: %d", esp_now_version);

    // Register ESP-NOW recv callback to time the whole transcation time
    ESP_LOGI("ESP-NOW", "Register ESP-NOW recv callback func");
    ESP_ERROR_CHECK(esp_now_register_recv_cb(receiveDataCB));

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t* peer = (esp_now_peer_info_t*) malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE("Peer", "Malloc peer information fail");
        esp_now_deinit();
        // return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    // TODO Need to know the WiFi channel?
    peer->channel = 0;
    peer->ifidx = WIFI_IF_AP; // I wanna use 
    peer->encrypt = false;
    memcpy(peer->peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    return ESP_OK;
}

// todo: why receiveDataCB drops some packets from PC
// maybe broadcast address?
// maybe matlab side?
void receiveDataCB(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    // if (esp_now_is_peer_exist(mac_addr) == false) {
    //     esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    //     if (peer == NULL) {
    //         ESP_LOGE("Callback", "Malloc peer information fail");
    //         vTaskDelete(NULL);
    //     }
    //     memset(peer, 0, sizeof(esp_now_peer_info_t));
    //     peer->channel = 0;
    //     peer->ifidx = ESPNOW_WIFI_IF;
    //     peer->encrypt = false;
    //     memcpy(peer->peer_addr, mac_addr, ESP_NOW_ETH_ALEN);
    //     ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    //     free(peer);
    // }

    // store data into the queue
    // ESP_LOGI("ESP-NOW", "Sent message back with %d bytes", data_len);

    if (data_len == sizeof(Command_Data_Packet_ESP_NOW)) {
        Command_Data_Packet_ESP_NOW* packet = (Command_Data_Packet_ESP_NOW*) data;
        // todo: Need to also check timestamp and CRC
        Command_Data commandData = packet -> commandData;

        ESP_LOGI(__func__, "Comm receiver Task: received one packet: omega_left: %.2f", commandData.leftAngularVelo);
        if (xQueueSendToBack( _dataInQueue, &commandData, ( TickType_t ) 0 ) != pdPASS ) {
            ESP_LOGI(__func__, "Comm receiver Task: queue full");
        }
    } else {
        ESP_LOGI(__func__, "ELSE! Comm receiver Task: received one packet");
    }
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
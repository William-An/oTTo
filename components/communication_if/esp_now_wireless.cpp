#include <stdio.h>
#include "communication_if.h"
#include "esp_now_wireless.h"
#include "esp_log.h"
#include "communication_struct.h"
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"


#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

uint8_t broadcast_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
void receiveDataCB(const uint8_t *mac_addr, const uint8_t *data, int data_len);
QueueHandle_t _dataInQueue;
QueueHandle_t _dataOutQueue;

EspNowWireless :: EspNowWireless(bool debugMode, QueueHandle_t dataInQueue, QueueHandle_t dataOutQueue) {
    _debugMode = debugMode;
    _dataInQueue = dataInQueue;
    _dataOutQueue = dataOutQueue;
    
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

    // Print mac addr
    otto_get_macAddr();

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

}

int EspNowWireless :: sendData(const void* data, size_t size) {
    return esp_now_send(broadcast_mac, (uint8_t *) data, size);
}

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
    }
}

esp_err_t otto_get_macAddr() {
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

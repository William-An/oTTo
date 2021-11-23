  
#ifndef __ESP_NOW_WIRELESS__
#define __ESP_NOW_WIRELESS__

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "communication_if.h"
#include <stdio.h>
#include <math.h>
#include "freertos/task.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

class EspNowWireless : public CommunicationIF {
    public:

        EspNowWireless(bool debugMode, QueueHandle_t dataInQueue, QueueHandle_t dataOutQueue);
        ~EspNowWireless(){};

        int sendData(const void* data, size_t size);
        int receiveData(void *buf, uint32_t length, TickType_t ticks_to_wait);
        uint32_t calculateCRC();

};

esp_err_t otto_get_macAddr();
#endif
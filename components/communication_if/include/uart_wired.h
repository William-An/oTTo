  
#ifndef __UART_WIRED__
#define __UART_WIRED__
/************************************************************
*
* @file:      uart_wired.h
* @author:    Xin Du
* @email:     du201@purdue.edu
* @version:   v1.0.0
* @date:      09/15/2021
* @brief:     uart wired include file for oTTo
*
************************************************************/

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "communication_if.h"
#include <stdio.h>
#include <math.h>
#include "freertos/task.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

class UartWired : public CommunicationIF {
    public:
        UartWired(bool debugMode);
        ~UartWired(){};

        // inherited interface
        int sendData(const void* data, size_t size);
        int receiveData(void *buf, uint32_t length, TickType_t ticks_to_wait);
        uint32_t calculateCRC();

};

#endif
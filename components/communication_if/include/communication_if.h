  
#ifndef __COMMUNICATION_IF__
#define __COMMUNICATION_IF__
/************************************************************
*
* @file:      communication_if.h
* @author:    Xin Du
* @email:     du201@purdue.edu
* @version:   v1.0.0
* @date:      09/15/2021
* @brief:     communication_if.h include file for oTTo
*
************************************************************/

#include <stdbool.h>
#include "freertos/FreeRTOS.h"

class CommunicationIF {
    public:
        CommunicationIF() {};
        char myString[10] = "hello";
        int sendData(const char* data);

        // int receiveData(const char* logName, const char* data);
    protected:
        bool _debugMode = false;
};

#endif
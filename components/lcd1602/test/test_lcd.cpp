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

void test_lcd(void)
{
    printf("test lcd tasks\n");
    LCD1602 lcd(uint8_t addr);
    ESP_ERROR_CHECK(lcd.begin());
    lcd.write_string("write sth");



}
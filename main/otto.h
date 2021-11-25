#ifndef __OTTO_H__
#define __OTTO_H__

/************************************************************
*
* @file:      otto.h
* @author:    Weili An
* @email:     an107@purdue.edu
* @version:   v1.0.0
* @date:      09/21/2021
* @brief:     Top level oTTo configuration and initialization
*             tasks function declaration
*
************************************************************/

#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/uart.h"

// Use port 0 of I2C
#define OTTO_I2C_PORT_NUM   0
// Use port 0 of UART
#define OTTO_UART_PORT_NUM  UART_NUM_0
// IMU Update rate
#define OTTO_IMU_RATE_HZ    100  
// OTTO Task priority defs 
#define OTTO_INIT_TASK_PRI  20
#define OTTO_TASK_PRI_10    10
#define OTTO_TASK_PRI_9    9
#define OTTO_TASK_PRI_8     8
#define OTTO_TASK_PRI_7     7
#define OTTO_TASK_PRI_6     6
#define OTTO_TASK_PRI_4     4
#define OTTO_TASK_PRI_3     3
#define OTTO_TASK_PRI_2     2
#define OTTO_TASK_PRI_1     1
#define OTTO_IMU_TASK_PRI   OTTO_TASK_PRI_7
#define OTTO_MOTOR_TASK_PRI OTTO_TASK_PRI_7
#define OTTO_COMM_RECEIVER_TASK_PRI  OTTO_TASK_PRI_6
#define OTTO_COMM_SENDER_TASK_PRI  OTTO_TASK_PRI_9
#define OTTO_DISP_TASK_PRI  OTTO_TASK_PRI_8
// OTTO Queue configs
#define OTTO_DATA_IN_QUEUE_LEN  10
#define OTTO_DATA_OUT_QUEUE_LEN  20

#define ESP_NOW_MODE 1



// Peripheral control
esp_err_t i2c_init(uint8_t i2c_portNum);
esp_err_t uart_init(uint8_t uart_portNum);
esp_err_t get_macAddr();

// Initialization tasks
// Tasks should abort on failed circumstances

/**
 * @brief Top entry for all initialization
 * 
 */
void otto_init(void *param);

/**
 * @brief IMU subsystem init and task setup
 * 
 */
void imu_task(void *param);

/**
 * @brief Stepper motor subsytem init and task setup
 * 
 */
void motor_task(void *param);

/**
 * @brief Initialize communication interface 
 *        with host PC and listen over queue
 * 
 */
void comm_sender_task(void *param);

/**
 * @brief Initialize communication interface 
 *        with host PC and listen over queue
 * 
 */
void comm_receiver_task(void *param);

/**
 * @brief Display unit init and task
 * 
 */
void display_task(void *param);


#endif // !__OTTO_H__
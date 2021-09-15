#ifndef __MOTOR_STEPPER__
#define __MOTOR_STEPPER__

/************************************************************
*
* @file:      motor_stepper.h
* @author:    Ruichao Zhang
* @email:     zhan3147@purue.edu
* @version:   v1.0.0
* @date:      09/09/2021
* @brief:     NEMA17 stepper(1.8 degree) with A4988 Driver
*
************************************************************/

#include "motor_driver.h"
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

// motor info and others
typedef struct Nema17Config_t {
    float fullStep;
    float wheelRadius;
} Nema17Config_t;

// pin number on controller to the A4988 control pins
typedef struct MotorIOConfig_t {
    gpio_num_t dir;
    gpio_num_t step;
    gpio_num_t ms1;
    gpio_num_t ms2;
    gpio_num_t ms3;
    gpio_num_t en;
} MotorIOConfig_t;

// step size that 4988 support
typedef enum {
    FULL_STEP = 1,
    HALF_STEP = 2,
    QUARTER_STEP = 4,
    EIGHTH_STEP = 8,
    SIXTEENTH_STEP = 16
} Step_Size_t;

// position of the motor
typedef enum {
    REFERECNCE = 1,
    OPPOSITE = 0
} Position_t;

class A4988_Driver : public GenericMotorDiver {
    public:
        A4988_Driver(Step_Size_t steps, Position_t pos, Nema17Config_t nema17);
        A4988_Driver();

        // Inherit methods
        ~A4988_Driver(){};
        esp_err_t setContinuous(float omega);
        esp_err_t setFixed(float angle, float omega);
        void setPos(Position_t pos);
        esp_err_t halt();
        float getPower();
        float getMaxOmega();
        float getOmega();
        float getAngle();
        float getRestAngle();
        bool isContinuous();
        bool isHalted();
        esp_err_t configIO(MotorIOConfig_t motorIO);

    private:
        Nema17Config_t nema17;
        Step_Size_t steps = SIXTEENTH_STEP;
        Position_t pos = REFERECNCE;
        MotorIOConfig_t motorIO;
};

#endif
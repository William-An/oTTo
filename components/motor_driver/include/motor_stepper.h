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
#include "driver/gpio.h"

typedef struct Nema17Config_t {
    float fullStep;
    float wheelRadius;
} Nema17Config_t;

typedef struct motorIOConfig_t {
    int dir;
    int step;
    int ms1;
    int ms2;
    int ms3;
    int en;
} motorIOConfig_t;

class A4988_Driver : public GenericMotorDiver {
    public:
        A4988_Driver(float steps, bool pos);
        A4988_Driver();

        // Inherit methods
        ~A4988_Driver(){};
        void setContinuous(float omega);
        void setFixed(float angle, float omega);
        void setPos(bool pos);
        void halt();
        float getPower();
        float getMaxOmega();
        float getOmega();
        float getAngle();
        float getRestAngle();
        bool isContinuous();
        bool isHalted();
        esp_err_t configIO(motorIOConfig_t motorIO);

    private:
        float steps = 1 / 16;
        bool pos = 1;
        motorIOConfig_t motorIO;
};

#endif
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

typedef struct Nema17Config_t {
    float fullStep;
    float wheelRadius;
};

class A4988_Driver : public GenericMotorDiver {
    public:
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

    private:
        float steps = 1 / 16;
        bool pos = 1;
};

#endif
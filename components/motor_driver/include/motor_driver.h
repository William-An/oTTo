#ifndef __MOTOR_DRIVER__
#define __MOTOR_DRIVER__
/************************************************************
*
* @file:      motor_driver.h
* @author:    Weili An
* @email:     an107@purdue.edu
* @version:   v1.0.0
* @date:      09/04/2021
* @brief:     Top motor driver include file for oTTo
*
************************************************************/

#include <stdbool.h>
#include "freertos/FreeRTOS.h"


class GenericMotorDiver {
    public:
        // GenericMotorDiver();
        ~GenericMotorDiver();

        // Motor control methods
        // Continuous rotation, omega in deg/s
        virtual esp_err_t setContinuous(float omega);

        // Rotate for a fixed angle with velocity specified
        // angle in deg, omega in deg/s
        // virtual esp_err_t setFixed(float angle, float omega);

        // Halt the motor
        virtual esp_err_t halt();

        // Getter for motor status
        // virtual float getPower() = 0;
        // virtual float getMaxOmega() = 0;
        float getOmega() {
            return omega;
        }

        float getAngle() {
            return angle;
        }

        float getRestAngle() {
            return restAngle;
        }

        bool isContinuous() {
            return continuous;
        }

        bool isHalted() {
            return halted;
        }
    
    protected:
        // Current angular velocity (deg/s)
        // Postive: CCW
        // negative: CW
        float omega = 0;
        
        // Current shaft angle with respect to start
        float angle = 0;

        // Remaining angle (degree) for the motor to step
        // positive is how many degree must the motor rotate
        //   CCW with shaft towards you
        // negative is how many degree must the motor rotate
        //   CW with shaft faces you
        float restAngle = 0;

        // Determine if motor need to rotate continuously
        bool continuous = false;

        // Determine if motor is halted
        bool halted = true;
};

#endif
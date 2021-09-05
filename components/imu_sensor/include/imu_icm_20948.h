#ifndef __IMU_ICM_20948__
#define __IMU_ICM_20948__
/************************************************************
*
* @file:      imu_icm_20948.h
* @author:    Weili An
* @email:     an107@purdue.edu
* @version:   v1.0.0
* @date:      09/04/2021
* @brief:     TDK InvenSense ICM 20948 9DoF driver
*
************************************************************/

#include "imu_sensor.h"
#include <inttypes.h>

class ICM20948IMU : public GenericIMU {
    public:
        ICM20948IMU(uint32_t freq);

        // Inherit methods
        ~ICM20948IMU(){};
        void calibrate();
        void updateAccel();
        void updateGyro();
        void updateMagnet();

    private:
        // TODO Might need some member fields here
};



#endif // __IMU_ICM_20948__
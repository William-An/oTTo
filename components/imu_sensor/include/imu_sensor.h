#ifndef __IMU_SENSOR__
#define __IMU_SENSOR__
/************************************************************
*
* @file:      imu_sensor.h
* @author:    Weili an
* @email:     an107@purdue.edu
* @version:   v1.0.0
* @date:      09/04/2021
* @brief:     Top IMU sensor driver for oTTo, base class for
*             all.
*
************************************************************/

#include "Adafruit_AHRS_FusionInterface.h"
#include "Adafruit_AHRS.h"
#include <stdint.h>

typedef struct Vector3_t
{
    float x;
    float y;
    float z;
} Vector3_t;

typedef struct AngleVector3_t
{
    float roll;
    float pitch;
    float yaw;
} AngleVector3_t;

class GenericIMU {
    // typedef enum FusionAlgorithm_t {
    //     MADGWICK, 
    //     MAHONY, 
    //     NXPFUSION
    // } FusionAlgorithm_t;

    public:
        GenericIMU(uint32_t freq);
        virtual ~GenericIMU();

        // Sensor calibration method
        virtual void calibrate() = 0;

        // Sensor data update methods, should set the class members for each sensor
        virtual void updateAccel() = 0;
        virtual void updateGyro() = 0;
        virtual void updateMagnet() = 0;
        void updateAll() {
            updateAccel();
            updateGyro();
            updateMagnet();
        }

        // Run selected fusion algorithm to update euler angles
        void runFusion();

        // Getter methods
        AngleVector3_t getEulerAngles() {
            return eulerAngles;
        };

        Vector3_t getAccel() {
            return accelVec;
        }
        
        Vector3_t getGyro() {
            return gyroVec;
        }

        Vector3_t getMagnet() {
            return magnetVec;
        }

        uint32_t getUpdateFreq() {
            return updateFreq;
        }

    protected:
        // Fusion algorithm type, forbid change
        // during program execution
        // ! Not yet implemented due to concern of runtime type conversion on ESP
        // ! Used static one instead
        // FusionAlgorithm_t currFusionAlgorithm;

        // Comment out unused fusion algorithm
        //Adafruit_NXPSensorFusion filter; // slowest
        //Adafruit_Madgwick filter;  // faster than NXP
        Adafruit_Mahony filter;  // fastest/smalleset

        // Unit in g
        Vector3_t accelVec;
        
        // Unit in Degree per sec
        Vector3_t gyroVec;

        // Unit in uT
        Vector3_t magnetVec;

        // Sensor update frequency, in Hz
        uint32_t updateFreq;
    private:
        // Private as we let the base fusion algorithm
        // handle this
        AngleVector3_t eulerAngles;
};

#endif 

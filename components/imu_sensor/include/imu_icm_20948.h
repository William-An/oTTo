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

/**
    Software License Agreement (BSD License)

    Copyright (c) 2019 Bryan Siepert for Adafruit Industries
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <inttypes.h>
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "imu_sensor.h"
#include "imu_icm_20x.h"

#ifndef I2C_MASTER_TIMEOUT_MS
  #define I2C_MASTER_TIMEOUT_MS       1000
#endif

#define ICM20948_I2CADDR_DEFAULT 0x69 ///< ICM20948 default i2c address
#define ICM20948_MAG_ID 0x09          ///< The chip ID for the magnetometer

#define ICM20948_UT_PER_LSB 0.15 ///< mag data LSB value (fixed)

// Magnetometer AK09916, https://www.y-ic.es/datasheet/78/SMDSW.020-2OZ.pdf
#define AK09916_I2C_ADDR 0x0C
#define AK09916_WIA2 0x01  ///< Magnetometer
#define AK09916_ST1 0x10   ///< Magnetometer
#define AK09916_HXL 0x11   ///< Magnetometer
#define AK09916_HXH 0x12   ///< Magnetometer
#define AK09916_HYL 0x13   ///< Magnetometer
#define AK09916_HYH 0x14   ///< Magnetometer
#define AK09916_HZL 0x15   ///< Magnetometer
#define AK09916_HZH 0x16   ///< Magnetometer
#define AK09916_ST2 0x18   ///< Magnetometer
#define AK09916_CNTL2 0x31 ///< Magnetometer
#define AK09916_CNTL3 0x32 ///< Magnetometer

/** The accelerometer data range */
typedef enum {
  ICM20948_ACCEL_RANGE_2_G = 0,
  ICM20948_ACCEL_RANGE_4_G,
  ICM20948_ACCEL_RANGE_8_G,
  ICM20948_ACCEL_RANGE_16_G,
} icm20948_accel_range_t;

/** The gyro data range */
typedef enum {
  ICM20948_GYRO_RANGE_250_DPS = 0,
  ICM20948_GYRO_RANGE_500_DPS,
  ICM20948_GYRO_RANGE_1000_DPS,
  ICM20948_GYRO_RANGE_2000_DPS,
} icm20948_gyro_range_t;

/**
 * @brief Predefined sampling rate to divisor coeff 
 *        for gyroscope
 * 
 */
typedef enum {
  ICM20948_GYRO_RATE_1000_HZ = 0,
  ICM20948_GYRO_RATE_500_HZ = 1,
  ICM20948_GYRO_RATE_250_HZ = 3,
  ICM20948_GYRO_RATE_100_HZ = 10,
  ICM20948_GYRO_RATE_50_HZ = 21,
  ICM20948_GYRO_RATE_10_HZ = 109
} icm20948_gyro_rate_t;

/**
 * @brief Predefined sampling rate to divisor coeff 
 *        for accelerometer
 * 
 */
typedef enum {
  ICM20948_ACCEL_RATE_1000_HZ = 0,
  ICM20948_ACCEL_RATE_500_HZ = 1,
  ICM20948_ACCEL_RATE_250_HZ = 3,
  ICM20948_ACCEL_RATE_100_HZ = 10,
  ICM20948_ACCEL_RATE_50_HZ = 21,
  ICM20948_ACCEL_RATE_10_HZ = 111
} icm20948_accel_rate_t;

/**
 * @brief Data rates/modes for the embedded AsahiKASEI AK09916 3-axis
 * magnetometer
 *
 */
typedef enum {
  AK09916_MAG_DATARATE_SHUTDOWN = 0x0, ///< Stops measurement updates
  AK09916_MAG_DATARATE_SINGLE =
      0x1, ///< Takes a single measurement then switches to
           ///< AK09916_MAG_DATARATE_SHUTDOWN
  AK09916_MAG_DATARATE_10_HZ = 0x2,  ///< updates at 10Hz
  AK09916_MAG_DATARATE_20_HZ = 0x4,  ///< updates at 20Hz
  AK09916_MAG_DATARATE_50_HZ = 0x6,  ///< updates at 50Hz
  AK09916_MAG_DATARATE_100_HZ = 0x8, ///< updates at 100Hz
} ak09916_data_rate_t;

class ICM20948IMU : public GenericIMU {
    public:
        ICM20948IMU(uint32_t freq, uint8_t addr);

        // Inherit methods
        ~ICM20948IMU(){};
        void calibrate();
        esp_err_t updateAll();

        //? Use selfTest functionality
        void selfTest();

        // Configure the sensor
        esp_err_t initMag();
        esp_err_t writeGyroRange(icm20948_gyro_range_t);
        esp_err_t writeAccelRange(icm20948_accel_range_t);
        esp_err_t writeMagRange(uint8_t);

        esp_err_t setAccelRate(icm20948_accel_rate_t new_accel_divisor);
        esp_err_t setGyroRate(icm20948_gyro_rate_t new_gyro_divisor);
        esp_err_t setMagRate(ak09916_data_rate_t new_mag_rate);
        esp_err_t getMagId(uint8_t *dst);

        // Begin transcation with the sensor and setup basic stuffs
        esp_err_t begin(i2c_port_t portNum);

        // ICM I2C Master config
        esp_err_t setI2CBypass(bool bypass_i2c);
        esp_err_t configureI2CMaster();
        esp_err_t enableI2CMaster(bool enable_i2c_master);
        esp_err_t resetI2CMaster();

        // ICM AUX transcation
        esp_err_t auxillaryRegisterTransaction(bool read,
                                               uint8_t slvAddr,
                                               uint8_t regAddr,
                                               uint8_t data,
                                               uint8_t *dst);

        // ICM 20948 Register read and write functions
        // TODO Wrap ICM read and write
        esp_err_t readReg(uint8_t regAddr, uint8_t* data, size_t len);
        // Future: Multiple bytes write support
        esp_err_t writeReg(uint8_t regAddr, uint8_t data);
        esp_err_t maskWriteReg(uint8_t regAddr, uint8_t regMask,uint8_t data, bool clearMasked);
        esp_err_t readMagReg(uint8_t regAddr, uint8_t *data);
        esp_err_t writeMagReg(uint8_t regAddr, uint8_t data);
        
        // TODO  DEBUG FUNC
        void logRaw();

        // Mock testing methods
        void updateMockAccel(Vector3_t);
        void updateMockGyro(Vector3_t);
        void updateMockMagnet(Vector3_t);

        // Temperature sensor getter and setter
        float getTemp() {
          return temp;
        }

    private:
        esp_err_t setBank(uint8_t);
        esp_err_t reset();
        esp_err_t readRaw();
        esp_err_t processRaw();

        // Onboard temp sensor reading, in C
        float temp;

        // I2C info
        i2c_port_t portNum;
        uint8_t icm_20948_addr;

        // Raw data
        int16_t rawAccelX, rawAccelY, rawAccelZ;
        int16_t rawGyroX, rawGyroY, rawGyroZ;
        int16_t rawMagX, rawMagY, rawMagZ;
        int16_t rawTemp;

        // Sensor config
        icm20948_accel_range_t accelRange;
        icm20948_gyro_range_t  gyroRange;

};



#endif // __IMU_ICM_20948__
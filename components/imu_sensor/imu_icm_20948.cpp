#include "imu_icm_20948.h"

// TODO
ICM20948IMU::ICM20948IMU(uint32_t freq) : GenericIMU(freq) {

}

// TODO
void ICM20948IMU::calibrate() {
    
}

// TODO
void ICM20948IMU::updateAccel() {

}

// TODO
void ICM20948IMU::updateGyro() {

}

// TODO
void ICM20948IMU::updateMagnet() {

}

void ICM20948IMU::updateMockAccel(Vector3_t accel) {
    accelVec = accel;
}

void ICM20948IMU::updateMockGyro(Vector3_t gyro) {
    gyroVec = gyro;
}

void ICM20948IMU::updateMockMagnet(Vector3_t magnet) {
    magnetVec = magnet;
}

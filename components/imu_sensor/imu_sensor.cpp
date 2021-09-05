#include "imu_sensor.h"

// base IMU class constructor
GenericIMU::GenericIMU(uint32_t freq) {
    this->updateFreq = freq;
    filter.begin(freq);
}

GenericIMU::~GenericIMU() {

}


void GenericIMU::runFusion() {
    filter.update(gyroVec.x, gyroVec.y, gyroVec.z, 
                  accelVec.x, accelVec.y, accelVec.z,
                  magnetVec.x, magnetVec.y, magnetVec.z);
    eulerAngles.pitch = filter.getPitch();
    eulerAngles.roll = filter.getRoll();
    eulerAngles.yaw = filter.getYaw();
}
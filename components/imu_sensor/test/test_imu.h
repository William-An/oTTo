#ifndef __TEST_IMU_H__
#define __TEST_IMU_H__

#include "imu_sensor.h"
#include "imu_icm_20948.h"

void test_fusion(i2c_port_t portNum);

#endif // !__TEST_IMU_H__
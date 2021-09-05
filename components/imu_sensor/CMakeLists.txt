idf_component_register(SRCS "imu_sensor.cpp" "imu_icm_20948.cpp"
                    SRC_DIRS .
                    INCLUDE_DIRS "include"
                    REQUIRES adafruit_ahrs)

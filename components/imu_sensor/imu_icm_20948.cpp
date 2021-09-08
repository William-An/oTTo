#include "imu_icm_20948.h"

/**
 * @brief Construct a new ICM20948IMU::ICM20948IMU object
 * 
 * @param freq: frequency of sensor data
 */
ICM20948IMU::ICM20948IMU(uint32_t freq, uint8_t addr) : GenericIMU(freq) {
    icm_20948_addr = addr;
}

/**
 * @brief Begin I2C transcation with ICM 20948 and perform basic setup
 *        including wake it up and set initial sensor range
 * 
 * @param i2cPortNum    : Which i2C port to use
 * @param i2cConf       : I2C driver configuration
 * @return esp_err_t
 */
esp_err_t ICM20948IMU::begin(i2c_port_t i2cPortNum, i2c_config_t i2cConf) {
    // Configure the I2C port
    this->conf = i2cConf;
    this->portNum = i2cPortNum;
    i2c_param_config(i2cPortNum, &(this->conf));

    esp_err_t driverErr = i2c_driver_install(portNum, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (driverErr != ESP_OK)
        return driverErr;

    // TODO Wake up ICM and configure basic

    return ESP_ERR_NOT_SUPPORTED;
}

/**
 * @brief Wrapper for default begin process
 * 
 * @return esp_err_t 
 */
esp_err_t ICM20948IMU::begin() {
    //! TODO Change according to chip sheet
    const uint32_t ICM20948_I2C_SDA_PIN = 10;
    const uint32_t ICM20948_I2C_SCL_PIN = 11;
    const uint32_t ICM20948_I2C_FREQ = 100000;

    // Use port 0 of I2C
    const uint32_t ICM20948_I2C_PORT_NUM = 0;

    // Default ICM 20948 I2C config
    const i2c_config_t DEFAULT_I2C_CONFIG = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = ICM20948_I2C_SDA_PIN,
        .scl_io_num = ICM20948_I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        { ICM20948_I2C_FREQ },
        .clk_flags = 0
    };

    return begin(ICM20948_I2C_PORT_NUM, DEFAULT_I2C_CONFIG);
}

/**
 * @brief Read a register from the ICM 20948 to data with length len
 * 
 * @param regAddr:      Read register addr       
 * @param data:         Pointer to data buffer
 * @param len:          Number of byte to read
 * @return esp_err_t 
 */
esp_err_t ICM20948IMU::readReg(uint8_t regAddr, uint8_t* data, size_t len) {

    // Write then read sequence for ICM 20948, 
    // adapted from master branch of ESP-IDF
    esp_err_t err = ESP_OK;

    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    assert (handle != NULL);

    // Start the transcation
    err = i2c_master_start(handle);
    if (err != ESP_OK) {
        goto end;
    }

    // Device addr on I2C
    err = i2c_master_write_byte(handle, icm_20948_addr << 1 | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) {
        goto end;
    }

    // Put register addr on I2C
    err = i2c_master_write(handle, &regAddr, 1, true);
    if (err != ESP_OK) {
        goto end;
    }

    // Start frame
    err = i2c_master_start(handle);
    if (err != ESP_OK) {
        goto end;
    }

    // Issue read
    err = i2c_master_write_byte(handle, icm_20948_addr << 1 | I2C_MASTER_READ, true);
    if (err != ESP_OK) {
        goto end;
    }

    // Read and place data on data buf
    err = i2c_master_read(handle, data, len, I2C_MASTER_LAST_NACK);
    if (err != ESP_OK) {
        goto end;
    }

    // Send stop
    i2c_master_stop(handle);

    // Compile and send transcation
    err = i2c_master_cmd_begin(portNum, handle, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    // End transcation
end:
    i2c_cmd_link_delete(handle);
    return err;
}

/**
 * @brief Write to a register in ICM 20948 with data
 * 
 * @param regAddr:      Write register addr
 * @param data:         Data to write into
 * @return esp_err_t 
 */
esp_err_t ICM20948IMU::writeReg(uint8_t regAddr, uint8_t data) {
    uint8_t write_buf[2] = {regAddr, data};

    esp_err_t err = ESP_OK;

    // Create command link
    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    assert (handle != NULL);

    // Start frame
    err = i2c_master_start(handle);
    if (err != ESP_OK) {
        goto end;
    }

    // Device addr
    err = i2c_master_write_byte(handle, icm_20948_addr << 1 | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) {
        goto end;
    }

    // Put reg addr and data on line, wait for ack
    err = i2c_master_write(handle, write_buf, sizeof(write_buf), true);
    if (err != ESP_OK) {
        goto end;
    }

    // Stop frame
    i2c_master_stop(handle);

    // Start transcation
    err = i2c_master_cmd_begin(portNum, handle, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

end:
    i2c_cmd_link_delete(handle);
    return err;
}


// TODO
void ICM20948IMU::calibrate() {
    
}

// TODO
esp_err_t ICM20948IMU::updateAccel() {
    return ESP_ERR_NOT_SUPPORTED;
}

// TODO
esp_err_t ICM20948IMU::updateGyro() {
    return ESP_ERR_NOT_SUPPORTED;
}

// TODO
esp_err_t ICM20948IMU::updateMagnet() {
    return ESP_ERR_NOT_SUPPORTED;
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

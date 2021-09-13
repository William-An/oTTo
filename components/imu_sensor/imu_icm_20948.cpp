#include "imu_icm_20948.h"

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
    esp_err_t err = ESP_OK;
    uint8_t buf = 0;

    // Configure the I2C port
    this->conf = i2cConf;
    this->portNum = i2cPortNum;
    err = i2c_param_config(i2cPortNum, &(this->conf));
    if (err != ESP_OK)
        return err;
    
    // Install I2C Driver
    err = i2c_driver_install(portNum, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK)
        return err;

    // TODO Wake up ICM and configure basic

    // WHO AM I
    err = readReg(ICM20X_B0_WHOAMI, &buf, 1);
    if (err != ESP_OK)
        return err;
    ESP_LOGI("[ICM]", "WHO AM I = %X", buf);

    // Perform soft reset of chip
    err = reset();
    if (err != ESP_OK)
        return err;

    // Wake up from sleep mode
    buf = 1;
    err = writeReg(ICM20X_B0_PWR_MGMT_1, buf);

    // Write gyro and accel range to largest
    setBank(2);
    err = writeGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
    if (err != ESP_OK)
        return err;

    err = writeAccelRange(ICM20948_ACCEL_RANGE_16_G);
    if (err != ESP_OK)
        return err;
    setBank(0);

    // Set gyro and accel sampling rate
    err = writeGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
    if (err != ESP_OK)
        return err;
  

    return err;
}

/**
 * @brief Wrapper for default begin process
 * 
 * @return esp_err_t 
 */
esp_err_t ICM20948IMU::begin() {
    // Default ICM 20948 I2C config
    i2c_config_t default_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = ICM20948_I2C_SDA_PIN,
        .scl_io_num = ICM20948_I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };
    default_conf.master.clk_speed = ICM20948_I2C_FREQ;


    return begin(ICM20948_I2C_PORT_NUM, default_conf);
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


/**
 * @brief Write to a register in ICM 20948 with masking and option
 *        to OR the masked bits
 * 
 * @param regAddr 
 * @param regMask 
 * @param data
 * @param clearMasked:  Whether to clear the original bits in the 
 *                      mask positions 
 * @return esp_err_t 
 */
esp_err_t ICM20948IMU::maskWriteReg(uint8_t regAddr, uint8_t regMask,uint8_t data, bool clearMasked) {
    uint8_t buf = 0;
    esp_err_t err = ESP_OK;

    // Read the current reg value
    err = readReg(regAddr, &buf, 1);
    if (err != ESP_OK)
        return err;
    
    // Whether to OR the data and write to reg
    buf = (data & regMask) | (clearMasked ? (~regMask) & buf : buf);
    err = writeReg(regAddr, buf);
    if (err != ESP_OK)
        return err;

}

/**
 * @brief Configure Gyroscope range
 * 
 * @param range 
 * @return esp_err_t 
 */
esp_err_t ICM20948IMU::writeGyroRange(icm20948_gyro_range_t range) {
    uint8_t buf = 0;
    esp_err_t err = ESP_OK;
    gyroRange = range;

    // Set bank
    err = setBank(2);
    if (err != ESP_OK)
        return err;

    // Read the current reg value
    err = readReg(ICM20X_B2_GYRO_CONFIG_1, &buf, 1);
    if (err != ESP_OK)
        return err;
    
    // ORing the new value and write to reg
    buf |= range << 1;
    err = writeReg(ICM20X_B2_GYRO_CONFIG_1, buf);
    if (err != ESP_OK)
        return err;

    return err;
}

/**
 * @brief Configure accelerator range
 * 
 * @param range 
 * @return esp_err_t 
 */
// TODO Improve to multiple byte write
esp_err_t ICM20948IMU::writeAccelRange(icm20948_accel_range_t range) {
    uint8_t buf = 0;
    esp_err_t err = ESP_OK;
    accelRange = range;

    // Set bank
    err = setBank(2);
    if (err != ESP_OK)
        return err;

    // Read the current reg value
    err = readReg(ICM20X_B2_ACCEL_CONFIG_1, &buf, 1);
    if (err != ESP_OK)
        return err;
    
    // ORing the new value and write to reg
    buf |= range << 1;
    err = writeReg(ICM20X_B2_ACCEL_CONFIG_1, buf);
    if (err != ESP_OK)
        return err;

    return err;
}

/**
 * @brief Configure magnetometer range
 * 
 * @param range 
 * @return esp_err_t 
 */
esp_err_t ICM20948IMU::writeMagRange(uint8_t range) {
    return ESP_ERR_NOT_SUPPORTED;
}

/**
 * @brief Set the sampling rate for the accelerometter
 * 
 * @param new_accel_divisor: sampling rate =
 *          1.125 kHz/(1 + new_accel_divisor[11:0])
 */
esp_err_t ICM20948IMU::setAccelRateDivisor(icm20948_accel_rate_t new_accel_divisor) {
    uint8_t buf = 0;
    esp_err_t err = ESP_OK;

    // Set bank
    err = setBank(2);
    if (err != ESP_OK)
        return err;

    // Set the sampling rate MSB
    err = writeReg(ICM20X_B2_ACCEL_SMPLRT_DIV_1, new_accel_divisor >> 8);
    if (err != ESP_OK)
        return err;

    // Set the sampling rate LSB
    err = writeReg(ICM20X_B2_ACCEL_SMPLRT_DIV_1 + 1, new_accel_divisor & 0xFF);
    if (err != ESP_OK)
        return err;
    
    err = setBank(0);
    return err;
}

/**
 * @brief Set the sampling rate for the gyroscope
 * 
 * @param new_gyro_divisor: sampling rate =
 *          1.1 kHz/(1 + new_gyro_divisor)
 */
esp_err_t ICM20948IMU::setGyroRateDivisor(icm20948_gyro_rate_t new_gyro_divisor) {
    uint8_t buf = 0;
    esp_err_t err = ESP_OK;

    // Set bank
    err = setBank(2);
    if (err != ESP_OK)
        return err;

    // Set the sampling rate
    err = writeReg(ICM20X_B2_GYRO_SMPLRT_DIV, new_gyro_divisor);
    if (err != ESP_OK)
        return err;
    
    err = setBank(0);
    return err;
}

// TODO
void ICM20948IMU::calibrate() {
    
}

/**
 * @brief Read raw sensor bytes into raw data member fields
 * 
 * @return esp_err_t 
 */
esp_err_t ICM20948IMU::readRaw() {
    esp_err_t err = ESP_OK;
    // Adapted from Adafruit ICM 20X Arduino library
    const uint8_t BUF_SIZE = 14 + 9;
    uint8_t buf[BUF_SIZE];

    // Begin reading
    err = setBank(0);
    if (err != ESP_OK)
        return err;
    
    // Read
    err = readReg(ICM20X_B0_ACCEL_XOUT_H, buf, BUF_SIZE);
    if (err != ESP_OK)
        return err;
    
    // Save buffer data into raw member fields
    rawAccelX = buf[0] << 8 | buf[1];
    rawAccelY = buf[2] << 8 | buf[3];
    rawAccelZ = buf[4] << 8 | buf[5];

    rawGyroX = buf[6] << 8 | buf[7];
    rawGyroX = buf[8] << 8 | buf[9];
    rawGyroX = buf[10] << 8 | buf[11];

    rawTemp = buf[12] << 8 | buf[13];

    rawMagX = ((buf[16] << 8) |
             (buf[15] & 0xFF)); // Mag data is read little endian
    rawMagY = ((buf[18] << 8) | (buf[17] & 0xFF));
    rawMagZ = ((buf[20] << 8) | (buf[19] & 0xFF));

    return err;
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

/**
 * @brief Set register bank
 * 
 * @param bankNum 
 * @return esp_err_t 
 */
esp_err_t ICM20948IMU::setBank(uint8_t bankNum) {
    return writeReg(ICM20X_B0_REG_BANK_SEL, bankNum);
}

/**
 * @brief Reset the registers to default states
 * 
 * @return esp_err_t 
 */
esp_err_t ICM20948IMU::reset() {
    esp_err_t err = ESP_OK;
    uint8_t buf;
    
    // Write to reset IMU
    err = writeReg(ICM20X_B0_PWR_MGMT_1, 1);
    if (err != ESP_OK)
        return err;
    
    // Poll until reset is complete
    while (true) {
        // Wait 20 ms
        vTaskDelay(20 / portTICK_RATE_MS);

        // Read PWR MGMT 1 reg
        err = readReg(ICM20X_B0_PWR_MGMT_1, &buf, 1);
        if (err != ESP_OK)
            return err;
        else if (((buf >> 7) & 1) == 0) {   
            // RESET bit will be auto cleared on reset, 
            // which is the sign that the soft reset is done 
            break;
        }
    }

    return err;
}

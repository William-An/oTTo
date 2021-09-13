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
    buf = 0x1;
    err = writeReg(ICM20X_B0_PWR_MGMT_1, buf);

    // Write gyro and accel range to largest
    setBank(2);
    err = writeGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
    if (err != ESP_OK)
        return err;

    err = writeAccelRange(ICM20948_ACCEL_RANGE_16_G);
    if (err != ESP_OK)
        return err;

    // Set gyro and accel sampling rate
    err = setGyroRate(ICM20948_GYRO_RATE_100_HZ);
    if (err != ESP_OK)
        return err;
    
    err = setAccelRate(ICM20948_ACCEL_RATE_100_HZ);
    if (err != ESP_OK)
        return err;
    
    // Init mag, setup comm and sampling rate
    err = initMag();
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

esp_err_t ICM20948IMU::updateAll() {
    esp_err_t err = ESP_OK;
    err = readRaw();
    if (err != ESP_OK)
        return err;
    return processRaw();
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
    return writeReg(regAddr, buf);
}

/**
 * @brief Read 1 byte from magnetometer register 
 * 
 * @param regAddr 
 * @param data 
 * @return esp_err_t 
 */
esp_err_t ICM20948IMU::readMagReg(uint8_t regAddr, uint8_t *data) {
    return auxillaryRegisterTransaction(true, AK09916_I2C_ADDR, regAddr, 0, data);
}

/**
 * @brief Write 1 byte to magnetometer register
 * 
 * @param regAddr 
 * @param data 
 * @return esp_err_t 
 */
esp_err_t ICM20948IMU::writeMagReg(uint8_t regAddr, uint8_t data) {
    return auxillaryRegisterTransaction(false, AK09916_I2C_ADDR, regAddr, data, NULL);
}

/**
 * @brief Initialize ICM20498 Magnetometer for comm
 * 
 * @return esp_err_t 
 */
esp_err_t ICM20948IMU::initMag() {
    uint8_t buf;
    esp_err_t err = ESP_OK;

    // Setup I2C interface
    err = setI2CBypass(false);
    if (err != ESP_OK)
        return err;
    err = configureI2CMaster();
    if (err != ESP_OK)
        return err;
    err = enableI2CMaster(true);
    if (err != ESP_OK)
        return err;
    
    // Test if aux I2C starts
    bool aux_i2c_setup_failed = true;
    for (int i = 0; i < I2C_MASTER_RESETS_BEFORE_FAIL; i++) {
        err = getMagId(&buf);
        ESP_ERROR_CHECK(err);
        if (err != ESP_OK)
            return err;
        else if (buf != ICM20948_MAG_ID) {
            resetI2CMaster();
        } else {
            aux_i2c_setup_failed = false;
            break;
        }
    }

    if (aux_i2c_setup_failed)
        return ESP_ERR_TIMEOUT;

    // Log magid
    ESP_LOGI("[ICM]", "Mag ID: %X", buf);

    // Set init mag data rate
    err = setMagRate(AK09916_MAG_DATARATE_100_HZ);
    if (err != ESP_OK)
        return err;
    
    // Setup slv0 proxy
    err = setBank(3);
    if (err != ESP_OK)
        return err;

    // Set slv0 proxy addr
    // Read data rate same as gyroscope, see sec 11.2 of ICM datasheet 
    // OR 0x80 to enable read
    err = writeReg(ICM20X_B3_I2C_SLV0_ADDR, 0x80 | AK09916_I2C_ADDR);
    if (err != ESP_OK)
        return err;
    
    // Set slv0 proxy start reg addr
    err = writeReg(ICM20X_B3_I2C_SLV0_REG, AK09916_ST1);
    if (err != ESP_OK)
        return err;
    
    // Set to read 9 bytes (ST1, 6 measurement bytes, one dummy byte, ST2)
    err = writeReg(ICM20X_B3_I2C_SLV0_CTRL, 0x80 | 0x09);
    if (err != ESP_OK)
        return err;

    return setBank(0);
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
esp_err_t ICM20948IMU::setAccelRate(icm20948_accel_rate_t new_accel_divisor) {
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
esp_err_t ICM20948IMU::setGyroRate(icm20948_gyro_rate_t new_gyro_divisor) {
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

/**
 * @brief Set the sampling rate for magnetometer
 *        According to datasheet at sec 9.3, first set the
 *        AK09916 into power-down mode and wait for at least 100 us 
 *        to set the new magnetometer rate
 *        https://www.y-ic.es/datasheet/78/SMDSW.020-2OZ.pdf 
 *        
 *        Note for the actual reading data rate, it is determined by
 *        the the gyroscope ODR is set as specified in ICM 20948 Datasheet
 *        sec 11.1
 * 
 * @param new_mag_rate 
 * @return esp_err_t 
 */
esp_err_t ICM20948IMU::setMagRate(ak09916_data_rate_t new_mag_rate) {
    esp_err_t err = ESP_OK;
    
    // Put in shutdown mode
    err = writeMagReg(AK09916_CNTL2, AK09916_MAG_DATARATE_SHUTDOWN);
    if (err != ESP_OK)
        return err;
    
    // Wait for at least 1 ms
    vTaskDelay(1 / portTICK_RATE_MS);

    // Set sampling rate
    return writeMagReg(AK09916_CNTL2, new_mag_rate);
}


/**
 * @brief Get magnetometer id to test if comm is ready
 * 
 * @return esp_err_t 
 */
esp_err_t ICM20948IMU::getMagId(uint8_t *dst) {
    return auxillaryRegisterTransaction(true, 0x80 | AK09916_I2C_ADDR, AK09916_WIA2, 0x0, dst);
}

/**
 * @brief Whether to enable ICM20948 I2C Bypass
 * 
 * @param bypass_i2c 
 * @return esp_err_t 
 */
esp_err_t ICM20948IMU::setI2CBypass(bool bypass_i2c) {
    esp_err_t err = ESP_OK;

    // Set bank
    err = setBank(0);
    if (err != ESP_OK)
        return err;
    
    // Mask write
    err = maskWriteReg(ICM20X_B0_REG_INT_PIN_CFG, 0b10, bypass_i2c ? 0b10 : 0b0, true);
    return err;
}

/**
 * @brief Set the I2C clock rate for the auxillary I2C bus to 345.60kHz and
 *        disable repeated start
 * 
 * @return esp_err_t 
 */
esp_err_t ICM20948IMU::configureI2CMaster() {
    esp_err_t err = ESP_OK;

    err = setBank(3);
    if (err != ESP_OK)
        return err;

    err = writeReg(ICM20X_B3_I2C_MST_CTRL, 0x17);
    if (err != ESP_OK)
        return err;

    return setBank(0);
}

/**
 * @brief Enable or disable the I2C mastercontroller
 * 
 * @param enable_i2c_master 
 * @return esp_err_t 
 */
esp_err_t ICM20948IMU::enableI2CMaster(bool enable_i2c_master) {
    esp_err_t err = ESP_OK;

    err = setBank(0);
    if (err != ESP_OK)
        return err;

    return maskWriteReg(ICM20X_B0_USER_CTRL, 0x20, enable_i2c_master ? 0x20 : 0x0, true);
}

/**
 * @brief Reset the ICM I2C Master
 * 
 * @return esp_err_t 
 */
esp_err_t ICM20948IMU::resetI2CMaster() {
    uint8_t buf;
    esp_err_t err = ESP_OK;

    err = setBank(0);
    if (err != ESP_OK)
        return err;

    err = maskWriteReg(ICM20X_B0_USER_CTRL, 0b10, 0b10, true);
    if (err != ESP_OK)
        return err;
    
    // Polling
    while (true) {
        err = readReg(ICM20X_B0_USER_CTRL, &buf, 1);
        if (err != ESP_OK)
            return err;
        if ((buf >> 1) & 1) // rst bit not yet clear
            vTaskDelay(20 / portTICK_RATE_MS);
        else 
            break;
    }
    return err;
}

/**
 * @brief Write/read a single byte to a given register address 
 *        for an I2C slave device on the auxiliary I2C bus using
 *        the slv4 port on ICM20948
 * 
 * @param read 
 * @param slvAddr 
 * @param regAddr 
 * @param data 
 * @param dst
 * @return esp_err_t 
 */
esp_err_t ICM20948IMU::auxillaryRegisterTransaction(bool read,
    uint8_t slvAddr,
    uint8_t regAddr,
    uint8_t data, uint8_t *dst) {
    uint8_t buf;
    esp_err_t err;

    err = setBank(3);
    if (err != ESP_OK)
        return err;

    if (read) {
        slvAddr |= 0x80; // set high bit for read, presumably for multi-byte reads
    } else { 
        // Write, put value in DO (data out) register
        err = writeReg(ICM20X_B3_I2C_SLV4_DO, data);
        if (err != ESP_OK)
            return err;
    }

    // Put slave and slave's reg addr into control regs
    err = writeReg(ICM20X_B3_I2C_SLV4_ADDR, slvAddr);
    if (err != ESP_OK)
        return err;

    err = writeReg(ICM20X_B3_I2C_SLV4_REG, regAddr);
    if (err != ESP_OK)
        return err;

    // Issue transcation
    err = writeReg(ICM20X_B3_I2C_SLV4_CTRL, 0x80);
    if (err != ESP_OK)
        return err;
    
    // Poll to see the result
    setBank(0);
    // Try 100 times
    for (int i = 0; i < NUM_FINISHED_CHECKS; i++)
    {
        err = readReg(ICM20X_B0_I2C_MST_STATUS, &buf, 1);
        if (err != ESP_OK)
            return err;
        if ((buf >> 6) & 1) {
            // Transcation done
            setBank(3);
            if (read) {
                return readReg(ICM20X_B3_I2C_SLV4_DI, dst, 1);
            }
            return err;
        }
        vTaskDelay(10 / portTICK_RATE_MS);
    }
    return ESP_ERR_TIMEOUT;
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
    rawAccelX = (buf[0] << 8) | (buf[1] & 0xff);
    rawAccelY = (buf[2] << 8) | (buf[3] & 0xff);
    rawAccelZ = (buf[4] << 8) | (buf[5] & 0xff);

    rawGyroX = (buf[6] << 8) | (buf[7] & 0xff);
    rawGyroY = (buf[8] << 8) | (buf[9] & 0xff);
    rawGyroZ = (buf[10] << 8) | (buf[11] & 0xff);

    rawTemp = (buf[12] << 8) | (buf[13] & 0xff);

    rawMagX = ((buf[16] << 8) |
             (buf[15] & 0xFF)); // Mag data is read little endian
    rawMagY = ((buf[18] << 8) | (buf[17] & 0xFF));
    rawMagZ = ((buf[20] << 8) | (buf[19] & 0xFF));

    return err;
}

/**
 * @brief Process raw sensor data read
 *        and update structs
 * 
 * @return esp_err_t 
 */
esp_err_t ICM20948IMU::processRaw() {
    // Set scales
    float accelScale = 1.0;    
    float gyroScale = 1.0;

    if (gyroRange == ICM20948_GYRO_RANGE_250_DPS)
        gyroScale = 131.0;
    else if (gyroRange == ICM20948_GYRO_RANGE_500_DPS)
        gyroScale = 65.5;
    else if (gyroRange == ICM20948_GYRO_RANGE_1000_DPS)
        gyroScale = 32.8;
    else if (gyroRange == ICM20948_GYRO_RANGE_2000_DPS)
        gyroScale = 16.4;

    if (accelRange == ICM20948_ACCEL_RANGE_2_G)
        accelScale = 16384.0;
    else if (accelRange == ICM20948_ACCEL_RANGE_4_G)
        accelScale = 8192.0;
    else if (accelRange == ICM20948_ACCEL_RANGE_8_G)
        accelScale = 4096.0;
    else if (accelRange == ICM20948_ACCEL_RANGE_16_G)
        accelScale = 2048.0;

    // Set processed member vars
    // in unit of g
    accelVec.x = rawAccelX / accelScale;
    accelVec.y = rawAccelY / accelScale;
    accelVec.z = rawAccelZ / accelScale;

    // dps
    gyroVec.x = (rawGyroX / gyroScale);
    gyroVec.y = (rawGyroY / gyroScale);
    gyroVec.z = (rawGyroZ / gyroScale);

    // in uT
    magnetVec.x = rawMagX * ICM20948_UT_PER_LSB;
    magnetVec.y = rawMagY * ICM20948_UT_PER_LSB;
    magnetVec.z = rawMagZ * ICM20948_UT_PER_LSB;

    temp = (rawTemp / 333.87) + 21.0;

    return ESP_OK;
}

void ICM20948IMU::logRaw() {
    ESP_LOGI("[RAW]", "RAW Accel %d %d %d", rawAccelX, rawAccelY, rawAccelZ);
    ESP_LOGI("[RAW]", "RAW Gyro %d %d %d", rawGyroX, rawGyroY, rawGyroZ);
    ESP_LOGI("[RAW]", "RAW Mag %d %d %d", rawMagX, rawMagY, rawMagZ);
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
    bankNum = bankNum << 4;
    return maskWriteReg(ICM20X_B0_REG_BANK_SEL, 0b00110000, bankNum, true);
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

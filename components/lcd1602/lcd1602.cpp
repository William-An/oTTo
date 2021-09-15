#include <stdio.h>
#include "lcd1602.h"

LCD1602::LCD1602(uint8_t addr) {
    mcp23008_addr = addr;
}

/**
 * @brief Default start methods for LCD1602
 * 
 * @return esp_err_t 
 */
esp_err_t LCD1602::begin() {
    return ESP_OK;
}

esp_err_t begin(i2c_port_t, i2c_config_t);

// Display control
esp_err_t LCD1602::reset() {
    return ESP_OK;
}

esp_err_t LCD1602::clear() {
    return ESP_OK;
}

esp_err_t LCD1602::enable_display(bool en) {
    return ESP_OK;
}

esp_err_t LCD1602::enable_cursor(bool en) {
    return ESP_OK;
}

esp_err_t LCD1602::enable_blink(bool en) {
    return ESP_OK;
}


// Cursor control
esp_err_t LCD1602::home() {
    return ESP_OK;
}

esp_err_t LCD1602::move_cursor(uint8_t col, uint8_t row) {
    return ESP_OK;
}


// Direction control
esp_err_t LCD1602::set_left_to_right(bool en) {
    return ESP_OK;
}

esp_err_t LCD1602::enable_auto_scroll(bool en) {
    return ESP_OK;
}

esp_err_t LCD1602::set_scroll(LCD1602_dir_t dir) {
    return ESP_OK;
}

esp_err_t LCD1602::move_cursor(LCD1602_dir_t dir) {
    return ESP_OK;
}

esp_err_t LCD1602::define_char(i2c_lcd1602_custom_index_t index, const uint8_t pixelmap[]) {
    return ESP_OK;
}

esp_err_t LCD1602::write_char(uint8_t chr) {
    return ESP_OK;
}

esp_err_t LCD1602::write_string(const char *str) {
    return ESP_OK;
}

esp_err_t LCD1602::readReg(uint8_t regAddr, uint8_t* data, size_t len) {

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
    err = i2c_master_write_byte(handle, mcp23008_addr << 1 | I2C_MASTER_WRITE, true);
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
    err = i2c_master_write_byte(handle, mcp23008_addr << 1 | I2C_MASTER_READ, true);
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
    err = i2c_master_cmd_begin(i2c_portNum, handle, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    // End transcation
end:
    i2c_cmd_link_delete(handle);
    return err;
}

// Future: Multiple bytes write support
esp_err_t LCD1602::writeReg(uint8_t regAddr, uint8_t data) {
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
    err = i2c_master_write_byte(handle, mcp23008_addr << 1 | I2C_MASTER_WRITE, true);
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
    err = i2c_master_cmd_begin(i2c_portNum, handle, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

end:
    i2c_cmd_link_delete(handle);
    return err;
}

esp_err_t LCD1602::maskWriteReg(uint8_t regAddr, uint8_t regMask,uint8_t data, bool clearMasked) {
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

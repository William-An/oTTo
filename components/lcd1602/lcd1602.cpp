#include <stdio.h>
#include "lcd1602.h"

// Adapted from https://github.com/DavidAntliff/esp32-i2c-lcd1602
/*
 * MIT License
 *
 * Copyright (c) 2018 David Antliff
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file
 *
 * @brief
 * The LCD1602 controller is an HD44780-compatible controller that normally operates
 * via an 8-bit or 4-bit wide parallel bus.
 *
 * https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 *
 * The LCD1602 controller is connected to a PCF8574A I/O expander via the I2C bus.
 * Only the top four bits are connected to the controller's data lines. The lower
 * four bits are used as control lines:
 *
 *   - B7: data bit 7
 *   - B6: data bit 6
 *   - B5: data bit 5
 *   - B4: data bit 4
 *   - B3: backlight (BL): off = 0, on = 1
 *   - B2: enable (EN): change from 1 to 0 to clock data into controller
 *   - B1: read/write (RW): write = 0, read = 1
 *   - B0: register select (RS): command = 0, data = 1
 *
 * Therefore to send a command byte requires the following operations:
 *
 *   // First nibble:
 *   val = command & 0xf0              // extract top nibble
 *   val |= 0x04                       // RS = 0 (command), RW = 0 (write), EN = 1
 *   i2c_write_byte(i2c_address, val)
 *   sleep(2ms)
 *   val &= 0xfb                       // EN = 0
 *   i2c_write_byte(i2c_address, val)
 *
 *   // Second nibble:
 *   val = command & 0x0f              // extract bottom nibble
 *   val |= 0x04                       // RS = 0 (command), RW = 0 (write), EN = 1
 *   i2c_write_byte(i2c_address, val)
 *   sleep(2ms)
 *   val &= 0xfb                       // EN = 0
 *   i2c_write_byte(i2c_address, val)
 *
 * Sending a data byte is very similar except that RS = 1 (data)
 *
 * When the controller powers up, it defaults to:
 *
 *   - display cleared
 *   - 8-bit interface, 1 line display, 5x8 dots per character
 *   - increment by 1 set
 *   - no shift
 *
 * The controller must be set to 4-bit operation before proper communication can commence.
 * The initialisation sequence for 4-bit operation is:
 *
 *   0. wait > 15ms after Vcc rises to 4.5V, or > 40ms after Vcc rises to 2.7V
 *   1. send nibble 0x03     // select 8-bit interface
 *   2. wait > 4.1ms
 *   3. send nibble 0x03     // select 8-bit interface again
 *   4. wait > 100us
 *   5. send command 0x32    // select 4-bit interface
 *   6. send command 0x28    // set 2 lines and 5x7(8?) dots per character
 *   7. send command 0x0c    // display on, cursor off
 *   8. send command 0x06    // move cursor right when writing, no scroll
 *   9. send command 0x80    // set cursor to home position (row 1, column 1)
 */

LCD1602::LCD1602(uint8_t addr) {
    mcp23008_addr = addr;
}

/**
 * @brief Start communication with the LCD1602, configure
 *        necessary regs on MCP23008 for proper actions
 * 
 * @param portNum I2C port to use, must be configured 
 *        and have driver installed
 * @return esp_err_t 
 */
esp_err_t LCD1602::begin(i2c_port_t portNum) {
    esp_err_t err = ESP_OK;
    this->i2c_portNum = portNum;

    // Set MCP GPIO to output mode, default is 0xFF
    err = writeReg(MCP23008_REG_IODIR, 0);
    ESP_ERROR_CHECK(err);
    if (err != ESP_OK)
        return err;

    // Set MCP GPIO to output mode, default is 0xFF
    err = writeReg(MCP23008_REG_IPOL, 0);
    ESP_ERROR_CHECK(err);
    if (err != ESP_OK)
        return err;

    //initialization of lcd display
    //wait for stabilization 500ms
    // vTaskDelay(500 / portTICK_RATE_MS);
    // //fuction set
    // err = writeReg(0b0010,0);
    // if (err != ESP_OK)
    //     return err;
    // //check busy flag
    // // !use delay instead of checking
    // vTaskDelay(500 / portTICK_RATE_MS);
    // //Display on/off control
    // err = writeReg(0b0000,0);
    // if (err != ESP_OK)
    //     return err;
    // err = writeReg(0b)
    // //check busy flag
    // // !use delay instead of checking
    // vTaskDelay(500 / portTICK_RATE_MS);
    // //Display clear
    // err = writeReg(0b0000,0);
    // if (err != ESP_OK)
    //     return err;
    // //Return home
    // err = writeReg();

    ESP_LOGI("Begin", "Set all to 0s");

    // Set default outputs to all 0s
    err = writeReg(MCP23008_REG_GPIO, 0x00);
    if (err != ESP_OK)
        return err;
    ESP_ERROR_CHECK(err);
    
    ESP_LOGI("Begin", "config");
    ets_delay_us(DELAY_POWER_ON);
    err = write_top_nibble(0b0011 << 4);
    ESP_ERROR_CHECK(err);
    ets_delay_us(DELAY_INIT_1);
    err = write_top_nibble(0b0011 << 4);
    ESP_ERROR_CHECK(err);
    ets_delay_us(DELAY_INIT_3);
    err = write_top_nibble(0b0011 << 4);
    ESP_ERROR_CHECK(err);
    
    // Function set
    err = write_top_nibble(0b0010 << 4);
    ESP_ERROR_CHECK(err);
    err = write_top_nibble(0b0010 << 4);
    ESP_ERROR_CHECK(err);
    err = write_top_nibble(0b1000 << 4);
    ESP_ERROR_CHECK(err);

    // Display on/off control
    err = write_top_nibble(FLAG_DISPLAY_CONTROL_DISPLAY_OFF << 4);
    ESP_ERROR_CHECK(err);
    err = write_top_nibble(FLAG_DISPLAY_CONTROL_DISPLAY_ON << 4);
    ESP_ERROR_CHECK(err);

    // Display clear
    err = write_top_nibble(0b0000 << 4);
    ESP_ERROR_CHECK(err);
    err = write_top_nibble(0b0001 << 4);
    ESP_ERROR_CHECK(err);

    // Entry mode set
    err = write_top_nibble(0b0000 << 4);
    ESP_ERROR_CHECK(err);
    err = write_top_nibble(0b0111 << 4);
    ESP_ERROR_CHECK(err);

    // Write an A
    write_string("OTTO Test");




    // ESP_LOGI("Begin", "Set all to 0 in 3s");
    // vTaskDelay(3000 / portTICK_RATE_MS);
    
    // // Set default outputs to all 0s
    // err = writeReg(MCP23008_REG_GPIO, 0);
    // if (err != ESP_OK)
    //     return err;

    // ESP_LOGI("Begin", "Set all to 1 in 3s");
    ESP_LOGI("Begin", "Done");
    vTaskDelay(300000 / portTICK_RATE_MS);

    // // Set default outputs to all 0s
    // err = writeReg(MCP23008_REG_GPIO, 0xff);
    // if (err != ESP_OK)
    //     return err;

    // // Wait at least 40ms after power rises above 2.7V before sending commands.
    // // vTaskDelay(DELAY_POWER_ON / portTICK_RATE_MS);
    // vTaskDelay(5000 / portTICK_RATE_MS);

    // Reset device
    ESP_LOGI("Begin", "Reseting chip");
    return reset();
}

// Display control
esp_err_t LCD1602::reset() {
    esp_err_t err = ESP_OK;
    esp_err_t first_err = ESP_OK;
    esp_err_t last_err = ESP_OK;

    
    // Set default outputs to all 0s except enable
    err = writeReg(MCP23008_REG_GPIO, 0);
    if (err != ESP_OK)
        return err;
    
    vTaskDelay(1000 / portTICK_RATE_MS);
    
    // Sync function for 4bit interface
    ESP_LOGI(__func__, "Sync");
    vTaskDelay(500 / portTICK_RATE_MS);
    write_top_nibble(0);
    vTaskDelay(500 / portTICK_RATE_MS);
    write_top_nibble(0);
    vTaskDelay(500 / portTICK_RATE_MS);
    write_top_nibble(0);
    vTaskDelay(500 / portTICK_RATE_MS);
    write_top_nibble(0);
    vTaskDelay(500 / portTICK_RATE_MS);
    write_top_nibble(0);
    vTaskDelay(500 / portTICK_RATE_MS);

    // Select 4-bit mode
    ESP_LOGI(__func__, "Set 4 bit");
    write_top_nibble(0x2 << 4);
    vTaskDelay(10 / portTICK_RATE_MS);
    write_top_nibble(0x2 << 4);
    vTaskDelay(10 / portTICK_RATE_MS);
    write_top_nibble(0b1000 << 4);
    vTaskDelay(10 / portTICK_RATE_MS);
    vTaskDelay(100000 / portTICK_RATE_MS);

    // Delay to simulate checking busy flag 
    vTaskDelay(DELAY_INIT_1 / portTICK_RATE_MS);

    // Display on/off control
    ESP_LOGI(__func__, "Display off");
    write_top_nibble(0x0 << 4);
    vTaskDelay(10 / portTICK_RATE_MS);
    // Enable cursor and all stuffs 
    write_top_nibble(0b1000 << 4);

    // Delay to simulate checking busy flag 
    vTaskDelay(DELAY_INIT_1 / portTICK_RATE_MS);

    // Display clear 
    ESP_LOGI(__func__, "Clear");
    write_top_nibble(0x0 << 4);
    write_top_nibble(0x1 << 4);

    // Delay to simulate checking busy flag 
    vTaskDelay(DELAY_INIT_1 / portTICK_RATE_MS);

    // Home 
    ESP_LOGI(__func__, "Home");
    write_top_nibble(0x0 << 4);
    write_top_nibble(0x2 << 4);

    // Delay to simulate checking busy flag 
    vTaskDelay(DELAY_INIT_1 / portTICK_RATE_MS);

    // Entry mode set
    ESP_LOGI(__func__, "Entry mode");
    write_top_nibble(0x0 << 4);
    write_top_nibble(0x06 << 4);

    // Delay to simulate checking busy flag 
    vTaskDelay(DELAY_INIT_1 / portTICK_RATE_MS);

    // Enable cursor and blink
    ESP_LOGI(__func__, "Turns on ");
    write_top_nibble(0x0 << 4);
    write_top_nibble(0xC << 4);

    // Delay to simulate checking busy flag 
    vTaskDelay(DELAY_INIT_1 / portTICK_RATE_MS);

    // Write an A
    ESP_LOGI(__func__, "Write something");
    write_top_nibble(0b0100 << 4 | 0b01);
    write_top_nibble(0b0001 << 4 | 0b01);

    // Delay to simulate checking busy flag 
    vTaskDelay(DELAY_INIT_1 / portTICK_RATE_MS);

    ESP_LOGI(__func__, "Finish reseting");
    for(;;);
    return err;
}

esp_err_t LCD1602::clear() {
    esp_err_t err = ESP_FAIL;

    err = write_command(COMMAND_CLEAR_DISPLAY);
    if (err == ESP_OK)
    {
        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    return ESP_OK;
}


esp_err_t LCD1602::enable_display(bool en) {
    esp_err_t err = ESP_FAIL;

    err = write_command(en?COMMAND_DISPLAY_CONTROL | FLAG_DISPLAY_CONTROL_DISPLAY_ON:COMMAND_DISPLAY_CONTROL |~FLAG_DISPLAY_CONTROL_DISPLAY_ON );
    if (err == ESP_OK)
    {
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
    
    return ESP_OK;
}

esp_err_t LCD1602::enable_cursor(bool en) {
    esp_err_t err = ESP_FAIL;

    err = write_command(en?COMMAND_DISPLAY_CONTROL | FLAG_DISPLAY_CONTROL_CURSOR_ON:COMMAND_DISPLAY_CONTROL | ~FLAG_DISPLAY_CONTROL_CURSOR_ON);
    if (err == ESP_OK)
    {
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
     

    return ESP_OK;
}

esp_err_t LCD1602::enable_blink(bool en) {
    esp_err_t err = ESP_FAIL;
//??
    err = write_command(en?COMMAND_DISPLAY_CONTROL | FLAG_DISPLAY_CONTROL_BLINK_ON:COMMAND_DISPLAY_CONTROL | ~FLAG_DISPLAY_CONTROL_BLINK_ON);
    if (err == ESP_OK)
    {
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
     

    return ESP_OK;
}


// Cursor control
esp_err_t LCD1602::home() {
    esp_err_t err = ESP_FAIL;
    
    err = write_command(COMMAND_RETURN_HOME);
    if (err == ESP_OK)
    {
        ets_delay_us(DELAY_RETURN_HOME);
    }
    return ESP_OK;
}

esp_err_t LCD1602::move_cursor(uint8_t col, uint8_t row) {
    esp_err_t err = ESP_FAIL;
    const int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
    if (row > 2)
    {
        row = 2;
    }
    if (col > 16)
    {
        col = 16;
    }
    err = write_command(COMMAND_SET_DDRAM_ADDR | (col + row_offsets[row]));
    
    return ESP_OK;
}

// Direction control
esp_err_t LCD1602::set_left_to_right(bool en) {
    esp_err_t err = ESP_FAIL;

    err = write_command(COMMAND_ENTRY_MODE_SET | FLAG_ENTRY_MODE_SET_ENTRY_INCREMENT);

    return ESP_OK;
}

esp_err_t LCD1602::enable_auto_scroll(bool en) {
    esp_err_t err = ESP_FAIL;   
    
    err = write_command(en?COMMAND_ENTRY_MODE_SET | FLAG_ENTRY_MODE_SET_ENTRY_SHIFT_ON: COMMAND_ENTRY_MODE_SET | ~FLAG_ENTRY_MODE_SET_ENTRY_SHIFT_ON);

    return ESP_OK;
}

esp_err_t LCD1602::set_scroll(LCD1602_dir_t dir) {
    esp_err_t err = ESP_FAIL;
    if (dir == LEFT){
        err = write_command(COMMAND_SHIFT | FLAG_SHIFT_MOVE_DISPLAY | FLAG_SHIFT_MOVE_LEFT);
    }
    if(dir == RIGHT){
        err = write_command(COMMAND_SHIFT | FLAG_SHIFT_MOVE_DISPLAY | FLAG_SHIFT_MOVE_RIGHT);
    }



    return ESP_OK;
}

esp_err_t LCD1602::move_cursor(LCD1602_dir_t dir) {
    esp_err_t err = ESP_FAIL;
    if (dir == LEFT){
        err = write_command(COMMAND_SHIFT | FLAG_SHIFT_MOVE_CURSOR | FLAG_SHIFT_MOVE_RIGHT);
    }
    if(dir == RIGHT){
        err = write_command(COMMAND_SHIFT | FLAG_SHIFT_MOVE_CURSOR | FLAG_SHIFT_MOVE_LEFT);
    }

    return ESP_OK;
}

esp_err_t LCD1602::define_char(i2c_lcd1602_custom_index_t index, const uint8_t pixelmap[]) {
    
    esp_err_t err = ESP_FAIL;
    index = (i2c_lcd1602_custom_index_t)((int)index & 0x07);  // only the first 8 indexes can be used for custom characters
    err = write_command(COMMAND_SET_CGRAM_ADDR | (index << 3));
    for (int i = 0; err == ESP_OK && i < 8; ++i)
    {
        err = write_data(pixelmap[i]);
    }
    return ESP_OK;
}

esp_err_t LCD1602::write_char(uint8_t chr) {
    esp_err_t err = ESP_FAIL;
    err = write_data(chr);

    return ESP_OK;
}

esp_err_t LCD1602::write_string(const char *str) {

    esp_err_t err = ESP_OK;
    for (int i = 0; err == ESP_OK && str[i]; ++i)
    {
        err = write_data(str[i]);
    }
    return ESP_OK;
}

// Refer to ICM 20948 for documentation
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

/**
 * @brief clock data from expander to LCD by 
 *        causing a falling edge on Enable
 * 
 * @param data 
 * @return esp_err_t 
 */
esp_err_t LCD1602::strobe_enable(uint8_t data) {
    esp_err_t err1 = writeReg(MCP23008_REG_GPIO, data & ~FLAG_ENABLE);
    ets_delay_us(1);
    
    esp_err_t err2 = writeReg(MCP23008_REG_GPIO, data | FLAG_ENABLE);
    ets_delay_us(1);

    esp_err_t err3 = writeReg(MCP23008_REG_GPIO, data & ~FLAG_ENABLE);
    ets_delay_us(1);

    return err1 == ESP_OK ? (ESP_OK ? err2 : err1):err3;
}

/**
 * @brief Send top nibble to the LCD controller
 * 
 * @param data 
 * @return esp_err_t 
 */
esp_err_t LCD1602::write_top_nibble(uint8_t data) {
    ESP_LOGD("LCD1602", "write_top_nibble 0x%02x", data);
    esp_err_t err1 = writeReg(MCP23008_REG_GPIO, data);
    esp_err_t err2 = strobe_enable(data);

    return err1 == ESP_OK ? err2 : err1;
}

/**
 * @brief Write a whole byte to LCD with register select
 * 
 * @param value 
 * @param register_select_flag 
 * @return esp_err_t 
 */
esp_err_t LCD1602::write(uint8_t value, uint8_t register_select_flag) {
    ESP_LOGD("LCD1602", "write 0x%02x | 0x%02x", value, register_select_flag);
    esp_err_t err1 = write_top_nibble((value & 0xf0) | register_select_flag);
    esp_err_t err2 = write_top_nibble(((value & 0x0f) << 4) | register_select_flag);
    return err1 == ESP_OK ? err2 : err1;
}

/**
 * @brief Write a command to LCD1602
 * 
 * @param command 
 * @return esp_err_t 
 */
esp_err_t LCD1602::write_command(uint8_t command) {
    ESP_LOGD("LCD1602", "write_command 0x%02x", command);
    return write(command, FLAG_RS_COMMAND);
}

/**
 * @brief Write data to LCD1602
 * 
 * @param data 
 * @return esp_err_t 
 */
esp_err_t LCD1602::write_data(uint8_t data) {
    ESP_LOGD("LCD1602", "write_data 0x%02x", data);
    return write(data, FLAG_RS_DATA);
}

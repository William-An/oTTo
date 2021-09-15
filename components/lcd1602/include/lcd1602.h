#ifndef __LCD1602_H__
#define __LCD1602_H__

/************************************************************
*
* @file:      lcd1602.h
* @author:    Weili An, Yuqing Fan
* @email:     {an107, fan230}@purdue.edu
* @version:   v1.0.0
* @date:      09/15/2021
* @brief:     LLCD 1602 Driver through MCP23008
*
************************************************************/

#include <inttypes.h>
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

// todo Need to refer to the sources
// Special characters for ROM Code A00

// Use the second set (0bxxxx1xxx) to avoid placing the null character within a string
#define I2C_LCD1602_CHARACTER_CUSTOM_0     0b00001000   ///< User-defined custom symbol in index 0
#define I2C_LCD1602_CHARACTER_CUSTOM_1     0b00001001   ///< User-defined custom symbol in index 1
#define I2C_LCD1602_CHARACTER_CUSTOM_2     0b00001010   ///< User-defined custom symbol in index 2
#define I2C_LCD1602_CHARACTER_CUSTOM_3     0b00001011   ///< User-defined custom symbol in index 3
#define I2C_LCD1602_CHARACTER_CUSTOM_4     0b00001100   ///< User-defined custom symbol in index 4
#define I2C_LCD1602_CHARACTER_CUSTOM_5     0b00001101   ///< User-defined custom symbol in index 5
#define I2C_LCD1602_CHARACTER_CUSTOM_6     0b00001110   ///< User-defined custom symbol in index 6
#define I2C_LCD1602_CHARACTER_CUSTOM_7     0b00001111   ///< User-defined custom symbol in index 7

#define I2C_LCD1602_CHARACTER_ALPHA        0b11100000   ///< Lower-case alpha symbol
#define I2C_LCD1602_CHARACTER_BETA         0b11100010   ///< Lower-case beta symbol
#define I2C_LCD1602_CHARACTER_THETA        0b11110010   ///< Lower-case theta symbol
#define I2C_LCD1602_CHARACTER_PI           0b11110111   ///< Lower-case pi symbol
#define I2C_LCD1602_CHARACTER_OMEGA        0b11110100   ///< Upper-case omega symbol
#define I2C_LCD1602_CHARACTER_SIGMA        0b11110110   ///< Upper-case sigma symbol
#define I2C_LCD1602_CHARACTER_INFINITY     0b11110011   ///< Infinity symbol
#define I2C_LCD1602_CHARACTER_DEGREE       0b11011111   ///< Degree symbol
#define I2C_LCD1602_CHARACTER_ARROW_RIGHT  0b01111110   ///< Arrow pointing right symbol
#define I2C_LCD1602_CHARACTER_ARROW_LEFT   0b01111111   ///< Arrow pointing left symbol
#define I2C_LCD1602_CHARACTER_SQUARE       0b11011011   ///< Square outline symbol
#define I2C_LCD1602_CHARACTER_DOT          0b10100101   ///< Centred dot symbol
#define I2C_LCD1602_CHARACTER_DIVIDE       0b11111101   ///< Division sign symbol
#define I2C_LCD1602_CHARACTER_BLOCK        0b11111111   ///< 5x8 filled block

/**
 * @brief Enum of valid indexes for definitions of user-defined characters.
 */
typedef enum
{
    I2C_LCD1602_INDEX_CUSTOM_0 = 0,                     ///< Index of first user-defined custom symbol
    I2C_LCD1602_INDEX_CUSTOM_1,                         ///< Index of second user-defined custom symbol
    I2C_LCD1602_INDEX_CUSTOM_2,                         ///< Index of third user-defined custom symbol
    I2C_LCD1602_INDEX_CUSTOM_3,                         ///< Index of fourth user-defined custom symbol
    I2C_LCD1602_INDEX_CUSTOM_4,                         ///< Index of fifth user-defined custom symbol
    I2C_LCD1602_INDEX_CUSTOM_5,                         ///< Index of sixth user-defined custom symbol
    I2C_LCD1602_INDEX_CUSTOM_6,                         ///< Index of seventh user-defined custom symbol
    I2C_LCD1602_INDEX_CUSTOM_7,                         ///< Index of eighth user-defined custom symbol
} i2c_lcd1602_custom_index_t;

typedef enum {
    LEFT,
    RIGHT
} LCD1602_dir_t;

// MCP 23008 I2C IO expander
#define MCP23008_I2C_ADDR 0b0100000

#define MCP23008_REG_IODIR      0x00
#define MCP23008_REG_IPOL       0x01
#define MCP23008_REG_GPINTEN    0x02
#define MCP23008_REG_DEFVAL     0x03
#define MCP23008_REG_INTCON     0x04
#define MCP23008_REG_IOCON      0x05
#define MCP23008_REG_GPPU       0x06
#define MCP23008_REG_INTF       0x07
#define MCP23008_REG_INTCAP     0x08
#define MCP23008_REG_GPIO       0x09
#define MCP23008_REG_OLAT       0x0A

#ifndef I2C_MASTER_TIMEOUT_MS
    #define I2C_MASTER_TIMEOUT_MS 1000
#endif

class LCD1602 {
    public:
        LCD1602(uint8_t addr);

        // Init methods
        esp_err_t begin();
        esp_err_t begin(i2c_port_t, i2c_config_t);

        // Display control
        esp_err_t reset();
        esp_err_t clear();
        esp_err_t enable_display(bool en);
        esp_err_t enable_cursor(bool en);
        esp_err_t enable_blink(bool en);

        // Cursor control
        esp_err_t home();
        esp_err_t move_cursor(uint8_t col, uint8_t row);

        // Direction control
        esp_err_t set_left_to_right(bool en);
        esp_err_t enable_auto_scroll(bool en);
        esp_err_t set_scroll(LCD1602_dir_t dir);
        esp_err_t move_cursor(LCD1602_dir_t dir);
        esp_err_t define_char(i2c_lcd1602_custom_index_t index, const uint8_t pixelmap[]);
        esp_err_t write_char(uint8_t chr);
        esp_err_t write_string(const char *str);

    private:
        uint8_t mcp23008_addr;
        uint8_t i2c_portNum;
        esp_err_t readReg(uint8_t regAddr, uint8_t* data, size_t len);
        // Future: Multiple bytes write support
        esp_err_t writeReg(uint8_t regAddr, uint8_t data);
        esp_err_t maskWriteReg(uint8_t regAddr, uint8_t regMask,uint8_t data, bool clearMasked);
};

#endif // !__LCD1602_H__
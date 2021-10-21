/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "test_motor.h"
#include "motor_driver.h"
#include "motor_stepper.h"

void test_stepper(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    A4988_Driver ref_wheel;

    Nema17Config_t nema17;
    nema17.fullStep = 1.8;
    A4988_Driver ops_wheel (SIXTEENTH_STEP, OPPOSITE, nema17);

    MotorIOConfig_t motor_pin;
    motor_pin.step = GPIO_NUM_18;
    motor_pin.en = GPIO_NUM_3;
    motor_pin.dir = GPIO_NUM_4;
    motor_pin.ms1 = GPIO_NUM_25;
    motor_pin.ms2 = GPIO_NUM_26;
    motor_pin.ms3 = GPIO_NUM_27;
    ESP_ERROR_CHECK(ref_wheel.configIO(motor_pin));

    motor_pin.step = GPIO_NUM_33;
    motor_pin.dir = GPIO_NUM_26;
    motor_pin.en = GPIO_NUM_3;
    motor_pin.ms1 = GPIO_NUM_12;
    motor_pin.ms2 = GPIO_NUM_13;
    motor_pin.ms3 = GPIO_NUM_14;
    ESP_ERROR_CHECK(ops_wheel.configIO(motor_pin));
    for (;;) {
        ref_wheel.setContinuous(720);
        ops_wheel.setContinuous(720);
        vTaskDelay(1 * 1000 / portTICK_RATE_MS);
        ref_wheel.setContinuous(-720);
        ops_wheel.setContinuous(-720);
        // ops_wheel.halt();
        vTaskDelay(1 * 1000 / portTICK_RATE_MS);
    }
}

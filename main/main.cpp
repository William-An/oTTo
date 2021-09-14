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
#include "esp_spi_flash.h"
#include "motor_driver.h"
#include "motor_stepper.h"

extern "C" void app_main(void);

void app_main(void)
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

    // printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
    //         (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    // for (int i = 10; i >= 0; i--) {
    //     printf("Restarting in %d seconds...\n", i);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
    // printf("Restarting now.\n");
    // fflush(stdout);
    // esp_restart();

    A4988_Driver a4988;
    MotorIOConfig_t motor_pin;
    motor_pin.step = GPIO_NUM_18;
    motor_pin.en = GPIO_NUM_2;
    motor_pin.dir = GPIO_NUM_4;
    motor_pin.ms1 = GPIO_NUM_25;
    motor_pin.ms2 = GPIO_NUM_26;
    motor_pin.ms3 = GPIO_NUM_27;
    ESP_ERROR_CHECK(a4988.configIO(motor_pin));
    for (;;) {
        a4988.setContinuous(720);
    }
}

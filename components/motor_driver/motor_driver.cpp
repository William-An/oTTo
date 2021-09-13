#include <stdio.h>
#include "motor_driver.h"
#include "motor_stepper.h"

A4988_Driver::A4988_Driver(float steps, bool pos) : GenericMotorDiver() {
    this->steps = steps;
    this->pos = pos;
}

A4988_Driver::A4988_Driver() : GenericMotorDiver() {
    this->steps = 1/16;
    this->pos = 1;
}

esp_err_t A4988_Driver::configIO(motorIOConfig_t motorIO) {
    esp_err_t err = ESP_OK;
    int normalOutputSel = (1ULL << motorIO.dir) | (1ULL << motorIO.en) | 
                    (1ULL << motorIO.ms1) | (1ULL << motorIO.ms2)  | (1ULL << motorIO.ms3);
    
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = normalOutputSel;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    err = gpio_config(&io_conf);
    if (err != ESP_OK)
        return err;

    int intrOutputSel = 1ULL << motorIO.step;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = intrOutputSel;
    err = gpio_config(&io_conf);
    if (err != ESP_OK)
        return err;

    return err;
}
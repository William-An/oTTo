#include <stdio.h>
#include "motor_driver.h"
#include "motor_stepper.h"

A4988_Driver::A4988_Driver(Step_Size_t steps, bool pos, Nema17Config_t nema17) {
    this->steps = steps;
    this->pos = pos;
    this->nema17 = nema17;
}

A4988_Driver::A4988_Driver() {
    this->steps = SIXTEENTH_STEP;
    this->pos = 1;
    (this->nema17).fullStep = 1.8;
}

esp_err_t A4988_Driver::configIO(MotorIOConfig_t motorIO) {
    esp_err_t err = ESP_OK;
    this->motorIO = motorIO;
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



    err = gpio_set_level(motorIO.en, 1);  // high is disable
    if (err != ESP_OK)
        return err;

    err = gpio_set_level(motorIO.dir, 0);
    if (err != ESP_OK)
        return err;

    if (steps == FULL_STEP) {
        err = gpio_set_level(motorIO.ms1, 0);
        err = gpio_set_level(motorIO.ms2, 0);
        err = gpio_set_level(motorIO.ms3, 0);
    } else if (steps == HALF_STEP) {
        err = gpio_set_level(motorIO.ms1, 1);
        err = gpio_set_level(motorIO.ms2, 0);
        err = gpio_set_level(motorIO.ms3, 0);
    } else if (steps == QUARTER_STEP) {
        err = gpio_set_level(motorIO.ms1, 0);
        err = gpio_set_level(motorIO.ms2, 1);
        err = gpio_set_level(motorIO.ms3, 0);
    } else if (steps == EIGHTH_STEP) {
        err = gpio_set_level(motorIO.ms1, 1);
        err = gpio_set_level(motorIO.ms2, 1);
        err = gpio_set_level(motorIO.ms3, 0);
    } else if (steps == SIXTEENTH_STEP) {
        err = gpio_set_level(motorIO.ms1, 1);
        err = gpio_set_level(motorIO.ms2, 1);
        err = gpio_set_level(motorIO.ms3, 1);
    } else {
        err = ESP_FAIL;
        return err;
    }
    if (err != ESP_OK)
        return err;

    err = mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, motorIO.step);
    if (err != ESP_OK)
        return err;

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;
    pwm_config.cmpr_a = 0.0;       
    pwm_config.cmpr_b = 0.0;       
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); 

    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    return err;
}

esp_err_t A4988_Driver::setContinuous(float omega) {
    esp_err_t err = ESP_OK;
    if (omega == 0) {
        err = ESP_FAIL;
        return err;
    }

    err = gpio_set_level(motorIO.en, 0);

    if (this->pos == 1) {
        if (omega > 0) {
            err = gpio_set_level(motorIO.dir, 1);
            if (err != ESP_OK)
                return err;
        } else {
            err = gpio_set_level(motorIO.dir, 0);
            if (err != ESP_OK)
                return err;
        }
    } else {
        if (omega > 0) {
            err = gpio_set_level(motorIO.dir, 0);
            if (err != ESP_OK)
                return err;
        } else {
            err = gpio_set_level(motorIO.dir, 1);
            if (err != ESP_OK)
                return err;
        }
    }

    float stepSize;
    stepSize = nema17.fullStep / (float)steps;

    float period = stepSize / omega;
    float freq = 1 / period;

    err = mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, freq);
    if (err != ESP_OK)
                return err;

    err = mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50);
    if (err != ESP_OK)
                return err;

    err = mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
    if (err != ESP_OK)
                return err;

    return err;
}

esp_err_t A4988_Driver::halt() {
    esp_err_t err = ESP_OK;
    err = mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    if (err != ESP_OK)
                return err;

    err = mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, 50);
    if (err != ESP_OK)
                return err;

    err = mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
    if (err != ESP_OK)
                return err;
    
    return err;
}
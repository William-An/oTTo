#include <stdio.h>
#include "motor_driver.h"
#include "motor_stepper.h"

A4988_Driver::A4988_Driver(Step_Size_t steps, Position_t pos, Nema17Config_t nema17) {
    this->steps = steps;
    this->pos = pos;
    this->nema17 = nema17;
}

A4988_Driver::A4988_Driver() {
    this->steps = SIXTEENTH_STEP;
    this->pos = REFERECNCE;
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

    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
    mcpwm_io_signals_t port;
    if (pos == REFERECNCE) {
        unit = MCPWM_UNIT_1;
        timer = MCPWM_TIMER_1;
        port = MCPWM1A;
    } else {
        unit = MCPWM_UNIT_0;
        timer = MCPWM_TIMER_0;
        port = MCPWM0A;
    }

    err = mcpwm_gpio_init(unit, port, motorIO.step);
    if (err != ESP_OK)
        return err;

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;
    pwm_config.cmpr_a = 0.0;       
    pwm_config.cmpr_b = 0.0;       
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;


    mcpwm_init(unit, timer, &pwm_config); 

    mcpwm_stop(unit, timer);
    return err;
}

esp_err_t A4988_Driver::setContinuous(float omega) {
    esp_err_t err = ESP_OK;
    if (omega == 0) {
        err = ESP_FAIL;
        return err;
    }

    err = gpio_set_level(motorIO.en, 0);

    if (this->pos == REFERECNCE) {
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

    float period = stepSize / abs(omega);
    float freq = 1 / period;

    mcpwm_unit_t unit;
    mcpwm_timer_t timer;

    if (pos == REFERECNCE) {
        unit = MCPWM_UNIT_1;
        timer = MCPWM_TIMER_1;
    } else {
        unit = MCPWM_UNIT_0;
        timer = MCPWM_TIMER_0;
    }

    err = mcpwm_set_frequency(unit, timer, freq);
    if (err != ESP_OK)
                return err;

    err = mcpwm_set_duty(unit, timer, MCPWM_OPR_A, 50);
    if (err != ESP_OK)
                return err;

    err = mcpwm_start(unit, timer);
    if (err != ESP_OK)
                return err;

    return err;
}

esp_err_t A4988_Driver::halt() {
    esp_err_t err = ESP_OK;
    

    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
    if (pos == REFERECNCE) {
        unit = MCPWM_UNIT_1;
        timer = MCPWM_TIMER_1;
    } else {
        unit = MCPWM_UNIT_0;
        timer = MCPWM_TIMER_0;
    }

    err = mcpwm_stop(unit, timer);
    if (err != ESP_OK)
                return err;

    err = mcpwm_set_frequency(unit, timer, 50);
    if (err != ESP_OK)
                return err;

    err = mcpwm_set_duty(unit, timer, MCPWM_OPR_A, 0);
    if (err != ESP_OK)
                return err;
    
    return err;
}

esp_err_t A4988_Driver::setFixed(float angle, float omega) {
    esp_err_t err = ESP_OK;
    float delay = angle / abs(omega);
    err = setContinuous(omega);
    if (err != ESP_OK)
                return err;

    vTaskDelay(delay * 1000 / portTICK_RATE_MS);

    err = halt();
    if (err != ESP_OK)
                return err;

    return err;
}
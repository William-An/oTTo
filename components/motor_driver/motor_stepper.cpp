#include <stdio.h>
#include <math.h>
#include "motor_driver.h"
#include "motor_stepper.h"

/**
 * @brief Construct a new A4988_Driver object
 * 
 * @param steps: full step of the stepper motor, nema17 usually have 1.8 degree as one full step
 * @param pos: if the current motor is the reference motor; the system can only have one reference, the other is opposite
 * @param nema17: motor and other info; full step of the stepper motor, nema17 usually have 1.8 degree as one full step
 */
A4988_Driver::A4988_Driver(Step_Size_t steps, Position_t pos, Nema17Config_t nema17) {
    this->steps = steps;
    this->pos = pos;
    this->nema17 = nema17;
}

/**
 * @brief Construct a new default A4988_Driver object
 * 
 */


A4988_Driver::A4988_Driver() {
    this->steps = SIXTEENTH_STEP;
    this->pos = REFERECNCE;
    (this->nema17).fullStep = 1.8;
}

/**
 * @brief configure the GPIOs used for A4988
 * 
 * @param motorIO motorIO has six fields: 
 *                step: driving signal; each pulse is one step
 *                dir: direction signal; 1 is CW, 0 is CCW
 *                en: active low enable signal
 *                ms1-3: step size signal
 */

esp_err_t A4988_Driver::configIO(MotorIOConfig_t motorIO) {
    esp_err_t err = ESP_OK;
    this->motorIO = motorIO;

    // enable the desired GPIOs
    uint64_t normalOutputSel = (1ULL << motorIO.dir) | (1ULL << motorIO.en) | 
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


    // set output to be default value
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

    // set up step pin to be pwm output
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

    // default value for pwm
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

/**
 * @brief set the motor to rotate at desire angular velocity indefinitely until halt() is inserted
 * 
 * @param omega the angular velocity
 */
esp_err_t A4988_Driver::setContinuous(float omega) {
    esp_err_t err = ESP_OK;

    // omega = 20.0F;
    if (omega == 0.0F) {
        return halt();
    }

    err = gpio_set_level(motorIO.en, 0);

    // determine the direction according to omega and motor postion
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

    // calculate pwm frequency
    float stepSize;
    stepSize = nema17.fullStep / (float)steps;

    float period = stepSize / abs(omega);
    if (period != 0.0F) {
        float freq = ceilf(1.0 / period); // Prevent truncate to 0

        // select unit and timer to be used for pwm
        mcpwm_unit_t unit;
        mcpwm_timer_t timer;

        if (pos == REFERECNCE) {
            unit = MCPWM_UNIT_1;
            timer = MCPWM_TIMER_1;
        } else {
            unit = MCPWM_UNIT_0;
            timer = MCPWM_TIMER_0;
        }

        // start pwm for motor rotation
        err = mcpwm_set_frequency(unit, timer, (uint32_t) freq);
        if (err != ESP_OK)
            return err;

        err = mcpwm_set_duty(unit, timer, MCPWM_OPR_A, 50);
        if (err != ESP_OK)
            return err;

        err = mcpwm_start(unit, timer);
        if (err != ESP_OK)
            return err;

        return err;
    } else {
        return halt();
    }
    return ESP_OK;
}

/**
 * @brief stop the motor
 * 
 */

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

/**
 * @brief set the motor to rotate at desire angular velocity for some angle
 * 
 * @param omega the angular velocity
 * @param angle total angle to be rotated
 */

esp_err_t A4988_Driver::setFixed(float angle, float omega) {
    esp_err_t err = ESP_OK;
    if (omega != 0.0F) {
        float delay = angle / abs(omega);
        err = setContinuous(omega);
        if (err != ESP_OK)
            return err;

        // TODO Inexact delay
        vTaskDelay(delay * 1000 / portTICK_RATE_MS);
    }
    return halt();
}
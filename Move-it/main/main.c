#include <unistd.h>
#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "esp_log.h"


/* Number of elements in a static array */
#define LENGTH(arr) (sizeof(arr) / sizeof(*arr))


/* Motor 1 forward control pin */
#define MOTOR1_FORWARD_GPIO GPIO_NUM_27
/* Motor 1 backward control pin */
#define MOTOR1_BACKWARD_GPIO GPIO_NUM_26
/* Motor 2 forward control pin */
#define MOTOR2_FORWARD_GPIO GPIO_NUM_33
/* Motor 2 backward control pin */
#define MOTOR2_BACKWARD_GPIO GPIO_NUM_32

/* Motor 1 power control pin */
#define MOTOR1_POWER_GPIO GPIO_NUM_14
/* Motor 2 power control pin */
#define MOTOR2_POWER_GPIO GPIO_NUM_25

/* MOTOR_POWER_INCREMENT is a constant that determines the step size for increasing or decreasing
 * the duty cycle for motor speed control */
#define MOTOR_POWER_INCREMENT 5.0
/* The MCPWM uses percentage to represent duty cycle, so the maximum is 100 */
#define MAX_DUTY_CYCLE 100.0
/* The MCPWM uses percentage to represent duty cycle, so the minimum is 0 */
#define MIN_DUTY_CYCLE 0.0


/**
 * Configures GPIO pins for motor control
 */
void configure_motor_control() {
    const int motor_control_pins[4] = {
        MOTOR1_FORWARD_GPIO, MOTOR1_BACKWARD_GPIO,
        MOTOR2_FORWARD_GPIO, MOTOR2_BACKWARD_GPIO
    };

    for (int i = 0; i < LENGTH(motor_control_pins); i++) {
        const int pin = motor_control_pins[i];

        /* Set all motor control pins to output mode (to allow setting voltage level) */
        ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_OUTPUT));
        /* Set level of all motor control pins to low initially */
        ESP_ERROR_CHECK(gpio_set_level(pin, 0));
    }

    mcpwm_config_t pwm_config = {
        /* The PWM signal can switch on and off `frequency` times per second */
        .frequency = 200,
        /* The initial duty for operator A, in percentage */
        .cmpr_a = 0,
        /* The initial duty for operator A, in percentage */
        .cmpr_b = 0,
        /* The counting method of timer, UP is the most commonly used */
        .counter_mode = MCPWM_UP_COUNTER,
        /* In this mode, the "ON" state of the PWM starts from the beginning of the PWM period */
        .duty_mode = MCPWM_DUTY_MODE_0
    };

    /* Configure power control pins for PWM */
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR1_POWER_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR2_POWER_GPIO));

    /* Initialize timer and MCPWM unit */
    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config));
}

/**
 * Util function to update duty cycle of motor power pins to a new value, relatively to the old one
 * @param increment percentage which will be added to the current duty cycle of both motors
 */
esp_err_t update_motors_duty(float increment) {
    float duty_current = mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A);
    ESP_LOGI("update_motors_duty", "Current duty cycle: %f", duty_current);

    float duty_new = duty_current + increment;
    if (duty_new > MAX_DUTY_CYCLE) {
        duty_new = MAX_DUTY_CYCLE;
    } else if (duty_new < MIN_DUTY_CYCLE) {
        duty_new = MIN_DUTY_CYCLE;
    }

    esp_err_t ret;
    /* Set new duty cycle for both motors */
    ret = mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, duty_new);
    ret = (ret != ESP_OK) ? ret : mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0B, duty_new);

    ESP_LOGI("update_motors_duty", "New duty cycle: %f", duty_new);
    return ret;
}

/**
 * Sets the duty o both motor power pins to a new value
 * @param duty_new percentage which will be set as the new duty cycle of both motors
 */
esp_err_t inline set_motors_duty(float duty_new) {
    return mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, duty_new) \
        || mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0B, duty_new);
}

/**
 * Sets the spin direction of both motors to forward
 */
esp_err_t inline spin_forward() {
    return gpio_set_level(MOTOR1_FORWARD_GPIO, 1) \
        || gpio_set_level(MOTOR1_BACKWARD_GPIO, 0) \
        || gpio_set_level(MOTOR2_FORWARD_GPIO, 1) \
        || gpio_set_level(MOTOR2_BACKWARD_GPIO, 0);
}

/**
 * Sets the spin direction of both motors to backward
 */
esp_err_t inline spin_backward() {
    return gpio_set_level(MOTOR1_FORWARD_GPIO, 0) \
        || gpio_set_level(MOTOR1_BACKWARD_GPIO, 1) \
        || gpio_set_level(MOTOR2_FORWARD_GPIO, 0) \
        || gpio_set_level(MOTOR2_BACKWARD_GPIO, 1);
}

void app_main(void) {
    ESP_LOGI("setup", "Configuring motor control pins");
    configure_motor_control();
    
    bool was_running = false;
    while (1) {
        if (was_running) {
            /* If the motor was running, we need to stop it */
            ESP_ERROR_CHECK(update_motors_duty(-1));
            gpio_set_level(MOTOR1_FORWARD_GPIO, 0);
            gpio_set_level(MOTOR1_BACKWARD_GPIO, 1);
            gpio_set_level(MOTOR2_FORWARD_GPIO, 0);
            gpio_set_level(MOTOR2_BACKWARD_GPIO, 1);
            was_running = false;
        } else {
            ESP_ERROR_CHECK(update_motors_duty(3));
            gpio_set_level(MOTOR1_FORWARD_GPIO, 1);
            gpio_set_level(MOTOR1_BACKWARD_GPIO, 0);
            gpio_set_level(MOTOR2_FORWARD_GPIO, 1);
            gpio_set_level(MOTOR2_BACKWARD_GPIO, 0);
            was_running = true;
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

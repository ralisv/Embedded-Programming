#include <unistd.h>
#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/touch_pad.h"
#include "driver/touch_sensor.h"
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


/* Touch sensor pin */
#define TOUCH_SENSOR1_GPIO TOUCH_PAD_NUM3
/* Touch sensor pin */
#define TOUCH_SENSOR2_GPIO TOUCH_PAD_NUM0
/* Arbitrary value, in your setup you migjt have to modify it */
#define TOUCH_DETECTION_THRESHOLD 500
/* The higher the touch resolution, the smoother the light-up and fade is going to be, in hz */
#define TOUCH_RESOLUTION 5


/**
 * Initializes touch sensors
 */
void configure_touch_sensors() {
    /* Before using a touch pad, you need to initialize the touch pad driver */
    ESP_ERROR_CHECK(touch_pad_init());

    /* Use the function touch_pad_set_fsm_mode() to select if touch pad
     * measurement (operated by FSM) should be started automatically by a hardware timer */
    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_SW));
    
    /* Enabling the touch sensor functionality for a particular GPIO is done
     * with touch_pad_config() */
    ESP_ERROR_CHECK(touch_pad_config(TOUCH_SENSOR1_GPIO, 200));
    ESP_ERROR_CHECK(touch_pad_config(TOUCH_SENSOR2_GPIO, 200));

    /* Start touch sensor by software */
    ESP_ERROR_CHECK(touch_pad_sw_start());

    /* Set sampling interval to 512 ms */
    ESP_ERROR_CHECK(touch_pad_set_measurement_interval(512));

    /* Configure sensor's sensitivity by setting pin's voltage levels */
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
}

/**
 * Function which makes an educated guess of whether the touch sensor is being touched or not
 * @returns true if the guess is that the sensor is being touched, false otherwise
 */
bool inline is_touching(uint16_t touch_value) {
    return touch_value < TOUCH_DETECTION_THRESHOLD;
}

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
    ret = (ret != ESP_OK) ? mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0B, duty_new) : ret;

    ESP_LOGI("update_motors_duty", "New duty cycle: %f", duty_new);
    return ret;
}   



void app_main(void) {
    ESP_LOGI("setup", "Configuring touch sensors at touch pads %u and %u", TOUCH_SENSOR1_GPIO, TOUCH_SENSOR2_GPIO);
    configure_touch_sensors();

    ESP_LOGI("setup", "Touch detection threshold is set to arbitrary value of %u", TOUCH_DETECTION_THRESHOLD);

    ESP_LOGI("setup", "Configuring motor control pins");
    configure_motor_control();

    ESP_ERROR_CHECK(update_motors_duty(50));
    vTaskDelay((1000 / TOUCH_RESOLUTION) / portTICK_PERIOD_MS);

    update_motors_duty(-50);
    vTaskDelay((1000 / TOUCH_RESOLUTION) / portTICK_PERIOD_MS);

    update_motors_duty(100);
    vTaskDelay((1000 / TOUCH_RESOLUTION) / portTICK_PERIOD_MS);

    update_motors_duty(-100);
    
    bool was_touched1 = false;
    bool was_touched2 = false;
    uint16_t touch_value1;
    uint16_t touch_value2;
    while (1) {
        touch_pad_read(TOUCH_SENSOR1_GPIO, &touch_value1);
        touch_pad_read(TOUCH_SENSOR2_GPIO, &touch_value2);

        /* If only one sensor is touched, turn the according motor forward */
        if (is_touching(touch_value1) && !was_touched1) {
            ESP_LOGI("loop", "Touch detected on sensor 1");
            update_motors_duty(50);

            was_touched1 = true;
            gpio_set_level(MOTOR1_FORWARD_GPIO, 1);
        } else if (!is_touching(touch_value1) && was_touched1) {
            ESP_LOGI("loop", "Stop touch detected on sensor 1");
            update_motors_duty(-50);

            was_touched1 = false;
            gpio_set_level(MOTOR1_FORWARD_GPIO, 0);
        }

        if (is_touching(touch_value2) && !was_touched2) {
            ESP_LOGI("loop", "Touch detected on sensor 2");
            
            update_motors_duty(50);
            was_touched2 = true;
            gpio_set_level(MOTOR2_FORWARD_GPIO, 1);
        } else if (!is_touching(touch_value2) && was_touched2) {
            ESP_LOGI("loop", "Stop touch detected on sensor 2");
            update_motors_duty(-50);

            was_touched2 = false;
            gpio_set_level(MOTOR2_FORWARD_GPIO, 0);
        }

        vTaskDelay((1000 / TOUCH_RESOLUTION) / portTICK_PERIOD_MS);
    }
}

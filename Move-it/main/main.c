#include <unistd.h>
#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/touch_pad.h"
#include "driver/touch_sensor.h"
#include "esp_log.h"




/* Motor 1 forward control pin */
#define MOTOR1_FORWARD_GPIO GPIO_NUM_27
/* Motor 1 backward control pin */
#define MOTOR1_BACKWARD_GPIO GPIO_NUM_26
/* Motor 2 forward control pin */
#define MOTOR2_FORWARD_GPIO GPIO_NUM_33
/* Motor 2 backward control pin */
#define MOTOR2_BACKWARD_GPIO GPIO_NUM_32


/* Touch sensor pin */
#define TOUCH_SENSOR1_GPIO TOUCH_PAD_NUM3
/* Touch sensor pin */
#define TOUCH_SENSOR2_GPIO TOUCH_PAD_NUM0
/* Arbitrary value, in your setup you migjt have to modify it */
#define TOUCH_DETECTION_THRESHOLD 500
/* The higher the touch resolution, the smoother the light-up and fade is going to be */
#define TOUCH_RESOLUTION 5 // hz


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

    for (int i = 0; i < 4; i++) {
        /* Set all motor control pins to output mode (to allow setting voltage level) */
        ESP_ERROR_CHECK(gpio_set_direction(motor_control_pins[i], GPIO_MODE_OUTPUT));
        /* Set level of all motor control pins to low initially */
        ESP_ERROR_CHECK(gpio_set_level(motor_control_pins[i], 0));
    }
}

void app_main(void) {
    ESP_LOGI("setup", "Configuring touch sensors at pins %u and %u", TOUCH_SENSOR1_GPIO, TOUCH_SENSOR2_GPIO);
    configure_touch_sensors();

    ESP_LOGI("setup", "Touch detection threshold is set to arbitrary value of %u", TOUCH_DETECTION_THRESHOLD);

    ESP_LOGI("setup", "Configuring motor control pins");
    configure_motor_control();

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
            was_touched1 = true;
            gpio_set_level(MOTOR1_FORWARD_GPIO, 1);
        } else if (!is_touching(touch_value1) && was_touched1) {
            ESP_LOGI("loop", "Stop touch detected on sensor 1");
            was_touched1 = false;
            gpio_set_level(MOTOR1_FORWARD_GPIO, 0);
        }

        if (is_touching(touch_value2) && !was_touched2) {
            ESP_LOGI("loop", "Touch detected on sensor 2");
            was_touched2 = true;
            gpio_set_level(MOTOR2_FORWARD_GPIO, 1);
        } else if (!is_touching(touch_value2) && was_touched2) {
            ESP_LOGI("loop", "Stop touch detected on sensor 2");
            was_touched2 = false;
            gpio_set_level(MOTOR2_FORWARD_GPIO, 0);
        }

        vTaskDelay((1000 / TOUCH_RESOLUTION) / portTICK_PERIOD_MS);
    }
}

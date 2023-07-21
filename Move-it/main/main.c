#include <unistd.h>
#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/touch_pad.h"
#include "driver/touch_sensor.h"
#include "esp_log.h"


/* Touch sensor pins */
#define TOUCH_SENSOR1_GPIO TOUCH_PAD_NUM0
#define TOUCH_SENSOR2_GPIO TOUCH_PAD_NUM3
/* Arbitrary value, in your setup you migjt have to modify it */
#define TOUCH_DETECTION_THRESHOLD 500
/* The higher the touch resolution, the smoother the light-up and fade is going to be */
#define TOUCH_RESOLUTION 20 // hz


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


void app_main(void) {
    ESP_LOGI("setup", "Configuring touch sensors at pins %u and %u", TOUCH_SENSOR1_GPIO, TOUCH_SENSOR2_GPIO);
    configure_touch_sensors();

    ESP_LOGI("setup", "Touch detection threshold is set to arbitrary value of %u", TOUCH_DETECTION_THRESHOLD);

    uint16_t touch_value1;
    uint16_t touch_value2;
    while (1) {
        touch_pad_read(TOUCH_SENSOR1_GPIO, &touch_value1);
        touch_pad_read(TOUCH_SENSOR2_GPIO, &touch_value2);
        ESP_LOGI("loop", "Reading arbitrary value of %u from the touch sensor 1", touch_value1);
        ESP_LOGI("loop", "Reading arbitrary value of %u from the touch sensor 2", touch_value2);
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

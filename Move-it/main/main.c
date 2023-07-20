#include <unistd.h>
#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/touch_pad.h"
#include "driver/touch_sensor.h"
#include "esp_log.h"


#define TOUCH_SENSOR_GPIO TOUCH_PAD_NUM0
/* Arbitrary value, in your setup you migjt have to modify it */
#define TOUCH_DETECTION_THRESHOLD 500
/* The higher the touch resolution, the smoother the light-up and fade is going to be */
#define TOUCH_RESOLUTION 20 // hz


/**
 * Initializes touch sensor  
 */
void configure_touch_sensor() {
    /* Before using a touch pad, you need to initialize the touch pad driver */
    ESP_ERROR_CHECK(touch_pad_init());

    /* Use the function touch_pad_set_fsm_mode() to select if touch pad
     * measurement (operated by FSM) should be started automatically by a hardware timer */
    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_SW));
    
    /* Enabling the touch sensor functionality for a particular GPIO is done
     * with touch_pad_config() */
    ESP_ERROR_CHECK(touch_pad_config(TOUCH_SENSOR_GPIO, 200));
    
    /* Start touch sensor by software */
    ESP_ERROR_CHECK(touch_pad_sw_start());

    /* Set sampling interval to 512 ms */
    ESP_ERROR_CHECK(touch_pad_set_measurement_interval(512));

    /* Configure sensor's sensitivity by setting pin's voltage levels */
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    
    /* Start the touch pad filter to process the raw reading data and eliminate possible noise */
    touch_pad_filter_start(10);
}


/**
 * Function which makes an educated guess of whether the touch sensor is being touched or not
 * @returns true if the guess is that the sensor is being touched, false otherwise
 */
bool inline is_touching(uint16_t touch_value) {
    return touch_value < TOUCH_DETECTION_THRESHOLD;
}


void app_main(void) {
    ESP_LOGI("setup", "Configuring touch sensor at pin %u", TOUCH_SENSOR_GPIO);
    configure_touch_sensor();

    ESP_LOGI("setup", "Touch detection threshold is set to arbitrary value of %u", TOUCH_DETECTION_THRESHOLD);

    uint16_t touch_value;
    while (1) {
        touch_pad_read(TOUCH_PAD_NUM0, &touch_value);
        ESP_LOGI("loop", "Reading arbitrary value of %u from the touch sensor", touch_value);
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

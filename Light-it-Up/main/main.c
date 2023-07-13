#include <unistd.h>
#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/touch_pad.h"
#include "driver/touch_sensor.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/ledc.h"


/* Tag used for logging in the main function */
static const char *TAG = "app_main";


#define TOUCH_SENSOR_GPIO TOUCH_PAD_NUM0
/* Arbitrary value, in your setup you migjt have to modify it */
#define TOUCH_DETECTION_THRESHOLD 500
/* The higher the touch resolution, the smoother the light-up and fade is going to be */
#define TOUCH_RESOLUTION 20


#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_GPIO 23
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RESOLUTION LEDC_TIMER_13_BIT
#define LEDC_DUTY 0
#define LEDC_FREQUENCY 100
#define LEDC_MAX_DUTY (pow(2, LEDC_DUTY_RESOLUTION) - 1)

#define LEDC_DUTY_INCREMENT (LEDC_MAX_DUTY / TOUCH_RESOLUTION)

#define TOTAL_FADE_TIME 5000 // ms
#define FADE_TIME (TOTAL_FADE_TIME / TOUCH_RESOLUTION)


#define min(x, y) (x < y : x ? y)
#define max(x, y) (x < y : y ? x)


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
 * Initializes LEDC timer and channel
 */
void configure_LED() {
    /* Configure LEDC timer */
    ledc_timer_config_t ledc_timer = {
        /* Resolution of the pwm duty (the amount of shades LED can reach) */
        .duty_resolution = LEDC_DUTY_RESOLUTION,
        /* Sets the frequency of the PWM signal to (LEDC_FREQUENCY) Hz, so LED can switch between (LEDC_FREQUENCY) brightness levels per second */
        .freq_hz = LEDC_FREQUENCY,
        /* Sets the speed mode of the timer */
        .speed_mode = LEDC_MODE,
        /* Assigns the LEDC timer number 0 to this configuration */
        .timer_num = LEDC_TIMER,
        /* Configure source clock */
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    /* Configure LEDC channel */
    const ledc_channel_config_t led_config = {
        /* The LED is located on pin number 23 */
        .gpio_num = LEDC_GPIO,
        /* Configure speed mode */
        .speed_mode = LEDC_MODE,
        /* Specify LEDC channel */
        .channel = LEDC_CHANNEL,
        /* Specify the timer for generating pwm signal */
        .timer_sel = LEDC_TIMER,
        /* Configure duty cycle (LED brightness) */
        .duty = LEDC_DUTY
    };
    ESP_ERROR_CHECK(ledc_channel_config(&led_config));

    /* Activate the fading functionality */
    ESP_ERROR_CHECK(ledc_fade_func_install(0));    
}


/**
 * Function for fading LED
 * @param current_duty The current state of duty
 * @returns new satte of duty
 */
uint16_t fade_LED(uint16_t current_duty) {
    const uint16_t new_duty = (LEDC_DUTY_INCREMENT > current_duty) ? 0 : current_duty - LEDC_DUTY_INCREMENT; // Check for underflow

    /* Set the target duty cycle and the time it takes to reach that duty cycle */
    ESP_ERROR_CHECK(ledc_set_fade_with_time(LEDC_MODE, LEDC_CHANNEL, new_duty, FADE_TIME));
    
    /* Start fading */
    ESP_ERROR_CHECK(ledc_fade_start(LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_NO_WAIT));

    return new_duty;
}

/**
 * Function for lighting the LED
 * @param current_duty The current state of duty
 * @returns new satte of duty
 */
uint16_t light_LED(uint16_t current_duty) {
    const uint16_t new_duty = (LEDC_DUTY_INCREMENT + current_duty > LEDC_MAX_DUTY) ? LEDC_MAX_DUTY : current_duty + LEDC_DUTY_INCREMENT; // Check for overflow

    /* Set the target duty cycle and the time it takes to reach that duty cycle */
    ESP_ERROR_CHECK(ledc_set_fade_with_time(LEDC_MODE, LEDC_CHANNEL, new_duty, FADE_TIME));
    
    /* Start fading */
    ESP_ERROR_CHECK(ledc_fade_start(LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_NO_WAIT));

    return new_duty;
}


/**
 * Function which makes an educated guess of whether the touch sensor is being touched or not
//  * @returns true if the touch sensor guesses the sensor is being touched, false otherwise
 */
bool inline is_touching(uint16_t touch_value) {
    return touch_value < TOUCH_DETECTION_THRESHOLD;
}


void app_main(void) {
    ESP_LOGI(TAG, "Configuring touch sensor at pin %u", TOUCH_SENSOR_GPIO);
    configure_touch_sensor();

    ESP_LOGI(TAG, "Touch detection threshold is set to arbitrary value of %u", TOUCH_DETECTION_THRESHOLD);

    ESP_LOGI(TAG, "Configuring LED at pin %u", LEDC_GPIO);
    configure_LED();
 
    bool was_touching = false;
    uint16_t duty = 0;
    uint16_t previous_duty = -1;

    uint16_t touch_value;
    while (1) {
        if (duty != previous_duty) {
            ESP_LOGI(TAG, "Current duty is %u", duty);
            previous_duty = duty;
        }
        
        touch_pad_read(TOUCH_PAD_NUM0, &touch_value);
        ESP_LOGI(TAG, "Reading arbitrary value of %u from the touch sensor", touch_value);
        
        if (!is_touching(touch_value) && duty > 0) {
            if (was_touching) {
                ESP_LOGI(TAG, "Fading LED since touch sensor is no longer touched");
                was_touching = false;
            }
            duty = fade_LED(duty);        
        } else if (is_touching(touch_value) && duty < LEDC_MAX_DUTY) {
            if (!was_touching) {
                ESP_LOGI(TAG, "Lighting LED up since touch is detected");
                was_touching = true;
            }
            duty = light_LED(duty);
        }

        /* Wait for the fade to complete */
        vTaskDelay(FADE_TIME / portTICK_PERIOD_MS);
    }
}

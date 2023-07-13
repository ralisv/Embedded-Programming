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
#define TOUCH_DETECTION_THRESHOLD 500

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_GPIO 23
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RESOLUTION LEDC_TIMER_10_BIT
#define LEDC_DUTY 512
#define LEDC_FREQUENCY 100
#define LEDC_MAX_DUTY (pow(2, LEDC_DUTY_RESOLUTION) - 1)

#define FADE_TIME 1000 // ms

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
 */
void fade_LED() {
    /* Set the target duty cycle and the time it takes to reach that duty cycle */
    ESP_ERROR_CHECK(ledc_set_fade_with_time(LEDC_MODE, LEDC_CHANNEL, 0, FADE_TIME));
    
    /* Start fading */
    ESP_ERROR_CHECK(ledc_fade_start(LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_NO_WAIT));
}

/**
 * Function for lighting the LED
 */
void light_LED() {
    /* Set the target duty cycle and the time it takes to reach that duty cycle */
    ESP_ERROR_CHECK(ledc_set_fade_with_time(LEDC_MODE, LEDC_CHANNEL, LEDC_MAX_DUTY, FADE_TIME));
    
    /* Start fading */
    ESP_ERROR_CHECK(ledc_fade_start(LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_NO_WAIT));
}


/**
 * Function which makes an educated guess of whether the touch sensor is being touched or not
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

    ESP_LOGI(TAG, "Fading LED initially");
    fade_LED();

    while (1) {
        uint16_t touch_value;
        touch_pad_read(TOUCH_PAD_NUM0, &touch_value);
        ESP_LOGI(TAG, "Reading arbitrary value of %u from the touch sensor", touch_value);
        if (was_touching && !is_touching(touch_value)) {
            ESP_LOGI(TAG, "Turning LED off since touch sensor is no longer touched");
            fade_LED();
            was_touching = false;
        } else if (!was_touching && is_touching(touch_value)) {
            ESP_LOGI(TAG, "Turning LED on since touch is detected");
            light_LED();
            was_touching = true;
        }

        /* Wait for the fade to complete */
        vTaskDelay(FADE_TIME / portTICK_PERIOD_MS);
    }
}

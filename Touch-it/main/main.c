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


#define TOUCH_SENSOR_GPIO TOUCH_PAD_NUM0
/* Arbitrary value, in your setup you migjt have to modify it */
#define TOUCH_DETECTION_THRESHOLD 500
/* The higher the touch resolution, the smoother the light-up and fade is going to be */
#define TOUCH_RESOLUTION 5


#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE

#define LEDC_GPIO_FORWARD 32
#define LEDC_GPIO_BACKWARD 33
#define LEDC_GPIO_LEFT 25
#define LEDC_GPIO_RIGHT 26

#define LEDC_CHANNEL_FORWARD LEDC_CHANNEL_0
#define LEDC_CHANNEL_BACKWARD LEDC_CHANNEL_1
#define LEDC_CHANNEL_LEFT LEDC_CHANNEL_2
#define LEDC_CHANNEL_RIGHT LEDC_CHANNEL_3

#define LEDC_DUTY_RESOLUTION LEDC_TIMER_13_BIT
#define LEDC_DUTY 0
#define LEDC_FREQUENCY 100
#define LEDC_MAX_DUTY (pow(2, LEDC_DUTY_RESOLUTION) - 1)

#define LEDC_DUTY_INCREMENT (LEDC_MAX_DUTY / TOUCH_RESOLUTION)

#define FADE_TIME (1000 / TOUCH_RESOLUTION)


/**
 * Initializes LEDC timer and channel
 */
void configure_LEDs() {
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

    /* Configure LEDC channels */
    const ledc_channel_config_t led_config_base = {
        .speed_mode = LEDC_MODE,
        .timer_sel = LEDC_TIMER,
        .duty = LEDC_DUTY
    };

    ledc_channel_config_t led_configs[4] = {
        {
            .channel = LEDC_CHANNEL_FORWARD,
            .gpio_num = LEDC_GPIO_FORWARD,
        },
        {
            .channel = LEDC_CHANNEL_BACKWARD,
            .gpio_num = LEDC_GPIO_BACKWARD,
        },
        {
            .channel = LEDC_CHANNEL_LEFT,
            .gpio_num = LEDC_GPIO_LEFT,
        },
        {
            .channel = LEDC_CHANNEL_RIGHT,
            .gpio_num = LEDC_GPIO_RIGHT,
        }
    };
    for (uint8_t i = 0; i < 4; i++) {
        ledc_channel_config_t led_config = led_config_base;
        led_config.channel = led_configs[i].channel;
        led_config.gpio_num = led_configs[i].gpio_num;
        ESP_ERROR_CHECK(ledc_channel_config(&led_config));
    };

    /* Activate the fading functionality */
    ESP_ERROR_CHECK(ledc_fade_func_install(0));    
}


/**
 * Function for fading the LED
 * @param new_duty The current state of duty
 * @param channel The channel of the LED
 * @returns new satte of duty
 */
uint16_t fade_LED(uint16_t new_duty, ledc_channel_t channel) {
    /* Set the target duty cycle and the time it takes to reach that duty cycle */
    ESP_ERROR_CHECK(ledc_set_fade_with_time(LEDC_MODE, channel, new_duty, FADE_TIME));
    
    /* Start fading */
    ESP_ERROR_CHECK(ledc_fade_start(LEDC_MODE, channel, LEDC_FADE_NO_WAIT));

    return new_duty;
}


void app_main(void) {
    ESP_LOGI("setup", "Configuring LEDs");
    configure_LEDs();
    
    bool tmp = false;
    while (1) {
        if (tmp) {
            fade_LED(LEDC_MAX_DUTY, LEDC_CHANNEL_FORWARD);
            fade_LED(0, LEDC_CHANNEL_BACKWARD);
            fade_LED(LEDC_MAX_DUTY, LEDC_CHANNEL_LEFT);
            fade_LED(0, LEDC_CHANNEL_RIGHT);
        } else {
            fade_LED(0, LEDC_CHANNEL_FORWARD);
            fade_LED(LEDC_MAX_DUTY, LEDC_CHANNEL_BACKWARD);
            fade_LED(0, LEDC_CHANNEL_LEFT);
            fade_LED(LEDC_MAX_DUTY, LEDC_CHANNEL_RIGHT);
        }
        tmp = !tmp;

        /* Wait for the fade to complete */
        vTaskDelay(FADE_TIME / portTICK_PERIOD_MS);
    }
}

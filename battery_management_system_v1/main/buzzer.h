#ifndef BUZZER_H
#define BUZZER_H

#include <stdio.h>  // printf, snprintf
#include <string.h> // memset, strcmp, strcasecmp
#include <inttypes.h> // int16_t, uint8_t, etc.
#include <iostream> // std::cout, std::endl

#include "freertos/FreeRTOS.h" // FreeRTOS types and functions (e.g., vTaskDelay, xQueueSend)
#include "freertos/task.h"     // FreeRTOS task functions (e.g., xTaskCreate)
#include "freertos/queue.h"    // FreeRTOS queue functions (e.g., xQueueCreate, xQueueSend)
#include "esp_err.h" // esp_err_t type and error codes (e.g., ESP_OK, ESP_FAIL)
#include "esp_log.h" // ESP32 logging functions (e.g., ESP_LOGI, ESP_LOGE)
#include "driver/gpio.h" // GPIO functions for controlling pins (e.g., gpio_set_level, gpio_set_direction)
#include "driver/ledc.h" // LEDC functions for PWM control (e.g., ledc_timer_config, ledc_channel_config)

#include "shared.h" // Header file for common definitions and utilities

// Uses LEDC PWM at 2700 Hz 
#define BUZZER_GPIO          GPIO_NUM_2
#define BUZZER_LEDC_CHANNEL  LEDC_CHANNEL_0
#define BUZZER_LEDC_TIMER    LEDC_TIMER_0
#define BUZZER_FREQ_HZ       2700  

// ============ BUZZER HELPERS ============

/**
  @brief Initializes the buzzer by configuring the LEDC timer 
  and channel for PWM output.
*/
static void buzzer_init() {
    ledc_timer_config_t timer = {};
    timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    timer.timer_num        = BUZZER_LEDC_TIMER;
    timer.duty_resolution  = LEDC_TIMER_10_BIT;  // 0–1023
    timer.freq_hz          = BUZZER_FREQ_HZ;
    timer.clk_cfg          = LEDC_AUTO_CLK;
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {};
    channel.speed_mode  = LEDC_LOW_SPEED_MODE;
    channel.channel     = BUZZER_LEDC_CHANNEL;
    channel.timer_sel   = BUZZER_LEDC_TIMER;
    channel.gpio_num    = BUZZER_GPIO;
    channel.duty        = 0;   // Start silent
    channel.hpoint      = 0;
    ledc_channel_config(&channel);
}

/**
  @brief Triggers a beep for a specified duration.
  @param duration_ms The duration of the beep in milliseconds.
*/
static void buzzer_beep(uint32_t duration_ms) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL, 512); // 50% duty
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL, 0);   // Silent
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL);
}

/**
  @brief Triggers a warning beep pattern.
*/
static void buzzer_warn() {
    buzzer_beep(100);
    vTaskDelay(pdMS_TO_TICKS(80));
    buzzer_beep(100);
}

/**
  @brief Triggers a critical alert beep pattern.
*/
static void buzzer_alert() {
    buzzer_beep(1000);
}

#endif // BUZZER_H
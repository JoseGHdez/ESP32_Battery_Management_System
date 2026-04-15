/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"

static const char *TAG = "example";

// Please consult the datasheet of your servo before changing the following parameters
//#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
//#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
//#define SERVO_MIN_DEGREE        -90   // Minimum angle
//#define SERVO_MAX_DEGREE        90    // Maximum angle

// SG90
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2100  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle
				      //
// MG996R
#define SERVO_STOP_PULSEWIDTH 1500 // Stop pulse width in microseconds


#define SERVO_PULSE_GPIO             26        // GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

static inline uint32_t example_angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

// MCPWM: Motor control pulse with modulator
extern "C" void app_main(void)
{
    // First, allocate a timer submodule

    ESP_LOGI(TAG, "Create timer and operator");
    
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {};
    timer_config.group_id = 0;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    timer_config.period_ticks = SERVO_TIMEBASE_PERIOD;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

	
    // Second, allocate an operator submodule

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {};
    operator_config.group_id = 0; // El operador debe estar en el mismo grupo que el timer
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    // Third connect timer and operator

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    // Now submodules in the operator submodule

    // First, allocate compartor submodule of the operator
    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config = {};
    comparator_config.flags.update_cmp_on_tez = true; // update threshold comparator when timer is 0
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    // Second, allocate generator submodule of the operator

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {};
    generator_config.gen_gpio_num = SERVO_PULSE_GPIO;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));
    
    // Now, initial setting for comparator and generator.

    // set the initial compare value, to start with the MG996R stopped
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, SERVO_STOP_PULSEWIDTH));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");

    // Now, setting of generator actions
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));
    // Now operation
    // First start timer

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    int nsteps = 1;
    int speed=100;
    int servo_pulse=SERVO_STOP_PULSEWIDTH;
    while (1) {

	//if(--nsteps==0)
	//{
	//	nsteps=3;
	//	speed=speed*-1;

	//}
	servo_pulse=SERVO_STOP_PULSEWIDTH-speed;
        ESP_LOGI(TAG, "Move to long press%d", servo_pulse);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator,servo_pulse));
        //Add delay, since it takes time for servo to rotate, usually 200ms/60degree rotation under 5V power supply
        vTaskDelay(pdMS_TO_TICKS(500));
	servo_pulse=SERVO_STOP_PULSEWIDTH;
        ESP_LOGI(TAG, "Wait long press%d", servo_pulse);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator,servo_pulse));
        vTaskDelay(pdMS_TO_TICKS(3000));

	servo_pulse=SERVO_STOP_PULSEWIDTH+speed;
        ESP_LOGI(TAG, "Move to first depress %d", servo_pulse);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator,servo_pulse));
        vTaskDelay(pdMS_TO_TICKS(200));
	servo_pulse=SERVO_STOP_PULSEWIDTH-speed;
        ESP_LOGI(TAG, "Move to short press %d", servo_pulse);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator,servo_pulse));
        //Add delay, since it takes time for servo to rotate, usually 200ms/60degree rotation under 5V power supply
        vTaskDelay(pdMS_TO_TICKS(500));
	servo_pulse=SERVO_STOP_PULSEWIDTH+speed;
        ESP_LOGI(TAG, "Move to final depress %d", servo_pulse);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator,servo_pulse));
        vTaskDelay(pdMS_TO_TICKS(200));




	ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY));

        }
}

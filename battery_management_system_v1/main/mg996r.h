#ifndef MG996R_H
#define MG996R_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"

static const char *SERVO_TAG = "servo_control";

// --- Servo control parameters ---
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microseconds (0 degrees)
#define SERVO_MAX_PULSEWIDTH_US 2100 // Maximum pulse width in microseconds (180 degrees)
#define SERVO_MIN_DEGREE        -90    // Minimum angle in degrees
#define SERVO_MAX_DEGREE        90 // Maximum angle in degrees

#define SERVO_STOP_PULSEWIDTH 1500 // Stop pulse width in microseconds

// Hardware configuration for the MG996R servo
#define SERVO_PULSE_GPIO             26       // Pin connected to the servo control signal (PWM output)
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms frequency (50Hz)

/**
 * @class ServoController
 * @brief A class to control the MG996R servo motor using the MCPWM peripheral 
 *        of the ESP32.
 */
class ServoController {
 public:
  // Constructor
  ServoController() : comparator(NULL), current_angle(0), speed(100), nsteps(1) {}

  void init_servo();
  void set_servo_angle(int angle);
  void set_servo_angle_smooth(int target_angle, int time_ms);
  void set_servo_pulse(uint32_t pulse_us);
  uint32_t angle_to_pulse_width(int angle);
  int get_servo_speed() const { return speed; }
  void stop_timer() {
    if (timer_handle) {
      mcpwm_timer_start_stop(timer_handle, MCPWM_TIMER_STOP_EMPTY);
    }
  }
  void start_timer() {
    if (timer_handle) {
      mcpwm_timer_start_stop(timer_handle, MCPWM_TIMER_START_NO_STOP);
    }
  }

 private:
  mcpwm_cmpr_handle_t comparator; // MCPWM comparator handle for controlling pulse width
  mcpwm_timer_handle_t timer_handle; // MCPWM timer handle for controlling the PWM frequency
  int current_angle; // Current angle of the servo
  int speed; // Movement speed
  int nsteps; // Step counter for smooth movement
};

/**
 * @brief Initializes the MCPWM peripheral for servo control.
 */
void ServoController::init_servo() {
  ESP_LOGI(SERVO_TAG, "Initializing MCPWM for the servo on GPIO %d", SERVO_PULSE_GPIO);
    
  // Create the timer (Timer)
  mcpwm_timer_config_t timer_config = {};
  timer_config.group_id = 0;
  timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  timer_config.resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ;
  timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
  timer_config.period_ticks = SERVO_TIMEBASE_PERIOD;
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer_handle));
    
  // Create the operator
  mcpwm_oper_handle_t oper = NULL;
  mcpwm_operator_config_t operator_config = {};
  operator_config.group_id = 0; 
  ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));
    
  // Connect the operator to the timer
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer_handle));
    
  // Create the comparator
  mcpwm_comparator_config_t comparator_config = {};
  comparator_config.flags.update_cmp_on_tez = true;
  ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));
    
  // Create the PWM generator
  mcpwm_gen_handle_t generator = NULL;
  mcpwm_generator_config_t generator_config = {};
  generator_config.gen_gpio_num = SERVO_PULSE_GPIO;
  ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));
    
  // Configure generator actions:
  // HIGH at the start of the timer (empty), then LOW when the comparator threshold is reached
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));
            
  // Initialize in neutral position
  uint32_t initial_pulse = SERVO_STOP_PULSEWIDTH;
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, initial_pulse));

  // Enable and start the timer
  ESP_ERROR_CHECK(mcpwm_timer_enable(timer_handle));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer_handle, MCPWM_TIMER_STOP_EMPTY));
}
              
/**
 * @brief Establishes a specific angle
 * @param angle Desired angle (0 - 180).
 */
void ServoController::set_servo_angle(int angle) {
  if (angle < SERVO_MIN_DEGREE) angle = SERVO_MIN_DEGREE;
  if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;
    
  if (comparator != NULL) {
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle));
    current_angle = angle;
  }
}

/**
 * @brief Smoothly transitions to a specified angle calculated by software.
 * @param target_angle Target angle (0 - 180).
 * @param time_ms Time in milliseconds that the complete movement should take.
 */
void ServoController::set_servo_angle_smooth(int target_angle, int time_ms) {
  if (target_angle < SERVO_MIN_DEGREE) target_angle = SERVO_MIN_DEGREE;
  if (target_angle > SERVO_MAX_DEGREE) target_angle = SERVO_MAX_DEGREE;
    
  if (time_ms <= 0 || current_angle == target_angle) {
    set_servo_angle(target_angle);
    return;
  }
    
  int step_delay_ms = 20; // Update every 20ms (50Hz update rate)
  int total_steps = time_ms / step_delay_ms;
  if (total_steps == 0) total_steps = 1;
    
  float angle_step = (float)(target_angle - current_angle) / total_steps;
  float temp_angle = current_angle;
    
  for (int i = 0; i < total_steps; i++) {
    temp_angle += angle_step;
        
    // Update the comparator with the new pulse width corresponding to the intermediate angle
    uint32_t pulse_us = angle_to_pulse_width((int)temp_angle);
    mcpwm_comparator_set_compare_value(comparator, pulse_us);
        
    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
  }
    
  // Ensure we end at the exact target angle
  set_servo_angle(target_angle);
}

/**
 * @brief Convert a given angle in degrees to the corresponding pulse width in microseconds for the MG996R servo.
 * @param angle Desired angle in degrees (0 - 180).
 */
inline uint32_t ServoController::angle_to_pulse_width(int angle) {
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / 
           (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

/**
 * @brief Establishes a specific pulse width in microseconds for the servo.
 * @param pulse_us Pulse width in microseconds.
 */
void ServoController::set_servo_pulse(uint32_t pulse_us) {
  if (comparator != NULL) {
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, pulse_us));
  }
}

#endif // MG996R_H
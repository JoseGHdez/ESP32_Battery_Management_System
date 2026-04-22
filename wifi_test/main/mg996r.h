#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"

static const char *SERVO_TAG = "servo_control";

// --- CONFIGURACIÓN DEL SERVO ---
// Ajusta estos valores según tu servo (SG90 estándar de 180 grados)
#define SERVO_MIN_PULSEWIDTH_US 500  // Ancho de pulso mínimo en microsegundos (0 grados)
#define SERVO_MAX_PULSEWIDTH_US 2100 // Ancho de pulso máximo en microsegundos (180 grados)
#define SERVO_MIN_DEGREE        -90    // Ángulo mínimo
#define SERVO_MAX_DEGREE        90 // Ángulo máximo

#define SERVO_STOP_PULSEWIDTH 1500 // Stop pulse width in microseconds

// Configuración de Hardware M5StickC PLUS2
#define SERVO_PULSE_GPIO             26       // Pin GPIO donde está conectado el servo
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us por tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms (frecuencia de 50Hz estándar para servos)

class ServoController {
 public:
  // Constructor para inicializar las variables
  ServoController() : comparator(NULL), current_angle(0), speed(100), nsteps(1) {}

  void init_servo();
  void set_servo_angle(int angle);
  void set_servo_angle_smooth(int target_angle, int time_ms);
  void set_servo_pulse(uint32_t pulse_us);
  uint32_t angle_to_pulse_width(int angle);
  int get_servo_speed() const { return speed; } // Getter para la velocidad del servo
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
  mcpwm_cmpr_handle_t comparator; // Manejador del comparador MCPWM
  mcpwm_timer_handle_t timer_handle; // Manejador del timer MCPWM
  int current_angle; // Ángulo actual del servo
  int speed; // Velocidad de movimiento (us/ms)
  int nsteps; // Número de pasos para la transición suave
};

/**
 * @brief Inicializa el periférico MCPWM para el control del servo.
 */
void ServoController::init_servo() {
  ESP_LOGI(SERVO_TAG, "Inicializando MCPWM para el servo en GPIO %d", SERVO_PULSE_GPIO);
    
  // 1. Crear el temporizador (Timer)
  mcpwm_timer_config_t timer_config = {};
  timer_config.group_id = 0;
  timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  timer_config.resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ;
  timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
  timer_config.period_ticks = SERVO_TIMEBASE_PERIOD;
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer_handle));
    
  // 2. Crear el operador
  mcpwm_oper_handle_t oper = NULL;
  mcpwm_operator_config_t operator_config = {};
  operator_config.group_id = 0; // El operador debe estar en el mismo grupo que el timer
  ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));
    
  // 3. Conectar timer y operador
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer_handle));
    
  // 4. Crear el comparador
  mcpwm_comparator_config_t comparator_config = {};
  comparator_config.flags.update_cmp_on_tez = true;
  ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));
    
  // 5. Crear el generador PWM
  mcpwm_gen_handle_t generator = NULL;
  mcpwm_generator_config_t generator_config = {};
  generator_config.gen_gpio_num = SERVO_PULSE_GPIO;
  ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));
    
  // 6. Configurar las acciones del generador
  // Poner en ALTO cuando el timer se reinicia (0)
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // Poner en BAJO cuando se alcanza el umbral del comparador
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));
            
  // 7. Iniciar en la posición 0
  uint32_t initial_pulse = SERVO_STOP_PULSEWIDTH;
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, initial_pulse));

  // 8. Habilitar y arrancar el timer
  ESP_ERROR_CHECK(mcpwm_timer_enable(timer_handle));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer_handle, MCPWM_TIMER_STOP_EMPTY));
}
              
/**
 * @brief Establece el ángulo del servo de forma inmediata usando MCPWM.
 * @param angle Ángulo deseado (0 - 180).
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
 * @brief Transición suave a un ángulo especificado calculada por software.
 * @param target_angle Ángulo final deseado.
 * @param time_ms Tiempo en milisegundos que debe tardar el movimiento completo.
 */
void ServoController::set_servo_angle_smooth(int target_angle, int time_ms) {
  if (target_angle < SERVO_MIN_DEGREE) target_angle = SERVO_MIN_DEGREE;
  if (target_angle > SERVO_MAX_DEGREE) target_angle = SERVO_MAX_DEGREE;
    
  if (time_ms <= 0 || current_angle == target_angle) {
    set_servo_angle(target_angle);
    return;
  }
    
  int step_delay_ms = 20; // Actualizar cada 20ms (sincronizado con los 50Hz del PWM)
  int total_steps = time_ms / step_delay_ms;
  if (total_steps == 0) total_steps = 1;
    
  float angle_step = (float)(target_angle - current_angle) / total_steps;
  float temp_angle = current_angle;
    
  for (int i = 0; i < total_steps; i++) {
    temp_angle += angle_step;
        
    // Actualizamos el comparador directamente para no afectar current_angle hasta el final
    uint32_t pulse_us = angle_to_pulse_width((int)temp_angle);
    mcpwm_comparator_set_compare_value(comparator, pulse_us);
        
    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
  }
    
  // Asegurar que llegamos exactamente a la posición final
  set_servo_angle(target_angle);
}

/**
 * @brief Convierte un ángulo a microsegundos basándose en los límites configurados.
 */
inline uint32_t ServoController::angle_to_pulse_width(int angle) {
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / 
           (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

/**
 * @brief Establece un ancho de pulso específico en microsegundos para el servo.
 * @param pulse_us Ancho de pulso en microsegundos.
 */
void ServoController::set_servo_pulse(uint32_t pulse_us) {
  if (comparator != NULL) {
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, pulse_us));
  }
}
#ifndef SHARED_H
#define SHARED_H

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/i2c_master.h"

#include "I2Crelay.h" // Header file for relay control functions
#include "ADS1115.h" // Header file for ADS1115 ADC functions
#include "mg996r.h" // Header file for MG996R servo control functions

// ============ UART CONFIGURATION ============
#define UART_NUM UART_NUM_0 /*!<@brief UART port number for console output and command input */
#define BUF_SIZE 256 /*!<@brief Buffer size for UART communication */

// ============ GPIO pins ============
#define GPIO_OUTPUT_IO_0    GPIO_NUM_18
#define GPIO_OUTPUT_IO_1    GPIO_NUM_19
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))

#define GPIO_INPUT_IO_0     GPIO_NUM_39
#define GPIO_INPUT_IO_1     GPIO_NUM_37
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))

#define ESP_INTR_FLAG_DEFAULT 0

// Program parameters and shared definitions for the battery management system
/** 
 * @brief Logger tag for the M5StickCPlus2 system 
 */
static const char *TAG = "M5_SYSTEM";

/** 
 * @brief Queue for sending messages to the display task
 */
static QueueHandle_t xQueueDisplay = NULL;

/** 
 * @brief Queue for receiving UART input data
 */
static QueueHandle_t uart_queue;

/**
 * @brief Button interruption queue
 */
static QueueHandle_t gpio_event_queue = NULL;

bool experiment_flag = false; // Flag to indicate whether we are in an experiment or not
bool is_waiting_discharge = false; // Flag to indicate whether we are waiting for the user to start the discharge phase

struct ExperimentParams {
  float charge_bias; // Bias voltage to apply during charging (to counteract sensor offset)
  float discharge_bias; // Bias voltage to apply during discharging (to counteract sensor offset)
  std::string mosquitto_ip; // IP address of the MQTT broker to publish results to
  std::string experiment_name; // Name of the experiment (used in MQTT topic and logging)
  std::string experiment_tags; // Custom tag for the experiment (used in MQTT topic and logging)
  std::vector<std::string> experiment_tags_list; // List of custom tags for the experiment (used in MQTT topic and logging)
  std::string battery_type; // Type of battery being tested (used in MQTT topic and logging)
  std::vector<std::string> tags; // Custom tags for the experiment (used in MQTT topic and logging)
  uint8_t charge_time_limit_minutes; // Time limit for the charging phase in minutes (safety cutoff)
  uint8_t discharge_time_limit_minutes; // Time limit for the discharging phase in minutes
  std::string device_name; // Name of the device running the experiment (used in MQTT topic and logging)
  std::string location; // Location where the experiment is conducted (used in MQTT topic and logging)
  uint16_t battery_capacity_mah; // Battery capacity in ampere-hours (used for calculating coulombs passed)
  uint8_t percentage_resistance_1; // Percentage of battery capacity at which to measure internal resistance for the first time
  uint8_t percentage_resistance_2; // Percentage of battery capacity at which to measure internal resistance for the second time
  uint8_t percentage_resistance_3; // Percentage of battery capacity at which to measure internal resistance for the third time
};

struct AppContext {
  ServoController *servoController; // Pointer to the ServoController instance
  I2CRelay *relayController; // Pointer to the I2CRelay instance
  ADS1115 *adsController; // Pointer to the ADS1115 instance
  ExperimentParams *experimentParams; // Struct to hold experiment configuration parameters
};

// Parameter block passed to MeasureCharging / MeasureDischarging tasks.
// caller_task is the ExperimentCycle handle to notify when the phase is done.
struct PhaseTaskParams {
    AppContext     *app_context;
    TaskHandle_t    caller_task;  // ExperimentCycle notified via ulTaskNotifyTake
    uint16_t        cycle_index;  // Which cycle this phase belongs to (for logging)
};

/**
 * @brief Scans the I2C bus for connected devices and prints their addresses to the console.
 * 
 * This function performs the following steps:
 * 1. Iterates through all possible I2C addresses (1 to 126).
 * 2. For each address, it creates an I2C command to write a byte (the address shifted left by 1 with the write bit) and checks for an acknowledgment from a device at that address.
 * 3. If a device acknowledges the address, it prints the address in hexadecimal format to the console.
 * 4. If there is an error during communication (other than no acknowledgment), it prints the error message for debugging purposes.
 * 5. This function is useful for diagnosing I2C communication issues and verifying that devices are properly connected to the bus before attempting to read or write data from them.
 */
// void ScanI2CBus() {
//   printf("Escaneando bus I2C...\n");
//   for (int i = 1; i < 127; i++) {
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
//     i2c_master_stop(cmd);
//     esp_err_t res = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
//     i2c_cmd_link_delete(cmd);
//     if (res == ESP_OK) {
//         printf("¡Dispositivo encontrado en dirección: 0x%02X!\n", i);
//     }
//     vTaskDelay(pdMS_TO_TICKS(1));
//   }
// }

#endif // SHARED_H
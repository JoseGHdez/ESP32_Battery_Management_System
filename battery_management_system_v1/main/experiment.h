#ifndef EXPERIMENT_H
#define EXPERIMENT_H

#include <stdio.h>    // printf, snprintf
#include <string.h>   // memset, strcmp, strcasecmp
#include <inttypes.h> // int16_t, uint8_t, etc.
#include <iostream>   // std::cout, std::endl
#include <cmath>      // std::abs for floating-point comparisons
#include <ctime>      // std::time for timestamping

#include "freertos/FreeRTOS.h" // FreeRTOS types and functions (e.g., vTaskDelay, xQueueSend)
#include "freertos/task.h"     // FreeRTOS task functions (e.g., xTaskCreate)
#include "freertos/queue.h"    // FreeRTOS queue functions (e.g., xQueueCreate, xQueueSend)
#include "esp_err.h" // esp_err_t type and error codes (e.g., ESP_OK, ESP_FAIL)
#include "esp_log.h" // ESP32 logging functions (e.g., ESP_LOGI, ESP_LOGE)
#include "esp_timer.h" // ESP32 timer functions (e.g., esp_timer_get_time)
#include "driver/gpio.h" // GPIO functions for controlling pins (e.g., gpio_set_level, gpio_set_direction)
#include "driver/uart.h" // UART functions for serial communication (e.g., uart_read_bytes, uart_write_bytes)
#include "driver/i2c.h" // I2C functions for communication with peripherals (e.g., i2c_master_write_to_device, i2c_master_write_read_device)
#include "esp_event.h"  // ESP32 event loop functions (e.g., esp_event_loop_create_default, esp_event_handler_register)
#include "mqtt_client.h" // MQTT client functions for publishing data (e.g., esp_mqtt_client_init, esp_mqtt_client_publish)
#include "cJSON.h"  // cJSON library for JSON parsing and generation (e.g., cJSON_Parse, cJSON_Print)

#include "wifi_mqtt.h" // Header file for WiFi and MQTT functions
#include "shared.h" // Header file for common definitions and utilities
#include "buzzer.h" // Header file for buzzer control functions

// Experiment configuration variables
#define EXPERIMENT_CICLES        1      // Number of charge/discharge cycles to perform in the experiment
#define EXPERIMENT_INTERVAL_MS   1000   // Sampling interval within each phase (ms)
#define CHARGE_DURATION_MS       30000  // How long to measure during charging phase (ms)
#define DISCHARGE_DURATION_MS    30000  // How long to measure during discharging phase (ms)
#define BATTERY_CAPACITY_AH      2.2f   // Battery capacity in Ah (used for coulomb counting approximation)

/**
  @brief Handle for the ExperimentCycle task.

  Tracks the running ExperimentCycle task so it can be safely stopped from 
  other tasks (e.g., ButtonTask or UARTTask). When stopping, it also ensures
  that any active MeasureCharging or MeasureDischarging task is killed and 
  that semaphores are released to prevent deadlocks or heap corruption.
*/
static TaskHandle_t experiment_task_handle = NULL;

/**
  @brief Handle for the active phase task (MeasureCharging or MeasureDischarging).

  This is used to track which phase task is currently running so that it can be
  safely stopped when ExperimentCycle is stopped. Since ExperimentCycle creates
  a new task for each phase, we need to keep track of it to ensure proper 
  cleanup and resource management.
*/
static TaskHandle_t phase_task_handle = NULL;

/**
  @brief Timestamp (in microseconds) when the experiment started.
*/
static int64_t experiment_start_us = 0;

/**
  @brief Semaphore to signal that the discharge phase can start.
  
  Blocked after charge phase. It is released by either pressing 
  Button B (GPIO 39) or by sending a "DISCHARGE" command over UART. 
*/
static SemaphoreHandle_t discharge_ready_sem = NULL;

/** @brief Semaphore to signal that the current phase should be skipped.
  
  Cuts the current phase short. When GPIO 39 is pressed mid-phase, 
  the running MeasureCharging/MeasureDischarging task checks this and
  exits early, handing control back to ExperimentCycle.
*/
static SemaphoreHandle_t phase_skip_sem = NULL;

// Forward declarations — defined later in this file
extern "C" void ExperimentCycle(void *pvParameters);
static void MeasureCharging(void *pvParameters);
static void MeasureDischarging(void *pvParameters);

uint16_t success_count = 0;
uint16_t failure_count = 0;
bool error_occured = false;

/** @brief Starts the ExperimentCycle task.
  
  Safe to call from ButtonTask or UARTTask. Does nothing if the task is already running.
*/
static void start_experiment_task(AppContext *context) {
    if (experiment_task_handle != NULL) {
        ESP_LOGW(TAG, "ExperimentCycle already running");
        return;
    }
    context->relayController->set_all_relays_mask(0x0); // Ensure all relays are off before starting
    experiment_flag = true;
    experiment_start_us = esp_timer_get_time(); // Record start timestamp
    xSemaphoreTake(phase_skip_sem, 0); 
    xSemaphoreTake(discharge_ready_sem, 0);
    xTaskCreate(ExperimentCycle, "EXP_CYCLE", 1024 * 4, context, 999, &experiment_task_handle);
    ESP_LOGI(TAG, "ExperimentCycle task started");
}

/** @brief Stops the ExperimentCycle task if it is running.
  
  Safe to call from ButtonTask or UARTTask. Does nothing if the task is not running.
*/
static void stop_experiment_task(AppContext *context) {
    if (experiment_task_handle == NULL) {
        ESP_LOGW(TAG, "ExperimentCycle not running");
        return;
    }
    context->relayController->set_all_relays_mask(0x0); // Ensure all relays are off
    experiment_flag = false;

    // Kill the active phase task first (MeasureCharging or MeasureDischarging).
    if (phase_task_handle != NULL) {
        vTaskDelete(phase_task_handle);
        phase_task_handle = NULL;
    }

    // Release semaphores in case the ExperimentCycle task is blocked waiting for them.
    if (discharge_ready_sem != NULL) {
        xSemaphoreGive(discharge_ready_sem);
    }
    if (phase_skip_sem != NULL) {
        xSemaphoreGive(phase_skip_sem);
    }

    vTaskDelete(experiment_task_handle);
    experiment_task_handle = NULL;
    ESP_LOGI(TAG, "ExperimentCycle task stopped");
}

// ============ EXPERIMENT PHASE TASKS ============

/**
  @brief Task function for measuring the charging phase of the experiment.

  This task is created by ExperimentCycle when it enters the charging phase. 
  It reads voltage and current from the ADS1115 at regular intervals, 
  checks for safety conditions (e.g., overvoltage), and updates the display 
  with the current measurements. It also keeps track of coulombs passed and 
  estimates state of charge based on the current readings.
*/
static void MeasureCharging(void *pvParameters) {
  PhaseTaskParams *phase_params = static_cast<PhaseTaskParams *>(pvParameters);
  AppContext *ctx = phase_params->app_context;
  ExperimentParams *exp_params = ctx->experimentParams;
  ADS1115 &adc = *(ctx->adsController);
  const uint16_t cycle = phase_params->cycle_index;
  auto phase_start_time = std::time(nullptr);

  success_count = 0;
  failure_count = 0;
  char uart_msg[160];
  char display_msg[128];
  uint8_t safety_breach_count = 0;
  float coulomb_count = 0.0f;
  uint32_t elapsed_s = 0;
  //float state_of_charge = 0.0f;
  float last_v1 = 0.0f, last_v2 = 0.0f, last_v3 = 0.0f, last_current = 0.0f;

  uart_write_bytes(UART_NUM, "--- CHARGING PHASE ---\r\n", 24);
  
  while (1) {
    auto config_cell_1 = adc.ConfigureADS1115(3); // Configure for Cell 1
    vTaskDelay(pdMS_TO_TICKS(80)); 
    auto voltage_cell_1  = adc.ReadVoltage(3);
    vTaskDelay(pdMS_TO_TICKS(80)); 
    auto config_cell_2 = adc.ConfigureADS1115(2); // Configure for Cell 2
    vTaskDelay(pdMS_TO_TICKS(80)); 
    auto voltage_cell_2  = adc.ReadVoltage(2, true);
    vTaskDelay(pdMS_TO_TICKS(80)); 
    auto config_cell_3 = adc.ConfigureADS1115(1); // Configure for Cell 3
    vTaskDelay(pdMS_TO_TICKS(80)); 
    auto voltage_cell_3  = adc.ReadVoltage(1, true);
    vTaskDelay(pdMS_TO_TICKS(80)); 
    auto config_current = adc.ConfigureADS1115(0); // Configure for Current
    vTaskDelay(pdMS_TO_TICKS(80)); 
    auto current  = adc.ReadCurrent(0, 0.000062f);
    vTaskDelay(pdMS_TO_TICKS(80)); 
      
    if (voltage_cell_1.second != ESP_OK || voltage_cell_2.second != ESP_OK 
        || voltage_cell_3.second != ESP_OK || current.second != ESP_OK
        || config_cell_1.second != ESP_OK || config_cell_2.second != ESP_OK
        || config_cell_3.second != ESP_OK || config_current.second != ESP_OK
      ) {
      ++failure_count;
      error_occured = true;
      if (voltage_cell_1.second != ESP_OK || config_cell_1.second != ESP_OK) {
        voltage_cell_1.first = last_v1; // Use last valid reading
        ESP_LOGE(TAG, "[CHG] Cell 1 read/config error! Using last valid reading.");
      }
      if (voltage_cell_2.second != ESP_OK || config_cell_2.second != ESP_OK) {
        voltage_cell_2.first = last_v2; // Use last valid reading
        ESP_LOGE(TAG, "[CHG] Cell 2 read/config error! Using last valid reading.");
      }
      if (voltage_cell_3.second != ESP_OK || config_cell_3.second != ESP_OK) {
        voltage_cell_3.first = last_v3; // Use last valid reading
        ESP_LOGE(TAG, "[CHG] Cell 3 read/config error! Using last valid reading.");
      }
      if (current.second != ESP_OK || config_current.second != ESP_OK) {
        current.first = last_current; // Use last valid reading
        ESP_LOGE(TAG, "[CHG] Current read/config error! Using last valid reading.");
      }
      ESP_LOGE(TAG, "[CHG] Sensor read error detected! V1: %.2f V, V2: %.2f V, V3: %.2f V, I: %.2f A", voltage_cell_1.first, voltage_cell_2.first, voltage_cell_3.first, current.first);
      buzzer_warn();
      buzzer_warn();
    }

    if (exp_params->timer_flag) {
      elapsed_s = (uint32_t)((esp_timer_get_time() - phase_start_time) / 1000000); // Elapsed time in s
      if (elapsed_s >= exp_params->charge_time_limit_minutes * 60) {
        ESP_LOGE(TAG, "[CHG] Time limit reached! Elapsed: %d s", elapsed_s);
        ctx->relayController->set_all_relays_mask(0x0);
        buzzer_warn();
        break;
      }
    }

    // Safety check
    if (voltage_cell_3.first >= 12.5f) {
      if (safety_breach_count < 5) {
        ESP_LOGW(TAG, "[CHG] High voltage detected! V: %.2f V. Breach count: %d",
                 voltage_cell_3.first, safety_breach_count + 1);
        safety_breach_count++;
      } else {
        ESP_LOGE(TAG, "[CHG] Battery is charged! V: %.2f V", 
                 voltage_cell_3.first);
        buzzer_warn();
        break;
      }
    }

    if (!error_occured) {
      coulomb_count += (std::fabs(current.first) * 
          (EXPERIMENT_INTERVAL_MS / 1000.0f)); // I * t (in seconds) gives coulombs
      last_v1 = voltage_cell_1.first;
      last_v2 = voltage_cell_2.first;
      last_v3 = voltage_cell_3.first;
      last_current = current.first;
      ESP_LOGI(TAG, "[CHG] Sample OK. Current: %.2f A, Coulombs: %.4f C", 
               current.first, coulomb_count);
    }
    
    // Show data on display
    snprintf(display_msg, sizeof(display_msg),
             "CHG - Cycle: %d\r\n"
             "C1: %.2fV | C2: %.2fV\r\n"
             "C3: %.2fV | Bat: %.2fV\r\n"
             "Current: %.2fA",
             cycle, voltage_cell_1.first, 
             voltage_cell_2.first == -1.0f ? -1.0f : (voltage_cell_2.first / 2), 
             voltage_cell_3.first == -1.0f ? -1.0f : (voltage_cell_3.first / 3), 
             voltage_cell_3.first, current.first);
      
    if (xQueueDisplay) {
      xQueueSend(xQueueDisplay, display_msg, 0);
    }

    // Log data to UART
    snprintf(uart_msg, sizeof(uart_msg),
             "[CHG]\r\n"
             "cycle: %d\r\n"
             "Cell 1: %.2fV\r\n"
             "Cell 2: %.2fV\r\n"
             "Cell 3: %.2fV\r\n"
             "Battery: %.2fV\r\n"
             "Current: %.2fA\r\n",
             cycle, voltage_cell_1.first, 
             (voltage_cell_2.first == -1.0f) ? -1.0f : (voltage_cell_2.first / 2), 
             (voltage_cell_3.first == -1.0f) ? -1.0f : (voltage_cell_3.first / 3), 
             voltage_cell_3.first, current.first);
    uart_write_bytes(UART_NUM, uart_msg, strlen(uart_msg));

    // Publish data to MQTT
    if (mqtt_client) {
      elapsed_s = (uint32_t)((esp_timer_get_time() - experiment_start_us) / 1000000);
      cJSON *root = cJSON_CreateObject();

      if (root != NULL) {
        cJSON_AddStringToObject(root, "phase", "charge");
        cJSON_AddNumberToObject(root, "cycle", cycle);
        cJSON_AddStringToObject(root, "device", 
          exp_params->device_name.c_str()); 
        cJSON_AddStringToObject(root, "exp_name", 
          exp_params->experiment_name.c_str());
        cJSON_AddStringToObject(root, "date", 
          std::string(phase_params->start_date).c_str());
        cJSON_AddStringToObject(root, "place", 
          exp_params->location.c_str());
        cJSON_AddNumberToObject(root, "t_s", elapsed_s);
        cJSON_AddStringToObject(root, "bat_type", 
          exp_params->battery_type.c_str());
        cJSON_AddNumberToObject(root, "bat_cap_Ah", 
          exp_params->battery_capacity_ah);
        cJSON_AddStringToObject(root, "sub_tag", 
          exp_params->current_tag.c_str());
        cJSON_AddNumberToObject(root, "cell_1",  voltage_cell_1.first);
        cJSON_AddNumberToObject(root, "cell_2",  
          voltage_cell_2.first == -1.0f ? -1.0f : (voltage_cell_2.first / 2.0));
        cJSON_AddNumberToObject(root, "cell_3", 
          voltage_cell_3.first == -1.0f ? -1.0f : (voltage_cell_3.first / 3.0));
        cJSON_AddNumberToObject(root, "battery", voltage_cell_3.first);
        cJSON_AddNumberToObject(root, "current", current.first);
        cJSON_AddNumberToObject(root, "coulombs", coulomb_count);
        cJSON_AddNumberToObject(root, "success_count", success_count);
        cJSON_AddNumberToObject(root, "failure_count", failure_count);
        
        // Generate the JSON string from the cJSON object
        char *mqtt_payload_dynamic = cJSON_PrintUnformatted(root);

        if (mqtt_payload_dynamic != NULL) {
          esp_mqtt_client_publish(mqtt_client, "m5stick/sensors/charge", 
            mqtt_payload_dynamic, 0, 1, 0);
          free(mqtt_payload_dynamic);
        } 

        cJSON_Delete(root);
      }
    }
  
    if (!error_occured) {
      ++success_count;
      error_occured = false;
      ESP_LOGV(TAG, "[CHG] Successful sample. Total successes: %d", 
               success_count);
      buzzer_beep(50);
    }

    error_occured = false; // Reset error flag for next cycle

    // Wait for the next sample interval, but exit early if Button B is pressed
    if (xSemaphoreTake(phase_skip_sem, pdMS_TO_TICKS(EXPERIMENT_INTERVAL_MS)) == pdTRUE) {
      uart_write_bytes(UART_NUM, "[CHG] Phase skipped by user\r\n", 29);
      break;
    }
  }

  phase_params->mas_charge = coulomb_count; // Ampere-seconds charged during this phase

  // Notify ExperimentCycle that this phase is complete
  xTaskNotifyGive(phase_params->caller_task);
  vTaskDelete(NULL);
}

// Runs during the discharging phase. Same structure as MeasureCharging but
// uses a different safety threshold direction and MQTT phase label.
static void MeasureDischarging(void *pvParameters) {
  PhaseTaskParams *phase_params = static_cast<PhaseTaskParams *>(pvParameters);
  AppContext *ctx = phase_params->app_context;
  ExperimentParams *exp_params = ctx->experimentParams;
  ADS1115 &adc = *(ctx->adsController);
  const uint16_t cycle = phase_params->cycle_index;

  char uart_msg[160];
  char display_msg[256];
  uint8_t safety_breach_count = 0;
  success_count = 0;
  failure_count = 0;
  uint32_t elapsed_s = 0;
  float state_of_charge = 0.0f;
  float last_current = 0.0f;
  float charge = exp_params->battery_capacity_ah * 3600.0f; // Total charge in coulombs (Ah * 3600 s/h)

  uart_write_bytes(UART_NUM, "--- DISCHARGING PHASE ---\r\n", 27);

  adc.ConfigureADS1115(3); // Configure for Cell 1
  vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
  auto voltage_cell_1  = adc.ReadVoltage(3);
  vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
  adc.ConfigureADS1115(2); // Configure for Cell 2
  vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
  auto voltage_cell_2  = adc.ReadVoltage(2, true);
  vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
  adc.ConfigureADS1115(1); // Configure for Cell 3
  vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
  auto voltage_cell_3  = adc.ReadVoltage(1, true);
  vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
  adc.ConfigureADS1115(0); // Configure for Current
  vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
  auto current  = adc.ReadCurrent(0, -0.00625f);
  last_current = current.first;
  vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
  float coulomb_count = 0.0f;
  float cell_resitance_1 = 0.0f;
  float cell_resitance_2 = 0.0f;
  float cell_resitance_3 = 0.0f;
  float voltage_no_load_1 = 0.0f;
  float voltage_no_load_2 = 0.0f;
  float voltage_no_load_3 = 0.0f;
  bool alerted_90_percent = false;
  bool alerted_80_percent = false;
  bool alerted_20_percent = false;
  bool alerted_50_percent = false;

  cell_resitance_1 = (phase_params->initial_voltage_cell_1 - voltage_cell_1.first) / std::fabs(current.first == 0.0f ? 0.001f : current.first);
  cell_resitance_2 = ((phase_params->initial_voltage_cell_2 / 2) - (voltage_cell_2.first / 2)) / std::fabs(current.first == 0.0f ? 0.001f : current.first);
  cell_resitance_3 = ((phase_params->initial_voltage_cell_3 / 3) - (voltage_cell_3.first / 3)) / std::fabs(current.first == 0.0f ? 0.001f : current.first);

  if (voltage_cell_1.first <= 3.2f || (voltage_cell_2.first / 2) <= 3.2f || (voltage_cell_3.first / 3) <= 3.2f) {
    ESP_LOGE(TAG, "[DIS] Safety threshold — aborting\nC1: %.2f V, C2: %.2f V, C3: %.2f V", voltage_cell_1.first, voltage_cell_2.first / 2, voltage_cell_3.first / 3);
    buzzer_alert();
    experiment_flag = false; // Finish experiments
    ctx->relayController->set_all_relays_mask(0x0);
    xTaskNotifyGive(phase_params->caller_task);
    vTaskDelete(NULL);
    return;
  }

  while (1) {
    auto config_cell_1 = adc.ConfigureADS1115(3); // Configure for Cell 1
    vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
    voltage_cell_1  = adc.ReadVoltage(3);
    vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
    auto config_cell_2 = adc.ConfigureADS1115(2); // Configure for Cell 2
    vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
    voltage_cell_2  = adc.ReadVoltage(2, true);
    vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
    auto config_cell_3 = adc.ConfigureADS1115(1); // Configure for Cell 3
    vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
    voltage_cell_3  = adc.ReadVoltage(1, true);
    vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
    auto config_current = adc.ConfigureADS1115(0); // Configure for Current
    vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
    current  = adc.ReadCurrent(0, -0.00625f); //-0.625
    vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free

    // Safety check — cut off if any cell drops below 3.5V during discharge
    if (voltage_cell_1.second != ESP_OK || voltage_cell_2.second != ESP_OK 
        || voltage_cell_3.second != ESP_OK || current.second != ESP_OK
        || config_cell_1.second != ESP_OK || config_cell_2.second != ESP_OK
        || config_cell_3.second != ESP_OK || config_current.second != ESP_OK) {
      ++failure_count;
      error_occured = true;
      ESP_LOGE(TAG, "[DIS] Sensor read error detected! V1: %.2f V, V2: %.2f V, V3: %.2f V, I: %.2f A", voltage_cell_1.first, voltage_cell_2.first, voltage_cell_3.first, current.first);
      buzzer_warn();
      buzzer_warn();
    }
    
    if ((voltage_cell_1.first <= 3.0f || (voltage_cell_2.first / 2) <= 3.0f || (voltage_cell_3.first / 3) <= 3.0f) && !error_occured) {
      ESP_LOGE(TAG, "[DIS] Low voltage detected! C1: %.2f V, C2: %.2f V, C3: %.2f V", voltage_cell_1.first, voltage_cell_2.first / 2, voltage_cell_3.first / 3);
      if (voltage_cell_1.first <= 2.8f || (voltage_cell_2.first / 2) <= 2.8f || (voltage_cell_3.first / 3) <= 2.8f) {
        buzzer_alert();
      }
      safety_breach_count++;
    }

    if (exp_params->timer_flag) {
      elapsed_s = (uint32_t)((esp_timer_get_time() - experiment_start_us) / 1000000); // Elapsed time in seconds
      if (elapsed_s >= exp_params->discharge_time_limit_minutes * 60) {
        ESP_LOGE(TAG, "[DIS] Time limit reached! Elapsed: %d s", elapsed_s);
        ctx->relayController->set_all_relays_mask(0x0);
        buzzer_warn();
        break;
      }
    }

    if ((safety_breach_count >= 3)) { // Allow a few transient breaches before cutting off
      ctx->relayController->set_all_relays_mask(0x0);
      buzzer_warn();
      break;
    }

    if (!error_occured) {
      ESP_LOGI(TAG, "[DIS] Sample OK. Current: %.2f A, Coulombs: %.4f C", current.first, coulomb_count);
      // Trapezoidal integration to estimate charge passed during this interval: Q = Q + 0.5 * (I0 + I1) * dt
      float dt_seconds = EXPERIMENT_INTERVAL_MS / 1000.0f;
      charge -= dt_seconds * 0.5f * std::fabs(last_current + current.first);

      // Estimate state of charge as percentage: SoC = (Q / Q_total) * 100
      state_of_charge = (charge / (exp_params->battery_capacity_ah * 3600.0f)) * 100.0f;

      // Coulomb counting for total charge passed
      coulomb_count += dt_seconds * 0.5f * (last_current + current.first);
      last_current = current.first;
      ESP_LOGI(TAG, "[DIS] Estimated SoC: %.2f%%\r\n"
                    "Charge: %.4f C\r\n"
                    "Coulombs: %.4f C\r\n", 
                    state_of_charge, charge, coulomb_count);
      
      if (!alerted_90_percent && state_of_charge <= 90.0f) {  //90
        ESP_LOGI(TAG, "[DIS] Battery at ~90%% capacity");
        buzzer_beep(200);
        alerted_90_percent = true;
        ctx -> relayController -> set_relay(1, false); // Cut the load for a moment to measure internal resistance without load
        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for voltages to stabilize
        std::pair<float, esp_err_t> aux_v1;
        std::pair<float, esp_err_t> aux_v2;
        std::pair<float, esp_err_t> aux_v3;
        std::pair<float, esp_err_t> conf_aux_v1;
        std::pair<float, esp_err_t> conf_aux_v2;
        std::pair<float, esp_err_t> conf_aux_v3;
        for (uint8_t i = 0; i < 5; ++i) {
          conf_aux_v1 = adc.ConfigureADS1115(3); // Configure for Cell 1
          while (conf_aux_v1.second != ESP_OK) {
            conf_aux_v1 = adc.ConfigureADS1115(3);
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is
          aux_v1 = adc.ReadVoltage(3);
          while (aux_v1.second != ESP_OK) {
            aux_v1 = adc.ReadVoltage(3);
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          voltage_no_load_1 += (aux_v1.second == ESP_OK) ? aux_v1.first : 0.0f;
          conf_aux_v2 = adc.ConfigureADS1115(2); // Configure for Cell 2
          while (conf_aux_v2.second != ESP_OK) {
            conf_aux_v2 = adc.ConfigureADS1115(2);
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
          aux_v2 = adc.ReadVoltage(2, true);
          while (aux_v2.second != ESP_OK) {
            aux_v2 = adc.ReadVoltage(2, true);
            vTaskDelay(pdMS_TO_TICKS(100)); 
          }
          voltage_no_load_2 += (aux_v2.second == ESP_OK) ? aux_v2.first / 2 : 0.0f;
          conf_aux_v3 = adc.ConfigureADS1115(1); // Configure for Cell 3
          while (conf_aux_v3.second != ESP_OK) {
            conf_aux_v3 = adc.ConfigureADS1115(1);
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
          aux_v3 = adc.ReadVoltage(1, true);
          while (aux_v3.second != ESP_OK) {
            aux_v3 = adc.ReadVoltage(1, true);
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          voltage_no_load_3 += (aux_v3.second == ESP_OK) ? aux_v3.first / 3 : 0.0f;
        }
        voltage_no_load_1 /= 5.0f;
        voltage_no_load_2 /= 5.0f;
        voltage_no_load_3 /= 5.0f;
        ESP_LOGI(TAG, "[DIS] Load voltages: V1: %.4f V, V2: %.4f V, V3: %.4f V, Current: %.2f A", voltage_cell_1.first, voltage_cell_2.first / 2, voltage_cell_3.first / 3, current.first);
        ESP_LOGI(TAG, "[DIS] No-load voltages: V1: %.4f V, V2: %.4f V, V3: %.4f V", voltage_no_load_1, voltage_no_load_2, voltage_no_load_3);
        cell_resitance_1 = (voltage_no_load_1 - voltage_cell_1.first) / std::fabs(current.first);
        cell_resitance_2 = ((voltage_no_load_2) - (voltage_cell_2.first / 2)) / std::fabs(current.first);
        cell_resitance_3 = ((voltage_no_load_3) - (voltage_cell_3.first / 3)) / std::fabs(current.first);
        ctx -> relayController -> set_relay(1, true); // Re-apply load
        snprintf(display_msg, sizeof(display_msg),
                 "[DIS] ~90%% SoC\r\n"
                 "R1: %.4f Ω\r\n"
                 "R2: %.4f Ω\r\n"
                 "R3: %.4f Ω",
                 cell_resitance_1, cell_resitance_2, cell_resitance_3);
      } else if (!alerted_80_percent && state_of_charge <= 80.0f) {  //90
        ESP_LOGI(TAG, "[DIS] Battery at ~80%% capacity");
        buzzer_beep(200);
        alerted_80_percent = true;
        ctx -> relayController -> set_relay(1, false); // Cut the load for a moment to measure internal resistance without load
        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for voltages to stabilize
        std::pair<float, esp_err_t> aux_v1;
        std::pair<float, esp_err_t> aux_v2;
        std::pair<float, esp_err_t> aux_v3;
        std::pair<float, esp_err_t> conf_aux_v1;
        std::pair<float, esp_err_t> conf_aux_v2;
        std::pair<float, esp_err_t> conf_aux_v3;
        for (uint8_t i = 0; i < 5; ++i) {
          conf_aux_v1 = adc.ConfigureADS1115(3); // Configure for Cell 1
          while (conf_aux_v1.second != ESP_OK) {
            conf_aux_v1 = adc.ConfigureADS1115(3);
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is
          aux_v1 = adc.ReadVoltage(3);
          while (aux_v1.second != ESP_OK) {
            aux_v1 = adc.ReadVoltage(3);
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          voltage_no_load_1 += (aux_v1.second == ESP_OK) ? aux_v1.first : 0.0f;
          conf_aux_v2 = adc.ConfigureADS1115(2); // Configure for Cell 2
          while (conf_aux_v2.second != ESP_OK) {
            conf_aux_v2 = adc.ConfigureADS1115(2);
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
          aux_v2 = adc.ReadVoltage(2, true);
          while (aux_v2.second != ESP_OK) {
            aux_v2 = adc.ReadVoltage(2, true);
            vTaskDelay(pdMS_TO_TICKS(100)); 
          }
          voltage_no_load_2 += (aux_v2.second == ESP_OK) ? aux_v2.first / 2 : 0.0f;
          conf_aux_v3 = adc.ConfigureADS1115(1); // Configure for Cell 3
          while (conf_aux_v3.second != ESP_OK) {
            conf_aux_v3 = adc.ConfigureADS1115(1);
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
          aux_v3 = adc.ReadVoltage(1, true);
          while (aux_v3.second != ESP_OK) {
            aux_v3 = adc.ReadVoltage(1, true);
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          voltage_no_load_3 += (aux_v3.second == ESP_OK) ? aux_v3.first / 3 : 0.0f;
        }
        voltage_no_load_1 /= 5.0f;
        voltage_no_load_2 /= 5.0f;
        voltage_no_load_3 /= 5.0f;
        ESP_LOGI(TAG, "[DIS] Load voltages: V1: %.4f V, V2: %.4f V, V3: %.4f V, Current: %.2f A", voltage_cell_1.first, voltage_cell_2.first / 2, voltage_cell_3.first / 3, current.first);
        ESP_LOGI(TAG, "[DIS] No-load voltages: V1: %.4f V, V2: %.4f V, V3: %.4f V", voltage_no_load_1, voltage_no_load_2, voltage_no_load_3);
        cell_resitance_1 = (voltage_no_load_1 - voltage_cell_1.first) / std::fabs(current.first);
        cell_resitance_2 = ((voltage_no_load_2) - (voltage_cell_2.first / 2)) / std::fabs(current.first);
        cell_resitance_3 = ((voltage_no_load_3) - (voltage_cell_3.first / 3)) / std::fabs(current.first);
        ctx -> relayController -> set_relay(1, true); // Re-apply load
        snprintf(display_msg, sizeof(display_msg),
                 "[DIS] ~80%% SoC\r\n"
                 "R1: %.4f Ω\r\n"
                 "R2: %.4f Ω\r\n"
                 "R3: %.4f Ω",
                 cell_resitance_1, cell_resitance_2, cell_resitance_3);
      } else if (!alerted_50_percent && state_of_charge <= 50.0f) {  //50
        ESP_LOGI(TAG, "[DIS] Battery at ~50%% capacity");
        buzzer_beep(500);
        alerted_50_percent = true;
        ctx -> relayController -> set_relay(1, false); // Cut the load for a moment to measure internal resistance without load
        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for voltages to stabilize
        std::pair<float, esp_err_t> aux_v1;
        std::pair<float, esp_err_t> aux_v2;
        std::pair<float, esp_err_t> aux_v3;
        std::pair<float, esp_err_t> conf_aux_v1;
        std::pair<float, esp_err_t> conf_aux_v2;
        std::pair<float, esp_err_t> conf_aux_v3;
        for (uint8_t i = 0; i < 5; ++i) {
          conf_aux_v1 = adc.ConfigureADS1115(3); // Configure for Cell 1
          while(conf_aux_v1.second != ESP_OK) {
            conf_aux_v1 = adc.ConfigureADS1115(3);
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          vTaskDelay(pdMS_TO_TICKS(80));
          aux_v1 = adc.ReadVoltage(3);
          while (aux_v1.second != ESP_OK) {
            aux_v1 = adc.ReadVoltage(3);
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          voltage_no_load_1 += (aux_v1.second == ESP_OK) ? aux_v1.first : 0.0f;
          conf_aux_v2 = adc.ConfigureADS1115(2); // Configure for Cell 2
          while (conf_aux_v2.second != ESP_OK) {
            conf_aux_v2 = adc.ConfigureADS1115(2);
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          vTaskDelay(pdMS_TO_TICKS(80));
          aux_v2 = adc.ReadVoltage(2, true);
          while (aux_v2.second != ESP_OK) {
            aux_v2 = adc.ReadVoltage(2, true);
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          voltage_no_load_2 += (aux_v2.second == ESP_OK) ? aux_v2.first / 2 : 0.0f;
          conf_aux_v3 = adc.ConfigureADS1115(1); // Configure for Cell 3
          while (conf_aux_v3.second != ESP_OK) {
            conf_aux_v3 = adc.ConfigureADS1115(1);
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          vTaskDelay(pdMS_TO_TICKS(80));
          aux_v3 = adc.ReadVoltage(1, true);
          while (aux_v3.second != ESP_OK) {
            aux_v3 = adc.ReadVoltage(1, true);
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          voltage_no_load_3 += (aux_v3.second == ESP_OK) ? aux_v3.first / 3 : 0.0f;
        }
        voltage_no_load_1 /= 5.0f;
        voltage_no_load_2 /= 5.0f;
        voltage_no_load_3 /= 5.0f;
        ESP_LOGI(TAG, "[DIS] Load voltages: V1: %.4f V, V2: %.4f V, V3: %.4f V, Current: %.2f A", voltage_cell_1.first, voltage_cell_2.first / 2, voltage_cell_3.first / 3, current.first);
        ESP_LOGI(TAG, "[DIS] No-load voltages: V1: %.4f V, V2: %.4f V, V3: %.4f V", voltage_no_load_1, voltage_no_load_2, voltage_no_load_3);
        cell_resitance_1 = (voltage_no_load_1 - voltage_cell_1.first) / std::fabs(current.first);
        cell_resitance_2 = ((voltage_no_load_2) - (voltage_cell_2.first / 2)) / std::fabs(current.first);
        cell_resitance_3 = ((voltage_no_load_3) - (voltage_cell_3.first / 3)) / std::fabs(current.first);
        ctx -> relayController -> set_relay(1, true); // Re-apply load
        snprintf(display_msg, sizeof(display_msg),
                 "[DIS] ~50%% SoC\r\n"
                 "R1: %.4f Ω\r\n"
                 "R2: %.4f Ω\r\n"
                 "R3: %.4f Ω",
                 cell_resitance_1, cell_resitance_2, cell_resitance_3);
      } else if (!alerted_20_percent && state_of_charge <= 20.0f) {  //70
        ESP_LOGI(TAG, "[DIS] Battery at ~20%% capacity");
        buzzer_alert();
        alerted_20_percent = true;
        ctx -> relayController -> set_relay(1, false); // Cut the load for a moment to measure internal resistance without load
        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for voltages to stabilize
        std::pair<float, esp_err_t> aux_v1;
        std::pair<float, esp_err_t> aux_v2;
        std::pair<float, esp_err_t> aux_v3;
        std::pair<float, esp_err_t> conf_aux_v1;
        std::pair<float, esp_err_t> conf_aux_v2;
        std::pair<float, esp_err_t> conf_aux_v3;
        for (uint8_t i = 0; i < 5; ++i) {
          vTaskDelay(pdMS_TO_TICKS(100));
          conf_aux_v1 = adc.ConfigureADS1115(3); // Configure for Cell 1
          while (conf_aux_v1.second != ESP_OK) {
            conf_aux_v1 = adc.ConfigureADS1115(3);
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          vTaskDelay(pdMS_TO_TICKS(80));
          aux_v1 = adc.ReadVoltage(3);
          while (aux_v1.second != ESP_OK) {
            aux_v1 = adc.ReadVoltage(3);
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          voltage_no_load_1 += (aux_v1.second == ESP_OK) ? aux_v1.first : 0.0f;
          conf_aux_v2 = adc.ConfigureADS1115(2); // Configure for Cell 2
          while (conf_aux_v2.second != ESP_OK) {
            conf_aux_v2 = adc.ConfigureADS1115(2);
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          vTaskDelay(pdMS_TO_TICKS(80));
          aux_v2 = adc.ReadVoltage(2, true);
          while (aux_v2.second != ESP_OK) {
            aux_v2 = adc.ReadVoltage(2, true);
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          voltage_no_load_2 += (aux_v2.second == ESP_OK) ? aux_v2.first / 2 : 0.0f;
          conf_aux_v3 = adc.ConfigureADS1115(1); // Configure for Cell 3
          while (conf_aux_v3.second != ESP_OK) {
            conf_aux_v3 = adc.ConfigureADS1115(1);
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          vTaskDelay(pdMS_TO_TICKS(80));
          aux_v3 = adc.ReadVoltage(1, true);
          while (aux_v3.second != ESP_OK) {
            aux_v3 = adc.ReadVoltage(1, true);
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          voltage_no_load_3 += (aux_v3.second == ESP_OK) ? aux_v3.first / 3 : 0.0f;
        }
        voltage_no_load_1 /= 5.0f;
        voltage_no_load_2 /= 5.0f;
        voltage_no_load_3 /= 5.0f;
        ESP_LOGI(TAG, "[DIS] Load voltages: V1: %.4f V, V2: %.4f V, V3: %.4f V, Current: %.2f A", voltage_cell_1.first, voltage_cell_2.first / 2, voltage_cell_3.first / 3, current.first);
        ESP_LOGI(TAG, "[DIS] No-load voltages: V1: %.4f V, V2: %.4f V, V3: %.4f V", voltage_no_load_1, voltage_no_load_2, voltage_no_load_3);
        cell_resitance_1 = (voltage_no_load_1 - voltage_cell_1.first) / std::fabs(current.first);
        cell_resitance_2 = ((voltage_no_load_2) - (voltage_cell_2.first / 2)) / std::fabs(current.first);
        cell_resitance_3 = ((voltage_no_load_3) - (voltage_cell_3.first / 3)) / std::fabs(current.first);
        ctx -> relayController -> set_relay(1, true); // Re-apply load
        snprintf(display_msg, sizeof(display_msg),
                 "[DIS] ~20%% SoC\r\n"
                 "R1: %.4f Ω\r\n"
                 "R2: %.4f Ω\r\n"
                 "R3: %.4f Ω",
                 cell_resitance_1, cell_resitance_2, cell_resitance_3);
      }
    }

    // Show data on display
    snprintf(display_msg, sizeof(display_msg),
             "DIS - Cycle: %d\r\n"
             "C1: %.2fV | C2: %.2fV\r\n"
             "C3: %.2fV | Bat: %.2fV\r\n"
             "Current: %.2fA",
             cycle, voltage_cell_1.first, (voltage_cell_2.first == -1.0f) ? -1.0f : (voltage_cell_2.first / 2), 
             (voltage_cell_3.first == -1.0f) ? -1.0f : (voltage_cell_3.first / 3), voltage_cell_3.first, current.first);
    if (xQueueDisplay) {
      xQueueSend(xQueueDisplay, display_msg, 0);
    }

    // Log data to UART
    snprintf(uart_msg, sizeof(uart_msg),
             "[DIS]\r\n"
             "cycle: %d\r\n"
             "Cell 1: %.2fV\r\n"
             "Cell 2: %.2fV\r\n"
             "Cell 3: %.2fV\r\n"
             "Battery: %.2fV\r\n"
             "Current: %.2fA\r\n",
             cycle, voltage_cell_1.first, (voltage_cell_2.first == -1.0f) ? -1.0f : (voltage_cell_2.first / 2), 
             (voltage_cell_3.first == -1.0f) ? -1.0f : (voltage_cell_3.first / 3), voltage_cell_3.first, current.first);
    uart_write_bytes(UART_NUM, uart_msg, strlen(uart_msg));

    // Publish data to MQTT
    if (mqtt_client) {
      elapsed_s = (uint32_t)((esp_timer_get_time() - experiment_start_us) / 1000000);
      cJSON *root = cJSON_CreateObject();
      if (root != NULL) {
        cJSON_AddStringToObject(root, "phase", "discharge");
        cJSON_AddNumberToObject(root, "cycle", cycle);
        cJSON_AddStringToObject(root, "device", exp_params->device_name.c_str()); 
        cJSON_AddStringToObject(root, "exp_name", exp_params->experiment_name.c_str());
        cJSON_AddStringToObject(root, "date", std::string(phase_params->start_date).c_str());
        cJSON_AddStringToObject(root, "place", exp_params->location.c_str());
        cJSON_AddNumberToObject(root, "t_s", elapsed_s);
        cJSON_AddStringToObject(root, "bat_type", exp_params->battery_type.c_str());
        cJSON_AddNumberToObject(root, "bat_cap_Ah", exp_params->battery_capacity_ah);
        cJSON_AddNumberToObject(root, "percent_r1", exp_params->percentage_resistance_1);
        cJSON_AddNumberToObject(root, "percent_r2", exp_params->percentage_resistance_2);
        cJSON_AddNumberToObject(root, "percent_r3", exp_params->percentage_resistance_3);
        cJSON_AddStringToObject(root, "sub_tag", exp_params->current_tag.c_str());
        cJSON_AddNumberToObject(root, "cell_1",  voltage_cell_1.first);
        cJSON_AddNumberToObject(root, "cell_2",  (voltage_cell_2.first == -1.0f) ? -1.0f : (voltage_cell_2.first / 2.0));
        cJSON_AddNumberToObject(root, "cell_3", (voltage_cell_3.first == -1.0f) ? -1.0f : (voltage_cell_3.first / 3.0));
        cJSON_AddNumberToObject(root, "battery", voltage_cell_3.first);
        cJSON_AddNumberToObject(root, "current", current.first);
        cJSON_AddNumberToObject(root, "coulombs", coulomb_count);
        cJSON_AddNumberToObject(root, "cell_res_1", cell_resitance_1);
        cJSON_AddNumberToObject(root, "cell_res_2", cell_resitance_2);
        cJSON_AddNumberToObject(root, "cell_res_3", cell_resitance_3);
        cJSON_AddNumberToObject(root, "success_count", success_count);
        cJSON_AddNumberToObject(root, "failure_count", failure_count);
        cJSON_AddNumberToObject(root, "soc_percent", state_of_charge);
        cJSON_AddNumberToObject(root, "cell_1_nload", voltage_no_load_1);
        cJSON_AddNumberToObject(root, "cell_2_nload", voltage_no_load_2);
        cJSON_AddNumberToObject(root, "cell_3_nload", voltage_no_load_3);

        // Generate the JSON string from the cJSON object
        char *mqtt_payload_dynamic = cJSON_PrintUnformatted(root);

        if (mqtt_payload_dynamic != NULL) {
          esp_mqtt_client_publish(mqtt_client, "m5stick/sensors/discharge", mqtt_payload_dynamic, 0, 1, 0);
          free(mqtt_payload_dynamic);
        } 

        cJSON_Delete(root);
      }
    }
  
    if (!error_occured) {
      ++success_count;
      error_occured = false;
      ESP_LOGV(TAG, "[DIS] Successful sample. Total successes: %d", success_count);
      buzzer_beep(50);
    }

    error_occured = false; // Reset error flag for next cycle

    // Wait for the next sample interval, but exit early if Button B is pressed
    if (xSemaphoreTake(phase_skip_sem, pdMS_TO_TICKS(EXPERIMENT_INTERVAL_MS)) == pdTRUE) {
      uart_write_bytes(UART_NUM, "[DIS] Phase skipped by user\r\n", 29);
      break;
    }
  }
  
  phase_params->mas_discharge = std::fabs(coulomb_count); // Ampere-seconds discharged during this phase
  xTaskNotifyGive(phase_params->caller_task);
  vTaskDelete(NULL);
}

// ============ EXPERIMENT ORCHESTRATOR ============

extern "C" void ExperimentCycle(void *pvParameters) {
  AppContext *app_context = static_cast<AppContext *>(pvParameters);
  char uart_msg[64];
  uint16_t cycle_count = 1;
  std::time_t date = std::time(nullptr);
  struct tm *now = localtime(&date);
  char date_str[32];
  strftime(date_str, sizeof(date_str), "%Y-%m-%d", now);
  ESP_LOGI(TAG, "ExperimentCycle started on %s, running %d cycles, sub_tag: %s", date_str, EXPERIMENT_CICLES, app_context->experimentParams->current_tag.c_str());

  while (cycle_count <= EXPERIMENT_CICLES && experiment_flag) {
    snprintf(uart_msg, sizeof(uart_msg), "\r\n=== CYCLE %d / %d ===\r\n",
             cycle_count, EXPERIMENT_CICLES);
    uart_write_bytes(UART_NUM, uart_msg, strlen(uart_msg));

    // --- CHARGING PHASE ---
    // PhaseTaskParams lives on the stack here; the phase task must finish
    // before we move on (guaranteed by ulTaskNotifyTake below).
    PhaseTaskParams charge_params = {
      .app_context = app_context,
      .caller_task = xTaskGetCurrentTaskHandle(),
      .cycle_index = cycle_count,
      .start_date  = date_str,
    };
    xTaskCreate(MeasureCharging, "CHG_PHASE", 1024 * 4, &charge_params, 1000, &phase_task_handle);
    // Block until MeasureCharging calls xTaskNotifyGive
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    phase_task_handle = NULL; // Phase finished normally

    gpio_set_level(GPIO_OUTPUT_IO_1, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));
    gpio_set_level(GPIO_OUTPUT_IO_1, 0);

    if (!experiment_flag) {
      break; // Aborted during charging
    }

    // Print successful count and failure count so far
    snprintf(uart_msg, sizeof(uart_msg), "Successful samples: %d, Failed samples: %d\r\n", success_count, failure_count);
    uart_write_bytes(UART_NUM, uart_msg, strlen(uart_msg));

    // Prompt the user to confirm the discharge phase.
    // They can either press Button B (GPIO 37) or send "DISCHARGE" via UART.
    // ExperimentCycle blocks here until one of those happens.
    is_waiting_discharge = true; // Used by the Button B ISR and UART task to know that giving the semaphore should trigger the discharge phase
    const char *prompt_uart = "Charging phase done.\r\nConnect load, then press Button B or send DISCHARGE to begin discharging.\r\n";
    uart_write_bytes(UART_NUM, prompt_uart, strlen(prompt_uart));

    char prompt_display[64];
    snprintf(prompt_display, sizeof(prompt_display),
             "Charge done\nPress B or\nsend DISCHARGE\nto start DIS");
    if (xQueueDisplay) xQueueSend(xQueueDisplay, prompt_display, 0);

    // Reset semaphore to a clean empty state before waiting
    xSemaphoreTake(discharge_ready_sem, 0);
    // Block until Button B or UART "DISCHARGE" gives the semaphore
    xSemaphoreTake(discharge_ready_sem, portMAX_DELAY);

    if (!experiment_flag) {
      break; // Experiment was stopped while waiting
    }

    
    // --- DISCHARGING PHASE ---
    PhaseTaskParams discharge_params = {
      .app_context = app_context,
      .caller_task = xTaskGetCurrentTaskHandle(),
      .cycle_index = cycle_count,
      .start_date  = date_str,
    };
    
    app_context->adsController->ConfigureADS1115(3); // Cell 1
    vTaskDelay(pdMS_TO_TICKS(80)); 
    discharge_params.initial_voltage_cell_1 = app_context->adsController->ReadVoltage(3).first;
    app_context->adsController->ConfigureADS1115(2); // Cell 2
    vTaskDelay(pdMS_TO_TICKS(80));
    discharge_params.initial_voltage_cell_2 = app_context->adsController->ReadVoltage(2, true).first;
    app_context->adsController->ConfigureADS1115(1); // Cell 3
    vTaskDelay(pdMS_TO_TICKS(80));
    discharge_params.initial_voltage_cell_3 = app_context->adsController->ReadVoltage(1, true).first;
    ESP_LOGI(TAG, "Initial voltages before discharge: C1: %.2f V, C2: %.2f V, C3: %.2f V", discharge_params.initial_voltage_cell_1, discharge_params.initial_voltage_cell_2, discharge_params.initial_voltage_cell_3);
    vTaskDelay(pdMS_TO_TICKS(1000));
    app_context->relayController->set_relay(1, true);
    
    xTaskCreate(MeasureDischarging, "DIS_PHASE", 1024 * 4, &discharge_params, 1000, &phase_task_handle);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    phase_task_handle = NULL; // Phase finished normally

    if (!experiment_flag) {
      break; // Aborted during discharging
    }

    app_context->relayController->set_relay(1, false);

    // Success and failure counts are updated by the phase tasks. Print them after each cycle.
    snprintf(uart_msg, sizeof(uart_msg), "Successful samples: %d, Failed samples: %d\r\n", success_count, failure_count);
    uart_write_bytes(UART_NUM, uart_msg, strlen(uart_msg));

    cycle_count++;
    snprintf(uart_msg, sizeof(uart_msg), "Cycle %d complete\r\n", cycle_count);
    uart_write_bytes(UART_NUM, uart_msg, strlen(uart_msg));
    snprintf(uart_msg, sizeof(uart_msg), 
             "Capacity ratio: %.2f%%\r\n"
             "State of Health: %.2f%%\r\n",
    (std::fabs(charge_params.mas_charge - discharge_params.mas_discharge) / charge_params.mas_charge) * 100.0f, 
    100.0f - (discharge_params.mas_discharge / (app_context->experimentParams->battery_capacity_ah * 3600)) * 100.0f);
    uart_write_bytes(UART_NUM, uart_msg, strlen(uart_msg));
    snprintf(uart_msg, sizeof(uart_msg), "Resting for %d seconds...\r\n", 20);
    uart_write_bytes(UART_NUM, uart_msg, strlen(uart_msg));
    vTaskDelay(pdMS_TO_TICKS(20 * 1000));
  }

  snprintf(uart_msg, sizeof(uart_msg),
           experiment_flag ? "All %d cycles complete\r\n" : "Experiment stopped at cycle %d\r\n",
           cycle_count);
  uart_write_bytes(UART_NUM, uart_msg, strlen(uart_msg));

  experiment_flag = false;
  phase_task_handle = NULL;
  experiment_task_handle = NULL;
  vTaskDelete(NULL);
}

#endif // EXPERIMENT_H
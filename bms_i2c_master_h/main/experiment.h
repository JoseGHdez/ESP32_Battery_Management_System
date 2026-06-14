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
#include "esp_event.h"
#include "mqtt_client.h"
#include "cJSON.h"

#include "wifi_mqtt.h" // Header file for WiFi and MQTT functions
#include "shared.h" // Header file for common definitions and utilities
#include "buzzer.h" // Header file for buzzer control functions

// Experiment configuration variables
#define EXPERIMENT_CICLES        5      // Number of charge/discharge cycles to perform in the experiment
#define EXPERIMENT_INTERVAL_MS   1000   // Sampling interval within each phase (ms)
#define CHARGE_DURATION_MS       30000  // How long to measure during charging phase (ms)
#define DISCHARGE_DURATION_MS    30000  // How long to measure during discharging phase (ms)
#define BATTERY_CAPACITY_AH      2.2f   // Battery capacity in mAh (used for coulomb counting approximation)

static TaskHandle_t experiment_task_handle = NULL; // Track the running ExperimentCycle task
static TaskHandle_t phase_task_handle = NULL;      // Track the active MeasureCharging/Discharging task
static int64_t experiment_start_us = 0;            // Timestamp (µs) when the experiment started

// Binary semaphore used to signal ExperimentCycle that the user has confirmed
// the start of the discharge phase. Released by Button B (GPIO 37) or "DISCHARGE" UART command.
static SemaphoreHandle_t discharge_ready_sem = NULL;

// Binary semaphore used to cut the current phase short. When GPIO 37 is pressed
// mid-phase, the running MeasureCharging/MeasureDischarging task checks this and
// exits early, handing control back to ExperimentCycle.
static SemaphoreHandle_t phase_skip_sem = NULL;

// Forward declarations — defined later in this file
extern "C" void ExperimentCycle(void *pvParameters);
static void MeasureCharging(void *pvParameters);
static void MeasureDischarging(void *pvParameters);

uint16_t success_count = 0;
uint16_t failure_count = 0;
bool error_occured = false;

// Starts ExperimentCycle as a FreeRTOS task. Safe to call from ButtonTask or UARTTask.
// Does nothing if the task is already running.
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
    xTaskCreate(ExperimentCycle, "EXP_CYCLE", 1024 * 4, context, 2, &experiment_task_handle);
    ESP_LOGI(TAG, "ExperimentCycle task started");
}

// Stops the ExperimentCycle task if it is running.
static void stop_experiment_task(AppContext *context) {
    if (experiment_task_handle == NULL) {
        ESP_LOGW(TAG, "ExperimentCycle not running");
        return;
    }
    context->relayController->set_all_relays_mask(0x0); // Ensure all relays are off
    experiment_flag = false;

    // Kill the active phase task first (MeasureCharging or MeasureDischarging).
    // vTaskDelete on ExperimentCycle alone leaves the child task running.
    if (phase_task_handle != NULL) {
        vTaskDelete(phase_task_handle);
        phase_task_handle = NULL;
    }

    // If ExperimentCycle is blocked on the discharge semaphore, unblock it so
    // it doesn't get deleted while holding a semaphore (causes heap corruption).
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

// Runs during the charging phase. Samples sensors every EXPERIMENT_INTERVAL_MS
// for CHARGE_DURATION_MS total, then notifies ExperimentCycle it is done.
static void MeasureCharging(void *pvParameters) {
  PhaseTaskParams *phase_params = static_cast<PhaseTaskParams *>(pvParameters);
  AppContext *ctx = phase_params->app_context;
  ADS1115 &adc = *(ctx->adsController);
  const uint16_t cycle = phase_params->cycle_index;

  success_count = 0;
  failure_count = 0;
  char uart_msg[160];
  char display_msg[128];
  uint8_t safety_breach_count = 0;
  float coulomb_count = 0.0f;

  uart_write_bytes(UART_NUM, "--- CHARGING PHASE ---\r\n", 24);
  
  while (1) {
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
    auto current  = adc.ReadCurrent(0, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
      
    if (voltage_cell_1.second != ESP_OK || voltage_cell_2.second != ESP_OK || voltage_cell_3.second != ESP_OK || current.second != ESP_OK) {
      ++failure_count;
      error_occured = true;
      ESP_LOGE(TAG, "[CHG] Sensor read error detected! V1: %.2f V, V2: %.2f V, V3: %.2f V, I: %.2f A", voltage_cell_1.first, voltage_cell_2.first, voltage_cell_3.first, current.first);
      buzzer_warn();
      buzzer_warn();
    }

    // Safety check
    if (voltage_cell_3.first >= 12.5f) {
      if (safety_breach_count < 5) {
        ESP_LOGW(TAG, "[CHG] High voltage detected! V: %.2f V. Breach count: %d", voltage_cell_3.first, safety_breach_count + 1);
        safety_breach_count++;
      } else {
        ESP_LOGE(TAG, "[CHG] Battery is charged! V: %.2f V", voltage_cell_3.first);
        buzzer_warn();
        break;
      }
    }

    if (!error_occured) {
      coulomb_count += (std::fabs(current.first) * (EXPERIMENT_INTERVAL_MS / 3600000.0f)); // Approximate coulombs passed during this interval
      ESP_LOGI(TAG, "[CHG] Sample OK. Current: %.2f A, Coulombs: %.4f Ah", current.first, coulomb_count / 3600.0f);
    }

    // Show data on display
    snprintf(display_msg, sizeof(display_msg),
             "CHG - Cycle: %d\r\n"
             "C1: %.2fV | C2: %.2fV\r\n"
             "C3: %.2fV | Bat: %.2fV\r\n"
             "Current: %.2fA",
             cycle, voltage_cell_1.first, voltage_cell_2.first == -1.0f ? -1.0f : (voltage_cell_2.first / 2), voltage_cell_3.first == -1.0f ? -1.0f : (voltage_cell_3.first / 3), voltage_cell_3.first, current.first);
      
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
             cycle, voltage_cell_1.first, (voltage_cell_2.first == -1.0f) ? -1.0f : (voltage_cell_2.first / 2), (voltage_cell_3.first == -1.0f) ? -1.0f : (voltage_cell_3.first / 3), voltage_cell_3.first, current.first);
    uart_write_bytes(UART_NUM, uart_msg, strlen(uart_msg));

    // Publish data to MQTT
    if (mqtt_client) {
      uint32_t elapsed_s = (uint32_t)((esp_timer_get_time() - experiment_start_us) / 1000000);
      cJSON *root = cJSON_CreateObject();
      std::time_t now = std::time(nullptr);
      if (root != NULL) {
        cJSON_AddStringToObject(root, "phase", "charge");
        cJSON_AddNumberToObject(root, "cycle", cycle);
        cJSON_AddStringToObject(root, "device", "m5stick"); 
        cJSON_AddStringToObject(root, "exp_name", "cycle_test");
        cJSON_AddStringToObject(root, "date", std::to_string(now).c_str());
        cJSON_AddStringToObject(root, "place", "lab");
        cJSON_AddNumberToObject(root, "t_s", elapsed_s);
        cJSON_AddStringToObject(root, "bat_type", "LiPo");
        cJSON_AddNumberToObject(root, "bat_cap_Ah", BATTERY_CAPACITY_AH);
        cJSON_AddStringToObject(root, "sub_tag", "A");
        cJSON_AddNumberToObject(root, "cell_1",  voltage_cell_1.first);
        cJSON_AddNumberToObject(root, "cell_2",  voltage_cell_2.first == -1.0f ? -1.0f : (voltage_cell_2.first / 2.0));
        cJSON_AddNumberToObject(root, "cell_3", voltage_cell_3.first == -1.0f ? -1.0f : (voltage_cell_3.first / 3.0));
        cJSON_AddNumberToObject(root, "battery", voltage_cell_3.first);
        cJSON_AddNumberToObject(root, "current", current.first);
        cJSON_AddNumberToObject(root, "coulombs", coulomb_count);
        cJSON_AddNumberToObject(root, "success_count", success_count);
        cJSON_AddNumberToObject(root, "failure_count", failure_count);
        
        // Generate the JSON string from the cJSON object
        char *mqtt_payload_dynamic = cJSON_PrintUnformatted(root);

        if (mqtt_payload_dynamic != NULL) {
          esp_mqtt_client_publish(mqtt_client, "m5stick/sensors/charge", mqtt_payload_dynamic, 0, 1, 0);
          free(mqtt_payload_dynamic);
        } 

        cJSON_Delete(root);
      }
    }
  
    if (!error_occured) {
      ++success_count;
      error_occured = false;
      ESP_LOGV(TAG, "[CHG] Successful sample. Total successes: %d", success_count);
      buzzer_beep(50);
    }

    error_occured = false; // Reset error flag for next cycle

    // Wait for the next sample interval, but exit early if Button B is pressed
    if (xSemaphoreTake(phase_skip_sem, pdMS_TO_TICKS(EXPERIMENT_INTERVAL_MS)) == pdTRUE) {
      uart_write_bytes(UART_NUM, "[CHG] Phase skipped by user\r\n", 29);
      break;
    }
  }

  // Notify ExperimentCycle that this phase is complete
  xTaskNotifyGive(phase_params->caller_task);
  vTaskDelete(NULL);
}

// Runs during the discharging phase. Same structure as MeasureCharging but
// uses a different safety threshold direction and MQTT phase label.
static void MeasureDischarging(void *pvParameters) {
  PhaseTaskParams *phase_params = static_cast<PhaseTaskParams *>(pvParameters);
  AppContext *ctx = phase_params->app_context;
  ADS1115 &adc = *(ctx->adsController);
  const uint16_t cycle = phase_params->cycle_index;

  char uart_msg[160];
  char display_msg[256];
  uint8_t safety_breach_count = 0;
  success_count = 0;
  failure_count = 0;

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
  auto current  = adc.ReadCurrent(0, 0.0f);
  vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
  float coulomb_count = 0.0f;
  float cell_resitance_1 = 0.0f;
  float cell_resitance_2 = 0.0f;
  float cell_resitance_3 = 0.0f;
  float capacity_80_percent = BATTERY_CAPACITY_AH * 3600 * 0.8f;
  float capacity_20_percent = BATTERY_CAPACITY_AH * 3600 * 0.2f;
  float capacity_50_percent = BATTERY_CAPACITY_AH * 3600 * 0.5f;

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
    adc.ConfigureADS1115(3); // Configure for Cell 1
    vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
    voltage_cell_1  = adc.ReadVoltage(3);
    vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
    adc.ConfigureADS1115(2); // Configure for Cell 2
    vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
    voltage_cell_2  = adc.ReadVoltage(2, true);
    vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
    adc.ConfigureADS1115(1); // Configure for Cell 3
    vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
    voltage_cell_3  = adc.ReadVoltage(1, true);
    vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
    adc.ConfigureADS1115(0); // Configure for Current
    vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free
    current  = adc.ReadCurrent(0, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free

    // Safety check — cut off if any cell drops below 3.5V during discharge
    if (voltage_cell_1.second != ESP_OK || voltage_cell_2.second != ESP_OK || voltage_cell_3.second != ESP_OK || current.second != ESP_OK) {
      ++failure_count;
      error_occured = true;
      ESP_LOGE(TAG, "[DIS] Sensor read error detected! V1: %.2f V, V2: %.2f V, V3: %.2f V, I: %.2f A", voltage_cell_1.first, voltage_cell_2.first, voltage_cell_3.first, current.first);
      buzzer_warn();
      buzzer_warn();
    }
    
    if ((voltage_cell_1.first <= 3.55f || (voltage_cell_2.first / 2) <= 3.55f || (voltage_cell_3.first / 3) <= 3.55f) && !error_occured) {
      ESP_LOGE(TAG, "[DIS] Low voltage detected! C1: %.2f V, C2: %.2f V, C3: %.2f V", voltage_cell_1.first, voltage_cell_2.first / 2, voltage_cell_3.first / 3);
      if (voltage_cell_1.first <= 3.3f || (voltage_cell_2.first / 2) <= 3.3f || (voltage_cell_3.first / 3) <= 3.3f) {
        buzzer_alert();
      }
      safety_breach_count++;
    }


    if (safety_breach_count >= 10) { // Allow a few transient breaches before cutting off
      ctx->relayController->set_all_relays_mask(0x0);
      buzzer_warn();
      break;
    }

    if (!error_occured) {
      coulomb_count += (std::fabs(current.first) * (EXPERIMENT_INTERVAL_MS / 3600000.0f)); // Approximate coulombs passed during this interval
      ESP_LOGI(TAG, "[DIS] Sample OK. Current: %.2f A, Coulombs: %.4f Ah", current.first, coulomb_count / 3600.0f);
    }

    if (coulomb_count == (BATTERY_CAPACITY_AH * 3600) - capacity_80_percent && !error_occured) {
      ESP_LOGI(TAG, "[DIS] Battery at ~80%% capacity");
      buzzer_beep(200);
      // app_context -> relayController -> set_relay(1, false); // Cut the load for a moment to measure internal resistance without load
      // vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for voltages to stabilize
      // float voltage_no_load_1 = 0.0f;
      // float voltage_no_load_2 = 0.0f;
      // float voltage_no_load_3 = 0.0f;
      // while (voltage_no_load_1.second != ESP_OK) {
      //   voltage_no_load_1 = adc.ReadVoltage(3);
      // }
      // while (voltage_no_load_2.second != ESP_OK) {
      //   voltage_no_load_2 = adc.ReadVoltage(2, true);
      // }
      // while (voltage_no_load_3.second != ESP_OK) {
      //   voltage_no_load_3 = adc.ReadVoltage(1, true);
      // }
      // cell_resitance_1 = (voltage_cell_1.first - adc.ReadVoltage(3).first) / current.first;
      // cell_resitance_2 = ((voltage_cell_2.first / 2) - adc.ReadVoltage(2, true).first) / current.first;
      // cell_resitance_3 = ((voltage_cell_3.first / 3) - adc.ReadVoltage(1, true).first) / current.first;
      // app_context -> relayController -> set_relay(1, true); // Re-apply load
    } else if (coulomb_count == (BATTERY_CAPACITY_AH * 3600) - capacity_50_percent && !error_occured) {
      ESP_LOGI(TAG, "[DIS] Battery at ~50%% capacity");
      buzzer_beep(500);
      // app_context -> relayController -> set_relay(1, false); // Cut the load for a moment to measure internal resistance without load
      // vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for voltages to stabilize
      // float voltage_no_load_1 = 0.0f;
      // float voltage_no_load_2 = 0.0f;
      // float voltage_no_load_3 = 0.0f;
      // while (voltage_no_load_1.second != ESP_OK) {
      //   voltage_no_load_1 = adc.ReadVoltage(3);
      // }
      // while (voltage_no_load_2.second != ESP_OK) {
      //   voltage_no_load_2 = adc.ReadVoltage(2, true);
      // }
      // while (voltage_no_load_3.second != ESP_OK) {
      //   voltage_no_load_3 = adc.ReadVoltage(1, true);
      // }
      // cell_resitance_1 = (voltage_cell_1.first - adc.ReadVoltage(3).first) / current.first;
      // cell_resitance_2 = ((voltage_cell_2.first / 2) - adc.ReadVoltage(2, true).first) / current.first;
      // cell_resitance_3 = ((voltage_cell_3.first / 3) - adc.ReadVoltage(1, true).first) / current.first;
      // app_context -> relayController -> set_relay(1, true); // Re-apply load
    } else if (coulomb_count == (BATTERY_CAPACITY_AH * 3600) - capacity_20_percent && !error_occured) {
      ESP_LOGI(TAG, "[DIS] Battery at ~20%% capacity");
      buzzer_alert();
      // app_context -> relayController -> set_relay(1, false); // Cut the load for a moment to measure internal resistance without load
      // vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for voltages to stabilize
      // float voltage_no_load_1 = 0.0f;
      // float voltage_no_load_2 = 0.0f;
      // float voltage_no_load_3 = 0.0f;
      // while (voltage_no_load_1.second != ESP_OK) {
      //   voltage_no_load_1 = adc.ReadVoltage(3);
      // }
      // while (voltage_no_load_2.second != ESP_OK) {
      //   voltage_no_load_2 = adc.ReadVoltage(2, true);
      // }
      // while (voltage_no_load_3.second != ESP_OK) {
      //   voltage_no_load_3 = adc.ReadVoltage(1, true);
      // }
      // cell_resitance_1 = (voltage_cell_1.first - adc.ReadVoltage(3).first) / current.first;
      // cell_resitance_2 = ((voltage_cell_2.first / 2) - adc.ReadVoltage(2, true).first) / current.first;
      // cell_resitance_3 = ((voltage_cell_3.first / 3) - adc.ReadVoltage(1, true).first) / current.first;
      // app_context -> relayController -> set_relay(1, true); // Re-apply load
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
      uint32_t elapsed_s = (uint32_t)((esp_timer_get_time() - experiment_start_us) / 1000000);
      cJSON *root = cJSON_CreateObject();
      std::time_t now = std::time(nullptr);
      if (root != NULL) {
        cJSON_AddStringToObject(root, "phase", "discharge");
        cJSON_AddNumberToObject(root, "cycle", cycle);
        cJSON_AddStringToObject(root, "device", "m5stick"); 
        cJSON_AddStringToObject(root, "exp_name", "cycle_test");
        cJSON_AddStringToObject(root, "date", std::to_string(now).c_str());
        cJSON_AddStringToObject(root, "place", "lab");
        cJSON_AddNumberToObject(root, "t_s", elapsed_s);
        cJSON_AddStringToObject(root, "bat_type", "LiPo");
        cJSON_AddNumberToObject(root, "bat_cap_Ah", BATTERY_CAPACITY_AH);
        cJSON_AddNumberToObject(root, "percent_r1", cell_resitance_1);
        cJSON_AddNumberToObject(root, "percent_r2", cell_resitance_2);
        cJSON_AddNumberToObject(root, "percent_r3", cell_resitance_3);
        cJSON_AddStringToObject(root, "sub_tag", "A");
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
  
  xTaskNotifyGive(phase_params->caller_task);
  vTaskDelete(NULL);
}

// ============ EXPERIMENT ORCHESTRATOR ============

extern "C" void ExperimentCycle(void *pvParameters) {
  AppContext *app_context = static_cast<AppContext *>(pvParameters);
  char uart_msg[64];
  uint16_t cycle_count = 0;

  ESP_LOGI(TAG, "ExperimentCycle started, running %d cycles", EXPERIMENT_CICLES);

  while (cycle_count < EXPERIMENT_CICLES && experiment_flag) {
    snprintf(uart_msg, sizeof(uart_msg), "\r\n=== CYCLE %d / %d ===\r\n",
             cycle_count + 1, EXPERIMENT_CICLES);
    uart_write_bytes(UART_NUM, uart_msg, strlen(uart_msg));

    // --- CHARGING PHASE ---
    // PhaseTaskParams lives on the stack here; the phase task must finish
    // before we move on (guaranteed by ulTaskNotifyTake below).
    PhaseTaskParams charge_params = {
      .app_context = app_context,
      .caller_task = xTaskGetCurrentTaskHandle(),
      .cycle_index = cycle_count,
    };
    xTaskCreate(MeasureCharging, "CHG_PHASE", 1024 * 4, &charge_params, 1, &phase_task_handle);
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

    app_context->relayController->set_relay(1, true);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // --- DISCHARGING PHASE ---
    PhaseTaskParams discharge_params = {
      .app_context = app_context,
      .caller_task = xTaskGetCurrentTaskHandle(),
      .cycle_index = cycle_count,
    };
    xTaskCreate(MeasureDischarging, "DIS_PHASE", 1024 * 4, &discharge_params, 1, &phase_task_handle);
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
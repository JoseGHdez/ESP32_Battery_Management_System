#include <stdio.h>  // printf, snprintf
#include <stdlib.h> // atoi, atof
#include <string.h> // memset, strcmp, strcasecmp
#include <inttypes.h> // int16_t, uint8_t, etc.
#include <iostream> // std::cout, std::endl
#include <array>    // std::array
#include <iomanip>  // std::setprecision, std::fixed
#include <unordered_map> // std::unordered_map for mapping strings to function pointers
#include <functional> // std::function for function pointers
#include <algorithm> // std::transform for string manipulation

#include "freertos/FreeRTOS.h" // FreeRTOS types and functions (e.g., vTaskDelay, xQueueSend)
#include "freertos/task.h"     // FreeRTOS task functions (e.g., xTaskCreate)
#include "freertos/queue.h"    // FreeRTOS queue functions (e.g., xQueueCreate, xQueueSend)
#include "esp_err.h" // esp_err_t type and error codes (e.g., ESP_OK, ESP_FAIL)
#include "esp_log.h" // ESP32 logging functions (e.g., ESP_LOGI, ESP_LOGE)
#include "esp_timer.h" // ESP32 timer functions (e.g., esp_timer_get_time)
#include "esp_vfs.h" // ESP32 Virtual File System functions (e.g., esp_vfs_spiffs_register)
#include "esp_spiffs.h" // ESP32 SPIFFS functions (e.g., esp_spiffs_info)
#include "driver/gpio.h" // GPIO functions for controlling pins (e.g., gpio_set_level, gpio_set_direction)
#include "driver/uart.h" // UART functions for serial communication (e.g., uart_read_bytes, uart_write_bytes)
#include "driver/i2c.h" // I2C functions for communication with peripherals (e.g., i2c_master_write_to_device, i2c_master_write_read_device)
#include "driver/ledc.h" // LEDC functions for PWM control (e.g., ledc_timer_config, ledc_channel_config)
#include "esp_adc/adc_oneshot.h" // ESP32 ADC one-shot mode functions (e.g., adc_oneshot_unit_init_cfg_t, adc_oneshot_read)
#include "esp_adc/adc_cali.h" // ESP32 ADC calibration functions (e.g., adc_cali_characteristics_t, adc_cali_calibrate)
#include "esp_adc/adc_cali_scheme.h" // ESP32 ADC calibration schemes (e.g., ADC_CALI_SCHEME_CURVE_FITTING)
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mqtt_client.h"

#include "sgm2578.h" // Display library for M5StickCPlus2
#include "st7789.h" // TFT display driver for ST7789-based displays (used in M5StickCPlus2)
#include "fontx.h" // Font rendering library for displaying text on the TFT screen
#include "bmpfile.h" // Bitmap file handling library (not used in this code but included for potential future use in displaying images on the TFT)
#include "I2Crelay.h" // Header file for relay control functions
#include "ADS1115.h" // Header file for ADS1115 ADC functions
#include "mg996r.h" // Header file for MG996R servo control functions

// I2C Configuration (M5StickCPlus2 Grove Port)
#define I2C_MASTER_SDA_IO         32        /*!<@brief I2C master SDA GPIO */
#define I2C_MASTER_SCL_IO         33        /*!<@brief I2C master SCL GPIO */
#define I2C_MASTER_NUM            I2C_NUM_0 /*!<@brief I2C master port number */
#define I2C_MASTER_FREQ_HZ        100000    /*!<@brief I2C master frequency in Hz */
#define I2C_MASTER_TIMEOUT_MS     1000      /*!<@brief I2C master timeout in ms */
#define RELAY_ADDR                0x26      /*!<@brief Relay I2C address */
#define REG_RELAY_CTRL            0x11      /*!<@brief Relay control register */

// ============ ADS1115 CONFIGURATION ============
#define ADS1115_ADDR          0x48      /*!<@brief ADS1115 I2C address */
#define ADS1115_REG_CONV      0x00      /*!<@brief ADS1115 conversion register */
#define ADS1115_REG_CONFIG    0x01      /*!<@brief ADS1115 configuration register */

// ============ INTERNAL ADC CONFIGURATION ============
#define BAT_ADC_CHAN              ADC_CHANNEL_0 /*!<@brief ADC channel for battery voltage reading (GPIO 36) */
#define BAT_VOLTAGE_DIVIDER       2.0 /*!<@brief Battery voltage divider ratio if needed */

// ============ UART CONFIGURATION ============
#define UART_NUM UART_NUM_0 /*!<@brief UART port number for console output and command input */
#define BUF_SIZE 256 /*!<@brief Buffer size for UART communication */

// ============ TASK CONFIGURATION ============
#define INTERVAL 400 /*!<@brief Delay interval for the task */
#define WAIT vTaskDelay(INTERVAL) /*!<@brief Delay code for the task */

// =========== M5STICKCPLUS2 CONFIGURATION ============
#define CONFIG_WIDTH 135     // Display width in pixels
#define CONFIG_HEIGHT 240    // Display height in pixels
#define CONFIG_MOSI_GPIO 15  // GPIO for SPI MOSI (Master Out Slave In)
#define CONFIG_SCLK_GPIO 13  // GPIO for SPI SCLK (Serial Clock)
#define CONFIG_CS_GPIO 5     // GPIO for SPI CS (Chip Select)
#define CONFIG_DC_GPIO 14    // GPIO for SPI DC (Data/Command)
#define CONFIG_RESET_GPIO 12 // GPIO for SPI Reset
#define CONFIG_BL_GPIO -1    // GPIO for SPI Backlight (set to -1 if not used, as M5StickCPlus2 has a fixed backlight control)
#define CONFIG_LED_GPIO 19   // GPIO for controlling an LED (optional, can be used for status indication)
#define CONFIG_OFFSETX 52    // X offset for the display (to center the content on the M5StickC screen)
#define CONFIG_OFFSETY 40    // Y offset for the display (to center the content on the M5StickC screen)

// Wifi configuration and MQTT
// --- SOLUCIÓN A ERRORES DE COMPILACIÓN (Macros faltantes) ---
#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
    #define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
    #define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
    #define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
    #define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
    #define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
    #define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#else
    // Valores por defecto si no están en menuconfig
    #define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
    #define EXAMPLE_H2E_IDENTIFIER ""
#endif

#ifndef ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD
    #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#endif

#define ESP_WIFI_SSID      "TP-Link_E3D6" //"Galaxy A02s85e9"//
#define ESP_WIFI_PASS      "76649769"//"vgnm2878"//
#define MQTT_BROKER_URI    "mqtt://192.168.1.102"//"mqtt://10.121.220.168"//
#define ESP_MAXIMUM_RETRY  20
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// Power hold GPIO for the display (used to ensure the display receives power)
#define POWER_HOLD_GPIO 4
// GPIO for enabling the SGM2578 power management IC (used to provide power to the display)
#define SGM2578_ENABLE_GPIO 27

// Buzzer — change BUZZER_GPIO to match your wiring.
// Uses LEDC PWM at 2700 Hz (passive piezo resonant frequency).
// Active buzzers also work — they ignore the frequency and just beep when driven HIGH.
#define BUZZER_GPIO          GPIO_NUM_2
#define BUZZER_LEDC_CHANNEL  LEDC_CHANNEL_0
#define BUZZER_LEDC_TIMER    LEDC_TIMER_0
#define BUZZER_FREQ_HZ       2700

// Button GPIOs
#define GPIO_OUTPUT_IO_0    GPIO_NUM_18
#define GPIO_OUTPUT_IO_1    GPIO_NUM_19
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))

#define GPIO_INPUT_IO_0     GPIO_NUM_39
#define GPIO_INPUT_IO_1     GPIO_NUM_37
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))

#define ESP_INTR_FLAG_DEFAULT 0

// Experiment configuration variables
#define EXPERIMENT_CICLES        5      // Number of charge/discharge cycles to perform in the experiment
#define EXPERIMENT_INTERVAL_MS   5000   // Sampling interval within each phase (ms)
#define CHARGE_DURATION_MS       30000  // How long to measure during charging phase (ms)
#define DISCHARGE_DURATION_MS    30000  // How long to measure during discharging phase (ms)

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

static const char *WIFI_TAG = "wifi";
static int s_retry_num = 0;
volatile int isConnected = 0;
static EventGroupHandle_t s_wifi_event_group;
esp_mqtt_client_handle_t mqtt_client = NULL;

struct AppContext {
  ServoController *servoController; // Pointer to the ServoController instance
  I2CRelay *relayController; // Pointer to the I2CRelay instance
  ADS1115 *adsController; // Pointer to the ADS1115 instance
};

// Parameter block passed to MeasureCharging / MeasureDischarging tasks.
// caller_task is the ExperimentCycle handle to notify when the phase is done.
struct PhaseTaskParams {
    AppContext     *app_context;
    TaskHandle_t    caller_task;  // ExperimentCycle notified via ulTaskNotifyTake
    uint16_t        cycle_index;  // Which cycle this phase belongs to (for logging)
};

bool experiment_flag = false; // Flag to indicate whether we are in an experiment or not
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

// ============ BUZZER HELPERS ============

// Initialise the LEDC peripheral for the buzzer. Call once from app_main.
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

// Beep for `duration_ms` milliseconds then go silent.
// Safe to call from any task — blocks the caller for the duration.
static void buzzer_beep(uint32_t duration_ms) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL, 512); // 50% duty
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL, 0);   // Silent
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL);
}

// Short double-beep pattern: used for warnings (e.g. approaching threshold).
static void buzzer_warn() {
    buzzer_beep(100);
    vTaskDelay(pdMS_TO_TICKS(80));
    buzzer_beep(100);
}

// Long continuous beep: used for critical safety cutoffs.
static void buzzer_alert() {
    buzzer_beep(1000);
}

// Forward declarations — defined later in this file
extern "C" void ExperimentCycle(void *pvParameters);
static void MeasureCharging(void *pvParameters);
static void MeasureDischarging(void *pvParameters);

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

// Interruption handler for buttons.
// Keep the ISR minimal — just queue the pin number and return immediately.
// NEVER call vTaskDelay or any blocking function from an ISR.
// WiFi glitch filtering is handled in ButtonTask instead.
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_event_queue, &gpio_num, NULL);
}

// Button event handler task.
// GPIO 36 and 39 receive spurious ~80µs glitch pulses from the WiFi/BT ADC
// calibration circuit (ESP32 errata 3.11). We filter them out by waiting
// 5ms after the interrupt fires, then checking if the pin is still LOW.
// A real button press holds the pin LOW for many milliseconds; glitches do not.
static void ButtonTask(void *pvParameters) {
    uint32_t gpio_num;
    AppContext *context = static_cast<AppContext *>(pvParameters);
    while (1) {
        if (xQueueReceive(gpio_event_queue, &gpio_num, portMAX_DELAY)) {
            // Wait longer than the WiFi glitch duration (~80µs) before sampling
            vTaskDelay(pdMS_TO_TICKS(5));
            // If pin already recovered, it was a WiFi glitch — discard it
            if (gpio_get_level((gpio_num_t)gpio_num) != 0) {
                continue;
            }
            ESP_LOGI(TAG, "Button GPIO %d pressed", gpio_num);

            if (gpio_num == GPIO_INPUT_IO_0) {
                // Button A: toggle experiment — start if idle, stop if running
                if (experiment_task_handle == NULL) {
                    uart_write_bytes(UART_NUM, "Starting experiment cycles...\r\n", 31);
                    char display_msg[64];
                    snprintf(display_msg, sizeof(display_msg), "Experiment started");
                    xQueueSend(xQueueDisplay, &display_msg, portMAX_DELAY);
                    vTaskDelay(pdMS_TO_TICKS(100)); // Small delay to ensure the display message is shown before starting the experiment
                    start_experiment_task(context);
                } else {
                    uart_write_bytes(UART_NUM, "Stopping experiment cycles...\r\n", 31);
                    char display_msg[64];
                    snprintf(display_msg, sizeof(display_msg), "Experiment stopped by user");
                    xQueueSend(xQueueDisplay, &display_msg, portMAX_DELAY);
                    stop_experiment_task(context);
                }
            } else if (gpio_num == GPIO_INPUT_IO_1) {
                // Button B: advance the experiment phase.
                // - If a phase task is running → skip it early (phase_skip_sem)
                // - If waiting for discharge confirmation → release that wait (discharge_ready_sem)
                // - If no experiment is running → stop (nothing to advance)
                if (experiment_task_handle != NULL) {
                    uart_write_bytes(UART_NUM, "Phase skip requested\r\n", 22);
                    // Give both — whichever one ExperimentCycle or the phase task
                    // is currently blocked on will consume it; the other is harmless.
                    xSemaphoreGive(phase_skip_sem);
                    xSemaphoreGive(discharge_ready_sem);
                } else {
                    stop_experiment_task(context);
                }
            }

            // Wait for button release before accepting the next press
            while (gpio_get_level((gpio_num_t)gpio_num) == 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }
}

/**
 * @brief FreeRTOS task that initializes the TFT display and waits for messages to update the screen.
 * 
 * This task performs the following steps:
 * 1. Initializes the GPIO pin to hold the power for the display and enables it.
 * 2. Initializes the SGM2578 power management IC to ensure the display receives power.
 * 3. Initializes the TFT display using the st7789 library, sets up the SPI communication, and fills the screen with black.
 * 4. Enters an infinite loop where it waits for messages from the xQueueDisplay queue. When a message is received, it updates the display with the new text, rotating it for the M5StickC orientation.
 * 5. The task logs the updated message to the console for debugging purposes.
 * 
 * @note: This task is designed to run indefinitely and should be created during the initialization phase of the program. It relies on messages being sent to the xQueueDisplay queue to update the screen, so other parts of the program should use this queue to communicate with the display task.
 * @param pvParameters - Pointer to task parameters (not used in this implementation).
 */
extern "C" void ShowText(void *pvParameters) {
  ESP_LOGI(TAG, "Tarea TFT iniciada");

  // Initialize the display
  gpio_reset_pin((gpio_num_t)POWER_HOLD_GPIO);
  gpio_set_direction((gpio_num_t)POWER_HOLD_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)POWER_HOLD_GPIO, 1);
    
  sgm2578_Enable(SGM2578_ENABLE_GPIO);

  FontxFile fx16G[2];
  InitFontx(fx16G,"/fonts/ILGH16XB.FNT","");
    
  TFT_t dev;
  spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO, CONFIG_BL_GPIO);
  lcdInit(&dev, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSETX, CONFIG_OFFSETY, true);
  lcdFillScreen(&dev, BLACK);

  char screen_text[128] = "Waiting..."; // Initial message
  lcdDrawString(&dev, fx16G, (CONFIG_WIDTH - 1) - 20, 10, (uint8_t*)screen_text, WHITE);

  while(1) {
    // Wait until receiving data
    if (xQueueReceive(xQueueDisplay, &screen_text, portMAX_DELAY)) {
      lcdFillScreen(&dev, BLACK);
      lcdSetFontDirection(&dev, 1); // M5StickC rotation
     
      uint16_t xpos = (CONFIG_WIDTH - 1) - 20;
      uint16_t ypos = 10;
            
      // String separation at newline characters
      char *line = strtok(screen_text, "\r\n");
            
      while (line != NULL) {
        lcdDrawString(&dev, fx16G, xpos, ypos, (uint8_t*)line, WHITE);
                
        if (xpos >= 18) { // Evitamos que xpos haga underflow
          xpos -= 18; 
        }
          
        line = strtok(NULL, "\r\n");
      }
       
      lcdDrawFinish(&dev);
      ESP_LOGI(TAG, "Pantalla actualizada");
    }
  }

  // Does not reach here
  while (1) {
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

/**
 * @brief Mounts the SPIFFS filesystem on the ESP32.
 * 
 * This function takes the base path, partition label, and maximum number of files as parameters to configure and mount the SPIFFS filesystem. It uses the esp_vfs_spiffs_register function to perform the mounting and checks for errors during the process. If the mounting is successful, it retrieves and logs the total and used space of the SPIFFS partition. This function is essential for enabling file storage capabilities on the ESP32, allowing other parts of the program to read from and write to files stored in SPIFFS.
 * @param path The base path to mount the SPIFFS filesystem (e.g., "/spiffs").
 * @param label The label of the SPIFFS partition to use. If set to NULL, the first partition with subtype=spiffs will be used.
 * @param max_files The maximum number of files that can be open simultaneously in the SPIFFS filesystem.
 * @return esp_err_t - Returns ESP_OK if the SPIFFS filesystem was mounted successfully, or an error code if there was a failure during mounting or initialization. The caller can use this return value to handle errors appropriately in the program.
 * @note: This function should be called during the initialization phase of the program before attempting to access any files in SPIFFS. Proper error handling is crucial to ensure that the filesystem is available for use and to diagnose any issues that may arise during mounting.
 */
extern "C" esp_err_t mountSPIFFS(const char * path, const char * label, int max_files) {
	esp_vfs_spiffs_conf_t conf = {
		.base_path = path,
		.partition_label = label,
		.max_files = static_cast<size_t>(max_files),
		.format_if_mount_failed =true
	};

	// Use settings defined above to initialize and mount SPIFFS filesystem.
	// Note: esp_vfs_spiffs_register is anall-in-one convenience function.
	esp_err_t ret = esp_vfs_spiffs_register(&conf);

	if (ret != ESP_OK) {
		if (ret ==ESP_FAIL) {
			ESP_LOGE(TAG, "Failed to mount or format filesystem");
		} else if (ret== ESP_ERR_NOT_FOUND) {
			ESP_LOGE(TAG, "Failed to find SPIFFS partition");
		} else {
			ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)",esp_err_to_name(ret));
		}
		return ret;
	}

	size_t total = 0, used = 0;
	ret = esp_spiffs_info(conf.partition_label, &total, &used);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to get SPIFFS partition information (%s)",esp_err_to_name(ret));
	} else {
		ESP_LOGI(TAG,"Mount %s to %s success", path, label);
		ESP_LOGI(TAG,"Partition size: total: %d, used: %d", total, used);
	}

	return ret;
}

// --- Connection handlers ---
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        isConnected = 0;
        if (s_retry_num < ESP_MAXIMUM_RETRY) {
          esp_wifi_connect();
          s_retry_num++;
          ESP_LOGI(WIFI_TAG, "retry to connect to the AP");
        } else {
          xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
          ESP_LOGI(WIFI_TAG, "connect to the AP fail");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        isConnected = 1;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void mqtt_event_handler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data) {
  AppContext *context = static_cast<AppContext *>(handler_args);
    
  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
  int msg_id;

  switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
      ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
      msg_id = esp_mqtt_client_subscribe(mqtt_client, "m5stick/sensors/charge", 0);
      ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
      msg_id = esp_mqtt_client_subscribe(mqtt_client, "m5stick/sensors/discharge", 0);
      ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
      msg_id = esp_mqtt_client_subscribe(mqtt_client, "m5stick/relay/#", 0);
      ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
      break;

    case MQTT_EVENT_DISCONNECTED:
      ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
      break;

    case MQTT_EVENT_SUBSCRIBED:
      ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d, return code=0x%02x", event->msg_id, (uint8_t)*event->data);
      break;

    case MQTT_EVENT_UNSUBSCRIBED:
      ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
      break;

    case MQTT_EVENT_PUBLISHED:
      ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
      printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
      printf("DATA=%.*s\r\n", event->data_len, event->data);
      break;

    case MQTT_EVENT_DATA:
      ESP_LOGI(TAG, "MQTT_EVENT_DATA");
      printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
      printf("DATA=%.*s\r\n", event->data_len, event->data);
      if (strncmp(event->topic, "m5stick/relay/", 14) == 0) {
        char relay_char = event->topic[14];
        int relay_num = relay_char - '0'; // Extrae el número (1-4)
                
        if (!experiment_flag) {
          if (relay_num >= 1 && relay_num <= 4 && event->data_len > 0) {
            bool turn_on = event->data;
            context->relayController -> set_relay(relay_num, turn_on);
            experiment_flag = turn_on; // Activar el modo experimento si se enciende un relé
            ESP_LOGI(TAG, "MQTT: Relé %d -> %s", relay_num, turn_on ? "ON" : "OFF");
          }
        }
      }
      break;

    case MQTT_EVENT_ERROR:
      ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
      if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
        ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
        ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
        ESP_LOGI(TAG, "Last captured errno : %d (%s)", event->error_handle->esp_transport_sock_errno,
          strerror(event->error_handle->esp_transport_sock_errno));
      } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
        ESP_LOGI(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
      } else {
        ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
      }
      break;

    default:
      ESP_LOGI(TAG, "Other event id:%d", event->event_id);
      break;
  }
}

void wifi_and_mqtt_init(AppContext *context) {
    ESP_ERROR_CHECK(nvs_flash_init());
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);

    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, ESP_WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, ESP_WIFI_PASS);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Esperar conexión antes de arrancar MQTT
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = MQTT_BROKER_URI;
    
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, context);
    esp_mqtt_client_start(mqtt_client);
}

// ============ I2C FUNCTIONS ============

/**
 * @brief Initializes the I2C master interface for communication with the relay controller and ADS1115 ADC.
 * 
 * This function configures the I2C parameters such as the mode (master), GPIO pins for SDA and SCL, pull-up resistors, and clock speed. It then applies the configuration using i2c_param_config and installs the I2C driver. The function returns an esp_err_t value indicating the success or failure of the initialization process, which can be used for error handling in the calling code.
 * @return esp_err_t - Returns ESP_OK if the I2C master was initialized successfully, or an error code if there was a failure during configuration or driver installation.
 */
esp_err_t i2c_master_init() {
    i2c_config_t conf;
    memset(&conf, 0, sizeof(i2c_config_t));
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// ============ ESP32 PIN READ VOLTAGE =============

/**
 * @brief Reads the battery voltage using the ESP32's internal ADC on a specified channel.
 * 
 * This function performs the following steps:
 * 1. Initializes the ADC in one-shot mode for the specified channel (GPIO 36 / ADC_CHANNEL_0).
 * 2. Configures the ADC attenuation to allow for a wider voltage range (up to approximately 3.3V).
 * 3. Reads the raw ADC value from the specified channel.
 * 4. Converts the raw ADC value to a voltage using the formula: Voltage = (Raw / 4095) * 3.3V, where 4095 is the maximum value for a 12-bit ADC.
 * 5. Deletes the ADC unit handle to free resources.
 * 6. Returns the calculated voltage in Volts. If there is an error during ADC initialization or reading, it returns -1.0 to indicate a failure.
 * 
 * @note: This function uses the ADC in one-shot mode, which means it initializes the ADC, performs a single reading, and then deinitializes it. This is suitable for infrequent voltage measurements to save power. For continuous monitoring, consider using the ADC in continuous mode instead.
 * @return float - The calculated battery voltage in Volts. Returns -1.0 if there was an error during ADC initialization or reading.
 * @configuration:
 * - ADC Unit: ADC_UNIT_1
 * - ADC Channel: ADC_CHANNEL_0 (GPIO 36)
 * - Attenuation: ADC_ATTEN_DB_12 (allows for input voltages up to approximately 3.3V)
 * - Bitwidth: ADC_BITWIDTH_DEFAULT (uses the maximum supported width, typically 12 bits for ESP32)
 * - The function assumes a voltage divider is used if the battery voltage exceeds the ADC input range, and the conversion formula should be adjusted accordingly based on the voltage divider ratio.
 */
float GetADCPinVoltage() { // TODO: Pasar configuración como parámetros para hacerlo más flexible (pin, atenuación, bitwidth)
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
      .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    adc_oneshot_chan_cfg_t config = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
    adc_oneshot_config_channel(adc1_handle, BAT_ADC_CHAN, &config);

    int adc_raw;
    adc_oneshot_read(adc1_handle, BAT_ADC_CHAN, &adc_raw);
    
    // Simple conversion to voltage (assuming a voltage divider is used if battery voltage exceeds ADC range)
    float voltage = (adc_raw * 3.3 / 4095.0);
    
    adc_oneshot_del_unit(adc1_handle);
    return voltage;
}

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
void ScanI2CBus() {
  printf("Escaneando bus I2C...\n");
  for (int i = 1; i < 127; i++) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t res = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(2));
    i2c_cmd_link_delete(cmd);
    if (res == ESP_OK) {
        printf("¡Dispositivo encontrado en dirección: 0x%02X!\n", i);
    }
  }
}

/**
 * @brief FreeRTOS task that periodically reads the battery voltage using the GetADCPinVoltage function and sends the voltage value to the display task via a queue.
 * 
 * This task performs the following steps:
 * 1. Enters an infinite loop where it reads the battery voltage every 10 seconds.
 * 2. Calls the GetADCPinVoltage function to read the voltage from the specified ADC channel.
 * 3. Formats the voltage value into a string message (e.g., "BAT: 3.70 V") to be sent to the display.
 * 4. Sends the formatted message to the display task using the xQueueDisplay queue.
 * 5. Additionally, it sends a message with the voltage reading to the UART for debugging purposes.
 * 6. Waits for 1 second (1000 ms) before repeating the process.
 */
extern "C" void BatteryTask(void *pvParameters) {
  char msg_to_queue[128]; 
  char uart_msg[128];
  //char mqtt_payload[128];

  AppContext *app_context = static_cast<AppContext *>(pvParameters);
  ADS1115 &adc = *(app_context->adsController);

  while (1) {
    // Read voltages and current from ADS1115
    float cell_1_voltage = adc.ReadVoltage(3);       
    float cell_2_voltage = adc.ReadVoltage(2, true); 
    float cell_3_voltage = adc.ReadVoltage(1, true); 
    float amp = adc.ReadCurrent(0);             
    
    // Create the message for the display if an experiment is not running. During experiments, the display is updated by the MeasureCharging/Discharging tasks instead, so we avoid overwriting that data with raw voltage readings.
    if (!experiment_task_handle && experiment_flag) {
      if (cell_1_voltage <= 3.4 || (cell_2_voltage / 2) <= 3.4 || (cell_3_voltage / 3) <= 3.4) {
        ESP_LOGE(TAG, "Auto-read: Low voltage detected! C1: %.2f V, C2: %.2f V, C3: %.2f V", cell_1_voltage, cell_2_voltage / 2, cell_3_voltage / 3);
        buzzer_alert();
        app_context->relayController->set_all_relays_mask(0x0); // Critical safety threshold reached
        experiment_flag = false; // Finish experiments
        continue;
      }

      if (cell_3_voltage >= 12.6f) {
        ESP_LOGE(TAG, "Auto-read: Battery is overcharged!");
        buzzer_warn();
        app_context->relayController->set_all_relays_mask(0x0); // Cut power to prevent overcharging
        continue;
      }
    }

    if (!experiment_task_handle) {
      snprintf(msg_to_queue, sizeof(msg_to_queue), 
               "Cell 1: %.2f V\n"
               "Cell 2: %.2f V\n"
               "Cell 3: %.2f V\n"
               "AMP: %.2f A",
               cell_1_voltage, cell_2_voltage / 2, cell_3_voltage / 3, amp);

      if (xQueueDisplay != NULL) {
        xQueueSend(xQueueDisplay, msg_to_queue, 0);
      }

      snprintf(uart_msg, sizeof(uart_msg), 
               "\r\nAuto-read:\r\n"
               "CELL 1 VOLT: %.2f V\r\n"
               "CELL 2 VOLT: %.2f V\r\n"
               "CELL 3 VOLT: %.2f V\r\n"
               "AMP: %.2f A\r\n", 
               cell_1_voltage, cell_2_voltage / 2, cell_3_voltage / 3, amp);
      uart_write_bytes(UART_NUM, uart_msg, strlen(uart_msg));
    }

    vTaskDelay(pdMS_TO_TICKS(10000)); // 10 second delay between readings
  }
}

// ============ EXPERIMENT PHASE TASKS ============

// Runs during the charging phase. Samples sensors every EXPERIMENT_INTERVAL_MS
// for CHARGE_DURATION_MS total, then notifies ExperimentCycle it is done.
static void MeasureCharging(void *pvParameters) {
  PhaseTaskParams *phase_params = static_cast<PhaseTaskParams *>(pvParameters);
  AppContext *ctx = phase_params->app_context;
  ADS1115 &adc = *(ctx->adsController);
  const uint16_t cycle = phase_params->cycle_index;

  char uart_msg[160];
  char mqtt_payload[128];
  char display_msg[128];
  uint8_t safety_breach_count = 0;

  uart_write_bytes(UART_NUM, "--- CHARGING PHASE ---\r\n", 24);

  while (1) {
    float voltage_cell_1  = adc.ReadVoltage(3);
    float voltage_cell_2  = adc.ReadVoltage(2, true);
    float voltage_cell_3  = adc.ReadVoltage(1, true);
    float current  = adc.ReadCurrent(0);
      
    // Safety check
    if (voltage_cell_3 >= 12.5f) {
      if (safety_breach_count < 5) {
        ESP_LOGW(TAG, "[CHG] High voltage detected! V: %.2f V. Breach count: %d", voltage_cell_3, safety_breach_count + 1);
        safety_breach_count++;
      } else {
        ESP_LOGE(TAG, "[CHG] Battery is charged! V: %.2f V", voltage_cell_3);
        buzzer_warn();
        break;
      }
    }

    // Show data on display
    snprintf(display_msg, sizeof(display_msg),
             "CHG\r\n"
             "Cycle: %d\r\n"
             "Cell 1: %.2fV\r\n"
             "Cell 2: %.2fV\r\n"
             "Cell 3: %.2fV\r\n"
             "Battery: %.2fV\r\n"
             "Current: %.2fA",
             cycle, voltage_cell_1, voltage_cell_2 / 2, voltage_cell_3 / 3, voltage_cell_3, current);
      
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
             cycle, voltage_cell_1, voltage_cell_2 / 2, voltage_cell_3 / 3, voltage_cell_3, current);
    uart_write_bytes(UART_NUM, uart_msg, strlen(uart_msg));

    // Publish data to MQTT
    if (mqtt_client) {
      uint32_t elapsed_s = (uint32_t)((esp_timer_get_time() - experiment_start_us) / 1000000);
      snprintf(mqtt_payload, sizeof(mqtt_payload),
               "{\"phase\":\"charge\",\"cycle\":%d,\"t_s\":%"PRIu32","
               "\"cell_1\":%.2f,\"cell_2\":%.2f,\"cell_3\":%.2f,\"battery\":%.2f,\"current\":%.2f}",
               cycle, elapsed_s, voltage_cell_1, voltage_cell_2 / 2, voltage_cell_3 / 3, voltage_cell_3, current);
      esp_mqtt_client_publish(mqtt_client, "m5stick/sensors/charge", mqtt_payload, 0, 1, 0);
    }
  
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
  char mqtt_payload[128];
  char display_msg[128];
  uint8_t safety_breach_count = 0;

  uart_write_bytes(UART_NUM, "--- DISCHARGING PHASE ---\r\n", 27);

  float voltage_cell_1  = adc.ReadVoltage(3);
  float voltage_cell_2  = adc.ReadVoltage(2, true);
  float voltage_cell_3  = adc.ReadVoltage(1, true);
  float current  = adc.ReadCurrent(0);

  if (voltage_cell_1 <= 3.2f || (voltage_cell_2 / 2) <= 3.2f || (voltage_cell_3 / 3) <= 3.2f) {
    ESP_LOGE(TAG, "[DIS] Safety threshold — aborting\nC1: %.2f V, C2: %.2f V, C3: %.2f V", voltage_cell_1, voltage_cell_2 / 2, voltage_cell_3 / 3);
    buzzer_alert();
    experiment_flag = false; // Finish experiments
    ctx->relayController->set_all_relays_mask(0x0);
    xTaskNotifyGive(phase_params->caller_task);
    vTaskDelete(NULL);
    return;
  }

  while (1) {
    voltage_cell_1  = adc.ReadVoltage(3);
    voltage_cell_2  = adc.ReadVoltage(2, true);
    voltage_cell_3  = adc.ReadVoltage(1, true);
    current  = adc.ReadCurrent(0);

    // Safety check — cut off if any cell drops below 3.5V during discharge
    if (voltage_cell_1 <= 3.52f || (voltage_cell_2 / 2) <= 3.52f || (voltage_cell_3 / 3) <= 3.52f) {
      ESP_LOGE(TAG, "[DIS] Low voltage detected! C1: %.2f V, C2: %.2f V, C3: %.2f V", voltage_cell_1, voltage_cell_2 / 2, voltage_cell_3 / 3);
      if (voltage_cell_1 <= 3.3f || (voltage_cell_2 / 2) <= 3.3f || (voltage_cell_3 / 3) <= 3.3f) {
        buzzer_alert();
      }
      safety_breach_count++;
    }

    if (safety_breach_count >= 20) { // Allow a few transient breaches before cutting off
      ctx->relayController->set_all_relays_mask(0x0);
      break;
    }

    // Show data on display
    snprintf(display_msg, sizeof(display_msg),
             "DIS\r\n"
             "Cycle: %d\r\n"
             "Cell 1: %.2fV\r\n"
             "Cell 2: %.2fV\r\n"
             "Cell 3: %.2fV\r\n"
             "Battery: %.2fV\r\n"
             "Current: %.2fA",
             cycle, voltage_cell_1, voltage_cell_2 / 2, voltage_cell_3 / 3, voltage_cell_3, current);
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
             cycle, voltage_cell_1, voltage_cell_2 / 2, voltage_cell_3 / 3, voltage_cell_3, current);
    uart_write_bytes(UART_NUM, uart_msg, strlen(uart_msg));

    // Publish data to MQTT
    if (mqtt_client) {
      uint32_t elapsed_s = (uint32_t)((esp_timer_get_time() - experiment_start_us) / 1000000);
      snprintf(mqtt_payload, sizeof(mqtt_payload),
               "{\"phase\":\"discharge\",\"cycle\":%d,\"t_s\":%"PRIu32","
               "\"cell_1\":%.2f,\"cell_2\":%.2f,\"cell_3\":%.2f,\"battery\":%.2f,\"current\":%.2f}",
               cycle, elapsed_s, voltage_cell_1, voltage_cell_2 / 2, voltage_cell_3 / 3, voltage_cell_3, current);
      esp_mqtt_client_publish(mqtt_client, "m5stick/sensors/discharge", mqtt_payload, 0, 1, 0);
    }
  
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
    xTaskCreate(MeasureCharging, "CHG_PHASE", 1024 * 4, &charge_params, 3, &phase_task_handle);
    // Block until MeasureCharging calls xTaskNotifyGive
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    phase_task_handle = NULL; // Phase finished normally

    gpio_set_level(GPIO_OUTPUT_IO_1, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));
    gpio_set_level(GPIO_OUTPUT_IO_1, 0);

    if (!experiment_flag) {
      break; // Aborted during charging
    }

    // Prompt the user to confirm the discharge phase.
    // They can either press Button B (GPIO 37) or send "DISCHARGE" via UART.
    // ExperimentCycle blocks here until one of those happens.
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
    xTaskCreate(MeasureDischarging, "DIS_PHASE", 1024 * 4, &discharge_params, 3, &phase_task_handle);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    phase_task_handle = NULL; // Phase finished normally

    if (!experiment_flag) {
      break; // Aborted during discharging
    }

    app_context->relayController->set_relay(1, false);

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

/**
 * @brief FreeRTOS task that handles UART communication for receiving commands and controlling relays or reading sensors.
 * 
 * This task performs the following steps:
 * 1. Enters an infinite loop where it waits for incoming data on the UART.
 * 2. When data is received, it reads the bytes into a buffer and processes the command.
 * 3. Recognizes specific commands such as "HELLO", "R1 ON", "R1 OFF", "R2 ON", "R2 OFF", "R3 ON", "R3 OFF", "R4 ON", "R4 OFF", "ALL ON", "ALL OFF", "VOLTAGE", and "CURRENT".
 * 4. For relay control commands, it calls the set_relay or set_all_relays_mask functions to update the relay states and sends a confirmation message back via UART.
 * 5. For "VOLTAGE" and "CURRENT" commands, it calls the GetVoltage and GetCurrentAmp functions respectively, formats the results into a message, and sends it back via UART.
 * 6. It also sends messages to the display task to update the screen with the current status or sensor readings.
 * 7. If an unrecognized command is received, it sends a "Comando no reconocido" message back via UART.
 * 8. After processing a command, it waits for 2 seconds before allowing the next command to be processed to prevent flooding the UART with responses.
 * 
 * @note: This task assumes that the UART is properly configured and that the xQueueDisplay queue is initialized for sending messages to the display task. It also relies on the set_relay, set_all_relays_mask, GetVoltage, and GetCurrentAmp functions to perform the necessary actions based on the received commands.
 * @param pvParameters - Pointer to task parameters (not used in this implementation).
 */
extern "C" void UARTTask(void *pvParameters) {
  // Get parameters from the task creation (if needed)
  AppContext *app_context = static_cast<AppContext *>(pvParameters);
  ServoController *servo_controller = app_context->servoController;
  I2CRelay *relay_controller = app_context->relayController;
  ADS1115 *ads_controller = app_context->adsController;

  char command[BUF_SIZE];
  int cmd_idx = 0;

  auto send_response = [&](const std::string& uart_msg, const std::string& tft_msg) {
    uart_write_bytes(UART_NUM, uart_msg.c_str(), uart_msg.length());
    
    if (!tft_msg.empty()) {
      char msg_to_queue[64];
      strncpy(msg_to_queue, tft_msg.c_str(), sizeof(msg_to_queue) - 1);
      msg_to_queue[sizeof(msg_to_queue) - 1] = '\0'; // Null-terminate the string
      xQueueSend(xQueueDisplay, &msg_to_queue, 0);
    }
  };

  // Dictionary mapper for commands to functions
  std::unordered_map<std::string, std::function<void()>> command_map = {
    {"HELLO",   [&]() { send_response("Hola desde ESP32!\r\n", ""); }},
    
    // Individual relay commands
    {"R1 ON",   [&]() { relay_controller->set_relay(1, true);  send_response("Encendiendo R1\r\n", "RELAY 1: ON"); experiment_flag = true; }},
    {"R1 OFF",  [&]() { relay_controller->set_relay(1, false); send_response("Apagando R1\r\n", "RELAY 1: OFF"); experiment_flag = false; }},
    {"R2 ON",   [&]() { relay_controller->set_relay(2, true);  send_response("Encendiendo R2\r\n", "RELAY 2: ON"); experiment_flag = true; }},
    {"R2 OFF",  [&]() { relay_controller->set_relay(2, false); send_response("Apagando R2\r\n", "RELAY 2: OFF"); experiment_flag = false; }},
    {"R3 ON",   [&]() { relay_controller->set_relay(3, true);  send_response("Encendiendo R3\r\n", "RELAY 3: ON"); experiment_flag = true; }},
    {"R3 OFF",  [&]() { relay_controller->set_relay(3, false); send_response("Apagando R3\r\n", "RELAY 3: OFF"); experiment_flag = false; }},
    {"R4 ON",   [&]() { relay_controller->set_relay(4, true);  send_response("Encendiendo R4\r\n", "RELAY 4: ON"); experiment_flag = true; }},
    {"R4 OFF",  [&]() { relay_controller->set_relay(4, false); send_response("Apagando R4\r\n", "RELAY 4: OFF"); experiment_flag = false; }},
    
    // Global relay commands
    {"ALL ON",  [&]() { relay_controller->set_all_relays_mask(0x0F); send_response("Encendiendo todos\r\n", "ALL: ON"); }},
    {"ALL OFF", [&]() { relay_controller->set_all_relays_mask(0x00); send_response("Apagando todos\r\n", "ALL: OFF"); }},
    
    // Sensor readings
    {"VOLTAGE A0", [&]() { 
      float v = ads_controller->ReadVoltage(0);
      send_response("Bat: " + std::to_string(v) + "V\r\n", "Bat: " + std::to_string(v) + "V"); 
    }},
    {"VOLTAGE A1", [&]() { 
      float v = ads_controller->ReadVoltage(1, true);
      send_response("Bat: " + std::to_string(v) + "V\r\n", "Bat: " + std::to_string(v) + "V"); 
    }},
    {"VOLTAGE A2", [&]() { 
      float v = ads_controller->ReadVoltage(2, true);
      send_response("V(A2): " + std::to_string(v) + "V\r\n", "V(A2): " + std::to_string(v) + "V"); 
    }},
    {"VOLTAGE A3", [&]() { 
      float v = ads_controller->ReadVoltage(3);
      send_response("V(A3): " + std::to_string(v) + "V\r\n", "V(A3): " + std::to_string(v) + "V"); 
    }},
    {"VOLTAGE PIN", [&]() { 
      float v = GetADCPinVoltage();
      send_response("Bat: " + std::to_string(v) + "V\r\n", "Bat: " + std::to_string(v) + "V"); 
    }},
    {"CURRENT", [&]() { 
      float ampers = ads_controller->ReadCurrent(0);
      send_response("Current: " + std::to_string(ampers) + "A\r\n", "Current: " + std::to_string(ampers) + "A"); 
    }},
    {"START CYCLES", [&]() {
      if (experiment_task_handle == NULL) {
        send_response("Starting experiment cycles...\r\n", "EXP: Running");
        start_experiment_task(app_context);
      } else {
        send_response("Experiment already running\r\n", "EXP: Running");
      }
    }},
    {"STOP CYCLES", [&]() {
      stop_experiment_task(app_context);
      send_response("Experiment stopped\r\n", "EXP: Stopped");
    }},
    {"DISCHARGE", [&]() {
      if (experiment_task_handle != NULL && discharge_ready_sem != NULL) {
        xSemaphoreGive(discharge_ready_sem);
        send_response("Discharge phase starting...\r\n", "DIS: Starting");
      } else {
        send_response("No experiment waiting for discharge\r\n", "");
      }
    }},
    {"START CHARGE", [&]() {
      send_response("Initiating charge sequence...\r\n", "Charging...");
      servo_controller -> start_timer();
      int servo_pulse = SERVO_STOP_PULSEWIDTH - servo_controller -> get_servo_speed();
      ESP_LOGI(TAG, "Move to long press%d", servo_pulse);
      servo_controller->set_servo_pulse(servo_pulse);
      //Add delay, since it takes time for servo to rotate, usually 200ms/60degree rotation under 5V power supply
      vTaskDelay(pdMS_TO_TICKS(500));

	    servo_pulse = SERVO_STOP_PULSEWIDTH;
      ESP_LOGI(TAG, "Wait long press%d", servo_pulse);
      servo_controller->set_servo_pulse(servo_pulse);
      vTaskDelay(pdMS_TO_TICKS(3000));

    	servo_pulse = SERVO_STOP_PULSEWIDTH + servo_controller -> get_servo_speed();
      ESP_LOGI(TAG, "Move to first depress %d", servo_pulse);
      servo_controller->set_servo_pulse(servo_pulse);
      vTaskDelay(pdMS_TO_TICKS(200));

	    servo_pulse = SERVO_STOP_PULSEWIDTH - servo_controller -> get_servo_speed();
      ESP_LOGI(TAG, "Move to short press %d", servo_pulse);
      servo_controller->set_servo_pulse(servo_pulse);
      vTaskDelay(pdMS_TO_TICKS(500));

	    servo_pulse = SERVO_STOP_PULSEWIDTH + servo_controller -> get_servo_speed();
      ESP_LOGI(TAG, "Move to final depress %d", servo_pulse);
      servo_controller -> set_servo_pulse(servo_pulse);
      vTaskDelay(pdMS_TO_TICKS(200));
      servo_controller -> stop_timer();
    }}
  };

  // Initial prompt
  vTaskDelay(pdMS_TO_TICKS(5000));
  uart_write_bytes(UART_NUM, "> ", 2);

  uart_event_t event;
  uint8_t dtmp[BUF_SIZE];

  // Task loop to handle UART events and process commands
  while (1) {
    if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
      if (event.type == UART_DATA) {
        int len = uart_read_bytes(UART_NUM, dtmp, event.size, portMAX_DELAY);
        
        for (int i = 0; i < len; i++) {
          char rx_char = dtmp[i];
          
          if (rx_char == '\n' || rx_char == '\r') {
            uart_write_bytes(UART_NUM, "\r\n", 2);
            
            if (cmd_idx > 0) {
              command[cmd_idx] = '\0'; 
              
              // To uppercase for case-insensitive command recognition
              std::string cmd_str(command);
              std::transform(cmd_str.begin(), cmd_str.end(), cmd_str.begin(), ::toupper);
              ESP_LOGI(TAG, "Command received: %s", cmd_str.c_str());

              auto it = command_map.find(cmd_str);
                if (it != command_map.end()) {
                  it->second();
                } else {
                  uart_write_bytes(UART_NUM, "Command not recognized\r\n", 23);
                }
              cmd_idx = 0; 
            }
            vTaskDelay(pdMS_TO_TICKS(1000)); 
            uart_write_bytes(UART_NUM, "\r\n> ", 4);
          }
          else if (rx_char == '\b' || rx_char == 127) {
            if (cmd_idx > 0) {
              cmd_idx--;
              uart_write_bytes(UART_NUM, "\b \b", 3);
            }
          }
          else {
            uart_write_bytes(UART_NUM, &rx_char, 1);
            if (cmd_idx < BUF_SIZE - 1) {
              command[cmd_idx++] = rx_char;
            }
          }
        }
      } 
      else if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
        uart_flush_input(UART_NUM);
        xQueueReset(uart_queue);
      }
    } else {
      uart_write_bytes(UART_NUM, "> ", 2);
    }
  }
}

// ============ MAIN FUNCTION ============

/**
 * @brief Starting point of the application. Initializes peripherals, creates tasks, and starts the scheduler.
 * 
 * This function performs the following steps:
 * 1. Initializes the SPIFFS file system for storing fonts.
 * 2. Initializes the I2C master for communication with the relay module and ADS1115.
 * 3. Scans the I2C bus to detect connected devices.
 * 4. Configures the UART for command input and output.
 * 5. Creates a FreeRTOS queue for sending messages to the display task.
 * 6. Creates the display task (ShowText) to handle screen updates.
 * 7. Creates the battery monitoring task (BatteryTask) to periodically read and display battery voltage.
 * 8. Creates the UART command handling task (UARTTask) to process incoming commands and control relays or read sensors.
 * Note: The main function does not return and runs indefinitely, managing the tasks and peripherals of the ESP32.
 * 
 * @return void
 */
extern "C" void app_main(void) {
  std::cout << "----- Initialising program -----" << std::endl;

  // Mount SPIFFS File System on FLASH
  ESP_LOGI(TAG, "Initializing SPIFFS");
  ESP_ERROR_CHECK(mountSPIFFS("/fonts", "storage1", 8));

  // Initialize I2C for relay and ADS1115
  if (i2c_master_init() == ESP_OK) {
    ESP_LOGI(TAG, "I2C master initialized successfully");
  }

  // Scan I2C bus to verify connections (useful for debugging, comment out in production)
  // ScanI2CBus();

  // Initialize peripherals
  static ServoController servo;
  static I2CRelay relay_controller;
  static ADS1115 ads_controller(I2C_MASTER_NUM, ADS1115_ADDR);
  servo.init_servo();

  static AppContext app_context = {
    .servoController = &servo,
    .relayController = &relay_controller,
    .adsController = &ads_controller
  };

  // Configure UART
  uart_config_t cfg;
  memset(&cfg, 0, sizeof(uart_config_t));
  cfg.baud_rate = 115200;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uart_param_config(UART_NUM, &cfg);
  uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue, 0);

  // Message queue for display updates
  xQueueDisplay = xQueueCreate(5, 64);

  // *** WiFi init must come before gpio_install_isr_service ***
  // esp_wifi_init() internally calls gpio_install_isr_service(), which resets
  // the ISR dispatch table and wipes any handlers registered before it.
  // Registering our handlers AFTER wifi init avoids this conflict.
  wifi_and_mqtt_init(&app_context);

  // Configure button GPIO pins.
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_NEGEDGE;
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;    // No-op for GPIO 34-39, external pull-up on PCB
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_config(&io_conf);

  // Create queue and task for button events
  gpio_event_queue = xQueueCreate(10, sizeof(uint32_t));
  // Create the discharge confirmation semaphore (starts empty — taken by ExperimentCycle)
  discharge_ready_sem = xSemaphoreCreateBinary();
  // Create the phase-skip semaphore (starts empty — given by Button B mid-phase)
  phase_skip_sem = xSemaphoreCreateBinary();
  // Initialise buzzer LEDC channel
  buzzer_init();
  xTaskCreate(ButtonTask, "button_task", 2048, &app_context, 10, NULL);

  // Install ISR service — WiFi may have already done this, ESP_ERR_INVALID_STATE is fine
  esp_err_t isr_err = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(isr_err));
  }
  gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
  gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);

  // Display task
  xTaskCreate(ShowText, "TFT", 1024 * 6, NULL, 4, NULL);

  // Battery monitoring task
  xTaskCreate(BatteryTask, "BAT_MONITOR", 1024 * 4, &app_context, 2, NULL);

  // UART command handling task
  xTaskCreate(UARTTask, "UART_COM", 1024 * 6, &app_context, 3, NULL);
}

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
#include <fstream> // std::ifstream for file reading (if needed for configuration)
#include <sstream> // std::istringstream for string splitting

#include "sgm2578.h" // Display library for M5StickCPlus2
#include "st7789.h" // TFT display driver for ST7789-based displays (used in M5StickCPlus2)
#include "fontx.h" // Font rendering library for displaying text on the TFT screen
#include "bmpfile.h" // Bitmap file handling library (not used in this code but included for potential future use in displaying images on the TFT)
#include "I2Crelay.h" // Header file for relay control functions
#include "ADS1115.h" // Header file for ADS1115 ADC functions
#include "mg996r.h" // Header file for MG996R servo control functions
#include "wifi_mqtt.h" // Header file for WiFi and MQTT functions
#include "shared.h" // Header file for common definitions and utilities
#include "experiment.h" // Header file for experiment functions and definitions
#include "buzzer.h" // Header file for buzzer control functions

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

// Power hold GPIO for the display (used to ensure the display receives power)
#define POWER_HOLD_GPIO 4
// GPIO for enabling the SGM2578 power management IC (used to provide power to the display)
#define SGM2578_ENABLE_GPIO 27

uint8_t current_tag_index = -1;

// Interruption handler for buttons.
// WiFi glitch filtering is handled in ButtonTask instead.
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_event_queue, &gpio_num, NULL);
}

std::vector<std::string> split_string(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(str);
    uint8_t size_limit = 10; 
    while (std::getline(tokenStream, token, delimiter) || tokens.size() >= size_limit) {
        tokens.push_back(token);
    }
    return tokens;
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
        // Button B: toggle experiment — start if idle, stop if running
        if (experiment_task_handle == NULL) {
          uart_write_bytes(UART_NUM, "Starting experiment cycles...\r\n", 31);
          char display_msg[64];
          snprintf(display_msg, sizeof(display_msg), "Experiment started");
          xQueueSend(xQueueDisplay, &display_msg, portMAX_DELAY);
          vTaskDelay(pdMS_TO_TICKS(100)); // Small delay to ensure the display message is shown before starting the experiment
          is_waiting_discharge = false; // Ensure we're not waiting for discharge confirmation when starting a new experiment
          start_experiment_task(context);
        }  else {
          uart_write_bytes(UART_NUM, "Stopping experiment cycles...\r\n", 31);
          char display_msg[64];
          snprintf(display_msg, sizeof(display_msg), "Experiment stopped by user");
          xQueueSend(xQueueDisplay, &display_msg, portMAX_DELAY);
          is_waiting_discharge = false; // Stop waiting and proceed to stop the experiment
          stop_experiment_task(context);
        }
      } else if (gpio_num == GPIO_INPUT_IO_1) {
        // Button A: advance the experiment phase.
        // - If a phase task is running -> skip it early (phase_skip_sem)
        // - If waiting for discharge confirmation -> release that wait (discharge_ready_sem)
        // - If no experiment is running -> stop (nothing to advance)
        if (experiment_task_handle != NULL) {
          if (is_waiting_discharge) {
            uart_write_bytes(UART_NUM, "Discharge confirmed\r\n", 21);
            is_waiting_discharge = false;
            xSemaphoreGive(discharge_ready_sem);
          } else {
            uart_write_bytes(UART_NUM, "Phase skip requested\r\n", 22);
            xSemaphoreGive(phase_skip_sem);
          }
        } else {
          stop_experiment_task(context);
        }
      } else if (gpio_num == GPIO_INPUT_IO_2) {
        // Button C: changes sub tags for the experiment
        current_tag_index = (current_tag_index + 1) % context->experimentParams->tags.size();
        context->experimentParams->current_tag = context->experimentParams->tags[current_tag_index];
        ESP_LOGI(TAG, "Sub-tag changed to: %s", context->experimentParams->current_tag.c_str());
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
 * 4. Enters an infinite loop where it waits for messages from the xQueueDisplay queue. When a message is received, it updates the display with the new text, rotating it for the M5StickC orientation.condition
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

  char screen_text[256] = "Waiting..."; // Initial message
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

void LeerConfiguracionSPIFFS(ExperimentParams &experiment_params) {
    // Abrir el archivo desde la partición SPIFFS
    std::ifstream file("/variables/exp_vars.txt");
    
    if (!file.is_open()) {
        ESP_LOGE("SPIFFS", "Error al abrir el archivo de configuración");
        return;
    }

    std::string line;
    // std::getline lee la línea entera hasta el salto de línea, respetando los espacios
    while (std::getline(file, line)) {
        
        // 1. Limpiar el posible retorno de carro '\r' (CRÍTICO en archivos creados en Windows)
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        // 2. Buscar la posición del separador '='
        size_t pos = line.find('=');
        if (pos != std::string::npos) {
            // 3. Extraer la clave y el valor
            std::string key = line.substr(0, pos);
            std::string value = line.substr(pos + 1);

            if (value.length() >= 2 && value.front() == '"' && value.back() == '"') {
                value = value.substr(1, value.length() - 2);
            }

            // --- Uso de las variables ---
            ESP_LOGI("SPIFFS", "Clave: '%s', Valor: '%s'", key.c_str(), value.c_str());
            
            if (key == "CHG_BIAS") {
            experiment_params.charge_bias = std::stof(value);
          } else if (key == "DCH_BIAS") {
            experiment_params.discharge_bias = std::stof(value);
          } else if (key == "CHG_TIME") {
            experiment_params.charge_time_limit_minutes = std::stoi(value);
          } else if (key == "DCH_TIME") {
            experiment_params.discharge_time_limit_minutes = std::stoi(value);
          } else if (key == "MSQT_IP") {
            experiment_params.mosquitto_ip = value;
          } else if (key == "BAT_TYPE") {
            experiment_params.battery_type = value;
          } else if (key == "EXP_NAME") {
            experiment_params.experiment_name = value;
          } else if (key == "EXP_SUBTAG") {
            experiment_params.experiment_tags = value;
            if (value.length() >= 2) {
              value = value.substr(1, value.length() - 2);
            }
            experiment_params.tags = split_string(value, ',');
          } else if (key == "DEVICE") {
            experiment_params.device_name = value;
          } else if (key == "LOCATION") {
            experiment_params.location = value;
          } else if (key == "BAT_CAP") {
            experiment_params.battery_capacity_ah = std::stof(value);
          } else if (key == "PCENT_R1") {
            experiment_params.percentage_resistance_1 = std::stof(value);
          } else if (key == "PCENT_R2") {
            experiment_params.percentage_resistance_2 = std::stof(value);
          } else if (key == "PCENT_R3") {
            experiment_params.percentage_resistance_3 = std::stof(value);
          } else if (key == "WIFI_SSID") {
            experiment_params.wifi_ssid = value;
          } else if (key == "WIFI_PASSWD") {
            experiment_params.wifi_password = value;
          } else if (key == "TIMER") {
            experiment_params.timer_flag = (value == "1") ? true : false;
          } else {
            ESP_LOGW(TAG, "Unknown parameter in file: %s", key.c_str());
          }
        }
    }
    file.close();
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
    if (experiment_flag) {
      vTaskDelay(pdMS_TO_TICKS(100000)); // During experiments, the display is updated by the MeasureCharging/Discharging tasks instead, so we avoid overwriting that data with raw voltage readings.
      continue;
    }

    // Read voltages and current from ADS1115
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
    auto amp  = adc.ReadCurrent(0, -0.00625f);
    vTaskDelay(pdMS_TO_TICKS(80)); // Small delay to ensure bus is free

    // Create the message for the display if an experiment is not running. During experiments, the display is updated by the MeasureCharging/Discharging tasks instead, so we avoid overwriting that data with raw voltage readings.
    if (!experiment_task_handle && experiment_flag) {
      if (voltage_cell_1.second != ESP_OK || voltage_cell_2.second != ESP_OK || voltage_cell_3.second != ESP_OK) {
        ESP_LOGE(TAG, "Error reading voltages: C1: %d, C2: %d, C3: %d", voltage_cell_1.second, voltage_cell_2.second, voltage_cell_3.second);
        buzzer_alert();
        continue; // Skip the rest of the loop and try again on the next iteration
      }

      if (voltage_cell_1.first <= 3.4 || (voltage_cell_2.first / 2) <= 3.4 || (voltage_cell_3.first / 3) <= 3.4) {
        ESP_LOGE(TAG, "Auto-read: Low voltage detected! C1: %.2f V, C2: %.2f V, C3: %.2f V", voltage_cell_1.first, voltage_cell_2.first / 2, voltage_cell_3.first / 3);
        buzzer_alert();
        app_context->relayController->set_all_relays_mask(0x0); // Critical safety threshold reached
        experiment_flag = false; // Finish experiments
        continue;
      }

      if (voltage_cell_3.first >= 12.6f) {
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
               voltage_cell_1.first, voltage_cell_2.first / 2, voltage_cell_3.first / 3, amp.first);

      if (xQueueDisplay != NULL) {
        xQueueSend(xQueueDisplay, msg_to_queue, 0);
      }

      snprintf(uart_msg, sizeof(uart_msg),
               "\r\nAuto-read:\r\n"
               "CELL 1 VOLT: %.2f V\r\n"
               "CELL 2 VOLT: %.2f V\r\n"
               "CELL 3 VOLT: %.2f V\r\n"
               "AMP: %.2f A\r\n",
               voltage_cell_1.first, voltage_cell_2.first / 2, voltage_cell_3.first / 3, amp.first);
      uart_write_bytes(UART_NUM, uart_msg, strlen(uart_msg));
    }

    vTaskDelay(pdMS_TO_TICKS(1000)); // 10 second delay between readings
  }
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
      auto v = ads_controller->ReadVoltage(0);
      send_response("Bat: " + std::to_string(v.first) + "V\r\n", "Bat: " + std::to_string(v.first) + "V");
    }},
    {"VOLTAGE A1", [&]() {
      auto v = ads_controller->ReadVoltage(1, true);
      send_response("Bat: " + std::to_string(v.first) + "V\r\n", "Bat: " + std::to_string(v.first) + "V");
    }},
    {"VOLTAGE A2", [&]() {
      auto v = ads_controller->ReadVoltage(2, true);
      send_response("V(A2): " + std::to_string(v.first) + "V\r\n", "V(A2): " + std::to_string(v.first) + "V");
    }},
    {"VOLTAGE A3", [&]() {
      auto v = ads_controller->ReadVoltage(3);
      send_response("V(A3): " + std::to_string(v.first) + "V\r\n", "V(A3): " + std::to_string(v.first) + "V");
    }},
    {"VOLTAGE PIN", [&]() {
      auto v = GetADCPinVoltage();
      send_response("Bat: " + std::to_string(v) + "V\r\n", "Bat: " + std::to_string(v) + "V");
    }},
    {"CURRENT", [&]() {
      auto v = ads_controller->ReadCurrent(0);
      send_response("Current: " + std::to_string(v.first) + "A\r\n", "Current: " + std::to_string(v.first) + "A");
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

/**
 * @brief Initializes the Grove I2C bus.
 *
 * This function initializes the I2C bus for communication with Grove modules.
 * It sets up the SDA and SCL pins and performs a bus clear operation.
*/
void InitGroveI2C() {
    gpio_num_t sda_pin = (gpio_num_t)I2C_MASTER_SDA_IO;
    gpio_num_t scl_pin = (gpio_num_t)I2C_MASTER_SCL_IO; 

    // Clear bus by toggling SCL and checking SDA
    gpio_set_direction(sda_pin, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(scl_pin, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_level(sda_pin, 1);
    gpio_set_level(scl_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(5));

    for (int i = 0; i < 9; i++) {
        gpio_set_level(scl_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(scl_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    // Generate a STOP condition to release the bus
    gpio_set_level(sda_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(scl_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(sda_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Initialize I2C master
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = { .clk_speed = 50000 }, // <-- 50kHz 
        .clk_flags = 0
    };
    
    i2c_param_config(I2C_NUM_0, &conf); 
    i2c_set_timeout(I2C_NUM_0, 20000); // 20000 cycles timeout (~400ms at 50kHz)
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
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
  ESP_ERROR_CHECK(mountSPIFFS("/variables", "storage3", 8));

  static ExperimentParams experiment_params;
  experiment_params.current_tag = "A";
  
  LeerConfiguracionSPIFFS(experiment_params);

  // Initialize I2C for relay and ADS1115
  InitGroveI2C();

  // Scan I2C bus to verify connections (useful for debugging, comment out in production)
  ScanI2CBus();

  // Initialize peripherals
  static ServoController servo;
  static I2CRelay relay_controller;
  static ADS1115 ads_controller(I2C_MASTER_NUM, ADS1115_ADDR);
  servo.init_servo();

  static AppContext app_context = {
    .servoController = &servo,
    .relayController = &relay_controller,
    .adsController = &ads_controller,
    .experimentParams = &experiment_params
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
  xQueueDisplay = xQueueCreate(5, 128 * sizeof(char)); // Queue of 5 messages, each up to 128 chars

  // *** WiFi init must come before gpio_install_isr_service ***
  // esp_wifi_init() internally calls gpio_install_isr_service(), which resets
  // the ISR dispatch table and wipes any handlers registered before it.
  // Registering our handlers AFTER wifi init avoids this conflict.
  wifi_and_mqtt_init(&app_context);

  // Syncronize time with SNTP for accurate timestamps in experiments and MQTT messages
  sincronizar_hora_sntp();

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
  gpio_isr_handler_add(GPIO_INPUT_IO_2, gpio_isr_handler, (void*) GPIO_INPUT_IO_2);

  // Display task
  xTaskCreate(ShowText, "TFT", 1024 * 6, NULL, 4, NULL);

  // Battery monitoring task
  xTaskCreate(BatteryTask, "BAT_MONITOR", 1024 * 4, &app_context, 2, NULL);

  // UART command handling task
  xTaskCreate(UARTTask, "UART_COM", 1024 * 6, &app_context, 3, NULL);
}

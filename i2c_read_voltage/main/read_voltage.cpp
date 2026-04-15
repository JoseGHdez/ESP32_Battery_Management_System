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
#include "esp_err.h" // esp_err_t type and error codes (e.g., ESP_OK, ESP_FAIL)
#include "esp_log.h" // ESP32 logging functions (e.g., ESP_LOGI, ESP_LOGE)
#include "esp_vfs.h" // ESP32 Virtual File System functions (e.g., esp_vfs_spiffs_register)
#include "esp_spiffs.h" // ESP32 SPIFFS functions (e.g., esp_spiffs_info)
#include "driver/gpio.h" // GPIO functions for controlling pins (e.g., gpio_set_level, gpio_set_direction)
#include "driver/uart.h" // UART functions for serial communication (e.g., uart_read_bytes, uart_write_bytes)
#include "driver/i2c.h" // I2C functions for communication with peripherals (e.g., i2c_master_write_to_device, i2c_master_write_read_device)
#include "driver/ledc.h" // LEDC functions for PWM control (e.g., ledc_timer_config, ledc_channel_config)
#include "esp_adc/adc_oneshot.h" // ESP32 ADC one-shot mode functions (e.g., adc_oneshot_unit_init_cfg_t, adc_oneshot_read)
#include "esp_adc/adc_cali.h" // ESP32 ADC calibration functions (e.g., adc_cali_characteristics_t, adc_cali_calibrate)
#include "esp_adc/adc_cali_scheme.h" // ESP32 ADC calibration schemes (e.g., ADC_CALI_SCHEME_CURVE_FITTING)

#include "sgm2578.h" // Display library for M5StickCPlus2
#include "st7789.h" // TFT display driver for ST7789-based displays (used in M5StickCPlus2)
#include "fontx.h" // Font rendering library for displaying text on the TFT screen
#include "bmpfile.h" // Bitmap file handling library (not used in this code but included for potential future use in displaying images on the TFT)
#include "I2Crelay.h" // Header file for relay control functions
#include "ADS1115.h" // Header file for ADS1115 ADC functions

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

// Power hold GPIO for the display (used to ensure the display receives power)
#define POWER_HOLD_GPIO 4
// GPIO for enabling the SGM2578 power management IC (used to provide power to the display)
#define SGM2578_ENABLE_GPIO 27

// ============ SERVO CONFIGURATION ============
#define SERVO_PIN             26   // ESP32 pin used for the servo
#define SERVO_MIN_PULSEWIDTH  410  // Ticks to 0.5ms (0 grades)
#define SERVO_MAX_PULSEWIDTH  2048 // Ticks to 2.5ms (180 grades)
#define SERVO_MAX_DEGREE      180  // Maximun angle for the servo

/** 
 * @brief Logger tag for the M5StickCPlus2 system 
 */
static const char *TAG = "M5_SYSTEM";

/** 
 * @brief Queue for sending messages to the display task
 */
static QueueHandle_t xQueueDisplay = NULL;

/** * @brief Queue for receiving UART input data
 */
static QueueHandle_t uart_queue;

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

  char texto_pantalla[64] = "Waiting..."; // Initial message

  while(1) {
    // Wait until receiving data
    if (xQueueReceive(xQueueDisplay, &texto_pantalla, portMAX_DELAY)) {
      lcdFillScreen(&dev, BLACK);
      lcdSetFontDirection(&dev, 1); // M5StickC rotation
            
      uint16_t xpos = (CONFIG_WIDTH - 1) - 20;
      uint16_t ypos = 10;
            
      lcdDrawString(&dev, fx16G, xpos, ypos, (uint8_t*)texto_pantalla, WHITE);
      lcdDrawFinish(&dev);
      ESP_LOGI(TAG, "Pantalla actualizada: %s", texto_pantalla);
    }
  }

  // Does not reach here, but if we wanted to clean up:
  // lcdFillScreen(&dev, BLACK);
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
    adc_oneshot_unit_init_cfg_t init_config1 = {.unit_id = ADC_UNIT_1};
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
    esp_err_t res = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(10));
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
 * 6. Waits for 10 seconds (10000 ms) before repeating the process.
 */
extern "C" void BatteryTask(void *pvParameters) {
    char msg_to_queue[64];
    ADS1115 adc(I2C_MASTER_NUM, ADS1115_ADDR); // Create an instance of the ADS1115 class for I2C communication

    while (1) {
        float v = adc.ReadVoltage(2); // Read voltage from channel 2 of the ADS1115
        float amp = adc.ReadCurrent(1); // Read current from channel 1 of the ADS1115 (if configured for current measurement)

        // Create message for the display
        snprintf(msg_to_queue, sizeof(msg_to_queue), "BAT: %.2f V\r\n AMP: %.2f A\r\n", v, amp);

        // Write to display queue
        if (xQueueDisplay != NULL) {
            xQueueSend(xQueueDisplay, &msg_to_queue, 0);
        }

        char uart_msg[64];
        snprintf(uart_msg, sizeof(uart_msg), "Auto-read: %.2fV\r\nAMP: %.2fA\r\n", v, amp);
        uart_write_bytes(UART_NUM, uart_msg, strlen(uart_msg));

        vTaskDelay(pdMS_TO_TICKS(10000)); // Delay
    }
}

/**
 * @brief Initializes the LEDC peripheral for controlling a servo motor connected to the specified GPIO pin.
 * 
 * This function performs the following steps:
 * 1. Configures the LEDC timer with a frequency of 50Hz, which is required for standard servo control, and sets the duty resolution to 14 bits (0-16383).
 * 2. Configures the LEDC channel to use the previously configured timer, assigns it to the specified GPIO pin for the servo signal, and initializes the duty cycle to 0 (no signal).
 * 3. Installs the LEDC fade function, which allows for smooth transitions when changing the servo angle without blocking the CPU.
 * 4. This function should be called during the initialization phase of the program before attempting to control the servo, as it sets up the necessary hardware configuration for generating the PWM signal required by the servo motor.
 * @note: The servo control signal is generated using the LEDC peripheral, which allows for precise control of the duty cycle to achieve the desired servo angle. The function assumes that the servo is connected to the specified GPIO pin and that the appropriate power supply is provided for the servo motor. Proper error handling should be implemented in a production environment to ensure that the LEDC peripheral is initialized successfully before attempting to control the servo.
 */
void init_servo() {
    // LEDC timer configuration
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num        = LEDC_TIMER_0;
    ledc_timer.duty_resolution  = LEDC_TIMER_14_BIT; // 14 bits resolution for duty cycle (0-16383)
    ledc_timer.freq_hz          = 50;  // 50Hz for standard servo control
    ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    // LEDC channel configuration
    ledc_channel_config_t ledc_channel = {};
    ledc_channel.speed_mode     = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel        = LEDC_CHANNEL_0;
    ledc_channel.timer_sel      = LEDC_TIMER_0;
    ledc_channel.intr_type      = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num       = SERVO_PIN;
    ledc_channel.duty           = 0;
    ledc_channel.hpoint         = 0;
    ledc_channel_config(&ledc_channel);

    // Fading service
    ledc_fade_func_install(0);
}

/**
 * @brief Sets the angle of the servo motor by calculating the appropriate duty cycle and updating the LEDC peripheral.
 * 
 * This function performs the following steps:
 * 1. Validates the input angle to ensure it is within the range of 0 to SERVO_MAX_DEGREE (180 degrees). If the angle is out of bounds, it is clamped to the nearest valid value.
 * 2. Uses a linear mapping (regla de tres) to convert the desired angle to the corresponding duty cycle in ticks, based on the defined minimum and maximum pulse widths for the servo.
 * 3. Updates the LEDC peripheral with the calculated duty cycle to set the servo to the desired angle. The ledc_set_duty function is called to set the duty cycle, and ledc_update_duty is called to apply the change to the hardware.
 * 4. This function allows for direct control of the servo angle by specifying the desired angle in degrees, and it handles the necessary calculations and hardware updates to achieve the correct position of the servo motor. Proper error handling should be implemented in a production environment to ensure that the input angle is valid and that the LEDC peripheral is updated successfully.
 * @param angle The desired angle for the servo motor, in degrees. Valid values are from 0 to SERVO_MAX_DEGREE (180 degrees). Values outside this range will be clamped to the nearest valid value.
 * @note: The servo motor should be connected to the specified GPIO pin and properly powered for the function to work correctly. The init_servo function should be called during initialization to set up the LEDC peripheral before using this function to control the servo angle.
 */
void set_servo_angle(int angle) {
    if (angle < 0) angle = 0;
    if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;

    // Map angle to duty cycle (ticks) using linear mapping
    uint32_t duty = SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * angle) / SERVO_MAX_DEGREE);
    
    // Hardware update
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

/**
 * @brief Sets the angle of the servo motor smoothly over a specified time duration by using the LEDC fade functionality.
 * 
 * This function performs the following steps:
 * 1. Validates the input angle to ensure it is within the range of 0 to SERVO_MAX_DEGREE (180 degrees). If the angle is out of bounds, it is clamped to the nearest valid value.
 * 2. Uses a linear mapping (regla de tres) to convert the desired angle to the corresponding duty cycle in ticks, based on the defined minimum and maximum pulse widths for the servo.
 * 3. Configures the LEDC fade functionality to transition from the current duty cycle to the target duty cycle over the specified time duration (time_ms). The ledc_set_fade_with_time function is called to set up the fade parameters, and ledc_fade_start is called to initiate the fade process.
 * 4. This function allows for smooth transitions of the servo angle by gradually changing the duty cycle over time, which can help reduce mechanical stress on the servo motor and provide a more visually appealing movement. Proper error handling should be implemented in a production environment to ensure that the input angle is valid and that the LEDC fade functionality is configured successfully.
 * @param angle The desired angle for the servo motor, in degrees. Valid values are from 0 to SERVO_MAX_DEGREE (180 degrees). Values outside this range will be clamped to the nearest valid value.
 * @param time_ms The duration of the fade transition in milliseconds. This specifies how long it should take for the servo to move from its current angle to the target angle. A value of 0 will result in an immediate change without fading.
 * @note: The servo motor should be connected to the specified GPIO pin and properly powered for the function to work correctly. The init_servo function should be called during initialization to set up the LEDC peripheral before using this function to control the servo angle with fading.
 */
void set_servo_angle_smooth(int angle, int time_ms) {
    if (angle < 0) angle = 0;
    if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;

    // Grade to duty cycle mapping
    uint32_t target_duty = SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * angle) / SERVO_MAX_DEGREE);
    
    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, target_duty, time_ms);
    
    // Start the fade (non-blocking)
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
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
  char command[BUF_SIZE];
  int cmd_idx = 0;
  
  I2CRelay relayController; 
  ADS1115 adsController(I2C_MASTER_NUM, ADS1115_ADDR);

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
    {"R1 ON",   [&]() { relayController.set_relay(1, true);  send_response("Encendiendo R1\r\n", "RELAY 1: ON"); }},
    {"R1 OFF",  [&]() { relayController.set_relay(1, false); send_response("Apagando R1\r\n", "RELAY 1: OFF"); }},
    {"R2 ON",   [&]() { relayController.set_relay(2, true);  send_response("Encendiendo R2\r\n", "RELAY 2: ON"); }},
    {"R2 OFF",  [&]() { relayController.set_relay(2, false); send_response("Apagando R2\r\n", "RELAY 2: OFF"); }},
    {"R3 ON",   [&]() { relayController.set_relay(3, true);  send_response("Encendiendo R3\r\n", "RELAY 3: ON"); }},
    {"R3 OFF",  [&]() { relayController.set_relay(3, false); send_response("Apagando R3\r\n", "RELAY 3: OFF"); }},
    {"R4 ON",   [&]() { relayController.set_relay(4, true);  send_response("Encendiendo R4\r\n", "RELAY 4: ON"); }},
    {"R4 OFF",  [&]() { relayController.set_relay(4, false); send_response("Apagando R4\r\n", "RELAY 4: OFF"); }},
    
    // Global relay commands
    {"ALL ON",  [&]() { relayController.set_all_relays_mask(0x0F); send_response("Encendiendo todos\r\n", "ALL: ON"); }},
    {"ALL OFF", [&]() { relayController.set_all_relays_mask(0x00); send_response("Apagando todos\r\n", "ALL: OFF"); }},
    
    // Sensor readings
    {"VOLTAGE A1", [&]() { 
      float v = adsController.ReadVoltage(1);
      send_response("Bat: " + std::to_string(v) + "V\r\n", "Bat: " + std::to_string(v) + "V"); 
    }},
    {"VOLTAGE A2", [&]() { 
      float v = adsController.ReadVoltage(2);
      send_response("V(A2): " + std::to_string(v) + "V\r\n", "V(A2): " + std::to_string(v) + "V"); 
    }},
    {"VOLTAGE A3", [&]() { 
      float v = adsController.ReadVoltage(3, true);
      send_response("V(A3): " + std::to_string(v) + "V\r\n", "V(A3): " + std::to_string(v) + "V"); 
    }},
    {"VOLTAGE PIN", [&]() { 
      float v = GetADCPinVoltage();
      send_response("Bat: " + std::to_string(v) + "V\r\n", "Bat: " + std::to_string(v) + "V"); 
    }},
    {"CURRENT", [&]() { 
      float ampers = adsController.ReadCurrent(2);
      send_response("Current: " + std::to_string(ampers) + "A\r\n", "Current: " + std::to_string(ampers) + "A"); 
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

              if (cmd_str.rfind("SERVO ", 0) == 0) { 
                int angle = std::stoi(cmd_str.substr(6)); 
                if (angle < 0 || angle > 180) {
                  uart_write_bytes(UART_NUM, "Error: Angle must be between 0 and 180\r\n", 40);
                  continue;
                }
                set_servo_angle(angle);
                send_response("Servo moved to " + std::to_string(angle) + " grades\r\n", "SERVO: " + std::to_string(angle) + " deg");
              } else if (cmd_str.rfind("SWEEP ", 0) == 0) {
                int target_angle = 0;
                int time_ms = 0;
                if (sscanf(cmd_str.c_str(), "SWEEP %d %d", &target_angle, &time_ms) == 2) {
                    set_servo_angle_smooth(target_angle, time_ms);
                    send_response("Servo moving to " + std::to_string(target_angle) + " in " + std::to_string(time_ms) + "ms\r\n", "SWEEP: " + std::to_string(target_angle));
                } else {
                    uart_write_bytes(UART_NUM, "Error. Use: SWEEP <angle> <time_ms>\r\n", 40);
                }
              } else {
                auto it = command_map.find(cmd_str);
                if (it != command_map.end()) {
                  it->second();
                } else {
                  uart_write_bytes(UART_NUM, "Command not recognized\r\n", 23);
                }
              }
              cmd_idx = 0; 
            }
            vTaskDelay(pdMS_TO_TICKS(1000)); 
            uart_write_bytes(UART_NUM, "> ", 2);
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
    ESP_LOGI(TAG, "I2C Relay inicializado");
  }
  
  // Scan I2C bus to verify connections (useful for debugging)
  ScanI2CBus();

  // Initialize servo control
  init_servo();

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

  // Display task
  xTaskCreate(ShowText, "TFT", 1024 * 6, NULL, 2, NULL);

  xTaskCreate(BatteryTask, "BAT_MONITOR", 1024 * 4, NULL, 1, NULL);

  // Command handling task (relays, voltage reading, etc.)
  xTaskCreate(UARTTask, "UART_COM", 1024 * 6, NULL, 1, NULL);
}

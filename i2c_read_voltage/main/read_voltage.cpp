#include <stdio.h>  // printf, snprintf
#include <stdlib.h> // atoi, atof
#include <string.h> // memset, strcmp, strcasecmp
#include <inttypes.h> // int16_t, uint8_t, etc.
#include <iostream> // std::cout, std::endl
#include <array>    // std::array
#include <iomanip>  // std::setprecision, std::fixed

#include "freertos/FreeRTOS.h" // FreeRTOS types and functions (e.g., vTaskDelay, xQueueSend)
#include "freertos/task.h"     // FreeRTOS task functions (e.g., xTaskCreate)
#include "esp_err.h" // esp_err_t type and error codes (e.g., ESP_OK, ESP_FAIL)
#include "esp_log.h" // ESP32 logging functions (e.g., ESP_LOGI, ESP_LOGE)
#include "esp_vfs.h" // ESP32 Virtual File System functions (e.g., esp_vfs_spiffs_register)
#include "esp_spiffs.h" // ESP32 SPIFFS functions (e.g., esp_spiffs_info)
#include "driver/gpio.h" // GPIO functions for controlling pins (e.g., gpio_set_level, gpio_set_direction)
#include "driver/uart.h" // UART functions for serial communication (e.g., uart_read_bytes, uart_write_bytes)
#include "driver/i2c.h" // I2C functions for communication with peripherals (e.g., i2c_master_write_to_device, i2c_master_write_read_device)
#include "esp_adc/adc_oneshot.h" // ESP32 ADC one-shot mode functions (e.g., adc_oneshot_unit_init_cfg_t, adc_oneshot_read)
#include "esp_adc/adc_cali.h" // ESP32 ADC calibration functions (e.g., adc_cali_characteristics_t, adc_cali_calibrate)
#include "esp_adc/adc_cali_scheme.h" // ESP32 ADC calibration schemes (e.g., ADC_CALI_SCHEME_CURVE_FITTING)

#include "sgm2578.h" // Display library for M5StickCPlus2
#include "st7789.h" // TFT display driver for ST7789-based displays (used in M5StickCPlus2)
#include "fontx.h" // Font rendering library for displaying text on the TFT screen
#include "bmpfile.h" // Bitmap file handling library (not used in this code but included for potential future use in displaying images on the TFT)
#include "I2Crelay.h" // Header file for relay control functions
#include "ADS1115.h" // Header file for ADS1115 ADC functions

// Configuración I2C (Puerto Grove M5StickCPlus2)
#define I2C_MASTER_SDA_IO         32        /*!<@brief I2C master SDA GPIO */
#define I2C_MASTER_SCL_IO         33        /*!<@brief I2C master SCL GPIO */
#define I2C_MASTER_NUM            I2C_NUM_0 /*!<@brief I2C master port number */
#define I2C_MASTER_FREQ_HZ        100000    /*!<@brief I2C master frequency in Hz */
#define I2C_MASTER_TIMEOUT_MS     1000      /*!<@brief I2C master timeout in ms */
#define RELAY_ADDR                0x26      /*!<@brief Relay I2C address */
#define REG_RELAY_CTRL            0x11      /*!<@brief Relay control register */

// ============ CONFIGURACIÓN ADS1115 ============
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

/** 
 * @brief Logger tag for the M5StickCPlus2 system 
 */
static const char *TAG = "M5_SYSTEM";

/** 
 * @brief Queue for sending messages to the display task
 */
static QueueHandle_t xQueueDisplay = NULL;

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
  #define POWER_HOLD_GPIO 4
  gpio_reset_pin((gpio_num_t)POWER_HOLD_GPIO);
  gpio_set_direction((gpio_num_t)POWER_HOLD_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)POWER_HOLD_GPIO, 1);
    
  #define SGM2578_ENABLE_GPIO 27
  sgm2578_Enable(SGM2578_ENABLE_GPIO);

  FontxFile fx16G[2];
  InitFontx(fx16G,"/fonts/ILGH16XB.FNT","");
    
  TFT_t dev;
  spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO, CONFIG_BL_GPIO);
  lcdInit(&dev, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSETX, CONFIG_OFFSETY, true);
  lcdFillScreen(&dev, BLACK);

  char texto_pantalla[64] = "Esperando..."; // Mensaje inicial

  while(1) {
    // Esperar a recibir un mensaje de la cola (espera infinita)
    if (xQueueReceive(xQueueDisplay, &texto_pantalla, portMAX_DELAY)) {
      lcdFillScreen(&dev, BLACK);
      lcdSetFontDirection(&dev, 1); // Rotación para M5StickC
            
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

/**
 * @brief Reads voltage from the ADS1115 ADC connected via I2C.
 * 
 * This function performs the following steps:
 * 1. Configures the ADS1115 to read from channel A0 with a gain of +/-6.144V and a data rate of 128SPS.
 * 2. Waits for the conversion to complete (at least 8ms for 128SPS).
 * 3. Reads the raw ADC value from the conversion register.
 * 4. Converts the raw value to a voltage using the ADS1115 resolution (0.1875mV per bit).
 * 5. Prints the raw ADC values and calculated voltage to the console for debugging purposes.
 * 
 * @return float - The calculated voltage in Volts. Returns -1.0 if there was an error during I2C communication.
 * @configuration:
 * - MUX bits for channel A1: 0xD0
 * - Gain: +/- 6.144V (0.1875mV per bit)
 * - Data rate: 128SPS (8ms conversion time)
 * - Onshot mode is used for single readings, so the configuration is sent 
 * before each read to ensure the correct channel and settings are applied. 
 * - Bit meaning for config bytes:
 *  - MSB (0xD0): 1(Start) 100(A1) 000(+/-6.144V) 0(Single-shot)
 * - LSB (0x83): 100(128 SPS) 0(Trad) 0(Alert Low) 11(Disable Comp) 
 */
float GetVoltageA1() {
    uint8_t config_data[3] = {
      ADS1115_REG_CONFIG, 
      0xD0,
      0x83  
    };

    // Configure the ADS1115 to read from channel A1 with the desired settings
    esp_err_t err_init = i2c_master_write_to_device(I2C_MASTER_NUM, ADS1115_ADDR, config_data, 3, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (err_init != ESP_OK) return -1.0;

    vTaskDelay(pdMS_TO_TICKS(20));  // Configuration delay

    // Read the conversion result from the ADS1115
    uint8_t reg = ADS1115_REG_CONV;
    uint8_t data[2] = {0, 0};
    esp_err_t err_read = i2c_master_write_read_device(I2C_MASTER_NUM, ADS1115_ADDR, &reg, 1, data, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    if (err_read != ESP_OK) return -1.0;

    // Convert the raw ADC value to a signed integer
    int16_t raw = (data[0] << 8) | data[1];

    // // --- BLOQUE DE DEBUG ---
    // std::cout << "--- Datos Crudos ---" << std::endl;
    // std::cout << "MSB (Byte 0): " << (int)data[0] << " (Hex: 0x" << std::hex << (int)data[0] << std::dec << ")" << std::endl;
    // std::cout << "LSB (Byte 1): " << (int)data[1] << " (Hex: 0x" << std::hex << (int)data[1] << std::dec << ")" << std::endl;
    std::cout << "Valor RAW: " << raw << std::endl;

    // Convert the raw ADC value to voltage (0.1875mV per bit for +/-6.144V range)
    float voltage = raw * 0.0001875f;
    return voltage;
}

float GetVoltageA2() {
    uint8_t config_data[3] = {
      ADS1115_REG_CONFIG, 
      0xE0, // Change later to read from A2 (MUX bits 110)
      0x83  
    };

    // Configure the ADS1115 to read from channel A2 with the desired settings
    esp_err_t err_init = i2c_master_write_to_device(I2C_MASTER_NUM, ADS1115_ADDR, config_data, 3, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (err_init != ESP_OK) return -1.0;

    vTaskDelay(pdMS_TO_TICKS(20));  // Configuration delay

    // Read the conversion result from the ADS1115
    uint8_t reg = ADS1115_REG_CONV;
    uint8_t data[2] = {0, 0};
    esp_err_t err_read = i2c_master_write_read_device(I2C_MASTER_NUM, ADS1115_ADDR, &reg, 1, data, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    if (err_read != ESP_OK) return -1.0;

    // Convert the raw ADC value to a signed integer
    int16_t raw = (data[0] << 8) | data[1];

    // // --- BLOQUE DE DEBUG ---
    // std::cout << "--- Datos Crudos ---" << std::endl;
    // std::cout << "MSB (Byte 0): " << (int)data[0] << " (Hex: 0x" << std::hex << (int)data[0] << std::dec << ")" << std::endl;
    // std::cout << "LSB (Byte 1): " << (int)data[1] << " (Hex: 0x" << std::hex << (int)data[1] << std::dec << ")" << std::endl;
    std::cout << "Valor RAW: " << raw << std::endl;

    // Convert the raw ADC value to voltage (0.1875mV per bit for +/-6.144V range)
    float voltage = raw * 0.0001875f;
    return voltage;
}

// Using Sensor medidor de Voltaje hasta 25V - FZ0430
float GetVoltageA3() {
    uint8_t config_data[3] = {
      ADS1115_REG_CONFIG, 
      0xF0, 
      0x83  
    };

    // Configure the ADS1115 to read from channel A3 with the desired settings
    esp_err_t err_init = i2c_master_write_to_device(I2C_MASTER_NUM, ADS1115_ADDR, config_data, 3, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (err_init != ESP_OK) return -1.0;

    vTaskDelay(pdMS_TO_TICKS(20));  // Configuration delay

    // Read the conversion result from the ADS1115
    uint8_t reg = ADS1115_REG_CONV;
    uint8_t data[2] = {0, 0};
    esp_err_t err_read = i2c_master_write_read_device(I2C_MASTER_NUM, ADS1115_ADDR, &reg, 1, data, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    if (err_read != ESP_OK) return -1.0;

    // Convert the raw ADC value to a signed integer
    int16_t raw = (data[0] << 8) | data[1];

    // // --- BLOQUE DE DEBUG ---
    // std::cout << "--- Datos Crudos ---" << std::endl;
    // std::cout << "MSB (Byte 0): " << (int)data[0] << " (Hex: 0x" << std::hex << (int)data[0] << std::dec << ")" << std::endl;
    // std::cout << "LSB (Byte 1): " << (int)data[1] << " (Hex: 0x" << std::hex << (int)data[1] << std::dec << ")" << std::endl;
    std::cout << "Valor RAW: " << raw << std::endl;

    // Convert the raw ADC value to voltage (0.1875mV per bit for +/-6.144V range)
    float v_pin = raw * 0.0001875f;
    
    // Multiplicamos por 5 para deshacer el divisor de tensión del FZ0430
    // Como tu lectura dio 0.995V en vez de 1.000V exactos, tu factor real 
    // es aproximadamente 5.025 (5V / 0.995V). Puedes usar 5.0f o el valor calibrado.
    float calibration_factor = 5.025f; 
    float real_battery_voltage = v_pin * calibration_factor;

    std::cout << "Voltaje en Pin S: " << v_pin << "V | Voltaje Real Batería: " << real_battery_voltage << "V" << std::endl;

    return real_battery_voltage;
}

/**
 * @brief Diagnosis function to verify correct operation of the ADS1115 and I2C communication.
 * 
 * This function performs the following steps:
 * 1. Reads the configuration register of the ADS1115 to verify if the chip accepted the settings (should be 0xC083 or 0x4083).
 * 2. Iterates through all 4 channels (A0 to A3) by configuring the MUX bits and reads the raw ADC value for each channel.
 * 3. Converts the raw ADC values to voltages and prints them to the console for verification.
 * 4. If the configuration read does not match the expected values, it prints an error message indicating a potential issue with the I2C connection or chip configuration.
 * 
 * @note: This function is intended for debugging purposes to ensure that the ADS1115 is properly configured and communicating over I2C before using it for actual sensor readings.
 * @configuration: 
 * - MUX bits for channels: A0=0xC0, A1=0xD0, A2=0xE0, A3=0xF0
 * - Gain: +/- 6.144V (0.1875mV per bit)
 * - Data rate: 128SPS (8ms conversion time)
 * - Mode: Single-shot
 * - Bit meaning for config bytes:
 * - MSB: 1(Start) MUX(100 for A0, 101 for A1, 110 for A2, 111 for A3) 000(+/-6.144V) 0(Single-shot)
 * - LSB: 100(128 SPS) 0(Trad) 0(Alert Low) 11(Disable Comp)
 */
// void DiagnosticADS1115() {
//     uint8_t config_reg = ADS1115_REG_CONFIG;
//     uint8_t data[2];

//     // Read configuration register to verify if the chip accepted our settings
//     i2c_master_write_read_device(I2C_MASTER_NUM, ADS1115_ADDR, &config_reg, 1, data, 2, pdMS_TO_TICKS(100));
//     uint16_t actual_config = (data[0] << 8) | data[1];
//     printf("--- Configuration Verification ---\n");
//     printf("Config in the chip: 0x%04X (Should be 0xC083)\n", actual_config);

//     if (actual_config != 0xC083 && actual_config != 0x4083) { // 0x4083 is when the OS bit returns to 0
//         printf("¡ERROR! The chip did not configure properly. Check SDA/SCL cables.\n");
//     }

//     // Channel MUX test: Read all 4 channels to verify correct readings
//     uint8_t mux_values[4] = {0xC0, 0xD0, 0xE0, 0xF0}; // MSB for A0, A1, A2, A3
    
//     for (int i = 0; i < 4; i++) {
//         uint8_t setup[3] = {ADS1115_REG_CONFIG, mux_values[i], 0x83};
//         i2c_master_write_to_device(I2C_MASTER_NUM, ADS1115_ADDR, setup, 3, pdMS_TO_TICKS(100));
//         vTaskDelay(pdMS_TO_TICKS(20));

//         uint8_t conv_reg = ADS1115_REG_CONV;
//         i2c_master_write_read_device(I2C_MASTER_NUM, ADS1115_ADDR, &conv_reg, 1, data, 2, pdMS_TO_TICKS(100));
        
//         int16_t raw = (data[0] << 8) | data[1];
//         printf("Channel A%d -> Raw: %d | Volt: %.3fV\n", i, raw, raw * 0.0001875f);
//     }
// }

/**
 * @brief Reads current from a sensor connected to the ADS1115 on channel A2.
 * 
 * This function performs the following steps:
 * 1. Configures the ADS1115 to read from channel A2 with a gain of +/-6.144V and a data rate of 128SPS.
 * 2. Waits for the conversion to complete (at least 8ms for 128SPS).
 * 3. Reads the raw ADC value from the conversion register.
 * 4. Converts the raw value to a voltage using the ADS1115 resolution (0.1875mV per bit).
 * 5. Applies a calibration formula to convert the sensor voltage to current (in Amperes), using a zero-point offset and sensitivity factor specific to the sensor being used.
 * 6. Implements a noise threshold to stabilize small readings around zero.
 * 7. Prints the sensor voltage and calculated current to the console for debugging purposes.
 * 
 * @return float - The calculated current in Amperes. Returns -1.0 if there was an error during I2C communication.
 * @configuration:
 * - MUX bits for channel A2: 0xE0
 * - Gain: +/- 6.144V (0.1875mV per bit)
 * - Data rate: 128SPS (8ms conversion time)
 * - Calibration: zero_point = 2.58394V, sensitivity = 0.033V/A (these values depend on the specific current sensor used and should be adjusted accordingly)
 * - Noise threshold: Readings between -0.001A and 0.001A are stabilized to 0A to reduce noise effects.
 * - Onshot mode is used for single readings, so the configuration is sent 
 * before each read to ensure the correct channel and settings are applied.
 * - Bit meaning for config bytes:
 * - MSB (0xE0): 1(Start) 111(A2) 000(+/-6.144V) 0(Single-shot)
 * - LSB (0x83): 100(128 SPS) 0(Trad) 0(Alert Low) 11(Disable Comp)
 */
float GetCurrentAmp() {
    uint8_t config_data[3] = { ADS1115_REG_CONFIG, 0xE0, 0x83 }; // Channel A2
    i2c_master_write_to_device(I2C_MASTER_NUM, ADS1115_ADDR, config_data, 3, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t reg = ADS1115_REG_CONV;
    uint8_t data[2];

    int16_t raw = 0;
  
    // Voltage in sensor output
    float v_sensor = 0.0; 
    for (int i = 0; i < 20; i++) {
      i2c_master_write_read_device(I2C_MASTER_NUM, ADS1115_ADDR, &reg, 1, data, 2, pdMS_TO_TICKS(100));
  
      raw = (data[0] << 8) | data[1];
  
      // Voltage in sensor output
      v_sensor += raw * 0.0001875f; 
    }

    v_sensor /= 20.0; // Average the readings

    // Calibration values (these depend on your specific sensor and setup, adjust accordingly)
    float zero_point = 2.5828f; 
    float sensitivity = 0.033f; // 33mV/A

    float amperage = (v_sensor - zero_point) / sensitivity;

    std::cout << "V_Sensor: " << v_sensor << "V | Amp: " << amperage << "A" << std::endl;

    return amperage;
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
    ESP_LOGI(TAG, "Tarea de batería iniciada (cada 10s)");

    while (1) {
        float v = GetADCPinVoltage();

        // Create message for the display
        snprintf(msg_to_queue, sizeof(msg_to_queue), "BAT: %.2f V", v);

        // Write to display queue
        if (xQueueDisplay != NULL) {
            xQueueSend(xQueueDisplay, &msg_to_queue, 0);
        }

        char uart_msg[64];
        snprintf(uart_msg, sizeof(uart_msg), "Auto-lectura: %.2fV\r\n", v);
        uart_write_bytes(UART_NUM, uart_msg, strlen(uart_msg));

        vTaskDelay(pdMS_TO_TICKS(10000)); // Delay
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
  char command[BUF_SIZE];
  int cmd_idx = 0;               // Índice actual de escritura en el buffer
  char msg_to_queue[64];
  I2CRelay relayController(RELAY_ADDR, I2C_MASTER_NUM);
  ADS1115 adsController(I2C_MASTER_NUM, ADS1115_ADDR);

  // Mostrar el prompt inicial por primera vez
  sleep(5); // Pequeña pausa para asegurar que el UART esté listo
  uart_write_bytes(UART_NUM, "> ", 2);

  while (1) {
    uint8_t rx_char;
    
    // Leemos 1 byte cada vez, esperando de forma indefinida hasta que llegue algo
    int len = uart_read_bytes(UART_NUM, &rx_char, 1, portMAX_DELAY);

    if (len > 0) {
      // 1. Detección de ENTER (Retorno de carro o salto de línea)
      if (rx_char == '\n' || rx_char == '\r') {
        
        // Hacemos un eco del salto de línea para que la terminal baje de renglón
        uart_write_bytes(UART_NUM, "\r\n", 2);

        // Solo procesamos si el usuario escribió algo
        if (cmd_idx > 0) {
          command[cmd_idx] = '\0'; // Terminador nulo de la cadena en C
          ESP_LOGI(TAG, "Comando recibido (%d bytes): %s", cmd_idx, command);

          // --- INICIO DE TU LÓGICA DE PROCESAMIENTO ---
          if (strcasecmp(command, "HELLO") == 0) {
            uart_write_bytes(UART_NUM, "Hola desde ESP32!\r\n", strlen("Hola desde ESP32!\r\n"));
          } else if (strcasecmp(command, "R1 ON") == 0) {
            uart_write_bytes(UART_NUM, "Encendiendo R1\r\n", strlen("Encendiendo R1\r\n"));
            relayController.set_relay(1, true);
            strcpy(msg_to_queue, "RELAY 1: ON");
            xQueueSend(xQueueDisplay, &msg_to_queue, 0);
          } else if (strcasecmp(command, "R1 OFF") == 0) {
            uart_write_bytes(UART_NUM, "Apagando R1\r\n", strlen("Apagando R1\r\n"));
            relayController.set_relay(1, false);
            strcpy(msg_to_queue, "RELAY 1: OFF");
            xQueueSend(xQueueDisplay, &msg_to_queue, 0);
          } else if (strcasecmp(command, "R2 ON") == 0) {
            uart_write_bytes(UART_NUM, "Encendiendo R2\r\n", strlen("Encendiendo R2\r\n"));
            relayController.set_relay(2, true);
            strcpy(msg_to_queue, "RELAY 2: ON");
            xQueueSend(xQueueDisplay, &msg_to_queue, 0);
          } else if (strcasecmp(command, "R2 OFF") == 0) {
            uart_write_bytes(UART_NUM, "Apagando R2\r\n", strlen("Apagando R2\r\n"));
            relayController.set_relay(2, false);
            strcpy(msg_to_queue, "RELAY 2: OFF");
            xQueueSend(xQueueDisplay, &msg_to_queue, 0);
          } else if (strcasecmp(command, "R3 ON") == 0) {
            uart_write_bytes(UART_NUM, "Encendiendo R3\r\n", strlen("Encendiendo R3\r\n"));
            relayController.set_relay(3, true);
            strcpy(msg_to_queue, "RELAY 3: ON");
            xQueueSend(xQueueDisplay, &msg_to_queue, 0);
          } else if (strcasecmp(command, "R3 OFF") == 0) {
            uart_write_bytes(UART_NUM, "Apagando R3\r\n", strlen("Apagando R3\r\n"));
            relayController.set_relay(3, false);
            strcpy(msg_to_queue, "RELAY 3: OFF");
            xQueueSend(xQueueDisplay, &msg_to_queue, 0);
          } else if (strcasecmp(command, "R4 ON") == 0) {
            uart_write_bytes(UART_NUM, "Encendiendo R4\r\n", strlen("Encendiendo R4\r\n"));
            relayController.set_relay(4, true);
            strcpy(msg_to_queue, "RELAY 4: ON");
            xQueueSend(xQueueDisplay, &msg_to_queue, 0);
          } else if (strcasecmp(command, "R4 OFF") == 0) {
            uart_write_bytes(UART_NUM, "Apagando R4\r\n", strlen("Apagando R4\r\n"));
            relayController.set_relay(4, false);
            strcpy(msg_to_queue, "RELAY 4: OFF");
            xQueueSend(xQueueDisplay, &msg_to_queue, 0);
          } else if (strcasecmp(command, "ALL ON") == 0) {
            uart_write_bytes(UART_NUM, "Encendiendo todos los relés\r\n", strlen("Encendiendo todos los relés\r\n"));
            relayController.set_all_relays_mask(0x0F);
            strcpy(msg_to_queue, "ALL: ON");
            xQueueSend(xQueueDisplay, &msg_to_queue, 0);
          } else if (strcasecmp(command, "ALL OFF") == 0) {
            uart_write_bytes(UART_NUM, "Apagando todos los relés\r\n", strlen("Apagando todos los relés\r\n"));
            relayController.set_all_relays_mask(0x00);
            strcpy(msg_to_queue, "ALL: OFF");
            xQueueSend(xQueueDisplay, &msg_to_queue, 0);
          } else if (strcasecmp(command, "VOLTAGE A1") == 0) {
            float v = adsController.ReadVoltage(1);
            std::string msg = "Bat: " + std::to_string(v) + "V\r\n";
            uart_write_bytes(UART_NUM, msg.c_str(), strlen(msg.c_str()));
            strcpy(msg_to_queue, msg.c_str());
            xQueueSend(xQueueDisplay, &msg_to_queue, 0);
          } else if (strcasecmp(command, "VOLTAGE A2") == 0) {
            float v = adsController.ReadVoltage(2);
            std::string msg = "V(A2): " + std::to_string(v) + "V\r\n";
            uart_write_bytes(UART_NUM, msg.c_str(), strlen(msg.c_str()));
            strcpy(msg_to_queue, msg.c_str());
            xQueueSend(xQueueDisplay, &msg_to_queue, 0);
          } else if (strcasecmp(command, "VOLTAGE A3") == 0) {
            float v = adsController.ReadVoltage(3, true);
            std::string msg = "V(A3): " + std::to_string(v) + "V\r\n";
            uart_write_bytes(UART_NUM, msg.c_str(), strlen(msg.c_str()));
            strcpy(msg_to_queue, msg.c_str());
            xQueueSend(xQueueDisplay, &msg_to_queue, 0);
          } else if (strcasecmp(command, "VOLTAGE PIN") == 0) {
            float v = GetADCPinVoltage();
            std::string msg = "Bat: " + std::to_string(v) + "V\r\n";
            uart_write_bytes(UART_NUM, msg.c_str(), strlen(msg.c_str()));
            strcpy(msg_to_queue, msg.c_str());
            xQueueSend(xQueueDisplay, &msg_to_queue, 0);
          } else if (strcasecmp(command, "CURRENT") == 0) {
            float ampers = GetCurrentAmp();
            std::string msg = "Current: " + std::to_string(ampers) + "A\r\n";
            uart_write_bytes(UART_NUM, msg.c_str(), strlen(msg.c_str()));
            strcpy(msg_to_queue, msg.c_str());
            xQueueSend(xQueueDisplay, &msg_to_queue, 0);
          } else {
            uart_write_bytes(UART_NUM, "Comando no reconocido\r\n", strlen("Comando no reconocido\r\n"));
          }
          // --- FIN DE TU LÓGICA DE PROCESAMIENTO ---

          // Reiniciamos el índice para el próximo comando
          cmd_idx = 0; 
        }

        // Una vez procesado, volvemos a imprimir el indicador visual para el usuario
        sleep(1); // Pequeña pausa para evitar que el prompt se mezcle con la respuesta
        uart_write_bytes(UART_NUM, "> ", 2);

      } 
      // 2. Detección de la tecla Retroceso (Backspace / Delete)
      else if (rx_char == '\b' || rx_char == 127) { 
        if (cmd_idx > 0) {
          cmd_idx--; // Eliminamos el carácter de nuestro buffer
          // Enviamos a la terminal: retroceso (\b), espacio vacio para limpiar visualmente, y otro retroceso
          uart_write_bytes(UART_NUM, "\b \b", 3); 
        }
      } 
      // 3. Guardado normal de cualquier otro carácter
      else {
        // Hacemos "eco" del carácter enviado para que el usuario pueda ver lo que está tecleando
        uart_write_bytes(UART_NUM, &rx_char, 1);

        // Guardamos en nuestro buffer, cuidando de no exceder el tamaño máximo
        if (cmd_idx < BUF_SIZE - 1) {
          command[cmd_idx++] = (char)rx_char;
        }
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

  // Configure UART
  uart_config_t cfg;
  memset(&cfg, 0, sizeof(uart_config_t));
  cfg.baud_rate = 115200;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

  uart_param_config(UART_NUM, &cfg);
  uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);

  // Message queue for display updates
  xQueueDisplay = xQueueCreate(5, 64);

  // Display task
  xTaskCreate(ShowText, "TFT", 1024 * 6, NULL, 2, NULL);

  //xTaskCreate(BatteryTask, "BAT_MONITOR", 1024 * 4, NULL, 1, NULL);

  // Command handling task (relays, voltage reading, etc.)
  xTaskCreate(UARTTask, "UART_COM", 1024 * 6, NULL, 1, NULL);
}

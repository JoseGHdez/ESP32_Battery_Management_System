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
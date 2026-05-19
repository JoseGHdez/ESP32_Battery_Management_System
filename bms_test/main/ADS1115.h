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

// ============ TASK CONFIGURATION ============
#define INTERVAL 400 /*!<@brief Delay interval for the task */
#define WAIT vTaskDelay(INTERVAL) /*!<@brief Delay code for the task */

class ADS1115 {
 public:
  ADS1115(i2c_port_t i2c_port = I2C_MASTER_NUM, uint8_t ads1115_addr = ADS1115_ADDR, 
          uint8_t ads1115_reg_config = ADS1115_REG_CONFIG, uint8_t ads1115_reg_conv = ADS1115_REG_CONV,
          int timeout_ms = I2C_MASTER_TIMEOUT_MS);
  float ReadVoltage(uint8_t channel, bool fz0430_flag = false);
  float CalibrateVoltage(float known_voltage, float factor);
  float ReadCurrent(uint8_t channel);
  void Diagnostic();

 private:
  i2c_port_t i2c_port_;
  int timeout_ms_;
  uint8_t ads1115_addr_;
  uint8_t ads1115_reg_config_;
  uint8_t ads1115_reg_conv_;
};

ADS1115::ADS1115(i2c_port_t port, uint8_t ads1115_addr, uint8_t ads1115_reg_config, uint8_t ads1115_reg_conv, int timeout_ms) {
  i2c_port_ = port;
  ads1115_addr_ = ads1115_addr;
  ads1115_reg_config_ = ads1115_reg_config;
  ads1115_reg_conv_ = ads1115_reg_conv;
  timeout_ms_ = timeout_ms;
}

float ADS1115::ReadVoltage(uint8_t channel, bool fz0430_flag) {
  if (channel > 3) return -1.0; // Invalid channel

  uint8_t mux_bits = 0x40 | (channel << 4); // MUX bits for single-ended input on the specified channel
  uint8_t config_data[3] = {
    ads1115_reg_config_, 
    mux_bits,
    0x83  // Gain: +/- 6.144V, Data rate: 128SPS, Single-shot mode
  };
  
  // Configure the ADS1115 to read from the specified channel with the desired settings
  esp_err_t err_init = i2c_master_write_to_device(i2c_port_, ads1115_addr_, config_data, 3, pdMS_TO_TICKS(timeout_ms_));
  if (err_init != ESP_OK) return -1.0;

  vTaskDelay(pdMS_TO_TICKS(20));  // Configuration delay

  // Read the conversion result from the ADS1115
  uint8_t reg = ads1115_reg_conv_;
  uint8_t data[2] = {0, 0};
  esp_err_t err_read = i2c_master_write_read_device(i2c_port_, ads1115_addr_, &reg, 1, data, 2, pdMS_TO_TICKS(timeout_ms_));
  if (err_read != ESP_OK) return -1.0;

  // Convert the raw ADC value to a signed integer
  int16_t raw = (data[0] << 8) | data[1];

  // Convert the raw ADC value to voltage (0.1875mV per bit for +/-6.144V range)
  float voltage = raw * 0.0001875f;

  return fz0430_flag ? CalibrateVoltage(voltage, 5.025f) : voltage;
}

float ADS1115::CalibrateVoltage(float known_voltage, float factor) {
  return known_voltage * factor; // Simple calibration using a known voltage and a factor
}

float ADS1115::ReadCurrent(uint8_t channel) {
  float voltage = ReadVoltage(channel);
  if (voltage < 0) return -1.0; // Error reading voltage

  // Example calibration for a current sensor with sensitivity of 0.033V/A
  float zero_point = 2.5478f; // Voltage at 0A (this should be determined experimentally for your specific sensor)
  voltage -= zero_point; // Remove zero-point offset

  float current = voltage / 0.033f; 
  return current;
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
void ADS1115::Diagnostic() {
  uint8_t config_reg = ads1115_reg_config_;
  uint8_t data[2];

  // Read configuration register to verify if the chip accepted our settings
  i2c_master_write_read_device(i2c_port_, ads1115_addr_, &config_reg, 1, data, 2, pdMS_TO_TICKS(100));
  uint16_t actual_config = (data[0] << 8) | data[1];
  printf("--- Configuration Verification ---\n");
  printf("Config in the chip: 0x%04X (Should be 0xC083)\n", actual_config);

  if (actual_config != 0xC083 && actual_config != 0x4083) { // 0x4083 is when the OS bit returns to 0
    printf("¡ERROR! The chip did not configure properly. Check SDA/SCL cables.\n");
  }

  // Channel MUX test: Read all 4 channels to verify correct readings
  uint8_t mux_values[4] = {0xC0, 0xD0, 0xE0, 0xF0}; // MSB for A0, A1, A2, A3
    
  for (int i = 0; i < 4; i++) {
    uint8_t setup[3] = {ads1115_reg_config_, mux_values[i], 0x83};
    i2c_master_write_to_device(i2c_port_, ads1115_addr_, setup, 3, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t conv_reg = ads1115_reg_conv_;
    i2c_master_write_read_device(i2c_port_, ads1115_addr_, &conv_reg, 1, data, 2, pdMS_TO_TICKS(100));
        
    int16_t raw = (data[0] << 8) | data[1];
    printf("Channel A%d -> Raw: %d | Volt: %.3fV\n", i, raw, raw * 0.0001875f);
 }
}
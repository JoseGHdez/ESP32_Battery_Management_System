#ifndef ADS1115_H
#define ADS1115_H

#include <stdio.h>  // printf, snprintf
#include <stdlib.h> // atoi, atof
#include <string.h> // memset, strcmp, strcasecmp
#include <inttypes.h> // int16_t, uint8_t, etc.
#include <iostream> // std::cout, std::endl
#include <array>    // std::array
#include <iomanip>  // std::setprecision, std::fixed
#include <utility>      // std::pair for returning multiple values from functions

#include "freertos/FreeRTOS.h" // FreeRTOS types and functions (e.g., vTaskDelay, xQueueSend)
#include "freertos/task.h"     // FreeRTOS task functions (e.g., xTaskCreate)
#include "esp_err.h" // esp_err_t type and error codes (e.g., ESP_OK, ESP_FAIL)
#include "esp_log.h" // ESP32 logging functions (e.g., ESP_LOGI, ESP_LOGE)
#include "driver/uart.h" // UART functions for serial communication (e.g., uart_read_bytes, uart_write_bytes)
#include "driver/i2c.h" // I2C functions for communication with peripherals (e.g., i2c_master_write_to_device, i2c_master_write_read_device)

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

/**
  @brief Mutex for I2C bus access 
*/
SemaphoreHandle_t i2c_bus_mutex = xSemaphoreCreateMutex();      

/**
  @brief Mutex for ADS1115 chip access (to prevent concurrent configuration 
  and reading) 
*/
SemaphoreHandle_t ads1115_chip_mutex = xSemaphoreCreateMutex(); 

/**
  @brief Function to reset the I2C bus in case of a bus error 
  (e.g., no ACK from device). This can help recover from transient 
  issues without needing to reset the entire ESP32.
*/
void reset_i2c_bus() {
  ESP_LOGW("I2C_RECOVERY", "Reiniciando el controlador I2C por error de bus...");
   
  // Uninstall previous driver
  i2c_driver_delete(I2C_NUM_0); 
    
  vTaskDelay(pdMS_TO_TICKS(50)); 
    
  i2c_config_t conf = {}; 
    
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = GPIO_NUM_32; 
  conf.scl_io_num = GPIO_NUM_33;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 50000; 
  conf.clk_flags = 0;
    
  if (i2c_param_config(I2C_NUM_0, &conf) != ESP_OK) {
    ESP_LOGE("I2C_RECOVERY", "Error al configurar parámetros I2C");
  }
  if (i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0) != ESP_OK) {
    ESP_LOGE("I2C_RECOVERY", "Error al instalar el driver I2C");
  }
    
  i2c_set_timeout(I2C_NUM_0, 20000); // 20 ms timeout for I2C operations
}

/**
  @class ADS1115
  @brief Driver class for the ADS1115 ADC. Provides methods to configure the 
  ADC, read voltage and current values, and perform diagnostics. 
*/
class ADS1115 {
 public:
  // Constructor
  ADS1115(i2c_port_t i2c_port = I2C_MASTER_NUM, uint8_t ads1115_addr = ADS1115_ADDR, 
          uint8_t ads1115_reg_config = ADS1115_REG_CONFIG, uint8_t ads1115_reg_conv = ADS1115_REG_CONV,
          int timeout_ms = I2C_MASTER_TIMEOUT_MS);

  // Methods
  std::pair<float, esp_err_t> ReadVoltage(uint8_t channel, bool fz0430_flag = false);
  float CalibrateVoltage(float known_voltage, float factor);
  std::pair<float, esp_err_t> ReadCurrent(uint8_t channel, float bias = 0.0f);
  void Diagnostic();
  std::pair<float, esp_err_t> ConfigureADS1115(uint8_t channel);

 private:
  // Attributes
  i2c_port_t i2c_port_;
  int timeout_ms_;
  uint8_t ads1115_addr_;
  uint8_t ads1115_reg_config_;
  uint8_t ads1115_reg_conv_;
  float point_zero_voltage_ = 2.43642; ////2.44752f;//2.54762f; // Voltage at 0A for current sensor calibration (this should be determined experimentally for your specific sensor)
};

/**
  @brief Constructor for the ADS1115 class.
*/
ADS1115::ADS1115(i2c_port_t port, uint8_t ads1115_addr, uint8_t ads1115_reg_config, uint8_t ads1115_reg_conv, int timeout_ms) {
  i2c_port_ = port;
  ads1115_addr_ = ads1115_addr;
  ads1115_reg_config_ = ads1115_reg_config;
  ads1115_reg_conv_ = ads1115_reg_conv;
  timeout_ms_ = timeout_ms;
}

/**
  @brief Configures the ADS1115 ADC for reading from a specific channel.

  This function sets up the ADS1115 to read from the specified channel (0-3) 
  by writing the appropriate configuration to the ADC. It uses a mutex to 
  ensure that only one task can configure or read from the ADC at a time. 
  If the I2C communication fails, it will attempt to reset the I2C bus and 
  retry a few times before giving up.

  @param channel The channel to configure (0-3).
  @return A pair containing the result and an error code.
*/
std::pair<float, esp_err_t> ADS1115::ConfigureADS1115(uint8_t channel) {
  if (xSemaphoreTake(ads1115_chip_mutex, portMAX_DELAY) != pdTRUE) return std::make_pair(-1.0, ESP_FAIL);

  uint8_t mux_bits = 0xC0 | (channel << 4);
  uint8_t config_data[3] = { ads1115_reg_config_, mux_bits, 0x83 };
  
  if (xSemaphoreTake(i2c_bus_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    esp_err_t err_init = i2c_master_write_to_device(i2c_port_, ads1115_addr_, config_data, 3, pdMS_TO_TICKS(timeout_ms_));
    xSemaphoreGive(i2c_bus_mutex);
    
    uint8_t counter = 0;
    while (counter < 5 && err_init != ESP_OK) {
      ESP_LOGW("ADS1115", "Config write attempt %d failed (ch%d): %s", counter + 1, channel, esp_err_to_name(err_init));
      vTaskDelay(pdMS_TO_TICKS(5));
      reset_i2c_bus(); // Reset I2C bus
      vTaskDelay(pdMS_TO_TICKS(5)); 
      if (xSemaphoreTake(i2c_bus_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        err_init = i2c_master_write_to_device(i2c_port_, ads1115_addr_, config_data, 3, pdMS_TO_TICKS(timeout_ms_));
        xSemaphoreGive(i2c_bus_mutex);
      }
      counter++;
    }
    
    if (err_init != ESP_OK) {
      ESP_LOGE("ADS1115", "Config write failed (ch%d): %s", channel, esp_err_to_name(err_init));
      xSemaphoreGive(ads1115_chip_mutex);
      return std::make_pair(-1.0, err_init);
    }

  } else {
    xSemaphoreGive(ads1115_chip_mutex);
    return std::make_pair(-1.0, ESP_FAIL);
  }

  xSemaphoreGive(ads1115_chip_mutex); 
  return std::make_pair(0.0, ESP_OK); 
}

/**
  @brief Reads the voltage from a specified channel of the ADS1115.

  This function reads the raw ADC value from the specified channel, converts it 
  to a voltage, and returns it. It uses mutexes to ensure safe access to the 
  ADC and I2C bus. If there are communication errors, it will attempt to reset 
  the I2C bus and retry a few times before giving up.

  @param channel The channel to read from (0-3).
  @param fz0430_flag If true, applies a calibration factor for FZ0430 cells.
  @return A pair containing the voltage and an error code.
*/
std::pair<float, esp_err_t> ADS1115::ReadVoltage(uint8_t channel, bool fz0430_flag) {
  if (channel > 3) return std::make_pair(-1.0, ESP_FAIL);

  if (xSemaphoreTake(ads1115_chip_mutex, portMAX_DELAY) != pdTRUE) return std::make_pair(-1.0, ESP_FAIL);

  uint8_t reg = ads1115_reg_conv_;
  uint8_t data[2] = {0, 0};
  esp_err_t err_read = ESP_FAIL;

  if (xSemaphoreTake(i2c_bus_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    err_read = i2c_master_write_read_device(i2c_port_, ads1115_addr_, &reg, 1, data, 2, pdMS_TO_TICKS(timeout_ms_));
    xSemaphoreGive(i2c_bus_mutex);
  } else {
    xSemaphoreGive(ads1115_chip_mutex);
    return std::make_pair(-1.0, ESP_FAIL);
  }
  
  // Retry mechanism
  uint8_t count = 0;
  while (count < 5 && err_read != ESP_OK) {
    ESP_LOGW("ADS1115", "Read attempt %d failed (ch%d): %s", count + 1, channel, esp_err_to_name(err_read));
    vTaskDelay(pdMS_TO_TICKS(5));
    reset_i2c_bus(); 
    vTaskDelay(pdMS_TO_TICKS(5));
    
    if (xSemaphoreTake(i2c_bus_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      err_read = i2c_master_write_read_device(i2c_port_, ads1115_addr_, &reg, 1, data, 2, pdMS_TO_TICKS(timeout_ms_));
      xSemaphoreGive(i2c_bus_mutex);
    }
    count++;
  }

  xSemaphoreGive(ads1115_chip_mutex);

  if (err_read != ESP_OK) {
    ESP_LOGE("ADS1115", "Conversion read failed (ch%d): %s", channel, esp_err_to_name(err_read));
    return std::make_pair(-1.0, err_read);
  }

  int16_t raw = (int16_t)((data[0] << 8) | data[1]);
  float voltage = raw * 0.00018754579f;

  return fz0430_flag ? std::make_pair(CalibrateVoltage(voltage, 5.025f), ESP_OK) : std::make_pair(voltage, ESP_OK);
}

/**
  @brief Calibrates a voltage reading using a known reference voltage and a 
  calibration factor.

  @param known_voltage A known reference voltage that was measured
  @param factor A calibration factor
  @return The calibrated voltage value.
*/
float ADS1115::CalibrateVoltage(float known_voltage, float factor) {
  return known_voltage * factor; // Simple calibration using a known voltage and a factor
}

/**
  @brief Reads the current from a specified channel of the ADS1115.
  @param channel The channel to read from (0-3).
  @param bias The bias voltage to subtract from the reading.
  @return A pair containing the current and an error code.
*/
std::pair<float, esp_err_t> ADS1115::ReadCurrent(uint8_t channel, float bias) {
  auto voltage_result = ReadVoltage(channel);
  float voltage = voltage_result.first;
  esp_err_t err = voltage_result.second;

  if (err != ESP_OK) {
    return std::make_pair(-1.0, err);
  }

  ESP_LOGV("ADS1115", "Voltage for current reading: %.3f V", voltage);

  // Example calibration for a current sensor with sensitivity of 0.033V/A
  voltage -= bias;
  voltage -= point_zero_voltage_; // Remove zero-point offset

  float current = voltage / 0.033f; 
  return std::make_pair(current, ESP_OK);
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

#endif // ADS1115_H
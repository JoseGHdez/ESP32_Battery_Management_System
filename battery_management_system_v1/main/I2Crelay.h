#ifndef I2CRELAY_H
#define I2CRELAY_H

#include <stdio.h>  // printf, snprintf
#include <string.h> // memset, strcmp, strcasecmp
#include <inttypes.h> // int16_t, uint8_t, etc.
#include <iostream> // std::cout, std::endl

#include "freertos/FreeRTOS.h" // FreeRTOS types and functions (e.g., vTaskDelay, xQueueSend)
#include "freertos/task.h"     // FreeRTOS task functions (e.g., xTaskCreate)
#include "esp_err.h" // esp_err_t type and error codes (e.g., ESP_OK, ESP_FAIL)
#include "esp_log.h" // ESP32 logging functions (e.g., ESP_LOGI, ESP_LOGE)
#include "driver/i2c.h" // I2C functions for communication with peripherals (e.g., i2c_master_write_to_device, i2c_master_write_read_device)

// I2C relay control parameters
#define I2C_MASTER_SDA_IO         32        /*!<@brief I2C master SDA GPIO */
#define I2C_MASTER_SCL_IO         33        /*!<@brief I2C master SCL GPIO */
#define I2C_MASTER_NUM            I2C_NUM_0 /*!<@brief I2C master port number */
#define I2C_MASTER_FREQ_HZ        50000    /*!<@brief I2C master frequency in Hz */
#define I2C_MASTER_TIMEOUT_MS     5000      /*!<@brief I2C master timeout in ms */
#define RELAY_ADDR                0x26      /*!<@brief Relay I2C address */
#define REG_RELAY_CTRL            0x11      /*!<@brief Relay control register */

// ============ TASK CONFIGURATION ============
#define INTERVAL 400 /*!<@brief Delay interval for the task */
#define WAIT vTaskDelay(INTERVAL) /*!<@brief Delay code for the task */

/** 
 * @brief Logger tag for the M5StickCPlus2 system 
 */
static const char *RELAYTAG = "I2C_RELAY";

class I2CRelay {
 public:
  I2CRelay(int relay_address = RELAY_ADDR, uint8_t register_address = REG_RELAY_CTRL, i2c_port_t i2c_port = I2C_MASTER_NUM, 
    int timeout_ms = I2C_MASTER_TIMEOUT_MS);
  void set_relay(int relay_num, bool state);
  esp_err_t set_all_relays_mask(uint8_t mask4bits);
  uint8_t get_relay_state() const { return relay_state_; }
  
 private:
  int relay_address_;
  uint8_t register_address_;
  i2c_port_t i2c_port_;
  uint8_t relay_state_;
  int timeout_ms_;
};

inline I2CRelay::I2CRelay(int addr, uint8_t reg_addr, i2c_port_t port, int timeout_ms) {
  relay_state_ = 0x00; // Initialize relay state to all OFF
  relay_address_ = addr;
  register_address_ = reg_addr;
  timeout_ms_ = timeout_ms;
  i2c_port_ = port;
}

/**
 * @brief Sets the state of a specific relay (1 to 4) to ON or OFF.
 * 
 * This function takes a relay number (1 to 4) and a boolean state (true for ON, false for OFF). It updates the global relay_state variable by setting or clearing the corresponding bit for the specified relay. Then, it sends the updated relay state to the relay controller via I2C by writing two bytes: the first byte is the register address (REG_RELAY_CTRL) and the second byte is the updated relay state. The function checks for errors during I2C communication and logs an error message if it fails, or a success message if the device is found and communication is successful.
 * @param relay_num An integer representing the relay number (1 to 4) to be controlled.
 * @param state A boolean value where true sets the relay to ON and false sets it to OFF.
 * @return void - This function does not return a value, but it updates the relay state and communicates with the relay controller over I2C. It also logs the result of the I2C communication for debugging purposes.
 */
inline void I2CRelay::set_relay(int relay_num, bool state) {
  if (relay_num < 1 || relay_num > 4) return;
  if (state) {
    relay_state_ |= (1 << (relay_num - 1));
  } else {
    relay_state_ &= ~(1 << (relay_num - 1));
  }

  uint8_t data[2] = {register_address_, relay_state_};
  esp_err_t err = i2c_master_write_to_device(i2c_port_, relay_address_, data, 2, pdMS_TO_TICKS(timeout_ms_));
  if (err != ESP_OK) {
    ESP_LOGE(RELAYTAG, "Error I2C (esperado): %s", esp_err_to_name(err));
  } else {
    ESP_LOGI(RELAYTAG, "I2C OK (dispositivo encontrado)");
  }
}

/**
 * @brief Sets the state of all 4 relays at once using a 4-bit mask.
 * 
 * This function takes a 4-bit mask where each bit represents the state of a relay (1 for ON, 0 for OFF). The function updates the global relay_state variable accordingly and sends the new state to the relay controller via I2C. The I2C communication is performed by writing two bytes: the first byte is the register address (REG_RELAY_CTRL) and the second byte is the updated relay state. The function returns an esp_err_t value indicating the success or failure of the I2C communication.
 * @param mask4bits A 4-bit value where each bit represents the desired state of a relay (bit 0 for relay 1, bit 1 for relay 2, bit 2 for relay 3, bit 3 for relay 4).
 * @return esp_err_t - Returns ESP_OK if the I2C communication was successful, or an error code if it failed.
 */
inline esp_err_t I2CRelay::set_all_relays_mask(uint8_t mask4bits) {
    relay_state_ = (mask4bits & 0x0F);
    uint8_t data[2] = {register_address_, relay_state_};
    return i2c_master_write_to_device(i2c_port_, relay_address_, data, 2, pdMS_TO_TICKS(timeout_ms_));
}

#endif
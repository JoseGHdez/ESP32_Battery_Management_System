#ifndef I2CRELAY_H
#define I2CRELAY_H

#include <stdio.h>  // printf, snprintf
#include <string.h> // memset, strcmp, strcasecmp
#include <inttypes.h> // int16_t, uint8_t, etc.
#include <iostream> // std::cout, std::endl

#include "freertos/FreeRTOS.h" // FreeRTOS types and functions
#include "freertos/task.h"     // FreeRTOS task functions
#include "esp_err.h" // esp_err_t type and error codes
#include "esp_log.h" // ESP32 logging functions

// NUEVO DRIVER I2C DE ESP-IDF v5
#include "driver/i2c_master.h" 

// I2C relay control parameters
#define I2C_MASTER_FREQ_HZ        50000    /*!<@brief I2C master frequency in Hz (50kHz para mayor estabilidad) */
#define I2C_MASTER_TIMEOUT_MS     50       /*!<@brief I2C master timeout in ms (No usar valores muy altos para evitar bloqueos) */
#define RELAY_ADDR                0x26      /*!<@brief Relay I2C address */
#define REG_RELAY_CTRL            0x11      /*!<@brief Relay control register */

// ============ TASK CONFIGURATION ============
#define INTERVAL 400 /*!<@brief Delay interval for the task */
#define WAIT vTaskDelay(INTERVAL) /*!<@brief Delay code for the task */

/** * @brief Logger tag for the M5StickCPlus2 system 
 */
static const char *RELAYTAG = "I2C_RELAY";

class I2CRelay {
 public:
  // Fíjate que ahora pasamos el i2c_master_bus_handle_t creado previamente
  I2CRelay(i2c_master_bus_handle_t bus_handle, int relay_address = RELAY_ADDR, 
           uint8_t register_address = REG_RELAY_CTRL, int timeout_ms = I2C_MASTER_TIMEOUT_MS);
  void set_relay(int relay_num, bool state);
  esp_err_t set_all_relays_mask(uint8_t mask4bits);
  uint8_t get_relay_state() const { return relay_state_; }
  
 private:
  int relay_address_;
  uint8_t register_address_;
  i2c_master_dev_handle_t dev_handle_; // <--- NUEVO MANEJADOR ESPECÍFICO DEL RELÉ
  uint8_t relay_state_;
  int timeout_ms_;
};

inline I2CRelay::I2CRelay(i2c_master_bus_handle_t bus_handle, int addr, uint8_t reg_addr, int timeout_ms) {
  relay_state_ = 0x00; // Initialize relay state to all OFF
  relay_address_ = addr;
  register_address_ = reg_addr;
  timeout_ms_ = timeout_ms;
  
  // Configuración de este dispositivo en concreto
  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = (uint16_t)relay_address_,
      .scl_speed_hz = I2C_MASTER_FREQ_HZ,
      .scl_wait_us = 0,
      .flags = {
          .disable_ack_check = false,
      }
  };

  // Añadimos el dispositivo al bus que nos han pasado y guardamos su "handle"
  esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle_);
  if (err != ESP_OK) {
      ESP_LOGE(RELAYTAG, "Fallo al registrar el relé en el bus I2C: %s", esp_err_to_name(err));
  } else {
      ESP_LOGI(RELAYTAG, "Relé registrado correctamente en el bus I2C");
  }
}

/**
 * @brief Sets the state of a specific relay (1 to 4) to ON or OFF.
 */
inline void I2CRelay::set_relay(int relay_num, bool state) {
  if (relay_num < 1 || relay_num > 4) return;
  if (state) {
    relay_state_ |= (1 << (relay_num - 1));
  } else {
    relay_state_ &= ~(1 << (relay_num - 1));
  }

  uint8_t data[2] = {register_address_, relay_state_};
  
  // NUEVA FUNCIÓN DE TRANSMISIÓN. No necesita pdMS_TO_TICKS
  esp_err_t err = i2c_master_transmit(dev_handle_, data, 2, timeout_ms_);
  
  if (err != ESP_OK) {
    ESP_LOGE(RELAYTAG, "Error I2C al conmutar relé %d: %s", relay_num, esp_err_to_name(err));
  } else {
    ESP_LOGI(RELAYTAG, "I2C OK: Relé %d -> %s", relay_num, state ? "ON" : "OFF");
  }
}

/**
 * @brief Sets the state of all 4 relays at once using a 4-bit mask.
 */
inline esp_err_t I2CRelay::set_all_relays_mask(uint8_t mask4bits) {
    relay_state_ = (mask4bits & 0x0F);
    uint8_t data[2] = {register_address_, relay_state_};
    
    // NUEVA FUNCIÓN DE TRANSMISIÓN.
    return i2c_master_transmit(dev_handle_, data, 2, timeout_ms_);
}

#endif
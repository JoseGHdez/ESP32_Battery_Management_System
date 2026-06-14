#ifndef ADS1115_H
#define ADS1115_H

#include <stdio.h>    // printf, snprintf
#include <stdlib.h>   // atoi, atof
#include <string.h>   // memset, strcmp, strcasecmp
#include <inttypes.h> // int16_t, uint8_t, etc.
#include <iostream>   // std::cout, std::endl
#include <array>      // std::array
#include <iomanip>    // std::setprecision, std::fixed
#include <utility>    // std::pair for returning multiple values from functions

#include "freertos/FreeRTOS.h" 
#include "freertos/task.h"     
#include "esp_err.h" 
#include "esp_log.h" 
#include "driver/uart.h" 

// NUEVO DRIVER I2C ORIENTADO A OBJETOS
#include "driver/i2c_master.h" 

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

// Mantenemos tus semáforos globales para sincronización entre tareas
extern SemaphoreHandle_t i2c_bus_mutex; 
extern SemaphoreHandle_t ads1115_chip_mutex;

class ADS1115 {
 public:
  // El constructor ahora recibe el bus maestro creado en el main, igual que el Relé
  ADS1115(i2c_master_bus_handle_t bus_handle, uint8_t ads1115_addr = ADS1115_ADDR, 
          uint8_t ads1115_reg_config = ADS1115_REG_CONFIG, uint8_t ads1115_reg_conv = ADS1115_REG_CONV,
          int timeout_ms = 50); // Timeout por defecto directo en ms (no uses valores muy altos)

  std::pair<float, esp_err_t> ReadVoltage(uint8_t channel, bool fz0430_flag = false);
  float CalibrateVoltage(float known_voltage, float factor);
  std::pair<float, esp_err_t> ReadCurrent(uint8_t channel, float bias = 0.0f);
  void Diagnostic();
  std::pair<float, esp_err_t> ConfigureADS1115(uint8_t channel);

 private:
  i2c_master_bus_handle_t bus_handle_;   // Manejador del bus físico compartido
  i2c_master_dev_handle_t dev_handle_;   // Manejador exclusivo de este ADS1115
  int timeout_ms_;
  uint8_t ads1115_addr_;
  uint8_t ads1115_reg_config_;
  uint8_t ads1115_reg_conv_;
  float point_zero_voltage_ = 2.54762f; // Calibración de tu sensor de corriente
};

// DENTRO DEL CONSTRUCTOR DE ADS1115 EN ADS1115.h:
inline ADS1115::ADS1115(i2c_master_bus_handle_t bus_handle, uint8_t ads1115_addr, 
                        uint8_t ads1115_reg_config, uint8_t ads1115_reg_conv, int timeout_ms) {
  bus_handle_ = bus_handle;
  ads1115_addr_ = ads1115_addr;
  ads1115_reg_config_ = ads1115_reg_config;
  ads1115_reg_conv_ = ads1115_reg_conv;
  timeout_ms_ = timeout_ms;

  // Inicialización limpia compatible 100% con C++23 campo por campo
  i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = ads1115_addr_,
    .scl_speed_hz = I2C_MASTER_FREQ_HZ, // <-- Bajar a 100kHz estabiliza la señal
    .scl_wait_us = 0,
    .flags = {
        .disable_ack_check = false
    }
};

  // Registramos este dispositivo específico en el bus maestro común
  esp_err_t err = i2c_master_bus_add_device(bus_handle_, &dev_cfg, &dev_handle_);
  if (err != ESP_OK) {
      ESP_LOGE("ADS1115", "Error al añadir el ADS1115 al bus: %s", esp_err_to_name(err));
  } else {
      ESP_LOGI("ADS1115", "ADS1115 enlazado correctamente al bus nuevo");
  }
}

// IMPLEMENTACIÓN DE CONFIGURACIÓN
inline std::pair<float, esp_err_t> ADS1115::ConfigureADS1115(uint8_t channel) {
  if (xSemaphoreTake(ads1115_chip_mutex, portMAX_DELAY) != pdTRUE) return std::make_pair(-1.0, ESP_FAIL);

  uint8_t mux_bits = 0xC0 | (channel << 4);
  uint8_t config_data[3] = { ads1115_reg_config_, mux_bits, 0x83 };
  
  esp_err_t err_init = ESP_FAIL;

  if (xSemaphoreTake(i2c_bus_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    // NUEVA FUNCIÓN: i2c_master_transmit (Usa dev_handle_ y el timeout directo en ms)
    err_init = i2c_master_transmit(dev_handle_, config_data, 3, timeout_ms_);
    xSemaphoreGive(i2c_bus_mutex);
    
    uint8_t counter = 0;
    // Mantenemos tu excelente bucle de reintentos contra las interferencias
    while (counter < 5 && err_init != ESP_OK) {
      ESP_LOGW("ADS1115", "Config write attempt %d failed (ch%d): %s. Reintentando...", counter + 1, channel, esp_err_to_name(err_init));
      vTaskDelay(pdMS_TO_TICKS(10));
      if (xSemaphoreTake(i2c_bus_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        err_init = i2c_master_transmit(dev_handle_, config_data, 3, timeout_ms_);
        xSemaphoreGive(i2c_bus_mutex);
      }

      if (err_init == ESP_ERR_INVALID_STATE || err_init == ESP_FAIL) {
        ESP_LOGW("ADS1115", "Reseteando bus por bloqueo del driver...");
        i2c_master_bus_reset(bus_handle_); // <-- Obliga al driver v5 a salir del pánico
        vTaskDelay(pdMS_TO_TICKS(10));
      }
      
      counter++;
    }
    

    if (err_init != ESP_OK) {
      ESP_LOGE("ADS1115", "Config write fallida tras 5 intentos (ch%d): %s", channel, esp_err_to_name(err_init));
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

// IMPLEMENTACIÓN DE LECTURA DE VOLTAJE
inline std::pair<float, esp_err_t> ADS1115::ReadVoltage(uint8_t channel, bool fz0430_flag) {
  if (channel > 3) return std::make_pair(-1.0, ESP_FAIL);

  if (xSemaphoreTake(ads1115_chip_mutex, portMAX_DELAY) != pdTRUE) return std::make_pair(-1.0, ESP_FAIL);

  vTaskDelay(pdMS_TO_TICKS(20)); // Tiempo de cortesía para estabilización

  uint8_t reg = ads1115_reg_conv_;
  uint8_t data[2] = {0, 0};
  esp_err_t err_read = ESP_FAIL;

  // --- PRIMER INTENTO ---
  if (xSemaphoreTake(i2c_bus_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    err_read = i2c_master_transmit_receive(dev_handle_, &reg, 1, data, 2, timeout_ms_);
    xSemaphoreGive(i2c_bus_mutex); // ¡LIBERAMOS EL BUS INMEDIATAMENTE!
    
    // --- BUCLE DE REINTENTOS ROBUSTO ---
    uint8_t count = 0;
    while (count < 5 && err_read != ESP_OK) {
      ESP_LOGW("ADS1115", "Read attempt %d failed (ch%d): %s. Reintentando...", count + 1, channel, esp_err_to_name(err_read));
      
      vTaskDelay(pdMS_TO_TICKS(10)); // Al soltar el bus, este delay realmente permite la recuperación por hardware
      
      if (err_read == ESP_ERR_TIMEOUT || err_read == ESP_ERR_INVALID_STATE || err_read == ESP_FAIL) {
        ESP_LOGE("BMS", "¡Bus I2C congelado! Aplicando reinicio de emergencia...");
        i2c_master_bus_reset(bus_handle_); // Fuerza al hardware a liberar las líneas SDA/SCL
      }

      if (xSemaphoreTake(i2c_bus_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        err_read = i2c_master_transmit_receive(dev_handle_, &reg, 1, data, 2, timeout_ms_);
        xSemaphoreGive(i2c_bus_mutex); // Liberamos tras cada reintento
      } 

      count++;
    }
  } else {
    xSemaphoreGive(ads1115_chip_mutex);
    return std::make_pair(-1.0, ESP_ERR_TIMEOUT);
  }

  xSemaphoreGive(ads1115_chip_mutex);

  if (err_read != ESP_OK) {
    ESP_LOGE("ADS1115", "Conversion read fallida tras 5 intentos (ch%d): %s", channel, esp_err_to_name(err_read));
    return std::make_pair(-1.0, err_read); 
  }

  int16_t raw = (int16_t)((data[0] << 8) | data[1]);
  
  // Respetamos exactamente tu factor de escala original
  float voltage = raw * 0.00018754579f;

  return fz0430_flag ? std::make_pair(CalibrateVoltage(voltage, 5.025f), ESP_OK) : std::make_pair(voltage, ESP_OK);
}

// IMPLEMENTACIÓN DE CALIBRACIÓN
inline float ADS1115::CalibrateVoltage(float known_voltage, float factor) {
  return known_voltage * factor; 
}

// IMPLEMENTACIÓN DE CORRIENTE
inline std::pair<float, esp_err_t> ADS1115::ReadCurrent(uint8_t channel, float bias) {
  auto voltage_result = ReadVoltage(channel);
  float voltage = voltage_result.first;
  esp_err_t err = voltage_result.second;

  if (err != ESP_OK) {
    return std::make_pair(-1.0, err);
  }

  ESP_LOGV("ADS1115", "Voltage for current reading: %.3f V", voltage);

  voltage -= bias;
  voltage -= point_zero_voltage_; // Quitar el offset del punto cero (2.54762f)

  float current = voltage / 0.033f; // Sensibilidad de tu sensor
  return std::make_pair(current, ESP_OK);
} 

// IMPLEMENTACIÓN DE DIAGNÓSTICO
inline void ADS1115::Diagnostic() {
  uint8_t config_reg = ads1115_reg_config_;
  uint8_t data[2] = {0, 0};

  // Verificación de configuración usando el nuevo driver
  esp_err_t err = i2c_master_transmit_receive(dev_handle_, &config_reg, 1, data, 2, timeout_ms_);
  
  uint16_t actual_config = (data[0] << 8) | data[1];
  printf("--- Verification de Configuration ADS1115 ---\n");
  if (err != ESP_OK) {
      printf("Error de comunicación I2C en Diagnóstico: %s\n", esp_err_to_name(err));
  } else {
      printf("Config en el chip: 0x%04X (Debería ser 0xC083 o 0x4083)\n", actual_config);
      if (actual_config != 0xC083 && actual_config != 0x4083) {
        printf("¡ERROR! El chip no se configuró bien. Revisa cables SDA/SCL o ruido.\n");
      }
  }

  // Test rápido de multiplexado por canales
  uint8_t mux_values[4] = {0xC0, 0xD0, 0xE0, 0xF0}; 
    
  for (int i = 0; i < 4; i++) {
    uint8_t setup[3] = {ads1115_reg_config_, mux_values[i], 0x83};
    
    // Escribimos la configuración temporal
    i2c_master_transmit(dev_handle_, setup, 3, timeout_ms_);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Leemos el resultado
    uint8_t conv_reg = ads1115_reg_conv_;
    err = i2c_master_transmit_receive(dev_handle_, &conv_reg, 1, data, 2, timeout_ms_);
        
    if (err == ESP_OK) {
        int16_t raw = (data[0] << 8) | data[1];
        printf("Canal A%d -> Raw: %d | Volt: %.3fV\n", i, raw, raw * 0.0001875f);
    } else {
        printf("Canal A%d -> Error de lectura por ruido.\n", i);
    }
 }
}

#endif // ADS1115_H
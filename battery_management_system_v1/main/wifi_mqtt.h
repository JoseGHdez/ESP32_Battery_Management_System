#ifndef WIFI_H
#define WIFI_H

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
#include <sys/time.h>
#include <ctime>

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
#include "esp_netif_sntp.h"
#include "esp_sntp.h"

#include "sgm2578.h" // Display library for M5StickCPlus2
#include "st7789.h" // TFT display driver for ST7789-based displays (used in M5StickCPlus2)
#include "fontx.h" // Font rendering library for displaying text on the TFT screen
#include "bmpfile.h" // Bitmap file handling library (not used in this code but included for potential future use in displaying images on the TFT)
#include "I2Crelay.h" // Header file for relay control functions
#include "ADS1115.h" // Header file for ADS1115 ADC functions
#include "mg996r.h" // Header file for MG996R servo control functions
#include "shared.h" // Header file for common definitions and utilities

// ============ TASK CONFIGURATION ============
#define INTERVAL 400 /*!<@brief Delay interval for the task */
#define WAIT vTaskDelay(INTERVAL) /*!<@brief Delay code for the task */

// Wifi configuration and MQTT
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
    // Default values
    #define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
    #define EXAMPLE_H2E_IDENTIFIER ""
#endif

#ifndef ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD
    #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#endif

#define ESP_WIFI_SSID      "TP-Link_E3D6" 
#define ESP_WIFI_PASS      "76649769"
#define MQTT_BROKER_URI    "mqtt://192.168.1.100"
#define ESP_MAXIMUM_RETRY  20
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *WIFI_TAG = "wifi";
static int s_retry_num = 0;
volatile int isConnected = 0;
static EventGroupHandle_t s_wifi_event_group;
esp_mqtt_client_handle_t mqtt_client = NULL;

// --- Connection handlers ---
/**
 * @brief Event handler for WiFi events, managing connection and disconnection
 *        events.
 * 
 * @param arg User-defined argument (not used in this handler).
 * @param event_base The base of the event (e.g., WIFI_EVENT, IP_EVENT).
 * @param event_id The specific event ID (e.g., WIFI_EVENT_STA_START, 
 *        WIFI_EVENT_STA_DISCONNECTED, IP_EVENT_STA_GOT_IP).
 * @param event_data Pointer to event-specific data (not used in this handler).
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base, 
  int32_t event_id, void* event_data) {
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
        ESP_LOGI(WIFI_TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        isConnected = 1;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief Event handler for MQTT events, managing connection, disconnection, 
 *        subscription, and message events.
 * 
 * @param handler_args Pointer to user-defined arguments
 * @param base The base of the event (e.g., MQTT_EVENT).
 * @param event_id The specific event ID (e.g., MQTT_EVENT_CONNECTED, 
 *        MQTT_EVENT_DATA).
 * @param event_data Pointer to event-specific data (esp_mqtt_event_handle_t).
 */
static void mqtt_event_handler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data) {
  AppContext *context = static_cast<AppContext *>(handler_args);
    
  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
  int msg_id;

  switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
      ESP_LOGI(WIFI_TAG, "MQTT_EVENT_CONNECTED");
      msg_id = esp_mqtt_client_subscribe(mqtt_client, "m5stick/sensors/charge", 0);
      ESP_LOGI(WIFI_TAG, "sent subscribe successful, msg_id=%d", msg_id);
      msg_id = esp_mqtt_client_subscribe(mqtt_client, "m5stick/sensors/discharge", 0);
      ESP_LOGI(WIFI_TAG, "sent subscribe successful, msg_id=%d", msg_id);
      msg_id = esp_mqtt_client_subscribe(mqtt_client, "m5stick/relay/#", 0);
      ESP_LOGI(WIFI_TAG, "sent subscribe successful, msg_id=%d", msg_id);
      break;

    case MQTT_EVENT_DISCONNECTED:
      ESP_LOGI(WIFI_TAG, "MQTT_EVENT_DISCONNECTED");
      break;

    case MQTT_EVENT_SUBSCRIBED:
      ESP_LOGI(WIFI_TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d, return code=0x%02x", event->msg_id, (uint8_t)*event->data);
      break;

    case MQTT_EVENT_UNSUBSCRIBED:
      ESP_LOGI(WIFI_TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
      break;

    case MQTT_EVENT_PUBLISHED:
      ESP_LOGI(WIFI_TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
      // printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
      // printf("DATA=%.*s\r\n", event->data_len, event->data);
      break;

    case MQTT_EVENT_DATA:
      ESP_LOGI(WIFI_TAG, "MQTT_EVENT_DATA");
      printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
      printf("DATA=%.*s\r\n", event->data_len, event->data);
      if (strncmp(event->topic, "m5stick/relay/", 14) == 0) {
        char relay_char = event->topic[14];
        int relay_num = relay_char - '0'; 
                
        if (!experiment_flag) {
          if (relay_num >= 1 && relay_num <= 4 && event->data_len > 0) {
            bool turn_on = event->data;
            context->relayController -> set_relay(relay_num, turn_on);
            experiment_flag = turn_on;
            ESP_LOGI(WIFI_TAG, "MQTT: Relé %d -> %s", relay_num, turn_on ? "ON" : "OFF");
          }
        }
      }
      break;

    case MQTT_EVENT_ERROR:
      ESP_LOGI(WIFI_TAG, "MQTT_EVENT_ERROR");
      if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
        ESP_LOGI(WIFI_TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
        ESP_LOGI(WIFI_TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
        ESP_LOGI(WIFI_TAG, "Last captured errno : %d (%s)", event->error_handle->esp_transport_sock_errno,
          strerror(event->error_handle->esp_transport_sock_errno));
      } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
        ESP_LOGI(WIFI_TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
      } else {
        ESP_LOGW(WIFI_TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
      }
      break;

    default:
      ESP_LOGI(WIFI_TAG, "Other event id:%d", event->event_id);
      break;
  }
}

/**
 * @brief Initializes WiFi and MQTT connections using the provided AppContext.
 * 
 * @param context Pointer to the AppContext containing experiment parameters.
 */
void wifi_and_mqtt_init(AppContext *context) {
  AppContext *ctx = context;
  ExperimentParams *exp_params = ctx->experimentParams;

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
  strcpy((char*)wifi_config.sta.ssid, exp_params->wifi_ssid.c_str());
  strcpy((char*)wifi_config.sta.password, exp_params->wifi_password.c_str());

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  // Wait for connection before initializing MQTT
  xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

  esp_mqtt_client_config_t mqtt_cfg = {};
  std::string broker_uri = "mqtt://" + exp_params->mosquitto_ip + ":1883";
  mqtt_cfg.broker.address.uri = broker_uri.c_str();
    
  mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_register_event(mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, context);
  esp_mqtt_client_start(mqtt_client);
}

/**
 * @brief Synchronizes the system time with an NTP server using SNTP.
 */
inline void sntp_time_sync(void) {
  ESP_LOGI("WIFI_TIME", "Inicializando SNTP (API Clasica)...");
    
  esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
  esp_sntp_setservername(0, "pool.ntp.org");
  esp_sntp_init();

  // Wait for SNTP synchronization with a timeout
  int tries = 0;
  const int max_tries = 10;
    
  while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++tries <= max_tries) {
    ESP_LOGI("WIFI_TIME", "Esperando respuesta del servidor NTP... (%d/%d)", tries, max_tries);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }

  // Set timezone to Central European Time (CET/CEST)
  setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
  tzset();
}

#endif // WIFI_H

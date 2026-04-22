#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#define WIFI_SSID      "TU_NOMBRE_DE_RED"
#define WIFI_PASS      "TU_CONTRASEÑA"
#define MAXIMUM_RETRY  5

static const char *TAG = "wifi_station";
static int s_retry_num = 0;

/* Manejador de eventos para Wi-Fi e IP */
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Reintentando conectar a la red...");
        } else {
            ESP_LOGI(TAG, "No se pudo conectar a la red Wi-Fi.");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Conectado exitosamente. IP asignada: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
    }
}

void wifi_init_sta(void) {
    // 1. Inicializar la pila de red LwIP
    ESP_ERROR_CHECK(esp_netif_init());

    // 2. Crear el bucle de eventos por defecto
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // 3. Inicializar el driver Wi-Fi con la configuración por defecto
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 4. Registrar el manejador de eventos
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    // 5. Configurar los parámetros de la conexión Wi-Fi
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK, // Nivel de seguridad mínimo
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    // 6. Arrancar el Wi-Fi (esto disparará el evento WIFI_EVENT_STA_START)
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta inicialización completa.");
}

void app_main(void) {
    // Inicializar NVS (Obligatorio para el Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Arrancar el módulo Wi-Fi
    wifi_init_sta();
    
    // NOTA: A partir de aquí puedes iniciar tus tareas para I2C (Relés), ADC (Batería) etc.
}
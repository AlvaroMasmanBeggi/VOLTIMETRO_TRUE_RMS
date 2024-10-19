#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"

#define WIFI_SSID "NOMBREWIFI"
#define WIFI_PASS "CONTRASEÑAWIFI"

static const char *TAG = "WiFi_Server";

float valores_float[5] = {1.23,3.23,4.53,2.54,32.1};
char matriz_char[5][11];

// Maneja la petición GET y responde en la pantalla
esp_err_t hello_get_handler(httpd_req_t *req) {
    char response[256];
    // Convierte el valor del float a string y lo almacena en response
    strcpy(response, "Valores en la matriz:<br>");

    // Agrega cada valor de la matriz a la respuesta
    for (int i = 0; i < 5; i++) {
        strcat(response, matriz_char[i]);
        strcat(response, "<br>");
    }
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Registra la URL para manejar la respuesta
httpd_uri_t hello = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = hello_get_handler,
    .user_ctx  = NULL
};

// Inicia el servidor HTTP
static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &hello);
    }
    return server;
}

// Función de evento de Wi-Fi
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Intentando reconectar...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "Dirección IP: " IPSTR, IP2STR(&event->ip_info.ip));
        start_webserver();  // Inicia el servidor web
    }
}

// Inicializa el Wi-Fi
void wifi_init_sta(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();
}

void app_main(void) {
// Asignar valores a la matriz de floats
    for (int i = 0; i < 5; i++) {
        // Convertir el float a string y almacenar en matriz_caracteres
        snprintf(matriz_char[i], sizeof(matriz_char[i]), "%.2f", valores_float[i]);
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    wifi_init_sta();  // Inicia la conexión Wi-Fi
}

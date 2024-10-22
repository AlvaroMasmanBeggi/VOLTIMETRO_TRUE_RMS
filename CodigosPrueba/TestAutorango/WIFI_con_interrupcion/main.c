#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"

#define WIFI_SSID "motog52"
#define WIFI_PASS "bruno312"

static const char *TAG = "WiFi_Server";

float valores_float[5] = {1.23,3.23,4.53,2.54,32.1};
char matriz_char[5][11];

#define PIN_TRIGGER 14  // Definir el pin de interrupción

volatile bool estado_variable = false, bandera=false, habilitar_wifi=false;  // Variable a cambiar

// Función de servicio de la interrupción
void IRAM_ATTR manejador_interrupcion() {
    estado_variable = !estado_variable;  // Cambiar el estado de la variable
    habilitar_wifi=true;
}

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
    bandera=true;
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
    esp_log_level_set("wifi", ESP_LOG_NONE);  // Desactiva todos los logs relacionados con Wi-Fi

    wifi_init_sta();  // Inicia la conexión Wi-Fi

    esp_wifi_stop();

    // Configurar el pin como entrada
    esp_rom_gpio_pad_select_gpio(PIN_TRIGGER);
    gpio_set_direction(PIN_TRIGGER, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_TRIGGER, GPIO_PULLDOWN_ONLY); // Configurar pull-up si es necesario

    // Configurar la interrupción
    gpio_set_intr_type(PIN_TRIGGER, GPIO_INTR_POSEDGE);  // Interrupción en flanco ascendente
    gpio_install_isr_service(0);  // Instalar el servicio de interrupciones
    gpio_isr_handler_add(PIN_TRIGGER, manejador_interrupcion, (void*) PIN_TRIGGER);  // Añadir el manejador de la interrupción

    esp_task_wdt_deinit();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(3000));  // Esperar 1 segundo
        // Aquí puedes hacer algo con el estado_variable
        printf("Estado de la variable: %s\n", estado_variable ? "VERDADERO" : "FALSO");

        if(habilitar_wifi==true){
            esp_wifi_start();
            habilitar_wifi=false;
        }
        if(bandera==true){
            esp_wifi_stop();
            bandera=false;
            estado_variable = !estado_variable;
        }
    }
}
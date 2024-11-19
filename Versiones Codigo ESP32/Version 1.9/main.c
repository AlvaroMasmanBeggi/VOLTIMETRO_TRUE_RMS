#include <driver/gpio.h>
#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <driver/adc.h>
#include <esp_system.h>
#include <math.h>
#include "driver/i2c.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_timer.h"


//La llave de 3k9 es el rango 1, la de 68k el 2 y la de 1M el 3

//El pin de lectura DC/AC corresponde a ADC1_CHANNEL_7 y es el pin 35 de la esp

//Pines de control de rango
//Pin de 3k9: 12
//Pin de 68k: 14
//Pin de 1M: 27

//Pin de display
//SDA es el 21
//SCL es el 22

//DatosWIFI
#define WIFI_SSID "motog52"
#define WIFI_PASS "bruno312"

//Pines de interrupciones
//Guardar dato es el 17
//Enviar dato es el 16

#define I2C_MASTER_SCL_IO 22    
#define I2C_MASTER_SDA_IO 21   
#define I2C_MASTER_NUM     I2C_NUM_0   //Master numero 0
#define I2C_MASTER_FREQ_HZ   100000  // Frecuencia de 100kHz
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define LCD_I2C_ADDRESS      0x27   // Dirección I2C del adaptador 

// Comandos básicos para el LCD
#define LCD_CMD_CLEAR_DISPLAY 0x01
#define LCD_CMD_RETURN_HOME   0x02
#define LCD_CMD_ENTRY_MODE    0x06
#define LCD_CMD_DISPLAY_ON    0x0C
#define LCD_CMD_FUNCTION_SET  0x28    // Modo de 4 bits, 2 líneas

#define PIN_TRIGGER_WIFI GPIO_NUM_16     // Definir el pin de interrupción
#define PIN_TRIGGER_GUARDAR GPIO_NUM_17  // Definir el pin de interrupción

#define pin4k7 GPIO_NUM_12
#define pin68k GPIO_NUM_14
#define pin1M GPIO_NUM_27
#define DCACinput ADC1_CHANNEL_7
#define FREQIN ADC1_CHANNEL_6

#define DEFAULT_VREF 1100 
#define max_adc 2.5
#define min_adc 0.2

int i = 0, k=0, numero_muestrasAC = 6000, numero_muestrasDC = 100, rango_actual = 3,esDC;
float lectura = 0, valor_inst = 0, volt_value = 0,data_to_save[5] = {0,0,0,0,0}, data_read[5] = {0,1,2,0,0},dato=2.65;
char matriz_char[5][11], buffer_display_superior[20], buffer_display_inferior[20];

volatile bool bandera=false, habilitar_wifi=false, guardar_datos=false;

// Función de servicio de la interrupción de memoria
void IRAM_ATTR rutina_servicio_memoria() {
    guardar_datos=true;  // Cambiar el estado de la variable
}

// Función de servicio de la interrupción de WIFI
void IRAM_ATTR rutina_servicio_WIFI() {
    habilitar_wifi=true;
}

// Inicializa el bus I2C
void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

// Enviar comando de 4 bits al LCD
void lcd_send_nibble(uint8_t data) {
    uint8_t upper_nibble = (data & 0xF0);   // Extraer los 4 bits altos
    uint8_t control = upper_nibble | 0x08;  // RS=0 para comando, En=1 para ciclo de habilitación

    uint8_t packet[4] = {
        control,             // 4 bits altos + En=1
        control | 0x04,      // 4 bits altos + En=0 (ciclo de habilitación)
    };

    // Enviar al LCD a través del bus I2C
    i2c_master_write_to_device(I2C_MASTER_NUM, LCD_I2C_ADDRESS, packet, sizeof(packet), 1000 / portTICK_PERIOD_MS);
}

// Enviar comando al LCD en modo de 4 bits
void lcd_send_command(uint8_t command) {
    lcd_send_nibble(command);            // Enviar 4 bits altos
    lcd_send_nibble(command << 4);       // Enviar 4 bits bajos
    vTaskDelay(2 / portTICK_PERIOD_MS);  // Retardo para procesar comando
}

// Enviar datos (caracter) al LCD
void lcd_send_data(uint8_t data) {
    uint8_t upper_nibble = (data & 0xF0);  // Extraer los 4 bits altos
    uint8_t lower_nibble = (data << 4);    // Extraer los 4 bits bajos

    uint8_t control_upper = upper_nibble | 0x09;  // RS=1 para datos, En=1
    uint8_t control_lower = lower_nibble | 0x09;  // RS=1 para datos, En=1

    uint8_t packet[4] = {
        control_upper,             // 4 bits altos + En=1
        control_upper | 0x04,      // 4 bits altos + En=0
        control_lower,             // 4 bits bajos + En=1
        control_lower | 0x04,      // 4 bits bajos + En=0
    };

    // Enviar datos al LCD a través del bus I2C
    i2c_master_write_to_device(I2C_MASTER_NUM, LCD_I2C_ADDRESS, packet, sizeof(packet), 1000 / portTICK_PERIOD_MS);
}

// Inicializar el LCD
void lcd_init() {
    vTaskDelay(50 / portTICK_PERIOD_MS);  // Esperar 50ms tras encendido

    lcd_send_command(0x03);  // Comando inicial (3 veces para asegurarse)
    vTaskDelay(5 / portTICK_PERIOD_MS);
    lcd_send_command(0x03);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    lcd_send_command(0x03);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    lcd_send_command(0x02);  // Configuración para modo de 4 bits

    // Configuración inicial del LCD
    lcd_send_command(LCD_CMD_FUNCTION_SET);  // Modo de 4 bits, 2 líneas
    lcd_send_command(LCD_CMD_DISPLAY_ON);    // Encender display sin cursor
    lcd_send_command(LCD_CMD_CLEAR_DISPLAY); // Limpiar display
    lcd_send_command(LCD_CMD_ENTRY_MODE);    // Modo entrada
}

// Función para enviar una cadena al LCD
void lcd_send_string(const char* str) {
    while (*str) {
        lcd_send_data(*str++);  // Enviar cada carácter como datos
    }
}

// Posicion del cursor
void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t row_offsets[] = { 0x00, 0x40 };  // Offsets de memoria para la primera y segunda fila
    if (row > 1) row = 1;  // El LCD 1602A tiene solo dos filas
    lcd_send_command(0x80 | (col + row_offsets[row]));  // Comando para mover el cursor
}

void lectura_flash(){
    // Crear una instancia de NVS
    nvs_handle_t my_handle;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READONLY, &my_handle));
    
    // Leer el arreglo de NVS
    size_t required_size = 5*sizeof(float);
    ESP_ERROR_CHECK(nvs_get_blob(my_handle, "float_array", data_read, &required_size));
  
    for (int i = 0; i < 5; i++) {
            printf("Datos leidos: %.2f\n", data_read[i]);
        }
    nvs_close(my_handle);
}

// Maneja la petición GET y responde en la pantalla
esp_err_t hello_get_handler(httpd_req_t *req) {
    // Asignar valores a la matriz de floats
    lectura_flash();
 for (int i = 0; i < 5; i++) {
        // Convertir el float a string y almacenar en matriz_caracteres
        snprintf(matriz_char[i], sizeof(matriz_char[i]), "%.2f", data_read[i]);
    }

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
        lcd_set_cursor(0, 0);
        lcd_send_string("Conectando... ");
        
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        //ESP_LOGI(TAG, "Dirección IP: " IPSTR, IP2STR(&event->ip_info.ip));
        // Formatear la dirección IP dentro del buffer
        lcd_set_cursor(0, 0);
        lcd_send_string("Ingrese a: ");

        sprintf(buffer_display_inferior, "IP: " IPSTR, IP2STR(&event->ip_info.ip));
        lcd_set_cursor(0, 1);
        lcd_send_string(buffer_display_inferior);
         vTaskDelay(3000 / portTICK_PERIOD_MS);
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

void config(){
    adc1_config_width(ADC_WIDTH_BIT_12);                         // Configura el ADC a 12 bits
    adc1_config_channel_atten(DCACinput, ADC_ATTEN_DB_12);  // Configura la atenuación para el canal 7 (GPIO 35)                        
    adc1_config_channel_atten(FREQIN, ADC_ATTEN_DB_12); // Configura la atenuación para el canal 6 (GPIO 34)
    
    esp_rom_gpio_pad_select_gpio(pin4k7);
    gpio_set_direction(pin4k7,GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(pin68k);
    gpio_set_direction(pin68k,GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(pin1M);
    gpio_set_direction(pin1M,GPIO_MODE_OUTPUT);

    // Configurar el pin  de wifi como entrada
    esp_rom_gpio_pad_select_gpio(PIN_TRIGGER_WIFI);
    gpio_set_direction(PIN_TRIGGER_WIFI, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_TRIGGER_WIFI, GPIO_PULLDOWN_ONLY); 

    // Configurar el pin de guardar como entrada
    esp_rom_gpio_pad_select_gpio(PIN_TRIGGER_GUARDAR);
    gpio_set_direction(PIN_TRIGGER_GUARDAR, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_TRIGGER_GUARDAR, GPIO_PULLDOWN_ONLY); 

    // Configurar las interrupciones
    gpio_install_isr_service(0);  // Instalar el servicio de interrupciones
    gpio_set_intr_type(PIN_TRIGGER_WIFI, GPIO_INTR_POSEDGE);  // Interrupción en flanco ascendente
    gpio_isr_handler_add(PIN_TRIGGER_WIFI, rutina_servicio_WIFI, (void*) PIN_TRIGGER_WIFI);  // Añadir el manejador de la interrupción
    gpio_set_intr_type(PIN_TRIGGER_GUARDAR, GPIO_INTR_POSEDGE);  // Interrupción en flanco ascendente
    gpio_isr_handler_add(PIN_TRIGGER_GUARDAR, rutina_servicio_memoria, (void*) PIN_TRIGGER_GUARDAR);  // Añadir el manejador de la interrupción

    // Desactiva todos los logs relacionados con Wi-Fi
    esp_log_level_set("wifi", ESP_LOG_NONE);

    //Inicio y chequeo de memoria flash
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND || ESP_ERR_NVS_INVALID_STATE) {
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Inicia la conexión Wi-Fi
    wifi_init_sta();  
    esp_wifi_stop();

    i2c_master_init();  // Inicializar I2C
    lcd_init();         // Inicializar 

    gpio_set_level(pin1M,1);
    gpio_set_level(pin68k,0);
    gpio_set_level(pin4k7,0);


}

void autorangoDC(float valor_actual){
    if(valor_actual>max_adc){
        switch(rango_actual){
            case 1:{
                lcd_set_cursor(0, 0);
                lcd_send_string("Rango máximo ");
                break;
            }
            case 2:{
                gpio_set_level(pin4k7,1);
                gpio_set_level(pin68k,0);
                rango_actual = 1;
                break;
            }
            case 3:{
                gpio_set_level(pin68k,1);
                gpio_set_level(pin1M,0);
                rango_actual = 2;
                break;
            }
        }   
    }
    
    if(valor_actual<min_adc){
        switch(rango_actual){
            case 1:{
                gpio_set_level(pin68k,1);
                gpio_set_level(pin4k7,0);
                rango_actual = 2;
                break;
            }
            case 2:{
                gpio_set_level(pin1M,1);
                gpio_set_level(pin68k,0);
                rango_actual = 3;
                break;
            }   
        }
    } 
}

void autorangoAC(float valor_actual){
    if(valor_actual>1.75){
        switch(rango_actual){
            case 1:{
                lcd_set_cursor(0, 0);
                lcd_send_string("Rango máximo ");
                break;
            }
            case 2:{
                gpio_set_level(pin4k7,1);
                gpio_set_level(pin68k,0);
                rango_actual = 1;
                break;
            }
            case 3:{
                gpio_set_level(pin68k,1);
                gpio_set_level(pin1M,0);
                rango_actual = 2;
                break;
            }
        }   
    }
    
    if(valor_actual<0.15){
        switch(rango_actual){
            case 1:{
                gpio_set_level(pin68k,1);
                gpio_set_level(pin4k7,0);
                rango_actual = 2;
                break;
            }
            case 2:{
                gpio_set_level(pin1M,1);
                gpio_set_level(pin68k,0);
                rango_actual = 3;
                break;
            }   
        }
    } 
}

float calculo_voltaje_DC(float dato){
    dato = ((dato * 3.3) / 4095)+0.11;   // Convierte la lectura del ADC a voltaje
    
    autorangoDC(dato);
   
    
    switch(rango_actual){
        case 1:{
                dato = -12279.48404*pow(dato,5)+19878.08366*pow(dato,4)-12519.545199*pow(dato,3)+3828.85629*pow(dato,2)-476.96148*dato+24.46554;
                dato = 0.000227511*pow(dato,2)+1.0223973*dato-1.11233982;
            break;
        }
        case 2: {
                dato = -0.00510281678*pow(dato,4)+0.0022684058*pow(dato,3)+0.0625534938*pow(dato,2)+6.24771582*dato-0.665277666;
                dato= -0.0000530263*pow(dato,5)+0.00194484*pow(dato,4)-0.02516799*pow(dato,3)+0.1378113*pow(dato,2)+0.71072067*dato+0.1714016;
            break;
        }
        case 3:{
                dato = -0.00282553*pow(dato,4)+0.0175085*pow(dato,3)-0.03770024*pow(dato,2)+0.5100984*dato-0.16514693;
            break;
        }
    }
    return dato;
}

float calculo_voltaje_AC(float dato){
    autorangoAC(dato);
   
    switch(rango_actual){
        case 1:{
                
            break;
        }
        case 2: {

            
            break;
        }
        case 3:{
            dato=0.05*pow(dato,4)-0.18*pow(dato,3)+0.25*pow(dato,2)+0.36*dato+0.02; 
            break;
        }
    }
    return dato;
}

int deteccionAC(){
    //tipo tension es igual a 1 cuando detecta ac, sino es dc
    int tipo_tension=0,muestras=300;
    float valores[300];

    for (i = 0; i < muestras; i++){
            lectura = adc1_get_raw(FREQIN);
            valores[i]=lectura;
            vTaskDelay(0.4 / portTICK_PERIOD_MS);        // Retardo para estabilizar
        }
    for(i=1;i < muestras; i++){
        if(valores[i]<1200){
            tipo_tension=1;
            return tipo_tension;
        }
    }
    return tipo_tension;
}

void lectura_tensionDC(){
    lectura = 0;
    for (i = 0; i < numero_muestrasDC; i++){
        lectura += adc1_get_raw(DCACinput);
        vTaskDelay(10 / portTICK_PERIOD_MS);        // Retardo para estabilizar
    }
    lectura /= numero_muestrasDC;
        
    volt_value = calculo_voltaje_DC(lectura);

    lcd_set_cursor(0, 0);
    lcd_send_string("   Voltaje DC:   ");

    sprintf(buffer_display_inferior, "      %.3f      ",volt_value );
    lcd_set_cursor(0, 1);
    lcd_send_string(buffer_display_inferior);
    esDC=1;

    vTaskDelay(500 / portTICK_PERIOD_MS); // Espera antes de la próxima lectura   
}

void lectura_tensionAC(){
    lectura = 0;
    valor_inst = 0;
    for (i = 0; i < numero_muestrasAC; i++){
           lectura = adc1_get_raw(DCACinput);
           vTaskDelay(0.017 / portTICK_PERIOD_MS);        // Retardo para estabilizar. El calculo del tiempo de muestreo se hace hasta el 7mo armonico de 400Hz
           valor_inst = lectura * lectura + valor_inst;

     }
    valor_inst /= numero_muestrasAC;
    volt_value = sqrt(valor_inst);
    volt_value = ((volt_value * 3.3) / 4095)+0.11;   // Convierte la lectura del ADC a voltaje
    
    volt_value = calculo_voltaje_AC(volt_value);
    
    lcd_set_cursor(0, 0);
    lcd_send_string("   Voltaje AC:   ");

    sprintf(buffer_display_inferior, "      %.3f      ",volt_value );
    lcd_set_cursor(0, 1);
    lcd_send_string(buffer_display_inferior);
    esDC=0;

    vTaskDelay(500 / portTICK_PERIOD_MS); // Espera antes de la próxima lectura
}

void app_main() {
    config();           
    
while (1) {
        if(guardar_datos==true){
            nvs_handle_t my_handle;
            ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &my_handle));

            // sprintf(data_to_save[k], "%.3f", volt_value);
           /* switch (esDC)
            {
            case 1:
                strcat(data_to_save[k]," DC<br>");
                break;
            
            case 0:
                strcat(data_to_save[k]," AC<br>");
                break;
            }*/
            data_to_save[k]=volt_value;
            k++;
        if(k>4)k=0;

        for (int i = 0; i < 5; i++) {
            printf("Datos a guardar\n: %.2f\n", data_to_save[i]);
        }

        // Guardar el arreglo en NVS
        ESP_ERROR_CHECK(nvs_set_blob(my_handle, "float_array", data_to_save, sizeof(data_to_save)));
        ESP_ERROR_CHECK(nvs_commit(my_handle)); // Asegurarse de que los datos se escriban
        nvs_close(my_handle);
        guardar_datos=false;
        } 

        if(habilitar_wifi==true){
            esp_wifi_start();
            habilitar_wifi=false;
        }
        if(bandera==true){
            esp_wifi_stop();
            bandera=false;
        }

        
        
    }
}
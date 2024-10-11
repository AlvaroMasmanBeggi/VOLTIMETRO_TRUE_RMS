#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <driver/adc.h>
#include <esp_system.h>
#include <math.h>
#include "driver/i2c.h"

//La llave de 5k6 es el rango 1, la de 82k el 2 y la de 1M el 3

//El pin de lectura DC/AC corresponde a ADC1_CHANNEL_7 y es el pin 35 de la esp

//Pines de control de rango
//Pin de 5k6: 12
//Pin de 82k: 14
//Pin de 1M: 27

//Pin de display
//SDA es el 21
//SCL es el 22

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

#define pin5k6 GPIO_NUM_12
#define pin82k GPIO_NUM_14
#define pin1M GPIO_NUM_27
#define DCACinput ADC1_CHANNEL_7
#define savePin GPIO_NUM_17
#define sendPin GPIO_NUM_16

#define DEFAULT_VREF 1100 
#define max_adc 3
#define min_adc 0.2

#define MAX_MUESTRAS 1000

int i = 0, valor_inst = 0, numero_muestrasAC = 1200, numero_muestrasDC = 100, rango_actual = 3;
float lectura = 0, volt_value = 0, valor_rms;

void config(){
    adc1_config_width(ADC_WIDTH_BIT_12);                         // Configura el ADC a 12 bits
    adc1_config_channel_atten(DCACinput, ADC_ATTEN_DB_12);  // Configura la atenuación para el canal 7 (GPIO 35)                        
    
    esp_rom_gpio_pad_select_gpio(pin5k6);
    gpio_set_direction(pin5k6,GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(pin82k);
    gpio_set_direction(pin82k,GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(pin1M);
    gpio_set_direction(pin1M,GPIO_MODE_OUTPUT);

    gpio_set_direction(savePin,GPIO_MODE_INPUT);
    gpio_set_pull_mode(savePin,GPIO_PULLDOWN_ONLY);
    gpio_set_direction(sendPin,GPIO_MODE_INPUT);
    gpio_set_pull_mode(sendPin,GPIO_PULLDOWN_ONLY);

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
    uint8_t upper_nibble = (data & 0xF0);  // Extraer los 4 bits altos
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
    lcd_send_nibble(command);         // Enviar 4 bits altos
    lcd_send_nibble(command << 4);    // Enviar 4 bits bajos
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

void autorango(float valor_actual){
    if(valor_actual>max_adc){
        switch(rango_actual){
            case 1:{
                printf("Rango máximo alcanzado\n");
                break;
            }
            case 2:{
                gpio_set_level(pin5k6,1);
                gpio_set_level(pin82k,0);
                rango_actual = 1;
                break;
            }
            case 3:{
                gpio_set_level(pin82k,1);
                gpio_set_level(pin1M,0);
                rango_actual = 2;
                break;
            }
        }   
    }
    
    if(valor_actual<min_adc){
        switch(rango_actual){
            case 1:{
                gpio_set_level(pin82k,1);
                gpio_set_level(pin5k6,0);
                rango_actual = 2;
                break;
            }
            case 2:{
                gpio_set_level(pin1M,1);
                gpio_set_level(pin82k,0);
                rango_actual = 3;
                break;
            }
            case 3:{
                printf("Ingrese tension mayor\n");
                break;
            }
        }
    } 
}

float calculo_voltaje(float dato){
    dato = ((dato * 3.3) / 4096) +0.11;   // Convierte la lectura del ADC a voltaje
    
    autorango(dato);

    switch(rango_actual){
        case 1:{
            break;
        }
        case 2: {
            break;
        }
        case 3:{
            break;
        }
    }
    return dato;
}

int deteccionAC(){
    //tipo tension es igual a 1 cuando detecta ac, sino es dc
    int tipo_tension=0,muestras=100;
    float valores[99], resta;

    for (i = 0; i < muestras; i++){
            lectura = adc1_get_raw(DCACinput);
            lectura = ((lectura * 3.3)/4096)+0.1;
            valores[i]=lectura;
            vTaskDelay(2 / portTICK_PERIOD_MS);        // Retardo para estabilizar
        }
    for(i=1;i < muestras; i++){
        resta=valores[0]-valores[i];
        if(resta<0){
            resta=resta*(-1);
        }
        if(resta>0.05){
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
        
    volt_value = calculo_voltaje(lectura);
    printf("Valor de voltaje DC en rango %d: %.3f\n", rango_actual,volt_value);

    vTaskDelay(3000 / portTICK_PERIOD_MS); // Espera antes de la próxima lectura   
}

void lectura_tensionAC(){
    lectura = 0;
    valor_inst = 0;
    for (i = 0; i < numero_muestrasAC; i++){
           lectura = adc1_get_raw(DCACinput);
           vTaskDelay(2 / portTICK_PERIOD_MS);        // Retardo para estabilizar
           valor_inst = lectura * lectura + valor_inst;
     }
    valor_inst /= numero_muestrasAC;
    valor_rms = sqrt(valor_inst);
    valor_rms = ((valor_rms * 3.3)/4096)+0.1;

    printf("Valor de voltaje AC en rango %d: %.3f\n", rango_actual,valor_rms);

    vTaskDelay(3000 / portTICK_PERIOD_MS); // Espera antes de la próxima lectura
}

void app_main() {
    config();           // Configuracion
    i2c_master_init();  // Inicializar I2C
    lcd_init();         // Inicializar LCD
   
}
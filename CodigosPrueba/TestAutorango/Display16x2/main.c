#include <stdio.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO    22    // Pin SCL del ESP32
#define I2C_MASTER_SDA_IO    21    // Pin SDA del ESP32
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   100000  // Frecuencia de 100kHz
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define LCD_I2C_ADDRESS      0x27   // Dirección I2C del adaptador PCF8574

// Comandos básicos para el LCD
#define LCD_CMD_CLEAR_DISPLAY 0x01
#define LCD_CMD_RETURN_HOME   0x02
#define LCD_CMD_ENTRY_MODE    0x06
#define LCD_CMD_DISPLAY_ON    0x0C
#define LCD_CMD_FUNCTION_SET  0x28  // Modo de 4 bits, 2 líneas

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

void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t row_offsets[] = { 0x00, 0x40 };  // Offsets de memoria para la primera y segunda fila
    if (row > 1) row = 1;  // El LCD 1602A tiene solo dos filas
    lcd_send_command(0x80 | (col + row_offsets[row]));  // Comando para mover el cursor
}

void app_main() {
    i2c_master_init();  // Inicializar I2C
    lcd_init();         // Inicializar LCD

    // Mover el cursor al inicio de la primera línea
    lcd_set_cursor(4, 0);

    // Mostrar "Hola Mundo"
    lcd_send_string("Dani  ");
}

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <driver/adc.h>
#include <esp_system.h>
#include <math.h>

#define EntradaBoton GPIO_NUM_23
#define EntradaTension GPIO_NUM_32

int i = 0;
int lectura = 0;
int numero_muestras = 64;
float volt_value = 0;
int valor_pico_inst=  0;

void config(){
    adc1_config_width(ADC_WIDTH_BIT_12);                         // Configura el ADC a 12 bits
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_12);  // Configura la atenuación para el canal 4 (GPIO 32)

    gpio_set_direction(EntradaBoton, GPIO_MODE_INPUT);
}

void lectura_tensionAC(){
    lectura = 0;
    for (i = 0; i < numero_muestras; i++){
           lectura = adc1_get_raw(ADC1_CHANNEL_4);
           vTaskDelay(10 / portTICK_PERIOD_MS);        // Retardo para estabilizar
           
     }
     lectura /= numero_muestras;


}

void lectura_tensionDC(){
    while (1) {
        // Lee el valor del ADC
        lectura = 0;
        for (i = 0; i < numero_muestras; i++){
            lectura += adc1_get_raw(ADC1_CHANNEL_4);
            vTaskDelay(10 / portTICK_PERIOD_MS);        // Retardo para estabilizar
        }
        lectura /= numero_muestras;

        // Convierte la lectura del ADC a voltaje
        volt_value = ((lectura * 3.3) / 4096) + 0.11;
        printf("Valor de voltaje: %.2f\n", volt_value);

        vTaskDelay(2000 / portTICK_PERIOD_MS); // Espera antes de la próxima lectura
    }
}

void app_main() {
    config();
    
    lectura_tensionDC();
    

}
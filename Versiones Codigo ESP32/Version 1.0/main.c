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
float lectura = 0;
int numero_muestras = 300;
float volt_value = 0;
float valor_rms, valor_pico;
int valor_inst = 0;

void config(){
    adc1_config_width(ADC_WIDTH_BIT_12);                         // Configura el ADC a 12 bits
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_12);  // Configura la atenuación para el canal 4 (GPIO 32)

    gpio_set_direction(EntradaBoton, GPIO_MODE_INPUT);
}

void lectura_tensionAC(){
    lectura = 0;
    valor_inst = 0;
    for (i = 0; i < numero_muestras; i++){
           lectura = ((adc1_get_raw(ADC1_CHANNEL_4)*3.3)/4096)+0.11;
           vTaskDelay(10 / portTICK_PERIOD_MS);        // Retardo para estabilizar
           valor_inst = lectura * lectura + valor_inst;
     }
    valor_inst /= numero_muestras;
    valor_rms = sqrt(valor_inst);
    valor_pico = valor_rms * 1.4142;

    printf("Valor RMS: %.2f\n", valor_rms);
    printf("Valor pico: %.2f\n", valor_pico);

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Espera antes de la próxima lectura
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

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Espera antes de la próxima lectura
    }
}

void app_main() {
    config();
    

}
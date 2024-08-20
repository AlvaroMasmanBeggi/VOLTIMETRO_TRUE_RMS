#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <driver/adc.h>
#include <esp_system.h>
#include <math.h>

//----------DECLARACIÓN DE VARIABLES------------//
#define EntradaBoton GPIO_NUM_23
#define EntradaTension GPIO_NUM_32

int i = 0;
float valor_max = 0;
float true_rms = 0;
int lectura = 0;
int numero_muestras = 64;
float volt_value = 0;
int valor_pico_inst=  0;

//----------CODIGO DE FUNCIONES-----------------//

void config(){
    adc1_config_width(ADC_WIDTH_BIT_12);                         // Configura el ADC a 12 bits
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_12);  // Configura la atenuación para el canal 4 (GPIO 32)

    gpio_set_direction(EntradaBoton, GPIO_MODE_INPUT); //Configura un boton como entrada digital
}

void lectura_tensionAC(){
    lectura = 0; 
    true_rms = 0;
    valor_max = 0;
    volt_value = 0;
    while(1){
    for(i = 0; i < numero_muestras; i++){
           lectura = adc1_get_raw(ADC1_CHANNEL_4);
           volt_value = ((lectura * 3.3) / 4096) + 0.1;
           valor_max = (volt_value*volt_value)+valor_max;
           vTaskDelay(60 / portTICK_PERIOD_MS);  
           lectura = 0;     
     }
     true_rms = sqrt(valor_max/numero_muestras);
     printf("True RMS: %.2f V\n",true_rms);
     vTaskDelay(2000 / portTICK_PERIOD_MS); // Espera antes de la próxima lectura
    }
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
        volt_value = ((lectura * 3.3) / 4096) + 0.09;
        printf("Valor de voltaje: %.2f V\n", volt_value);

        vTaskDelay(2000 / portTICK_PERIOD_MS); // Espera antes de la próxima lectura
        
    }
}

//----------PROGRAMA PRINCIPAL---------------//
void app_main() {
    config();
    

    lectura_tensionDC();

}
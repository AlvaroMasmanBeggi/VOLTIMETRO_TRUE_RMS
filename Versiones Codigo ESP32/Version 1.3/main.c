#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <driver/adc.h>
#include <esp_system.h>
#include <math.h>

//El pin de lectura DC corresponde a ADC1_CHANNEL_4 y es el pin 32 de la esp
//El pin de lectura AC corresponde a ADC1_CHANNEL_6 y es el pin 34 de la esp

#define control_DEMUX 
#define control_MUX_0 GPIO_NUM_25 
#define control_MUX_1 GPIO_NUM_27 

int i = 0;
float lectura = 0;
int numero_muestras = 300;
float volt_value = 0;
float valor_rms;
int valor_inst = 0;
int mediciones[100]; //Variables para determinar si es AC o DC
bool esDC;

void config(){
    adc1_config_width(ADC_WIDTH_BIT_12);                         // Configura el ADC a 12 bits
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_12);  // Configura la atenuación para el canal 4 (GPIO 32)                        
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_12);  // Configura la atenuación para el canal 6 (GPIO 34)
    gpio_set_direction(control_MUX_0,GPIO_MODE_OUTPUT);
    gpio_set_direction(control_MUX_1,GPIO_MODE_OUTPUT);

}

void lectura_tensionAC(){
    lectura = 0;
    valor_inst = 0;
    for (i = 0; i < numero_muestras; i++){
           lectura = ((adc1_get_raw(ADC1_CHANNEL_6)*3.3)/4096)+0.11;
           vTaskDelay(10 / portTICK_PERIOD_MS);        // Retardo para estabilizar
           valor_inst = lectura * lectura + valor_inst;
     }
    valor_inst /= numero_muestras;
    valor_rms = sqrt(valor_inst);

    printf("Valor RMS: %.2f\n", valor_rms);

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
        printf("Valor de voltaje DC: %.2f\n", volt_value);

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Espera antes de la próxima lectura
    }
}

bool determinacion_tension(){    //Determina si la entrada es AC o DC. Devuelve un true si es DC, y false si es AC
     // Lee el valor del ADC
     esDC = true;
     for (i = 0; i < 100; i++)
     {
         mediciones[i] = adc1_get_raw(ADC1_CHANNEL_4);
         vTaskDelay(10 / portTICK_PERIOD_MS); // Retardo para estabilizar
        }

    for (i = 1; i < 100; i++) {
    if (abs(mediciones[i] - mediciones[i-1]) > 13) {    //13 es la minima desviacion que debe tener la entrada para que sea reconocida como AC. 13 equivale a 10mV
        esDC = false;
        break;
    }
}
        return esDC;
}

void app_main() {
    config();
    while (1){
        lectura_tensionDC();
    }

}
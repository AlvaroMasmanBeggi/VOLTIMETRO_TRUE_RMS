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

#define control_MUX_0 GPIO_NUM_25 
#define control_MUX_1 GPIO_NUM_27 
#define output_disable GPIO_NUM_12

#define MAX_MUESTRAS 1000

int i = 0, valor_inst = 0,numero_muestras = 300, max = 3, min = 0.2;
float lectura = 0, volt_value = 0, valor_rms;

void config(){
    adc1_config_width(ADC_WIDTH_BIT_12);                         // Configura el ADC a 12 bits
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_12);  // Configura la atenuación para el canal 4 (GPIO 32)                        
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_12);  // Configura la atenuación para el canal 6 (GPIO 34)
    gpio_set_direction(control_MUX_0,GPIO_MODE_OUTPUT);
    gpio_set_direction(control_MUX_1,GPIO_MODE_OUTPUT);
    gpio_set_direction(output_disable,GPIO_MODE_OUTPUT);

    gpio_set_level(control_MUX_1,1);
    gpio_set_level(control_MUX_0,1);
    gpio_set_level(output_disable,1);
}

void autorango(float valor_actual){
    if(valor_actual<min){
        switch(gpio_get_level(control_MUX_0)){
        case 0: 
            gpio_set_level(control_MUX_0,1);
            break;

        case 1: 
            if(gpio_get_level(control_MUX_1)==1 && gpio_get_level(output_disable==0)) gpio_set_level(output_disable,1);
            if(gpio_get_level(control_MUX_1)==1 && gpio_get_level(output_disable==1)) printf("Ingrese una entrada mayor\n\n");
            if(gpio_get_level(control_MUX_1)==0) {
                gpio_set_level(control_MUX_0,0);
                gpio_set_level(control_MUX_1,1);
            }
            break;
        }
    }

    if(valor_actual>max){
        switch(gpio_get_level(output_disable)){
            case 0: 
              switch(gpio_get_level(control_MUX_0)){
                case 0:
                    if(gpio_get_level(control_MUX_1==1)){
                        gpio_set_level(control_MUX_0,1);
                        gpio_set_level(control_MUX_1,0);
                    }
                    else {
                        printf("Rango máximo alcanzado\n\n");
                    }
                    break;
        
                case 1: 
                    gpio_set_level(control_MUX_0,0);
                    break;
            }
            break;

        case 1: 
            gpio_set_level(output_disable,0);
            break;
        }
    }
}

void lectura_tensionAC(){
    lectura = 0;
    valor_inst = 0;
    for (i = 0; i < numero_muestras; i++){
           lectura = adc1_get_raw(ADC1_CHANNEL_6);
           vTaskDelay(5 / portTICK_PERIOD_MS);        // Retardo para estabilizar
           valor_inst = lectura * lectura + valor_inst;
     }
    valor_inst /= numero_muestras;
    valor_rms = sqrt(valor_inst);
    valor_rms = ((valor_rms * 3.3)/4096)+0.11;

    printf("Valor AC RMS: %.2f\n", valor_rms);

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Espera antes de la próxima lectura
}

void lectura_tensionDC(){
    while (1) {
        // Lee el valor del ADC
        lectura = 0;
        for (i = 0; i < numero_muestras; i++){
            lectura += adc1_get_raw(ADC1_CHANNEL_4);
          //  autorango(lectura);
            vTaskDelay(10 / portTICK_PERIOD_MS);        // Retardo para estabilizar
        }
        lectura /= numero_muestras;

        volt_value = ((lectura * 3.3) / 4096) + 0.11;   // Convierte la lectura del ADC a voltaje
        printf("Valor de voltaje DC: %.2f\n", volt_value);

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Espera antes de la próxima lectura
    }
}

float medirFrecuencia() {
    uint32_t num_transiciones = 0;
    uint32_t num_muestras = 1000; // Número de muestras para la medición de frecuencia
    int adc_val_anterior = adc1_get_raw(ADC1_CHANNEL_6);
    int adc_val_actual;
    
    for (i = 0; i < num_muestras; i++) {
        vTaskDelay(10 / portTICK_PERIOD_MS); // Esperar el intervalo de muestreo

        adc_val_actual = adc1_get_raw(ADC1_CHANNEL_6);

        // Detectar transición de alto a bajo o bajo a alto
        if ((adc_val_anterior < 2048 && adc_val_actual >= 2048) ||  (adc_val_anterior >= 2048 && adc_val_actual < 2048)) {
            num_transiciones++;
        }
        
        adc_val_anterior = adc_val_actual;
    }

    // Calcular la frecuencia en Hz
    float tiempo_medicion_segundos = num_muestras * 10 / 1000.0; // Tiempo en segundos
    float frecuencia = (float)num_transiciones / (4 * tiempo_medicion_segundos); // Cuatro transiciones por ciclo
    return frecuencia;
}

void app_main() {
    config();

    while (1){
        printf("%.2f",medirFrecuencia());
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}
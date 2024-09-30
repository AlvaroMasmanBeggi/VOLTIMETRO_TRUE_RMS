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

//Pines de control
//Pin de 56k es el 5
//Pin de 100k es el 18
//Pin de 560k es el 19
//Pin de 1M es el 21

//Pines de entrada AC

//Pin de entrada DC es el 34

//Pin de display
//SDA es el 25
//SCL es el 26

//Pines de interrupciones
//Guardar dato es el 23
//Enviar dato es el 22

#define SDA GPIO_NUM_25 
#define SCL GPIO_NUM_26 
#define pin56k GPIO_NUM_13
#define pin100k GPIO_NUM_12
#define pin560k GPIO_NUM_14
#define pin1M GPIO_NUM_27
#define DCinput ADC1_CHANNEL_6
#define ACinput ADC1_CHANNEL_5
#define savePin GPIO_NUM_23
#define sendPin GPIO_NUM_22

#define max_adc 3
#define min_adc 0.2

#define MAX_MUESTRAS 1000

int i = 0, valor_inst = 0, numero_muestras = 300, rango_actual=4;
float lectura = 0, volt_value = 0, valor_rms;

void config(){
    adc1_config_width(ADC_WIDTH_BIT_12);                         // Configura el ADC a 12 bits
    adc1_config_channel_atten(DCinput, ADC_ATTEN_DB_12);  // Configura la atenuación para el canal 6 (GPIO 34)                        
    adc1_config_channel_atten(ACinput, ADC_ATTEN_DB_12);  // Configura la atenuación para el canal 5 (GPIO 33)
    
    gpio_set_direction(SDA,GPIO_MODE_OUTPUT);
    gpio_set_direction(SCL,GPIO_MODE_OUTPUT);
    gpio_set_direction(pin56k,GPIO_MODE_OUTPUT);
    gpio_set_direction(pin100k,GPIO_MODE_OUTPUT);
    gpio_set_direction(pin560k,GPIO_MODE_OUTPUT);
    gpio_set_direction(pin1M,GPIO_MODE_OUTPUT);

    gpio_set_direction(savePin,GPIO_MODE_INPUT);
    gpio_set_pull_mode(savePin,GPIO_PULLDOWN_ONLY);
    gpio_set_direction(sendPin,GPIO_MODE_INPUT);
    gpio_set_pull_mode(sendPin,GPIO_PULLDOWN_ONLY);

    gpio_set_level(pin1M,1);
    gpio_set_level(pin560k,0);
    gpio_set_level(pin100k,0);
    gpio_set_level(pin56k,0);
}

void autorango(float valor_actual){
    if(valor_actual>max_adc){
       if(gpio_get_level(pin56k)){
           printf("Rango máximo alcanzado\n");
       }
       else if(gpio_get_level(pin100k)){
           gpio_set_level(pin56k,1);
           gpio_set_level(pin100k,0);
           rango_actual = 1;
       }
       else if(gpio_get_level(pin560k)){
           gpio_set_level(pin100k,1);
           gpio_set_level(pin560k,0);
           rango_actual = 2;
       }
       else {
           gpio_set_level(pin560k,1);
           gpio_set_level(pin1M,0);
           rango_actual = 3;
       }
    }
    
    if(valor_actual<min_adc){
        if(gpio_get_level(pin56k)){
           gpio_set_level(pin100k,1);
           gpio_set_level(pin56k,0);
           rango_actual = 2;
           
       }
       else if(gpio_get_level(pin100k)){
           gpio_set_level(pin560k,1);
           gpio_set_level(pin100k,0);
           rango_actual = 3;
       }
       else if(gpio_get_level(pin560k)){
           gpio_set_level(pin1M,1);
           gpio_set_level(pin560k,0);
           rango_actual = 4;
       }
       else {
           printf("Ingrese tension mayor\n");
       }
    }
}

float calculo_voltaje(int rango,float dato){
    float resultado=0;
    dato = ((dato * 3.3) / 4096);   // Convierte la lectura del ADC a voltaje
   //autorango(dato);
    switch(rango){
        case 1:{
            break;
        }
        case 2: {
            resultado = -12.6536 * pow(dato, 3) +24.12888 * pow(dato, 2) +26.13991 * dato +2.52276;
            break;
        }
        case 3:{
            resultado = 1.24089 * pow(dato, 3) -4.66145 * pow(dato, 2) +13.18033 * dato -1.87623;
            break;
        }
        case 4:{
            resultado = -1.2108 * pow(dato, 3) +9.1719 * pow(dato, 2) -4.3426 * dato + 1.2022;
            break;
        }
    }
    return resultado;
}

void lectura_tensionDC(){
    while (1) {
        // Lee el valor del ADC
        lectura = 0;
        for (i = 0; i < numero_muestras; i++){
            lectura += adc1_get_raw(DCinput);
            vTaskDelay(10 / portTICK_PERIOD_MS);        // Retardo para estabilizar
        }
        lectura /= numero_muestras;
        printf("Lectura antes del calculo %.2f\n", lectura);
        volt_value = calculo_voltaje(rango_actual,lectura);
        
        printf("Valor de voltaje DC en rango %d: %.2f\n", rango_actual, volt_value);

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Espera antes de la próxima lectura
    }
}

void lectura_tensionAC(){
    lectura = 0;
    valor_inst = 0;
    for (i = 0; i < numero_muestras; i++){
           lectura = adc1_get_raw(ACinput);
           vTaskDelay(5 / portTICK_PERIOD_MS);        // Retardo para estabilizar
           valor_inst = lectura * lectura + valor_inst;
     }
    valor_inst /= numero_muestras;
    valor_rms = sqrt(valor_inst);
    valor_rms = ((valor_rms * 3.3)/4096)+0.11;

    printf("Valor AC RMS: %.2f\n", valor_rms);

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Espera antes de la próxima lectura
}

void app_main() {
    config();
    lectura_tensionDC();
  
}
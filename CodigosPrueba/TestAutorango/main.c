#include <esp_system.h>
#include <driver/gpio.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define pinA GPIO_NUM_2
#define pinB GPIO_NUM_4
#define pinC GPIO_NUM_16
#define pinD GPIO_NUM_17

void app_main() {
    gpio_set_direction(pinA, GPIO_MODE_OUTPUT);
    gpio_set_direction(pinB,GPIO_MODE_OUTPUT);
    gpio_set_direction(pinC,GPIO_MODE_OUTPUT);
    gpio_set_direction(pinD,GPIO_MODE_OUTPUT);

    gpio_set_level(pinA,0);
    gpio_set_level(pinB,0);
    gpio_set_level(pinC,0);
    gpio_set_level(pinD,0);

    while(1){
        gpio_set_level(pinB,1);
        printf("Puerto B: 50mV\n");
        printf("%d, %d, %d, %d\n", gpio_get_level(pinA),gpio_get_level(pinB),gpio_get_level(pinC), gpio_get_level(pinD));
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        gpio_set_level(pinB,0);
        gpio_set_level(pinC,1);
        printf("Puerto C: 265mV\n");
         printf("%d, %d, %d, %d\n", gpio_get_level(pinA),gpio_get_level(pinB),gpio_get_level(pinC), gpio_get_level(pinD));
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        gpio_set_level(pinC,0);
        gpio_set_level(pinD,1);
        printf("Puerto D: 455mV\n");
         printf("%d, %d, %d, %d\n", gpio_get_level(pinA),gpio_get_level(pinB),gpio_get_level(pinC), gpio_get_level(pinD));
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        gpio_set_level(pinD,0);
        gpio_set_level(pinA,1);
        printf("Puerto A: 28mV\n");
         printf("%d, %d, %d, %d\n", gpio_get_level(pinA),gpio_get_level(pinB),gpio_get_level(pinC), gpio_get_level(pinD));
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        gpio_set_level(pinA,0);
    }
}
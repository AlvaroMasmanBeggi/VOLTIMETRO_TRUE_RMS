#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_timer.h"

void app_main() {
    // Obtener el tiempo de inicio
    int64_t startTime = esp_timer_get_time();
    printf("Inicio de la medición...\n");
    
    // Simular un retraso o una acción (por ejemplo, 2 segundos)
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    // Obtener el tiempo de fin
    int64_t endTime = esp_timer_get_time();
    printf("Fin de la medición...\n");
    
    // Calcular el tiempo transcurrido
    double elapsedTime = endTime - startTime;
    printf("Tiempo transcurrido: %.4f segundos\n", elapsedTime/1000000);
}
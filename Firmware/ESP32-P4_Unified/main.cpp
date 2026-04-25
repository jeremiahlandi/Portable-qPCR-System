#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void) {
    while (1) {
        printf("qPCR System Online - ESP32-P4 Heartbeat...\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
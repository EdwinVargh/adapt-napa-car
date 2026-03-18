#ifndef ARBITRATOR_H
#define ARBITRATOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdbool.h>
#include "motor_driver.h"
#include "esp_adc/adc_oneshot.h"

void car_control_task(void *pvParameters);

typedef struct {
    int16_t x;
    int16_t y;
    bool hard_override;
    bool emergency_stop;
    bool is_connected;
} supervisor_cmd_t;

extern QueueHandle_t supervisor_queue;

#endif // ARBITRATOR_H
// pir_sensor.h
// PIR motion sensor management for ESP32 Pumpkin

#ifndef PIR_SENSOR_H
#define PIR_SENSOR_H

#include "freertos/event_groups.h"
#include "esp_err.h"

/**
 * @brief Initialize PIR sensor GPIO
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pir_sensor_init(void);

/**
 * @brief Start PIR sensor monitoring task
 * @param event_group Event group for motion detection synchronization
 */
void pir_sensor_start_task(EventGroupHandle_t event_group);

#endif // PIR_SENSOR_H

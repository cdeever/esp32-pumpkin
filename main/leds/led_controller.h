// led_controller.h
// LED strip control for ESP32 Pumpkin

#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include "freertos/event_groups.h"
#include "esp_err.h"
#include "led_strip.h"

/**
 * @brief Initialize LED strip
 * @return LED strip handle, or NULL on error
 */
led_strip_handle_t led_controller_init(void);

/**
 * @brief Start unified LED effect task (handles both flame and strobe)
 * @param led_strip LED strip handle
 * @param event_group Event group for motion detection synchronization
 */
void led_controller_start_task(led_strip_handle_t led_strip,
                               EventGroupHandle_t event_group);

/**
 * @brief Clear all LEDs
 * @param led_strip LED strip handle
 */
void led_controller_clear(led_strip_handle_t led_strip);

#endif // LED_CONTROLLER_H

// audio_manager.h
// Audio playback management for ESP32 Pumpkin

#ifndef AUDIO_MANAGER_H
#define AUDIO_MANAGER_H

#include "freertos/event_groups.h"
#include "esp_err.h"

/**
 * @brief Initialize I2S audio subsystem
 *
 * Initializes the I2S peripheral with the audio sample selected
 * at compile time via menuconfig (idf.py menuconfig).
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t audio_manager_init(void);

/**
 * @brief Start audio playback task
 * @param event_group Event group for motion detection synchronization
 */
void audio_manager_start_task(EventGroupHandle_t event_group);

#endif // AUDIO_MANAGER_H

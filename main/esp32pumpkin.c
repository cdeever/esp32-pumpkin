#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "config.h"
#include "audio/audio_manager.h"
#include "sensors/pir_sensor.h"
#include "leds/led_controller.h"

// Event group for motion detection
EventGroupHandle_t xSystemEvents;
#define EVENT_MOTION_DETECTED (1 << 0)

void app_main() {

    ESP_LOGI(TAG_MAIN, "ESP32 Pumpkin starting...");

    // Create event group for motion detection
    xSystemEvents = xEventGroupCreate();
    if (xSystemEvents == NULL) {
        ESP_LOGE(TAG_MAIN, "Failed to create event group");
        return;
    }

    // Initialize PIR sensor
    esp_err_t ret = pir_sensor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Failed to initialize PIR sensor");
        return;
    }

    // Initialize audio subsystem
    ret = audio_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Failed to initialize audio manager");
        return;
    }

    // Audio sample is selected at compile time via menuconfig

    // Initialize LED strip
    led_strip_handle_t led_strip = led_controller_init();
    if (led_strip == NULL) {
        ESP_LOGE(TAG_MAIN, "Failed to initialize LED controller");
        return;
    }

    // Start all tasks
    pir_sensor_start_task(xSystemEvents);
    audio_manager_start_task(xSystemEvents);
    led_controller_start_task(led_strip, xSystemEvents);

    ESP_LOGI(TAG_MAIN, "All modules initialized - ESP32 Pumpkin ready!");
}

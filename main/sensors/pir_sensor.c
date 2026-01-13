// pir_sensor.c
// PIR motion sensor implementation with GPIO interrupt

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "config.h"
#include "pir_sensor.h"

#define EVENT_MOTION_DETECTED (1 << 0)

// Event group handle (passed from main)
static EventGroupHandle_t pir_event_group = NULL;

// Last trigger time for debouncing
static volatile uint32_t last_trigger_time = 0;

// ISR handler for PIR sensor - keep minimal work here
static void IRAM_ATTR pir_isr_handler(void* arg) {
    uint32_t now = xTaskGetTickCountFromISR();

    // Debounce check
    if ((now - last_trigger_time) > pdMS_TO_TICKS(CONFIG_PIR_DEBOUNCE_MS)) {
        last_trigger_time = now;

        // Set event bit from ISR
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xEventGroupSetBitsFromISR(pir_event_group, EVENT_MOTION_DETECTED,
                                  &xHigherPriorityTaskWoken);

        // Yield if needed
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

esp_err_t pir_sensor_init(void) {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_POSEDGE;   // Trigger on rising edge
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_GPIO_PIR_SENSOR);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_PIR, "Failed to configure PIR sensor GPIO: %s", esp_err_to_name(ret));
        return ret;
    }

    // Install GPIO ISR service
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE means service already installed (ok)
        ESP_LOGE(TAG_PIR, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        return ret;
    }

    // Add ISR handler for PIR sensor pin
    ret = gpio_isr_handler_add(CONFIG_GPIO_PIR_SENSOR, pir_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_PIR, "Failed to add ISR handler: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG_PIR, "PIR sensor initialized on GPIO %d with interrupt", CONFIG_GPIO_PIR_SENSOR);
    return ESP_OK;
}

void pir_sensor_start_task(EventGroupHandle_t event_group) {
    pir_event_group = event_group;
    ESP_LOGI(TAG_PIR, "PIR sensor ready (interrupt-driven)");
}

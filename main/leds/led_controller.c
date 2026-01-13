// led_controller.c
// LED strip control implementation with unified state machine

#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_err.h"

#include "led_strip.h"
#include "led_strip_rmt.h"

#include "config.h"
#include "led_controller.h"

#define EVENT_MOTION_DETECTED (1 << 0)

led_strip_handle_t led_controller_init(void) {
    led_strip_handle_t led_strip;

    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = CONFIG_GPIO_LED_STRIP,
        .max_leds = CONFIG_LED_COUNT,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = CONFIG_LED_RMT_RESOLUTION,
        .flags.with_dma = false,
    };

    esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_LED, "Failed to create LED strip: %s", esp_err_to_name(ret));
        return NULL;
    }

    ESP_LOGI(TAG_LED, "LED strip initialized on GPIO %d with %d LEDs", CONFIG_GPIO_LED_STRIP, CONFIG_LED_COUNT);
    return led_strip;
}

void led_controller_clear(led_strip_handle_t led_strip) {
    ESP_ERROR_CHECK(led_strip_clear(led_strip));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
}

typedef struct {
    led_strip_handle_t led_strip;
    EventGroupHandle_t event_group;
} led_task_params_t;

// Strobe effect helper function
static void run_strobe_effect(led_strip_handle_t led_strip) {
    int total_blinks = CONFIG_STROBE_DURATION_MS / (2 * CONFIG_STROBE_BLINK_MS);

    ESP_LOGI(TAG_STROBE, "Starting strobe effect");
    vTaskDelay(pdMS_TO_TICKS(CONFIG_STROBE_DELAY_MS));
    led_controller_clear(led_strip);

    for (int i = 0; i < total_blinks; i++) {
        // Turn the LEDs on with low brightness (white color)
        for (int j = 0; j < CONFIG_LED_COUNT; j++) {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, j, CONFIG_STROBE_BRIGHTNESS,
                                                CONFIG_STROBE_BRIGHTNESS, CONFIG_STROBE_BRIGHTNESS));
        }
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));
        vTaskDelay(pdMS_TO_TICKS(CONFIG_STROBE_BLINK_MS));

        // Turn the LEDs off
        led_controller_clear(led_strip);
        vTaskDelay(pdMS_TO_TICKS(CONFIG_STROBE_BLINK_MS));
    }
    ESP_LOGI(TAG_STROBE, "Strobe effect complete");
}

// Flame effect single frame helper function
static void render_flame_frame(led_strip_handle_t led_strip) {
    led_controller_clear(led_strip);

    // Create a flickering effect by adjusting brightness and color
    for (int i = 0; i < CONFIG_LED_COUNT; i++) {
        int red = CONFIG_FLAME_BASE_RED + (rand() % CONFIG_FLAME_FLICKER_RANGE - CONFIG_FLAME_FLICKER_RANGE / 2);
        int green = CONFIG_FLAME_BASE_GREEN + (rand() % (CONFIG_FLAME_FLICKER_RANGE / 2) - CONFIG_FLAME_FLICKER_RANGE / 4);
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, green, red, 0));
    }
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
}

// Unified LED state machine task
static void led_state_machine_task(void *arg) {
    led_task_params_t *params = (led_task_params_t *)arg;
    led_strip_handle_t led_strip = params->led_strip;
    EventGroupHandle_t event_group = params->event_group;

    ESP_LOGI(TAG_LED, "LED state machine task started");

    while (1) {
        // Check for motion detection event
        EventBits_t bits = xEventGroupWaitBits(event_group, EVENT_MOTION_DETECTED,
                                               pdFALSE, pdFALSE, pdMS_TO_TICKS(50));

        if (bits & EVENT_MOTION_DETECTED) {
            // Motion detected - run strobe effect
            run_strobe_effect(led_strip);
            // Clear the motion bit after strobe completes
            xEventGroupClearBits(event_group, EVENT_MOTION_DETECTED);
        } else {
            // No motion - render single flame frame
            render_flame_frame(led_strip);
            // Add randomized delay for natural flickering
            vTaskDelay(pdMS_TO_TICKS(CONFIG_FLAME_BASE_DELAY_MS + (rand() % CONFIG_FLAME_RANDOM_DELAY_MS)));
        }
    }
}

void led_controller_start_task(led_strip_handle_t led_strip,
                               EventGroupHandle_t event_group) {
    static led_task_params_t params;
    params.led_strip = led_strip;
    params.event_group = event_group;

    BaseType_t xReturned = xTaskCreate(led_state_machine_task, "led_state_machine",
                                       CONFIG_TASK_STACK_LED_FLAME, &params,
                                       CONFIG_TASK_PRIORITY_LED, NULL);
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG_LED, "Failed to create LED state machine task");
    } else {
        ESP_LOGI(TAG_LED, "LED state machine task created successfully");
    }
}

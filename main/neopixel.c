// neopixel.c

#include <stdio.h>
#include <stdlib.h> 

#include "driver/rmt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"

#include "neopixel.h"

#define NUM_LEDS 10  // We're using 10 LEDs

#define LED_STRIP_GPIO_PIN    18     // GPIO pin where the NeoPixel string is connected
#define LED_STRIP_NUM_LEDS    10     // Number of LEDs in your NeoPixel string

extern volatile int motion_detected;
extern SemaphoreHandle_t xLedMutex;

// Initialize the NeoPixel LED strip with the given GPIO pin and brightness
led_strip_handle_t neopixel_init() {
    //gpio_num_t gpio_pin, uint8_t brightness

    led_strip_handle_t led_strip;

    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = 39, // The GPIO that connected to the LED strip's data line
        .max_leds = 10, // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812, // LED strip model
        .flags.invert_out = false, // whether to invert the output signal (useful when your hardware has a level inverter)
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // different clock source can lead to different power consumption
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false, // whether to enable the DMA feature
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    return led_strip;
}


void strobe_lights_task(void *arg) {
    led_strip_handle_t led_strip = (led_strip_handle_t)arg;

    int blink_duration = 50;  // Blink on/off every 200 milliseconds (adjust for speed)
    int total_blinks = 3000 / (2 * blink_duration);  // Total number of blinks in 3 seconds (on + off counts as one blink)

    while (1) {
        if (motion_detected >= 1 && 
            xSemaphoreTake(xLedMutex, portMAX_DELAY) == pdTRUE)  {

            vTaskDelay(pdMS_TO_TICKS(500)); // slight delay to wait for sound to catch up
            clear_lights(led_strip);

            for (int i = 0; i < total_blinks; i++) {
                // Turn the LEDs on with low brightness (white color)
                for (int j = 0; j < 10; j++) {
                    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, j, 5, 5, 5));  // Low brightness white
                }
                ESP_ERROR_CHECK(led_strip_refresh(led_strip));  // Send data to the LED strip
        
                vTaskDelay(pdMS_TO_TICKS(blink_duration));  // Wait for blink_duration (200 ms)

                // Turn the LEDs off
                clear_lights(led_strip);
        
                vTaskDelay(pdMS_TO_TICKS(blink_duration));  // Wait for blink_duration (200 ms)
            }
            xSemaphoreGive(xLedMutex);
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void show_flame_task(void *arg) {
    led_strip_handle_t led_strip = (led_strip_handle_t)arg;

    int base_red = 80;
    int base_green = 10;
    int flicker_range = 30;

    while (1) {
        if (motion_detected == 0 && 
            xSemaphoreTake(xLedMutex, portMAX_DELAY) == pdTRUE) {
            clear_lights(led_strip);

            // Create a flickering effect by adjusting brightness and color
            for (int i = 0; i < 10; i++) {
                // Randomize brightness between a safer range to reduce power consumption
        //     int brightness = rand() % 50 + 100;  // Random brightness from 100 to 150
                
                int red = base_red + (rand() % flicker_range - flicker_range / 2);
                int green = base_green + (rand() % (flicker_range / 2) - flicker_range / 4);
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, green, red, 0));        
            }
            // Refresh the strip to update the LEDs
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));

            xSemaphoreGive(xLedMutex);

            // Add a small random delay to mimic natural flickering
            vTaskDelay(pdMS_TO_TICKS(50 + (rand() % 50)));  // Random delay between 50 and 100 ms

        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void clear_lights(led_strip_handle_t led_strip) {

    ESP_ERROR_CHECK(led_strip_clear(led_strip));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));

}
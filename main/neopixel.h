// neopixel.h

#ifndef NEOPIXEL_H
#define NEOPIXEL_H

#include <stdint.h>
#include "driver/gpio.h"

#include "led_strip.h"
#include "led_strip_interface.h"
#include "led_strip_rmt.h"

// Initialize the NeoPixel LED strip with a specific GPIO pin and brightness level
led_strip_handle_t neopixel_init();

void strobe_lights_task(void *arg);

void show_flame_task(void *arg); 

void clear_lights(led_strip_handle_t led_strip);

#endif  // NEOPIXEL_H

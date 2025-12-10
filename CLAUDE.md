# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an ESP32-based Halloween pumpkin project that creates animated LED flame effects and plays audio when motion is detected. The project uses FreeRTOS tasks for concurrent operation of motion detection, audio playback, and LED control.

**Target Hardware**: ESP32-S2 (configured in sdkconfig)
**Framework**: ESP-IDF (Espressif IoT Development Framework)

## Build & Flash Commands

```bash
# Configure the project (opens menuconfig)
idf.py menuconfig

# Build the project
idf.py build

# Flash to ESP32
idf.py flash

# Monitor serial output
idf.py monitor

# Flash and immediately start monitoring
idf.py flash monitor

# Clean build artifacts
idf.py fullclean
```

## Architecture Overview

### FreeRTOS Task Structure

The application uses 4 concurrent FreeRTOS tasks orchestrated in `main/esp32pumpkin.c`:

1. **check_pir_sensor_task** (esp32pumpkin.c:92): Continuously polls PIR sensor on GPIO 38, sets motion_detected flag when triggered, implements debouncing with 20-second cooldown
2. **play_audio_task** (esp32pumpkin.c:73): Monitors motion_detected flag and streams PCM audio data via I2S when motion occurs
3. **show_flame_task** (neopixel.c:86): Renders idle flame animation with randomized red/orange flickering when no motion detected
4. **strobe_lights_task** (neopixel.c:52): Triggers white strobe effect on motion detection

### Synchronization Mechanisms

- **Shared State**: Global volatile `motion_detected` flag for communication between PIR sensor task and LED/audio tasks
- **Mutex Protection**: `xLedMutex` semaphore ensures flame and strobe tasks don't access LED strip simultaneously
- **Task Coordination**:
  - Flame task runs when `motion_detected == 0` (idle state)
  - Strobe task runs when `motion_detected >= 1` (triggered state)
  - Both tasks check motion flag and attempt to acquire mutex before LED operations

### Hardware Interfaces

**I2S Audio** (esp32pumpkin.c:33):
- GPIO 37: Bit Clock (BCK)
- GPIO 33: Word Select (WS/LR Clock)
- GPIO 34: Data Out (DO)
- Sample rate from audio header file
- 16-bit mono output, 8 DMA buffers of 1024 bytes

**WS2812B LED Strip** (neopixel.c:26):
- GPIO 39: Data line
- 10 LEDs configured via RMT driver
- GRB pixel format, 10MHz RMT resolution

**PIR Motion Sensor** (esp32pumpkin.c:63):
- GPIO 38: Motion detection input

### Audio Sample System

Audio files are converted to C header files containing PCM data arrays. Each header (e.g., `132806__nanakisan__evil-laugh-12.h`) exports:
```c
extern const unsigned int sampleRate;
extern const unsigned int sampleCount;
extern const signed char samples[];  // 8-bit PCM data
```

Switch audio by changing the include in esp32pumpkin.c:12. Multiple pre-converted samples available in main/ directory.

## Code Organization

- **main/esp32pumpkin.c**: Application entry point (app_main), I2S initialization, PIR sensor setup, audio playback task
- **main/neopixel.c**: LED strip initialization, flame animation algorithm, strobe effect implementation
- **main/neopixel.h**: Public interface for LED functions
- **main/*.h**: Pre-converted audio sample headers (8-bit PCM arrays)
- **CMakeLists.txt**: Root project configuration
- **main/CMakeLists.txt**: Main component registration, links esp_driver_i2s, esp_driver_gpio, driver components
- **sdkconfig**: ESP-IDF configuration (ESP32-S2 target, 2MB flash)

## Development Notes

### Managing Audio Files

To add new audio samples:
1. Convert audio to header using Bitluni's utility: https://bitluni.net/wp-content/uploads/2018/01/Audio2Header.html
2. Place .h file in main/ directory
3. Update include in esp32pumpkin.c:11-12
4. Rebuild project

### GPIO Pin Definitions

All GPIO assignments are hardcoded in esp32pumpkin.c and neopixel.c. Current configuration:
- I2S: GPIOs 37, 33, 34
- PIR: GPIO 38
- LED: GPIO 39

Note: Pin definitions appear in multiple locations (PIR_IO, RGB_IO macros vs neopixel_init hardcoded value). Verify consistency when modifying.

### LED Effect Parameters

Flame effect tuning (neopixel.c:86-91):
- `base_red`: Base red channel value (default: 80)
- `base_green`: Base green channel value (default: 10)
- `flicker_range`: Randomization range for color variation (default: 30)
- Refresh rate: 50-100ms random delay

Strobe effect tuning (neopixel.c:52-56):
- `blink_duration`: On/off cycle time (default: 50ms)
- Total duration: 3 seconds
- White color: RGB(5,5,5) - intentionally low brightness

### Motion Detection Behavior

PIR sensor logic (esp32pumpkin.c:92):
1. Polls GPIO every 1000ms when idle
2. On detection: triggers for 20ms, then resets flag
3. 20-second cooldown before next detection possible
4. Audio and strobe tasks check flag independently

### Known Configuration Details

- Project uses managed component: espressif__led_strip (in managed_components/)
- ESP32-S2 target with 2MB flash
- No custom Cursor or Copilot rules present

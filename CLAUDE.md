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

Audio files are converted to C header files containing PCM data arrays. Each header exports:
```c
extern const unsigned int sampleRate;
extern const unsigned int sampleCount;
extern const signed char samples[];  // 8-bit PCM data
```

**Audio samples are selected at compile time via menuconfig.** Only the selected sample is compiled into the binary, significantly reducing flash usage.

Available audio samples (in `main/audio/samples/`):
- **Evil Laugh** (795KB) - default, classic Halloween sound
- **Scream #14** (841KB) - high-pitched scream
- **Angry Beast** (2.2MB) - growling beast, longest duration
- **Zombie Growl** (556KB) - smallest file
- **Ghost Moan** (2.8MB) - largest file
- **Alien Snarl** (576KB) - alien creature sound
- **Generic Scream** (720KB) - general scream effect

To change the audio sample:
```bash
idf.py menuconfig
# Navigate to: Component config -> ESP32 Pumpkin Audio Configuration
# Select desired audio sample
idf.py fullclean build flash
```

**Important**: Always run `idf.py fullclean` after changing the audio selection in menuconfig to ensure the correct sample is compiled.

The audio playback task converts 8-bit signed samples to 16-bit by left-shifting 8 bits before sending to I2S.

## Code Organization

- **main/esp32pumpkin.c**: Application entry point (app_main), initializes all subsystems and starts tasks
- **main/audio/audio_manager.c**: I2S initialization, audio playback task, compile-time sample selection
- **main/audio/audio_manager.h**: Public interface for audio management
- **main/audio/samples/*.h**: Pre-converted audio sample headers (8-bit PCM arrays)
- **main/sensors/pir_sensor.c**: PIR motion sensor handling
- **main/leds/led_controller.c**: LED strip control, flame and strobe effects
- **main/config.h**: Shared configuration constants and GPIO pin definitions
- **main/Kconfig.projbuild**: Menuconfig options for audio sample selection
- **CMakeLists.txt**: Root project configuration
- **main/CMakeLists.txt**: Main component registration, links esp_driver_i2s, esp_driver_gpio, driver components
- **sdkconfig**: ESP-IDF configuration (ESP32-S2 target, 2MB flash)

## Development Notes

### Managing Audio Files

**Selecting an audio sample:**
1. Run `idf.py menuconfig`
2. Navigate to: Component config → ESP32 Pumpkin Audio Configuration
3. Select desired audio sample
4. Save configuration and exit
5. Run `idf.py fullclean build` to compile with the new sample

**Adding new audio samples:**
1. Convert audio to header using Bitluni's utility: https://bitluni.net/wp-content/uploads/2018/01/Audio2Header.html
2. Place .h file in `main/audio/samples/` directory
3. Edit `main/Kconfig.projbuild`:
   - Add new `config PUMPKIN_AUDIO_<NAME>` option in the choice block
   - Include file size and description
4. Edit `main/audio/audio_manager.c`:
   - Add new `#elif defined(CONFIG_PUMPKIN_AUDIO_<NAME>)` case (around line 18-30)
   - Include the new header file
5. Run `idf.py menuconfig` to select the new sample
6. Rebuild project with `idf.py fullclean build`

### GPIO Pin Definitions

All GPIO assignments are hardcoded in esp32pumpkin.c and neopixel.c. Current configuration:
- I2S: GPIOs 37 (BCK), 33 (WS), 34 (DO)
- PIR: GPIO 38
- LED: GPIO 39

**Important**: Pin definitions appear in multiple locations:
- esp32pumpkin.c:24-30 defines macros for I2S_BCK_IO, I2S_WS_IO, I2S_DO_IO, PIR_IO, RGB_IO
- neopixel.c:33 hardcodes GPIO 39 in led_strip_config_t
- neopixel.c:19-20 has unused LED_STRIP_GPIO_PIN macro (value 18)

When modifying GPIO assignments, update all locations to maintain consistency.

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

### Component Dependencies

The main component (main/CMakeLists.txt:4) requires:
- `esp_driver_i2s`: I2S audio peripheral driver
- `esp_driver_gpio`: GPIO control
- `driver`: General driver support

The project uses managed component `espressif__led_strip` for WS2812B LED control via RMT peripheral.

### Important Implementation Details

**Task Priorities**: All tasks created with priority 5 (esp32pumpkin.c:116-123). Tasks run cooperatively based on delays and mutex availability.

**Audio Playback**: The play_audio_task runs continuously but only outputs samples when motion_detected >= 1. After playing once, it delays 100ms and checks flag again (esp32pumpkin.c:89).

**LED Mutex Pattern**: Both flame and strobe tasks check motion_detected AND acquire xLedMutex before proceeding. This ensures only one effect runs at a time. Tasks that fail to acquire mutex wait 100ms and retry.

**Pixel Format**: LEDs use GRB format (not RGB). When calling led_strip_set_pixel(), parameters are (strip, index, G, R, B) - note the order in neopixel.c:68 and neopixel.c:105.

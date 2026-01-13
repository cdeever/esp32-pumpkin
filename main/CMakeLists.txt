
# Collect all source files from modules
file(GLOB AUDIO_SOURCES "audio/*.c")
file(GLOB SENSOR_SOURCES "sensors/*.c")
file(GLOB LED_SOURCES "leds/*.c")

# Register main component with all modules
idf_component_register(
    SRCS
        "esp32pumpkin.c"
        ${AUDIO_SOURCES}
        ${SENSOR_SOURCES}
        ${LED_SOURCES}
    INCLUDE_DIRS
        "."
        "audio"
        "sensors"
        "leds"
    PRIV_REQUIRES
        esp_driver_i2s
        esp_driver_gpio
        driver
)
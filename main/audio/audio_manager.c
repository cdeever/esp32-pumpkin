// audio_manager.c
// Audio playback management implementation

#include "driver/i2s.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "config.h"
#include "audio_manager.h"

#define EVENT_MOTION_DETECTED (1 << 0)

// Include only the selected audio sample at compile time
#if defined(CONFIG_PUMPKIN_AUDIO_EVIL_LAUGH)
    #include "samples/evil_laugh.h"
#elif defined(CONFIG_PUMPKIN_AUDIO_SCREAM)
    #include "samples/scream14.h"
#elif defined(CONFIG_PUMPKIN_AUDIO_ANGRY_BEAST)
    #include "samples/angry_beast.h"
#elif defined(CONFIG_PUMPKIN_AUDIO_ZOMBIE_GROWL)
    #include "samples/zombie_growl.h"
#elif defined(CONFIG_PUMPKIN_AUDIO_GHOST_MOAN)
    #include "samples/ghost_moan.h"
#elif defined(CONFIG_PUMPKIN_AUDIO_ALIEN_SNARL)
    #include "samples/alien_snarl.h"
#elif defined(CONFIG_PUMPKIN_AUDIO_GENERIC_SCREAM)
    #include "samples/generic_scream.h"
#else
    #error "No audio sample selected in menuconfig! Run 'idf.py menuconfig' to select an audio sample."
#endif

// Event group handle (passed from main)
static EventGroupHandle_t audio_event_group = NULL;

esp_err_t audio_manager_init(void) {
    // I2S driver configuration
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = sampleRate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = CONFIG_I2S_DMA_BUF_COUNT,
        .dma_buf_len = CONFIG_I2S_DMA_BUF_LEN,
        .use_apll = false,
        .tx_desc_auto_clear = true,
    };

    // I2S pin configuration
    i2s_pin_config_t pin_config = {
        .bck_io_num = CONFIG_GPIO_I2S_BCK,
        .ws_io_num = CONFIG_GPIO_I2S_WS,
        .data_out_num = CONFIG_GPIO_I2S_DO,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    // Install I2S driver
    esp_err_t ret = i2s_driver_install(CONFIG_I2S_PORT_NUM, &i2s_config, 0, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_AUDIO, "Failed to install I2S driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2s_set_pin(CONFIG_I2S_PORT_NUM, &pin_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_AUDIO, "Failed to set I2S pins: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2s_set_clk(CONFIG_I2S_PORT_NUM, sampleRate, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_AUDIO, "Failed to set I2S clock: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG_AUDIO, "I2S initialized successfully at %u Hz", sampleRate);
    return ESP_OK;
}

static void audio_playback_task(void *pvParameter) {
    int16_t sample16;
    size_t bytes_written;

    while (1) {
        // Wait for motion detection event
        EventBits_t bits = xEventGroupWaitBits(audio_event_group, EVENT_MOTION_DETECTED,
                                               pdTRUE, pdFALSE, pdMS_TO_TICKS(100));

        if (bits & EVENT_MOTION_DETECTED) {
            ESP_LOGI(TAG_AUDIO, "Playing audio sample (rate: %u Hz, count: %u)...",
                     sampleRate, sampleCount);

            for (unsigned int i = 0; i < sampleCount; i++) {
                // Convert 8-bit signed sample to 16-bit signed (centered around 0)
                sample16 = (int16_t)samples[i] << 8;

                // Send the sample to the I2S peripheral
                i2s_write(CONFIG_I2S_PORT_NUM, &sample16, sizeof(sample16), &bytes_written, portMAX_DELAY);
            }
            ESP_LOGI(TAG_AUDIO, "Audio playback complete");
        }
    }
}

void audio_manager_start_task(EventGroupHandle_t event_group) {
    audio_event_group = event_group;

    BaseType_t xReturned = xTaskCreate(audio_playback_task, "audio_playback",
                                       CONFIG_TASK_STACK_AUDIO, NULL,
                                       CONFIG_TASK_PRIORITY_AUDIO, NULL);
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG_AUDIO, "Failed to create audio playback task");
    } else {
        ESP_LOGI(TAG_AUDIO, "Audio playback task created successfully");
    }
}

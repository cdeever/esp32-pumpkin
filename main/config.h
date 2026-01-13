// config.h
// Central configuration file for ESP32 Pumpkin project

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// GPIO Pin Assignments
// ============================================================================
#define CONFIG_GPIO_I2S_BCK        (37)
#define CONFIG_GPIO_I2S_WS         (33)
#define CONFIG_GPIO_I2S_DO         (34)
#define CONFIG_GPIO_PIR_SENSOR     (38)
#define CONFIG_GPIO_LED_STRIP      (39)

// ============================================================================
// I2S Configuration
// ============================================================================
#define CONFIG_I2S_PORT_NUM        (0)
#define CONFIG_I2S_DMA_BUF_COUNT   (8)
#define CONFIG_I2S_DMA_BUF_LEN     (1024)

// ============================================================================
// LED Configuration
// ============================================================================
#define CONFIG_LED_COUNT           (10)
#define CONFIG_LED_RMT_RESOLUTION  (10000000)  // 10MHz

// ============================================================================
// Timing Constants (milliseconds)
// ============================================================================
#define CONFIG_PIR_DEBOUNCE_MS     (20000)  // 20 second debounce
#define CONFIG_PIR_POLL_INTERVAL_MS (1000)   // 1 second poll interval
#define CONFIG_STROBE_DURATION_MS  (3000)   // 3 second strobe
#define CONFIG_STROBE_BLINK_MS     (50)     // 50ms blink duration
#define CONFIG_STROBE_DELAY_MS     (500)    // Delay before strobe starts
#define CONFIG_FLAME_BASE_DELAY_MS (50)     // Base delay for flame effect
#define CONFIG_FLAME_RANDOM_DELAY_MS (50)   // Random additional delay for flame

// ============================================================================
// LED Effect Parameters
// ============================================================================
#define CONFIG_FLAME_BASE_RED      (80)
#define CONFIG_FLAME_BASE_GREEN    (10)
#define CONFIG_FLAME_FLICKER_RANGE (30)
#define CONFIG_STROBE_BRIGHTNESS   (5)      // RGB value for strobe

// ============================================================================
// Task Stack Sizes (bytes)
// ============================================================================
#define CONFIG_TASK_STACK_PIR      (2048)
#define CONFIG_TASK_STACK_AUDIO    (2048)
#define CONFIG_TASK_STACK_LED_STROBE (2048)
#define CONFIG_TASK_STACK_LED_FLAME  (2048)

// ============================================================================
// Task Priorities
// ============================================================================
#define CONFIG_TASK_PRIORITY_PIR   (5)
#define CONFIG_TASK_PRIORITY_AUDIO (5)
#define CONFIG_TASK_PRIORITY_LED   (5)

// ============================================================================
// Logging Tags
// ============================================================================
#define TAG_MAIN     "MAIN"
#define TAG_PIR      "PIR"
#define TAG_AUDIO    "AUDIO"
#define TAG_LED      "LED"
#define TAG_STROBE   "STROBE"
#define TAG_FLAME    "FLAME"

#endif // CONFIG_H

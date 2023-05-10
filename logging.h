//
// GTM LAB - goTenna Mesh protocol playground
// ------------------------------------------
// Copyright 2021-2023 by https://github.com/sybip (gpg 0x8295E0C0)
// Released under MIT license (see LICENSE file for full details)
//

#ifndef LOGGING_H
#define LOGGING_H

#ifdef ARDUINO_ARCH_ESP32
// ESP logging library and macros used below
// https://github.com/espressif/ESP8266_RTOS_SDK/tree/master/components/log
// If compiling for non-ESP platform, port the library from
// (or write your own macros if preferred)

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE  // DO NOT EDIT THIS LINE
#include "esp_log.h"

#define LOG_ ESP_LOG_LEVEL_LOCAL
#define HEXD(tag, buf, len) ESP_LOG_BUFFER_HEXDUMP(tag, buf, len, ESP_LOG_DEBUG)
#define HEXV(tag, buf, len) ESP_LOG_BUFFER_HEXDUMP(tag, buf, len, ESP_LOG_VERBOSE)

#else  // ARDUINO_ARCH_ESP32
// we need these to be consistent
typedef enum {
    ESP_LOG_NONE = 0,
    ESP_LOG_ERROR,
    ESP_LOG_WARN,
    ESP_LOG_INFO,
    ESP_LOG_DEBUG,
    ESP_LOG_VERBOSE,
    ESP_LOG_MAX
} esp_log_level_t;

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE  // DO NOT EDIT THIS LINE

// Define these for your NON-ESP platform
#define LOG_(level, TAG, format, ...) if (level <= logLevel) printf("%d (%d) %s: " format "\n", level, millis(), TAG, ##__VA_ARGS__)
#define HEXV
#define HEXD

// not needed
#define esp_log_level_set

#endif  // ARDUINO_ARCH_ESP32

// Logging macros, common to ESP and non-ESP
#define LOGE( format, ... ) LOG_(ESP_LOG_ERROR,   TAG, format, ##__VA_ARGS__)
#define LOGW( format, ... ) LOG_(ESP_LOG_WARN,    TAG, format, ##__VA_ARGS__)
#define LOGI( format, ... ) LOG_(ESP_LOG_INFO,    TAG, format, ##__VA_ARGS__)
#define LOGD( format, ... ) LOG_(ESP_LOG_DEBUG,   TAG, format, ##__VA_ARGS__)
#define LOGV( format, ... ) LOG_(ESP_LOG_VERBOSE, TAG, format, ##__VA_ARGS__)

// Binary output macros
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

#endif  // LOGGING_H

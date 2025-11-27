#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h" // multitasking
#include "freertos/task.h"
#include "driver/uart.h" // UART Ports
#include "driver/gpio.h" // Control GPIO pins
#include "esp_log.h" // Log info
#include "esp_sleep.h" // Power saving

#define TAG "GPS_LORA"

// UART definitions
#define GPS_UART_NUM      UART_2
#define GPS_UART_RX_PIN   23
// #define GPS_UART_TX_PIN   21  // not used

#define LORA_UART_NUM     UART_0
#define LORA_UART_RX_PIN  17
#define LORA_UART_TX_PIN  16

#define LORA_RST_PIN      4

#define UART_BUF_SIZE     1024 // how much data can be stored while reading from UART
#define RESPONSE_TIMEOUT  2000 // ms

// --- Helper functions ---

// Send AT command and wait for response
// Ask the LoRa to do something and wait for expected reply

bool send_at_command(const char* cmd, const char* expected) {

    uart_write_bytes(LORA_UART_NUM, cmd, strlen(cmd));
    uart_write_bytes(LORA_UART_NUM, "\r\n", 2);
    ESP_LOGI(TAG, "Sent: %s", cmd);

    uint8_t buf[128];
    int len;
    uint32_t start = (uint32_t)esp_timer_get_time() / 1000; // ms
    while (((uint32_t)esp_timer_get_time() / 1000 - start) < RESPONSE_TIMEOUT) {
        len = uart_read_bytes(LORA_UART_NUM, buf, sizeof(buf) - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            buf[len] = 0;
            ESP_LOGI(TAG, "Received: %s", buf);
            if (strstr((char*)buf, expected) != NULL) return true;
        }
    }
    ESP_LOGE(TAG, "Timeout or unexpected response");
    return false;
}

// Read full NMEA sentence from GPS
// Look at GPS Calcs and grab complete location sentences

bool read_gps_sentence(char* buffer, size_t max_len) {
    size_t idx = 0;
    char ch;
    uint32_t start = (uint32_t)esp_timer_get_time() / 1000;
    while ((uint32_t)esp_timer_get_time() / 1000 - start < 2000) { // 2s timeout
        if (uart_read_bytes(GPS_UART_NUM, (uint8_t*)&ch, 1, pdMS_TO_TICKS(100)) > 0) {
            if (ch == '\n') {
                buffer[idx] = 0;
                if (strncmp(buffer, "$GPGGA", 6) == 0 || strncmp(buffer, "$GPRMC", 6) == 0) {
                    return true;
                }
                idx = 0; // reset for next sentence
            } else if (ch != '\r' && idx < max_len - 1) {
                buffer[idx++] = ch;
            }
        }
    }
    return false;
}

// Parse NMEA GGA latitude/longitude
// Take the GPS string and convert it into numbers the computer can understand

bool parse_gga(const char* nmea, float* latitude, float* longitude) {
    char parts[15][16];
    int idx = 0;
    const char* start = nmea;
    for (int i = 0; i < 15; i++) memset(parts[i], 0, sizeof(parts[i]));

    for (const char* p = nmea; *p; p++) {
        if (*p == ',' || *(p+1) == 0) {
            size_t len = p - start + (*(p+1)==0 ? 1 : 0);
            if (len >= sizeof(parts[idx])) len = sizeof(parts[idx])-1;
            strncpy(parts[idx], start, len);
            parts[idx][len] = 0;
            idx++;
            start = p + 1;
        }
    }
    if (idx < 6) return false;

    float lat = atof(parts[2]);
    float lon = atof(parts[4]);
    if (lat == 0 || lon == 0) return false;

    int lat_deg = (int)(lat / 100);
    int lon_deg = (int)(lon / 100);
    float lat_min = lat - lat_deg*100;
    float lon_min = lon - lon_deg*100;

    *latitude = lat_deg + lat_min/60.0f;
    *longitude = lon_deg + lon_min/60.0f;

    if (parts[3][0]=='S') *latitude *= -1;
    if (parts[5][0]=='W') *longitude *= -1;

    return true;
}

// Convert float to string with 5 decimals
// Turn a number into a text string to send over LoRa.

void format_coord(float val, char* buf, size_t len) {
    snprintf(buf, len, "%.5f", val);
}

// --- Main application ---
void app_main(void) {

    ESP_LOGI(TAG, "ESP32-C6 GPS->LoRa Startup");

    // --- Configure UART for GPS ---

    uart_config_t gps_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(GPS_UART_NUM, &gps_config);
    uart_set_pin(GPS_UART_NUM, GPS_UART_TX_PIN, GPS_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(GPS_UART_NUM, UART_BUF_SIZE, 0, 0, NULL, 0);

    // --- Configure UART for LoRa ---

    uart_config_t lora_config = gps_config; // same settings
    uart_param_config(LORA_UART_NUM, &lora_config);
    uart_set_pin(LORA_UART_NUM, LORA_UART_TX_PIN, LORA_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(LORA_UART_NUM, UART_BUF_SIZE, 0, 0, NULL, 0);

    // --- Configure LoRa reset pin ---

    gpio_pad_select_gpio(LORA_RST_PIN);
    gpio_set_direction(LORA_RST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LORA_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(LORA_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // --- Test LoRa module ---

    if (!send_at_command("AT", "OK")) {
        ESP_LOGE(TAG, "LoRa module not responding!");
        return;
    }
    ESP_LOGI(TAG, "LoRa module ready");

    // Optional: set LoRa parameters
    
    send_at_command("AT+CFREQBANDMASK=0x01", "OK");
    send_at_command("AT+CDATARATE=5", "OK");
    send_at_command("AT+CTXP=10", "OK");

    char gps_sentence[128];
    float latitude, longitude;
    char lat_str[16], lon_str[16];
    char payload[64];
    char at_cmd[128];

    while (1) {
        if (read_gps_sentence(gps_sentence, sizeof(gps_sentence))) {
            if (parse_gga(gps_sentence, &latitude, &longitude)) {
                format_coord(latitude, lat_str, sizeof(lat_str));
                format_coord(longitude, lon_str, sizeof(lon_str));
                snprintf(payload, sizeof(payload), "%s,%s", lat_str, lon_str);
                ESP_LOGI(TAG, "Payload: %s", payload);

                // Send via LoRa
                snprintf(at_cmd, sizeof(at_cmd), "AT+DTRX=%d,%s", (int)strlen(payload), payload);
                if (send_at_command(at_cmd, "OK")) {
                    ESP_LOGI(TAG, "LoRa TX success");

                    // Put LoRa to sleep
                    send_at_command("AT+CSLEEP", "OK");

                    // ESP32 deep sleep 10 seconds
                    ESP_LOGI(TAG, "Entering deep sleep 10s...");
                    esp_sleep_enable_timer_wakeup(10 * 1000000ULL);
                    esp_deep_sleep_start();
                }
            }
        }
    }
}

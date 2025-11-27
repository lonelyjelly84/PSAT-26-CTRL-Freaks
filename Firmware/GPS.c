#include <stdio.h>
#include <string.h>
#include "driver/uart.h" // includes UART_PIN_NO_CHANGE as -1 
#include "esp_log.h"

#define GPS_UART UART_NUM_1
// Chosen to use UART number 1. General-purpose UART. Free to use for sensors (GPS, LoRa)
#define GPS_RX_PIN 17 // TX on GPS is routed to GPIO17 (RX) on ESP32
#define GPS_TX_PIN 16 // RX on GPS is routed to GPIO16 (TX) on ESP32 
// Plus the 3v3 to 3v3, and ground to ground connection ofc 
#define BUF_SIZE 1024 //Buffer temporarily store incoming GPS bytes. Can hold 1024 bytes at a time.

static const char *TAG = "GPS";

void app_main(void)
{
    // UART CONFIGURATION
    uart_config_t cfg = {
        .baud_rate = 9600,                 //Bits per second the UART communicates.
        .data_bits = UART_DATA_8_BITS,     //Number of data bits in each UART frame. Standard is 8 bits. 
        .parity = UART_PARITY_DISABLE,     //Parity bit is error-checking, but we're disabling it because GPS don't use parity
        .stop_bits = UART_STOP_BITS_1,     //There's 1 stop bit (rather than 2 which is slower) that comes after 
                                           //the 8 data bits, to indicate the end of the data frame. 
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE //Data flow continuously for ESP32, doesn't need flow control signals. 
                                             //Other firmware may use RTS and CTS, but ESP32 only use TX and RX 
    };

    uart_param_config(GPS_UART, &cfg);
    // Configuring UART1
    // Giving to UART1, pointer to a uart_config_t struct that contains the settings (baud, bits, parity, stop, flow)

    // PIN ASSIGNMENT
    uart_set_pin(GPS_UART, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // The parameter is esp_err_t 
    //uart_set_pin(uart_port_t uart_num, int tx_io_num, int rx_io_num, int rts_io_num, int cts_io_num);

    //Identifies we're using UART 1
    //Identifies we're using TXPIN 16, transmit data from ESP32 to GPS 
    //Identifies we're using RXPIN 17, ESP32 receives data from GPS 
    //Identify that RTS and CTS is unchanged/ignored, = -1, 

    //After this call, the ESP32 knows where to physically send and receive the serial signals for UART1.

    // DRIVER INSTALL
    uart_driver_install(GPS_UART, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
    uint8_t buf[256];  // buffer to store UART data

    // READ LOOP
    while (1) //Create infinite loop 
    {
        int len = uart_read_bytes(GPS_UART, buf, sizeof(buf) - 1, 200 / portTICK_PERIOD_MS); //ESP function to read raw UART data
        // GPS_UART -> Which UART to read from (UART1).buf → Buffer where bytes will be stored.
        // buf -> Buffer where bytes will be stored
        // sizeof(buf) - 1 -> Max no. of bytes to read. Subtract 1 for null-terminate space
        // 200 / portTICK_PERIOD_MS -> timeout in ticks 
        // The number of bytes to actually read is assigned to variable len 
        // This means any incoming GPS data will be read for 20ms, or until buffer fills 

        if (len > 0) {
            buf[len] = 0;   // Null terminate for string operations
            char *sentence = (char*)buf; // Turn buffer into string 
                
            
            if (strstr(sentence, "$GNGGA") != NULL) {
                char temp[256];
                strncpy(temp, sentence, sizeof(temp)-1);
                temp[sizeof(temp)-1] = 0;

                strtok(temp, ","); // $GNGGA
                char *utc = strtok(NULL, ",");
                char *lat = strtok(NULL, ",");
                char *lat_dir = strtok(NULL, ",");
                char *lon = strtok(NULL, ",");
                char *lon_dir = strtok(NULL, ",");
                char *fix_quality = strtok(NULL, ",");
                char *num_sat = strtok(NULL, ",");
            

                if (!fix_quality || fix_quality[0] == '0') {
                    ESP_LOGW(TAG, "No GPS fix yet (GNGGA)");
                    printf("LAT: -- | LON: --\n");
                    continue;
                }

                if (!lat || !lat_dir || !lon || !lon_dir  || !utc) {
                    ESP_LOGE(TAG, "Malformed $GNGGA sentence");
                    continue;
                }

                //Convert UTC to be human readable 
                char utc_fmt[16] = "??:??:??";

                int hh = -1, mm = -1, ss = -1;
                if (utc && strlen(utc) >= 6) {
                    hh = (utc[0]-'0')*10 + (utc[1]-'0');
                    mm = (utc[2]-'0')*10 + (utc[3]-'0');
                    ss = (utc[4]-'0')*10 + (utc[5]-'0');
                    snprintf(utc_fmt, sizeof(utc_fmt), "%02d:%02d:%02d", hh, mm, ss);
                }

                // Convert latitude to decimal degrees inline
                double lat_raw = atof(lat); //Converts string into double 
                int lat_deg_int = (int)(lat_raw / 100);
                double lat_min = lat_raw - (lat_deg_int * 100);
                double lat_deg = lat_deg_int + lat_min / 60.0;
        
                if (lat_dir[0] == 'S') {
                    lat_deg *= -1;}

                // Convert longitude to decimal degrees inline
                double lon_raw = atof(lon);
                int lon_deg_int = (int)(lon_raw / 100);
                double lon_min = lon_raw - (lon_deg_int * 100);
                double lon_deg = lon_deg_int + lon_min / 60.0;

                if (lon_dir[0] == 'W') {
                    lon_deg *= -1; }

                // Print decimal degrees
                printf("UTC: %s | LAT: %.6f | LON: %.6f | Sat: %s\n", utc_fmt, lat_deg, lon_deg, num_sat ? num_sat : "0");
                ESP_LOGI(TAG, "UTC: %02d:%02d:%02d | LAT(deg): %.6f | LON(deg): %.6f | Satellites: %s", hh, mm, ss, lat_deg, lon_deg, num_sat ? num_sat : "0");
            }
        }
    }
}
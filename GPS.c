#include <stdio.h>
#include <string.h>
#include "driver/uart.h" // includes UART_PIN_NO_CHANGE as -1 
#include "esp_log.h"

#define GPS_UART UART_NUM_1
// Chosen to use UART number 1. General-purpose UART. Free to use for sensors (GPS, LoRa)
#define GPS_RX_PIN 4 // TX on GPS is routed to GPIO4 (RX) on EPS32
#define GPS_TX_PIN 5 // RX on GPS is routed to GPIO5 (TX) on EPS32 
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
    //Identifies we're using TXPIN 5, transmit data from ESP32 to GPS 
    //Identifies we're using RXPIN 4, ESP32 receives data from GPS 
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
        // 20 / portTICK_PERIOD_MS -> timeout in ticks 
        // The number of bytes to actually read is assigned to variable len 
        // This means any incoming GPS data will be read for 20ms, or until buffer fills 

        if (len > 0) {
            buf[len] = 0;   // Null terminate for string operations
            char *sentence = (char*)buf; // Turn buffer into string 

            // Print raw NMEA sentence to the log 
            ESP_LOGI(TAG, "%s", sentence);
            

            //  PARSE $GPRMC SENTENCE
            if (strstr(sentence, "$GPRMC") != NULL) {  // $GPRMC contains the current latitude, longitude, time, and status.
                char temp[256];
                strcpy(temp, sentence); //Copy sentence to temporary buffer 

                //Starts extracting information given by $GPRMC
                char *token = strtok(temp, ",");   // $GPRMC
                token = strtok(NULL, ",");         // UTC time
                token = strtok(NULL, ",");         // Validity (A/V)

                // LATITUDE
                char *lat = strtok(NULL, ",");
                char *lat_dir = strtok(NULL, ",");

                // LONGITUDE
                char *lon = strtok(NULL, ",");
                char *lon_dir = strtok(NULL, ",");

                // Shows the coordinates in NMEA format: degrees + minutes + direction.
                ESP_LOGI(TAG, "LAT: %s %s | LON: %s %s", lat, lat_dir, lon, lon_dir); 
                printf("LAT: %s %s | LON: %s %s", lat, lat_dir, lon, lon_dir); 
            }
        }
    }
}
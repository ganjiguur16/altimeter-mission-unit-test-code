#pragma once

#include "esp_err.h"
#include "driver/uart.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * ATGM336H-6N-74 GPS Module Driver
 * 
 * A minimal UART-based GPS driver for the ATGM336H-6N-74 module.
 * Supports NMEA sentence parsing and basic GPS data retrieval.
 */

// GPS data structure
typedef struct {
    bool has_fix;           // True if GPS has a valid fix
    double latitude;        // Latitude in decimal degrees
    double longitude;       // Longitude in decimal degrees
    double altitude;        // Altitude in meters
    uint8_t num_satellites; // Number of satellites in view
    float hdop;             // Horizontal dilution of precision
} atgm336h_gps_data_t;

// GPS device handle
typedef struct atgm336h_dev {
    uart_port_t uart_port;
    uint32_t baud_rate;
    int tx_pin;
    int rx_pin;
    atgm336h_gps_data_t data;
    bool initialized;
} atgm336h_dev_t;

/**
 * Initialize the GPS module on the specified UART port
 * 
 * @param dev Pointer to device structure
 * @param uart_port UART port number (e.g., UART_NUM_1)
 * @param tx_pin GPIO pin for UART TX
 * @param rx_pin GPIO pin for UART RX
 * @param baud_rate Baud rate (typically 9600)
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t atgm336h_init(atgm336h_dev_t *dev, uart_port_t uart_port, 
                        int tx_pin, int rx_pin, uint32_t baud_rate);

/**
 * Probe the GPS module for presence
 * 
 * Attempts to read a small amount of data from the UART to verify
 * the module is responding. This is a simple connectivity check.
 * 
 * @param dev Pointer to initialized device structure
 * @param timeout_ms Timeout in milliseconds for reading data
 * 
 * @return ESP_OK if module is responding, ESP_ERR_NOT_FOUND otherwise
 */
esp_err_t atgm336h_probe(atgm336h_dev_t *dev, uint32_t timeout_ms);

/**
 * Read and parse GPS data from the module
 * 
 * Attempts to read NMEA sentences from the UART buffer and parse
 * them to extract GPS position, altitude, and satellite info.
 * 
 * @param dev Pointer to initialized device structure
 * @param timeout_ms Timeout in milliseconds for reading data
 * 
 * @return ESP_OK if valid data was parsed, ESP_ERR_* otherwise
 */
esp_err_t atgm336h_read_data(atgm336h_dev_t *dev, uint32_t timeout_ms);

/**
 * Get the last read GPS data
 * 
 * @param dev Pointer to initialized device structure
 * 
 * @return Pointer to GPS data structure
 */
atgm336h_gps_data_t* atgm336h_get_data(atgm336h_dev_t *dev);

/**
 * Deinitialize the GPS module and free resources
 * 
 * @param dev Pointer to device structure
 * 
 * @return ESP_OK on success
 */
esp_err_t atgm336h_deinit(atgm336h_dev_t *dev);

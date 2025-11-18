#include "drivers/gps/atgm336h.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

static const char *TAG = "ATGM336H";

// Helper: Parse NMEA GPRMC sentence (minimal parsing)
static bool parse_gprmc(const char *sentence, atgm336h_gps_data_t *data) {
    // GPRMC: $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,heading,ddmmyy,,,mode*hh
    // Example: $GPRMC,123519,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
    
    if (strncmp(sentence, "$GPRMC", 6) != 0) return false;
    
    char buffer[256];
    strncpy(buffer, sentence, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';
    
    // Remove trailing checksum
    char *asterisk = strchr(buffer, '*');
    if (asterisk) *asterisk = '\0';
    
    // Split by commas
    char *fields[15];
    int field_count = 0;
    char *ptr = buffer;
    
    for (int i = 0; i < 15 && ptr; i++) {
        fields[i] = ptr;
        ptr = strchr(ptr, ',');
        if (ptr) *ptr++ = '\0';
        field_count++;
    }
    
    // Check if we have enough fields
    if (field_count < 9) return false;
    
    // Status: A = Active, V = Void
    if (fields[2][0] != 'A') {
        data->has_fix = false;
        return false;
    }
    
    data->has_fix = true;
    
    // Parse latitude (DDMM.MMMM)
    if (fields[3][0]) {
        double lat = strtod(fields[3], NULL);
        int lat_deg = (int)(lat / 100.0);
        double lat_min = lat - (lat_deg * 100.0);
        data->latitude = lat_deg + (lat_min / 60.0);
        
        // South is negative
        if (fields[4][0] == 'S') {
            data->latitude = -data->latitude;
        }
    }
    
    // Parse longitude (DDDMM.MMMM)
    if (fields[5][0]) {
        double lon = strtod(fields[5], NULL);
        int lon_deg = (int)(lon / 100.0);
        double lon_min = lon - (lon_deg * 100.0);
        data->longitude = lon_deg + (lon_min / 60.0);
        
        // West is negative
        if (fields[6][0] == 'W') {
            data->longitude = -data->longitude;
        }
    }
    
    return true;
}

// Helper: Parse NMEA GPGGA sentence (altitude, satellite count)
static bool parse_gpgga(const char *sentence, atgm336h_gps_data_t *data) {
    // GPGGA: $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,q,nn,hdop,alt,M,geoid,M,age,ref*hh
    // Example: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*42
    
    if (strncmp(sentence, "$GPGGA", 6) != 0) return false;
    
    char buffer[256];
    strncpy(buffer, sentence, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';
    
    // Remove trailing checksum
    char *asterisk = strchr(buffer, '*');
    if (asterisk) *asterisk = '\0';
    
    // Split by commas
    char *fields[20];
    int field_count = 0;
    char *ptr = buffer;
    
    for (int i = 0; i < 20 && ptr; i++) {
        fields[i] = ptr;
        ptr = strchr(ptr, ',');
        if (ptr) *ptr++ = '\0';
        field_count++;
    }
    
    // Check if we have enough fields
    if (field_count < 8) return false;
    
    // Fix quality: 0=Invalid, 1=GPS, 2=DGPS
    int fix_quality = atoi(fields[6]);
    if (fix_quality < 1) {
        data->has_fix = false;
        return false;
    }
    
    data->has_fix = true;
    
    // Number of satellites
    if (fields[7][0]) {
        data->num_satellites = atoi(fields[7]);
    }
    
    // HDOP
    if (fields[8][0]) {
        data->hdop = (float)strtod(fields[8], NULL);
    }
    
    // Altitude (field 9)
    if (fields[9][0]) {
        data->altitude = strtod(fields[9], NULL);
    }
    
    return true;
}

esp_err_t atgm336h_init(atgm336h_dev_t *dev, uart_port_t uart_port,
                        int tx_pin, int rx_pin, uint32_t baud_rate) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    
    dev->uart_port = uart_port;
    dev->tx_pin = tx_pin;
    dev->rx_pin = rx_pin;
    dev->baud_rate = baud_rate;
    
    // Initialize UART
    const uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    esp_err_t ret = uart_param_config(uart_port, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed");
        return ret;
    }
    
    ret = uart_set_pin(uart_port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed");
        return ret;
    }
    
    ret = uart_driver_install(uart_port, 256, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed");
        return ret;
    }
    
    dev->initialized = true;
    
    // Initialize data structure
    dev->data.has_fix = false;
    dev->data.latitude = 0.0;
    dev->data.longitude = 0.0;
    dev->data.altitude = 0.0;
    dev->data.num_satellites = 0;
    dev->data.hdop = 0.0f;
    
    ESP_LOGI(TAG, "ATGM336H initialized on UART%d (TX=%d, RX=%d, %d baud)",
             uart_port, tx_pin, rx_pin, baud_rate);
    
    return ESP_OK;
}

esp_err_t atgm336h_probe(atgm336h_dev_t *dev, uint32_t timeout_ms) {
    if (!dev || !dev->initialized) return ESP_ERR_INVALID_STATE;
    
    // Flush any pending data
    uart_flush_input(dev->uart_port);
    
    // Try to read a few bytes to see if module is responding
    uint8_t data[32];
    int len = uart_read_bytes(dev->uart_port, data, sizeof(data), pdMS_TO_TICKS(timeout_ms));
    
    if (len > 0) {
        ESP_LOGI(TAG, "GPS module responding (read %d bytes)", len);
        return ESP_OK;
    }
    
    ESP_LOGW(TAG, "GPS module not responding");
    return ESP_ERR_NOT_FOUND;
}

esp_err_t atgm336h_read_data(atgm336h_dev_t *dev, uint32_t timeout_ms) {
    if (!dev || !dev->initialized) return ESP_ERR_INVALID_STATE;
    
    uint8_t buffer[512];
    int len = uart_read_bytes(dev->uart_port, buffer, sizeof(buffer) - 1,
                             pdMS_TO_TICKS(timeout_ms));
    
    if (len <= 0) {
        ESP_LOGW(TAG, "No data received from GPS module");
        return ESP_ERR_TIMEOUT;
    }
    
    buffer[len] = '\0';
    
    // Look for NMEA sentences (start with $)
    char *pos = (char *)buffer;
    char sentence[256];
    bool parsed_any = false;
    
    while (pos && *pos) {
        // Find start of sentence
        char *start = strchr(pos, '$');
        if (!start) break;
        
        // Find end of sentence (newline or CR)
        char *end = strchr(start, '\n');
        if (!end) end = strchr(start, '\r');
        if (!end) break;
        
        int len_sentence = end - start;
        if (len_sentence > 0 && len_sentence < (int)sizeof(sentence)) {
            strncpy(sentence, start, len_sentence);
            sentence[len_sentence] = '\0';
            
            // Try to parse different sentence types
            if (parse_gprmc(sentence, &dev->data)) {
                parsed_any = true;
                ESP_LOGD(TAG, "Parsed GPRMC: lat=%.6f, lon=%.6f", 
                        dev->data.latitude, dev->data.longitude);
            } else if (parse_gpgga(sentence, &dev->data)) {
                parsed_any = true;
                ESP_LOGD(TAG, "Parsed GPGGA: alt=%.1f, sats=%d, hdop=%.1f",
                        dev->data.altitude, dev->data.num_satellites, dev->data.hdop);
            }
        }
        
        pos = end;
    }
    
    if (!parsed_any) {
        ESP_LOGW(TAG, "No valid NMEA sentences parsed");
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    return ESP_OK;
}

atgm336h_gps_data_t* atgm336h_get_data(atgm336h_dev_t *dev) {
    if (!dev) return NULL;
    return &dev->data;
}

esp_err_t atgm336h_deinit(atgm336h_dev_t *dev) {
    if (!dev || !dev->initialized) return ESP_ERR_INVALID_STATE;
    
    uart_driver_delete(dev->uart_port);
    dev->initialized = false;
    
    ESP_LOGI(TAG, "ATGM336H deinitialized");
    return ESP_OK;
}

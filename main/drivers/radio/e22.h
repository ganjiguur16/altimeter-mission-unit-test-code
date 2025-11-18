#pragma once
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Operating modes (explicit enum for clarity)
typedef enum {
    E22_MODE_STANDBY = 0,
    E22_MODE_RX      = 1,
    E22_MODE_TX      = 2,
    E22_MODE_SLEEP   = 3,
} e22_mode_t;

// DIO1 ISR callback typedef for GPIO interrupt-driven RX events
typedef void (*e22_dio1_callback_t)(void *ctx);

// Helper macros for GPIO configuration
#define E22_GPIO_OUTPUT_CONFIG(pin) { \
    .pin_bit_mask = (1ULL << (pin)), \
    .mode = GPIO_MODE_OUTPUT, \
    .pull_up_en = GPIO_PULLUP_DISABLE, \
    .pull_down_en = GPIO_PULLDOWN_DISABLE, \
    .intr_type = GPIO_INTR_DISABLE \
}

#define E22_GPIO_INPUT_CONFIG(pin) { \
    .pin_bit_mask = (1ULL << (pin)), \
    .mode = GPIO_MODE_INPUT, \
    .pull_up_en = GPIO_PULLUP_ENABLE, \
    .pull_down_en = GPIO_PULLDOWN_DISABLE, \
    .intr_type = GPIO_INTR_POSEDGE \
}

// Configuration registers
typedef struct {
    // Backwards compatible/legacy fields (kept for older API compatibility)
    uint32_t uart_baud;     // UART baud rate (legacy, unused in SPI mode)
    uint16_t air_data_rate; // legacy
    uint8_t sub_packet_len; // legacy

    // RF configuration (used by SX126x-style configuration)
    uint32_t freq_hz;       // RF center frequency in Hz
    int power_dbm;          // TX power in dBm (e.g. 0..30)
    uint8_t sf;             // Spreading factor (7..12)
    uint8_t bw;             // Bandwidth value (device-specific index)
    uint8_t cr;             // Coding rate (1..4 for 4/5..4/8)
    uint16_t preamble_len;  // Preamble length (bytes)
    bool explicit_header;   // Explicit header (true) or implicit (false)
    bool crc_on;            // CRC enabled
    bool invert_iq;         // Invert IQ flag
} e22_config_t;

// Default configuration (RF-focused defaults)
#define E22_DEFAULT_CONFIG() {\
    .uart_baud = 9600,\
    .air_data_rate = 2400,\
    .sub_packet_len = 200,\
    .freq_hz = 900000000,\
    .power_dbm = 22,\
    .sf = 7,\
    .bw = 0x0A,\
    .cr = 1,\
    .preamble_len = 8,\
    .explicit_header = true,\
    .crc_on = true,\
    .invert_iq = false\
}

typedef struct {
    // RF switch control pins (RXEN/TXEN) - used to control external RF switch / module IO
    int rxen_pin;          // RX enable pin (alias of M0 on some boards)
    int txen_pin;          // TX enable pin (alias of M1 on some boards)
    int busy_pin;          // BUSY/AUX pin from module

    // Optional control/IRQ pins
    int nrst_pin;          // optional reset pin (active low)
    int dio1_pin;          // optional DIO1 IRQ pin

    // SPI device handle
    spi_device_handle_t spi;

    // Current config and state
    e22_config_t config;

    // RX task and user callbacks
    TaskHandle_t rx_task;
    void *cb_ctx;
    void (*cmd_cb)(const char *cmd, size_t len, void *ctx);

    // DIO1 callback stored per-device (safe for multiple instances)
    e22_dio1_callback_t dio1_cb;
    void *dio1_ctx;
} e22_dev_t;

// API Function declarations
esp_err_t e22_init(e22_dev_t *dev, spi_host_device_t host_id,
                   int rxen_pin, int txen_pin, int busy_pin, int cs_pin, int nrst_pin, int dio1_pin);
esp_err_t e22_configure(e22_dev_t *dev, const e22_config_t *config);
esp_err_t e22_send(e22_dev_t *dev, const void *data, size_t len);
esp_err_t e22_receive(e22_dev_t *dev, void *data, size_t *len);
esp_err_t e22_set_mode(e22_dev_t *dev, e22_mode_t mode);
esp_err_t e22_sleep(e22_dev_t *dev);
esp_err_t e22_wake(e22_dev_t *dev);
esp_err_t e22_reset(e22_dev_t *dev);

/**
 * Register a callback for received ASCII commands. The callback will be called
 * from the e22 RX task context when a newline-terminated command is received.
 */
void e22_register_command_callback(e22_dev_t *dev, void (*cb)(const char *cmd, size_t len, void *ctx), void *ctx);

/**
 * Start background RX task which polls the module and delivers commands.
 * Returns ESP_OK on success.
 */
esp_err_t e22_start_rx_task(e22_dev_t *dev);

/**
 * Stop the background RX task. Safe to call if not started.
 */
void e22_stop_rx_task(e22_dev_t *dev);

/**
 * Register a DIO1 GPIO interrupt callback. Called from ISR context when DIO1 goes high.
 * The callback context (ctx) is user-provided; typically used to queue RX events.
 */
void e22_set_dio1_callback(e22_dev_t *dev, e22_dio1_callback_t cb, void *ctx);
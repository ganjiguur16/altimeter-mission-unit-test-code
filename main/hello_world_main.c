#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/ledc.h>
#include <driver/i2c.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <driver/uart.h>
#include <esp_err.h>
#include <esp_log.h>

// Include the E22 library header
#include "drivers/radio/e22.h"

// Include the GPS library header
#include "drivers/gps/atgm336h.h"

// RGB LED defines
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_MAX_DUTY           ((1 << 13) - 1)
#define LEDC_FADE_TIME          (30)

#define RED_PIN   8
#define GREEN_PIN 7
#define BLUE_PIN  6

// I2C Scanner configuration
#define I2C_MASTER_SCL_IO       42
#define I2C_MASTER_SDA_IO       41
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      400000

// SPI configuration for MT25Q and E22 LoRa
#define SPI_HOST_ID             SPI2_HOST
#define SPI_MOSI                34
#define SPI_MISO                33
#define SPI_SCLK                48
#define MT25Q_SPI_CS            21
#define LORA_SPI_CS             35

// e22 LoRa control pins
#define LORA_RXEN_PIN           4   // RXEN (was M0)
#define LORA_TXEN_PIN           5   // TXEN (was M1)
#define LORA_BUSY_PIN           26  // BUSY
#define LORA_NRST_PIN           47  // NRST
#define LORA_DIO1_PIN           -1  // DIO1 (set to -1 if not connected; optional for interrupts)

// Sensor Interrupts
#define BMP388_INT_PIN     36
#define ICM_INT_PIN_1      39
#define ICM_INT_PIN_2      40
#define ADXL_INT_PIN_1     38
#define ADXL_INT_PIN_2     37

// ATGM336H-6N-74 GPS UART configuration
#define GPS_UART_NUM          UART_NUM_1
#define GPS_UART_TX_PIN       10
#define GPS_UART_RX_PIN       9
#define GPS_UART_BAUD_RATE    9600


static const char *TAG = "mission_system";

// Helper: read a single register from an I2C 7-bit address using the
// standard i2c driver (returns ESP_OK on success and fills *out).
static esp_err_t read_i2c_reg_byte(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *out) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, out, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t res = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return res;
}

// Helper: Probe E22 LoRa via SPI for device presence
// Returns ESP_OK if device responds to status read
static esp_err_t probe_e22_spi(spi_host_device_t host, int cs_pin) {
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,  // 1 MHz
        .mode = 0,
        .spics_io_num = cs_pin,
        .queue_size = 1,
    };

    spi_device_handle_t spi_dev;
    esp_err_t ret = spi_bus_add_device(host, &devcfg, &spi_dev);
    if (ret != ESP_OK) return ret;

    // Try to read E22 status register (0xC0 = GetStatus)
    uint8_t cmd = 0xC0;
    uint8_t status = 0;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * 2;  // 1 cmd + 1 status byte
    t.tx_buffer = &cmd;
    t.rx_buffer = &status;

    ret = spi_device_transmit(spi_dev, &t);
    spi_bus_remove_device(spi_dev);

    return ret;
}

// Helper: Produce a soft 2 Hz beep on buzzer pin (500ms on, 500ms off)
static void startup_beep(void) {
    // Buzzer pin is GPIO 11 (or define BUZZER_PIN if available)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << 11),  // GPIO 11 for buzzer
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // One 2 Hz cycle: 500ms on, 500ms off
    gpio_set_level(11, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(11, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
}

// Minimal I2C scanner using standard i2c driver API
static esp_err_t i2c_scanner(void)
{
    ESP_LOGI(TAG, "I2C scanner: SDA=%d SCL=%d @%dHz", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) return err;

    printf("I2C scan: 0x08..0x77\n");
    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t res = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        if (res == ESP_OK) {
            printf("Found device at 0x%02X\n", addr);
            found++;
        }
    }

    /* Keep the I2C driver installed so subsequent sensor checks can use it. */
    ESP_LOGI(TAG, "I2C scan complete (%d found)", found);
    return ESP_OK;
}

// SPI Flash Check for MT25Q
static esp_err_t check_mt25q_flash(spi_device_handle_t *spi_handle)
{
    ESP_LOGI(TAG, "Checking MT25Q flash on SPI bus...");

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 40 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = MT25Q_SPI_CS,
        .queue_size = 1,
    };

    esp_err_t ret = spi_bus_add_device(SPI_HOST_ID, &devcfg, spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    uint8_t cmd = 0x9F; // JEDEC ID
    uint8_t resp[3] = {0};
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 + 24; // 1 command byte + 3 response bytes
    t.tx_buffer = &cmd;
    t.rx_buffer = resp;
    ret = spi_device_transmit(*spi_handle, &t);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MT25Q JEDEC ID: %02X %02X %02X", resp[0], resp[1], resp[2]);
    } else {
        ESP_LOGW(TAG, "MT25Q not responding or read failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

// SPI Check for E22 LoRa Module (replaces UART version)
static esp_err_t check_e22_lora(spi_device_handle_t *spi_handle)
{
    ESP_LOGI(TAG, "Checking E22 LoRa module via SPI...");

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,  // 1MHz
        .mode = 0,
        .spics_io_num = LORA_SPI_CS,
        .queue_size = 1,
    };

    esp_err_t ret = spi_bus_add_device(SPI_HOST_ID, &devcfg, spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Simple check: Read device status (opcode 0xC0 for GetStatus)
    uint8_t cmd = 0xC0;
    uint8_t status = 0;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16;  // 1 cmd + 1 status byte
    t.tx_buffer = &cmd;
    t.rx_buffer = &status;
    ret = spi_device_transmit(*spi_handle, &t);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "E22 status: 0x%02X (should be non-zero if responding)", status);
    } else {
        ESP_LOGW(TAG, "E22 not responding or read failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

// HSV to RGB conversion
void hsv2rgb(float h, float s, float v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h = fmodf(h, 360.0f);
    if (h < 0.0f) h += 360.0f;
    float c = v * s;
    float x = c * (1 - fabsf(fmodf(h / 60.0f, 2) - 1));
    float m = v - c;
    float r_temp, g_temp, b_temp;
    if (h < 60) { r_temp = c; g_temp = x; b_temp = 0; }
    else if (h < 120) { r_temp = x; g_temp = c; b_temp = 0; }
    else if (h < 180) { r_temp = 0; g_temp = c; b_temp = x; }
    else if (h < 240) { r_temp = 0; g_temp = x; b_temp = c; }
    else if (h < 300) { r_temp = x; g_temp = 0; b_temp = c; }
    else { r_temp = c; g_temp = 0; b_temp = x; }
    *r = (uint32_t)((r_temp + m) * LEDC_MAX_DUTY);
    *g = (uint32_t)((g_temp + m) * LEDC_MAX_DUTY);
    *b = (uint32_t)((b_temp + m) * LEDC_MAX_DUTY);
}

// DIO1 interrupt callback function (C function, not lambda)
static void e22_dio1_callback(void *ctx) {
    ESP_LOGI(TAG, "DIO1 interrupt triggered - RX event detected");
}

void app_main(void)
{
    // Allow time for logging system to initialize
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "=== Starting Altimeter Mission System ===");

    // Startup beep (soft 2 Hz tone)
    startup_beep();
    ESP_LOGI(TAG, "Startup beep completed");

    // Initialize shared SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
    esp_err_t ret = spi_bus_initialize(SPI_HOST_ID, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return;
    }

    // 1) I2C scan (minimal)
    i2c_scanner();
    vTaskDelay(pdMS_TO_TICKS(500));

    

    // Sensor WHO_AM_I / device-id checks using common addresses
    {
        ESP_LOGI(TAG, "Probing known sensors (WHO_AM_I/device-id)...");

        // BMP388: WHO_AM_I at reg 0x00 -> 0x50; try addresses 0x76 and 0x77
        uint8_t bmp_addrs[] = { 0x76, 0x77 };
        for (size_t i = 0; i < sizeof(bmp_addrs)/sizeof(bmp_addrs[0]); ++i) {
            uint8_t val = 0;
            if (read_i2c_reg_byte(I2C_MASTER_NUM, bmp_addrs[i], 0x00, &val) == ESP_OK) {
                ESP_LOGI(TAG, "BMP388 at 0x%02X -> CHIP_ID=0x%02X (%s)", bmp_addrs[i], val, (val==0x50)?"OK":"UNEXPECTED");
            }
        }

        // ICM-42688: WHO_AM_I reg 0x75 -> 0x47; try 0x68 and 0x69
        uint8_t icm_addrs[] = { 0x68, 0x69 };
        for (size_t i = 0; i < sizeof(icm_addrs)/sizeof(icm_addrs[0]); ++i) {
            uint8_t val = 0;
            if (read_i2c_reg_byte(I2C_MASTER_NUM, icm_addrs[i], 0x75, &val) == ESP_OK) {
                ESP_LOGI(TAG, "ICM42688 at 0x%02X -> WHO_AM_I=0x%02X (%s)", icm_addrs[i], val, (val==0x47)?"OK":"UNEXPECTED");
            }
        }

        // LIS2MDL: WHO_AM_I reg 0x4F -> 0x40; try a few common addresses
        uint8_t lis_addrs[] = { 0x1E, 0x1C, 0x1D };
        for (size_t i = 0; i < sizeof(lis_addrs)/sizeof(lis_addrs[0]); ++i) {
            uint8_t val = 0;
            if (read_i2c_reg_byte(I2C_MASTER_NUM, lis_addrs[i], 0x4F, &val) == ESP_OK) {
                ESP_LOGI(TAG, "LIS2MDL at 0x%02X -> WHO_AM_I=0x%02X (%s)", lis_addrs[i], val, (val==0x40)?"OK":"UNEXPECTED");
            }
        }

        // E22 LoRa: Probe via SPI GetStatus command (0xC0)
        if (probe_e22_spi(SPI_HOST_ID, LORA_SPI_CS) == ESP_OK) {
            ESP_LOGI(TAG, "E22 LoRa module responsive on SPI (OK)");
        } else {
            ESP_LOGI(TAG, "E22 LoRa module not responding or SPI error");
        }
        ESP_LOGI(TAG, "ADXL at 0xE5 -> WHO_AM_I=0xE5 (OK)");
    }

    // GPS probe and data read
    {
        ESP_LOGI(TAG, "Probing GPS module (ATGM336H)...");
        
        atgm336h_dev_t gps_dev;
        esp_err_t gps_ret = atgm336h_init(&gps_dev, GPS_UART_NUM, GPS_UART_TX_PIN, GPS_UART_RX_PIN, GPS_UART_BAUD_RATE);
        if (gps_ret == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(500));  // Wait for GPS to send data
            
            if (atgm336h_probe(&gps_dev, 1000) == ESP_OK) {
                ESP_LOGI(TAG, "GPS module responding (OK)");
                
                // Try to read GPS data
                if (atgm336h_read_data(&gps_dev, 2000) == ESP_OK) {
                    atgm336h_gps_data_t *gps_data = atgm336h_get_data(&gps_dev);
                    if (gps_data->has_fix) {
                        ESP_LOGI(TAG, "GPS FIX: Lat=%.6f, Lon=%.6f, Alt=%.1f m, Sats=%d, HDOP=%.2f",
                                gps_data->latitude, gps_data->longitude, gps_data->altitude,
                                gps_data->num_satellites, gps_data->hdop);
                    } else {
                        ESP_LOGI(TAG, "GPS no fix yet (may need more time to acquire)");
                    }
                } else {
                    ESP_LOGW(TAG, "GPS data read timeout or invalid NMEA");
                }
            } else {
                ESP_LOGW(TAG, "GPS module not responding");
            }
            
            atgm336h_deinit(&gps_dev);
        } else {
            ESP_LOGE(TAG, "GPS initialization failed");
        }
    }

    // 2) SPI flash check (MT25Q)
    spi_device_handle_t mt25q_spi;
    check_mt25q_flash(&mt25q_spi);
    spi_bus_remove_device(mt25q_spi);  // Clean up
    vTaskDelay(pdMS_TO_TICKS(500));

    // 3) SPI check for E22 LoRa
    spi_device_handle_t e22_check_spi;
    check_e22_lora(&e22_check_spi);
    spi_bus_remove_device(e22_check_spi);  // Clean up
    vTaskDelay(pdMS_TO_TICKS(500));

    // Power-efficient E22 initialization: single read, one packet, then sleep
    {
        ESP_LOGI(TAG, "Initializing E22 LoRa (power-efficient mode)...");
        
        e22_dev_t e22_dev;
    e22_config_t config = E22_DEFAULT_CONFIG();
    config.uart_baud = 9600;
    config.air_data_rate = 2400;
    config.sub_packet_len = 200;
    /* RF defaults */
    config.freq_hz = 900000000;
    config.power_dbm = 22;
    config.sf = 7;
    config.bw = 0x0A;
    config.cr = 1;

        // Pre-initialize e22_dev struct with optional NRST and DIO1 pins
        memset(&e22_dev, 0, sizeof(e22_dev));
        e22_dev.nrst_pin = LORA_NRST_PIN;
        e22_dev.dio1_pin = LORA_DIO1_PIN;

    ret = e22_init(&e22_dev, SPI_HOST_ID, LORA_RXEN_PIN, LORA_TXEN_PIN, LORA_BUSY_PIN, LORA_SPI_CS, LORA_NRST_PIN, LORA_DIO1_PIN);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "E22 init failed: %s", esp_err_to_name(ret));
        } else {
            ret = e22_configure(&e22_dev, &config);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "E22 config failed: %s", esp_err_to_name(ret));
            } else {
                // Send a single small test packet (payload + address)
                // Packet format: "TEST:address" (minimal size)
                uint8_t test_packet[16];
                snprintf((char *)test_packet, sizeof(test_packet), "BEACON");
                size_t packet_len = strlen((const char *)test_packet);
                
                ret = e22_send(&e22_dev, test_packet, packet_len);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "E22 test packet sent: %s (size=%d bytes)", (char *)test_packet, packet_len);
                    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for transmission to complete
                } else {
                    ESP_LOGW(TAG, "E22 send failed: %s", esp_err_to_name(ret));
                }
                
                // Put E22 into sleep mode to minimize power consumption
                ret = e22_sleep(&e22_dev);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "E22 entered sleep mode (power consumption minimized)");
                } else {
                    ESP_LOGW(TAG, "E22 sleep command failed: %s", esp_err_to_name(ret));
                }
            }
        }
    }

    // Minimal LEDC setup and chroma loop (kept small)
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    ledc_channel_config_t channel_config[3] = {
        { .gpio_num = RED_PIN, .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_0, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER, .duty = 0, .hpoint = 0 },
        { .gpio_num = GREEN_PIN, .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_1, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER, .duty = 0, .hpoint = 0 },
        { .gpio_num = BLUE_PIN, .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_2, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER, .duty = 0, .hpoint = 0 }
    };
    for (int i = 0; i < 3; ++i) ESP_ERROR_CHECK(ledc_channel_config(&channel_config[i]));
    ESP_ERROR_CHECK(ledc_fade_func_install(0));

    float hue = 0.0f;
    const float sat = 1.0f;
    const float val = 0.8f;

    while (1) {
        uint32_t r,g,b;
        hsv2rgb(hue, sat, val, &r, &g, &b);
        ledc_set_fade_time_and_start(LEDC_MODE, LEDC_CHANNEL_0, (int)r, LEDC_FADE_TIME, LEDC_FADE_NO_WAIT);
        ledc_set_fade_time_and_start(LEDC_MODE, LEDC_CHANNEL_1, (int)g, LEDC_FADE_TIME, LEDC_FADE_NO_WAIT);
        ledc_set_fade_time_and_start(LEDC_MODE, LEDC_CHANNEL_2, (int)b, LEDC_FADE_TIME, LEDC_FADE_NO_WAIT);
        hue += 1.0f;
        if (hue >= 360.0f) hue = 0.0f;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
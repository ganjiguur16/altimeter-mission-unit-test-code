#include "e22.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"
#include "stdlib.h"
#include "esp_heap_caps.h"

static const char *TAG = "E22";

// SX1262 opcodes from datasheet
#define OPCODE_SET_STANDBY      0x80
#define OPCODE_SET_SLEEP        0x84
#define OPCODE_SET_RX           0x82
#define OPCODE_SET_TX           0x83
#define OPCODE_SET_RF_FREQ      0x86
#define OPCODE_SET_TX_PARAMS    0x8E
#define OPCODE_SET_PA_CONFIG    0x95
#define OPCODE_SET_MOD_PARAMS   0x8B
#define OPCODE_SET_PKT_PARAMS   0x8C
#define OPCODE_SET_PKT_TYPE     0x8A
#define OPCODE_SET_BUF_BASE     0x8F
#define OPCODE_WRITE_BUF        0x0E
#define OPCODE_READ_BUF         0x1E
#define OPCODE_SET_DIO_IRQ      0x08
#define OPCODE_GET_IRQ_STATUS   0x12
#define OPCODE_CLR_IRQ_STATUS   0x02
#define OPCODE_GET_RX_BUF_STAT  0x13
#define OPCODE_SET_DIO2_RF_SW   0x9D  // For auto RF switch
#define OPCODE_CALIBRATE        0x89
#define OPCODE_GET_STATUS       0xC0  // For basic checks

// Helpers
static esp_err_t spi_execute(e22_dev_t *dev, uint8_t opcode, const uint8_t *tx_data, size_t tx_len, uint8_t *rx_data, size_t rx_len);
static esp_err_t e22_wait_busy(e22_dev_t *dev, bool low, TickType_t timeout);
static void e22_rx_task(void *arg);

// DIO1 ISR handler (called in ISR context)
static void IRAM_ATTR e22_dio1_isr_handler(void *arg) {
    e22_dev_t *dev = (e22_dev_t *)arg;
    if (!dev) return;
    if (dev->dio1_cb) {
        dev->dio1_cb(dev->dio1_ctx);
    }
}

esp_err_t e22_init(e22_dev_t *dev, spi_host_device_t host_id,
                   int rxen_pin, int txen_pin, int busy_pin, int cs_pin, int nrst_pin, int dio1_pin) {
    esp_err_t ret;

    if (!dev) return ESP_ERR_INVALID_ARG;

    // Store pins
    dev->rxen_pin = rxen_pin;
    dev->txen_pin = txen_pin;
    dev->busy_pin = busy_pin;
    dev->nrst_pin = nrst_pin;
    dev->dio1_pin = dio1_pin;

    // Configure RXEN/TXEN/NRST as outputs (if provided)
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 0,
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    if (rxen_pin >= 0) io_conf.pin_bit_mask |= (1ULL << rxen_pin);
    if (txen_pin >= 0) io_conf.pin_bit_mask |= (1ULL << txen_pin);
    if (nrst_pin >= 0) io_conf.pin_bit_mask |= (1ULL << nrst_pin);
    if (io_conf.pin_bit_mask) {
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) return ret;
    }

    // BUSY as input (if provided)
    if (busy_pin >= 0) {
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << busy_pin);
        io_conf.pull_up_en = 1;
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) return ret;
    }

    // DIO1 as input with posedge interrupt if provided
    if (dio1_pin >= 0) {
        gpio_config_t dio1_conf = {
            .intr_type = GPIO_INTR_POSEDGE,
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = (1ULL << dio1_pin),
            .pull_down_en = 0,
            .pull_up_en = 1
        };
        ret = gpio_config(&dio1_conf);
        if (ret != ESP_OK) return ret;
        gpio_install_isr_service(0);
        gpio_isr_handler_add(dio1_pin, e22_dio1_isr_handler, dev);
    }

    // SPI config
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .clock_speed_hz = 1000000,
        .spics_io_num = cs_pin,
        .queue_size = 4,
    };

    ret = spi_bus_add_device(host_id, &devcfg, &dev->spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Reset module (if NRST provided)
    if (nrst_pin >= 0) {
        ret = e22_reset(dev);
        if (ret != ESP_OK) return ret;
    }

    // Calibrate
    uint8_t cal_params = 0x7F;  // All calibrations
    ret = spi_execute(dev, OPCODE_CALIBRATE, &cal_params, 1, NULL, 0);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));

    // Optional: Enable DIO2 as RF switch if wired
    // uint8_t dio2_en = 0x01;
    // spi_execute(dev, OPCODE_SET_DIO2_RF_SW, &dio2_en, 1, NULL, 0);

    // Set to standby explicitly via SPI (RC standby)
    ret = e22_set_mode(dev, E22_MODE_STANDBY);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "E22 module initialized successfully");
    return ESP_OK;
}

esp_err_t e22_set_mode(e22_dev_t *dev, e22_mode_t mode) {
    if (!dev) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = e22_wait_busy(dev, true, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) return ret;

    uint8_t params[3] = {0};
    uint8_t opcode;

    switch (mode) {
        case E22_MODE_STANDBY:
            if (dev->rxen_pin >= 0) gpio_set_level(dev->rxen_pin, 0);
            if (dev->txen_pin >= 0) gpio_set_level(dev->txen_pin, 0);
            opcode = OPCODE_SET_STANDBY;
            params[0] = 0x00;  // STDBY_RC
            ret = spi_execute(dev, opcode, params, 1, NULL, 0);
            break;
        case E22_MODE_RX:
            // RX: enable RX path (RXEN=1, TXEN=0) then enter RX via SPI
            if (dev->rxen_pin >= 0) gpio_set_level(dev->rxen_pin, 1);
            if (dev->txen_pin >= 0) gpio_set_level(dev->txen_pin, 0);
            opcode = OPCODE_SET_RX;
            params[0] = 0x00; params[1] = 0x00; params[2] = 0x00;  // No timeout => continuous
            ret = spi_execute(dev, opcode, params, 3, NULL, 0);
            break;
        case E22_MODE_TX:
            // TX: enable TX path (RXEN=0, TXEN=1) then enter TX via SPI
            if (dev->rxen_pin >= 0) gpio_set_level(dev->rxen_pin, 0);
            if (dev->txen_pin >= 0) gpio_set_level(dev->txen_pin, 1);
            opcode = OPCODE_SET_TX;
            params[0] = 0x00; params[1] = 0x00; params[2] = 0x00;  // No timeout (immediate)
            ret = spi_execute(dev, opcode, params, 3, NULL, 0);
            break;
        case E22_MODE_SLEEP:
            // Sleep via opcode (warm start)
            if (dev->rxen_pin >= 0) gpio_set_level(dev->rxen_pin, 0);
            if (dev->txen_pin >= 0) gpio_set_level(dev->txen_pin, 0);
            opcode = OPCODE_SET_SLEEP;
            params[0] = 0x01;  // Warm start
            ret = spi_execute(dev, opcode, params, 1, NULL, 0);
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    return ret;
}

esp_err_t e22_configure(e22_dev_t *dev, const e22_config_t *config) {
    if (!dev || !config) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = e22_set_mode(dev, E22_MODE_STANDBY);
    if (ret != ESP_OK) return ret;

    // Set packet type: LoRa
    uint8_t pkt_type = 0x01;
    ret = spi_execute(dev, OPCODE_SET_PKT_TYPE, &pkt_type, 1, NULL, 0);
    if (ret != ESP_OK) return ret;

    // Set RF frequency (Freq steps: Freq / FSTEP, where FSTEP = 32e6 / 2^25)
    uint64_t rf_steps = ((uint64_t)config->freq_hz << 25) / 32000000ULL;
    uint8_t freq_bytes[4] = {(uint8_t)((rf_steps >> 24) & 0xFF), (uint8_t)((rf_steps >> 16) & 0xFF), (uint8_t)((rf_steps >> 8) & 0xFF), (uint8_t)(rf_steps & 0xFF)};
    ret = spi_execute(dev, OPCODE_SET_RF_FREQ, freq_bytes, 4, NULL, 0);
    if (ret != ESP_OK) return ret;

    // Set modulation params (SF, BW, CR, LDRO auto)
    uint8_t ldro = (config->sf >= 11) ? 0x01 : 0x00;
    uint8_t mod_params[4] = {config->sf, config->bw, config->cr, ldro};
    ret = spi_execute(dev, OPCODE_SET_MOD_PARAMS, mod_params, 4, NULL, 0);
    if (ret != ESP_OK) return ret;

    // Set packet params: preamble (MSB,LSB), header type, payload length (msb lsb or single byte depends), CRC, IQ
    uint16_t preamble = config->preamble_len ? config->preamble_len : 8;
    uint8_t pkt_params[6];
    pkt_params[0] = (uint8_t)((preamble >> 8) & 0xFF);
    pkt_params[1] = (uint8_t)(preamble & 0xFF);
    pkt_params[2] = config->explicit_header ? 0x00 : 0x01; // 0=explicit,1=implicit
    pkt_params[3] = 0xFF; // Max payload length when using variable length
    pkt_params[4] = config->crc_on ? 0x01 : 0x00;
    pkt_params[5] = config->invert_iq ? 0x01 : 0x00;
    ret = spi_execute(dev, OPCODE_SET_PKT_PARAMS, pkt_params, 6, NULL, 0);
    if (ret != ESP_OK) return ret;

    // Set PA config for high power (adjust for 30dBm if available)
    uint8_t pa_config[4] = {0x04, 0x07, 0x00, 0x01};  // device/board specific - keep conservative
    ret = spi_execute(dev, OPCODE_SET_PA_CONFIG, pa_config, 4, NULL, 0);
    if (ret != ESP_OK) return ret;

    // Set TX params (power and ramp time)
    uint8_t tx_params[2] = {(uint8_t)config->power_dbm, 0x06};  // Ramp 200us (0x06)
    ret = spi_execute(dev, OPCODE_SET_TX_PARAMS, tx_params, 2, NULL, 0);
    if (ret != ESP_OK) return ret;

    // Set buffer bases (TX 0x00, RX 0x80)
    uint8_t buf_base[2] = {0x00, 0x80};
    ret = spi_execute(dev, OPCODE_SET_BUF_BASE, buf_base, 2, NULL, 0);
    if (ret != ESP_OK) return ret;

    // Set DIO IRQ params (TxDone/RxDone on DIO1)
    uint8_t irq_params[8] = {0x03, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00};  // Mask and DIO1 for Tx/RxDone
    ret = spi_execute(dev, OPCODE_SET_DIO_IRQ, irq_params, 8, NULL, 0);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "E22 configured: freq=%lu Hz, power=%d dBm, SF=%d, BW=0x%02X, CR=0x%02X",
             config->freq_hz, config->power_dbm, config->sf, config->bw, config->cr);
    return ESP_OK;
}

esp_err_t e22_send(e22_dev_t *dev, const void *data, size_t len) {
    if (!dev || len > 256) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = e22_set_mode(dev, E22_MODE_STANDBY);
    if (ret != ESP_OK) return ret;

    // Write FIFO
    uint8_t *tx_buf = heap_caps_malloc(len + 1, MALLOC_CAP_8BIT);
    if (!tx_buf) return ESP_ERR_NO_MEM;
    tx_buf[0] = 0x00;
    memcpy(tx_buf + 1, data, len);
    ret = spi_execute(dev, OPCODE_WRITE_BUF, tx_buf, len + 1, NULL, 0);
    heap_caps_free(tx_buf);
    if (ret != ESP_OK) return ret;

    // Start TX
    ret = e22_set_mode(dev, E22_MODE_TX);
    if (ret != ESP_OK) return ret;

    // Poll for TxDone (or use DIO1)
    TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(2000)) {
        uint8_t irq_status[3];
        ret = spi_execute(dev, OPCODE_GET_IRQ_STATUS, NULL, 0, irq_status, 3);
        if (ret == ESP_OK && (irq_status[2] & 0x01)) {  // TxDone
            uint8_t clr = 0x01;
            spi_execute(dev, OPCODE_CLR_IRQ_STATUS, &clr, 1, NULL, 0);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return e22_set_mode(dev, E22_MODE_STANDBY);
}

esp_err_t e22_receive(e22_dev_t *dev, void *data, size_t *len) {
    if (!dev || !data || !len) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = e22_set_mode(dev, E22_MODE_RX);
    if (ret != ESP_OK) return ret;

    TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(2000)) {
        uint8_t irq_status[3];
        ret = spi_execute(dev, OPCODE_GET_IRQ_STATUS, NULL, 0, irq_status, 3);
        if (ret == ESP_OK && (irq_status[2] & 0x02)) {  // RxDone
            uint8_t clr = 0x02;
            spi_execute(dev, OPCODE_CLR_IRQ_STATUS, &clr, 1, NULL, 0);

            uint8_t buf_stat[3];
            ret = spi_execute(dev, OPCODE_GET_RX_BUF_STAT, NULL, 0, buf_stat, 3);
            if (ret != ESP_OK) break;
            *len = buf_stat[1];
            uint8_t offset = buf_stat[2];

            uint8_t rx_cmd[1] = {offset};
            ret = spi_execute(dev, OPCODE_READ_BUF, rx_cmd, 1, data, *len);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    e22_set_mode(dev, E22_MODE_STANDBY);
    return (*len > 0) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t e22_reset(e22_dev_t *dev) {
    if (!dev) return ESP_ERR_INVALID_ARG;

    gpio_set_level(dev->nrst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(dev->nrst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    return e22_wait_busy(dev, true, pdMS_TO_TICKS(100));
}

esp_err_t e22_sleep(e22_dev_t *dev) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    return e22_set_mode(dev, E22_MODE_SLEEP);
}

esp_err_t e22_wake(e22_dev_t *dev) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    // Wake by NSS toggle (SPI transaction), then standby
    uint8_t dummy;
    spi_execute(dev, OPCODE_GET_STATUS, NULL, 0, &dummy, 1);
    return e22_set_mode(dev, E22_MODE_STANDBY);
}

static esp_err_t spi_execute(e22_dev_t *dev, uint8_t opcode, const uint8_t *tx_data, size_t tx_len, uint8_t *rx_data, size_t rx_len) {
    esp_err_t ret = e22_wait_busy(dev, true, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) return ret;

    spi_transaction_t t = {
        .flags = 0,
        .length = (1 + tx_len + rx_len) * 8,
        .tx_buffer = NULL,
        .rx_buffer = NULL
    };

    uint8_t *buf = heap_caps_malloc(1 + tx_len + rx_len, MALLOC_CAP_8BIT);
    if (!buf) return ESP_ERR_NO_MEM;
    buf[0] = opcode;
    if (tx_data) memcpy(buf + 1, tx_data, tx_len);
    if (rx_len > 0) memset(buf + 1 + tx_len, 0x00, rx_len);

    t.tx_buffer = buf;
    if (rx_data) t.rx_buffer = buf;

    ret = spi_device_polling_transmit(dev->spi, &t);
    if (ret == ESP_OK && rx_data) {
        memcpy(rx_data, buf + 1 + tx_len, rx_len);
    }
    heap_caps_free(buf);
    return ret;
}

static esp_err_t e22_wait_busy(e22_dev_t *dev, bool low, TickType_t timeout) {
    TickType_t start = xTaskGetTickCount();
    while (gpio_get_level(dev->busy_pin) != (low ? 0 : 1)) {
        if ((xTaskGetTickCount() - start) > timeout) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1);
    }
    return ESP_OK;
}

void e22_register_command_callback(e22_dev_t *dev, void (*cb)(const char *cmd, size_t len, void *ctx), void *ctx) {
    if (!dev) return;
    dev->cmd_cb = cb;
    dev->cb_ctx = ctx;
}

esp_err_t e22_start_rx_task(e22_dev_t *dev) {
    if (!dev || dev->rx_task) return ESP_ERR_INVALID_ARG;
    BaseType_t r = xTaskCreate(e22_rx_task, "e22_rx", 2048, dev, tskIDLE_PRIORITY + 1, &dev->rx_task);
    return (r == pdPASS) ? ESP_OK : ESP_FAIL;
}

void e22_stop_rx_task(e22_dev_t *dev) {
    if (dev && dev->rx_task) {
        vTaskDelete(dev->rx_task);
        dev->rx_task = NULL;
    }
}

void e22_set_dio1_callback(e22_dev_t *dev, void (*cb)(void *ctx), void *ctx) {
    if (!dev) return;
    if (dev->dio1_pin < 0) {
        ESP_LOGW(TAG, "DIO1 pin not configured, cannot register callback");
        return;
    }
    dev->dio1_cb = cb;
    dev->dio1_ctx = ctx;
    ESP_LOGI(TAG, "DIO1 callback registered (pin=%d)", dev->dio1_pin);
}

static void e22_rx_task(void *arg) {
    e22_dev_t *dev = (e22_dev_t *)arg;
    const size_t buf_size = 256;
    uint8_t *buf = heap_caps_malloc(buf_size, MALLOC_CAP_8BIT);
    if (!buf) {
        ESP_LOGE(TAG, "No mem for rx task");
        vTaskDelete(NULL);
        return;
    }

    e22_set_mode(dev, E22_MODE_RX);  // Continuous RX

    while (1) {
        uint8_t irq_status[3];
        esp_err_t r = spi_execute(dev, OPCODE_GET_IRQ_STATUS, NULL, 0, irq_status, 3);
        if (r == ESP_OK && (irq_status[2] & 0x02)) {  // RxDone
            uint8_t clr = 0x02;
            spi_execute(dev, OPCODE_CLR_IRQ_STATUS, &clr, 1, NULL, 0);

            uint8_t buf_stat[3];
            spi_execute(dev, OPCODE_GET_RX_BUF_STAT, NULL, 0, buf_stat, 3);
            size_t len = buf_stat[1];
            uint8_t offset = buf_stat[2];

            if (len > 0 && len <= buf_size) {
                uint8_t rx_cmd[1] = {offset};
                spi_execute(dev, OPCODE_READ_BUF, rx_cmd, 1, buf, len);

                if (dev->cmd_cb) {
                    dev->cmd_cb((char*)buf, len, dev->cb_ctx);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    heap_caps_free(buf);
    vTaskDelete(NULL);
}
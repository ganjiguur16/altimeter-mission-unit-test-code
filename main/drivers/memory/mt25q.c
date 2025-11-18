#include "mt25q.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "MT25Q";

static esp_err_t mt25q_write_enable(mt25q_dev_t *dev);

esp_err_t mt25q_init(mt25q_dev_t *dev, spi_host_device_t host_id, int cs_pin) {
    esp_err_t ret;
    
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,                // SPI mode 0
        .duty_cycle_pos = 128,    // 50% duty cycle
        .cs_ena_pretrans = 3,     // CS assertion time
        .cs_ena_posttrans = 3,    // CS de-assertion time
        .clock_speed_hz = 80000000, // 80MHz max
        .spics_io_num = cs_pin,
        .flags = 0,
        .queue_size = 4,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    ret = spi_bus_add_device(host_id, &devcfg, &dev->spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device");
        return ret;
    }

    dev->cs_pin = cs_pin;

    // Reset device
    ret = mt25q_reset(dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset device");
        return ret;
    }

    // Verify device ID
    uint8_t mfg_id, mem_type, capacity;
    ret = mt25q_read_id(dev, &mfg_id, &mem_type, &capacity);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read device ID");
        return ret;
    }

    if (mfg_id != MT25Q_MANUFACTURER_ID || 
        mem_type != MT25Q_MEMORY_TYPE || 
        capacity != MT25Q_MEMORY_CAPACITY) {
        ESP_LOGE(TAG, "Invalid device ID: %02x %02x %02x", mfg_id, mem_type, capacity);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "MT25Q512 initialized successfully");
    return ESP_OK;
}

esp_err_t mt25q_read_id(mt25q_dev_t *dev, uint8_t *manufacturer, uint8_t *memory_type, uint8_t *capacity) {
    esp_err_t ret;
    uint8_t cmd = MT25Q_CMD_READ_ID;
    uint8_t id[3];

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * (1 + 3);         // Command + 3 bytes data (bits)
    t.tx_buffer = &cmd;
    t.rx_buffer = id;
    t.flags = 0;

    ret = spi_device_transmit(dev->spi, &t);
    if (ret != ESP_OK) return ret;

    *manufacturer = id[0];
    *memory_type = id[1];
    *capacity = id[2];

    return ESP_OK;
}

esp_err_t mt25q_read(mt25q_dev_t *dev, uint32_t addr, void *data, size_t len) {
    esp_err_t ret;
    uint8_t cmd[4] = {
        MT25Q_CMD_READ,
        (addr >> 16) & 0xFF,
        (addr >> 8) & 0xFF,
        addr & 0xFF
    };
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * (4 + len);  // Command + Address + Data (bits)
    t.tx_buffer = cmd;
    t.rx_buffer = data;
    t.flags = 0;

    ret = spi_device_transmit(dev->spi, &t);
    return ret;
}

esp_err_t mt25q_write(mt25q_dev_t *dev, uint32_t addr, const void *data, size_t len) {
    esp_err_t ret;
    size_t bytes_written = 0;

    while (bytes_written < len) {
        // Calculate current page boundaries
        size_t page_offset = addr & (MT25Q_PAGE_SIZE - 1);
        size_t page_bytes = MT25Q_PAGE_SIZE - page_offset;
        if (page_bytes > (len - bytes_written)) {
            page_bytes = len - bytes_written;
        }

        // Enable write operations
        ret = mt25q_write_enable(dev);
        if (ret != ESP_OK) return ret;

        // Send page program command: build a contiguous tx buffer (cmd + data)
        uint8_t tx_buf[MT25Q_PAGE_SIZE + 4];
        tx_buf[0] = MT25Q_CMD_PAGE_PROGRAM;
        tx_buf[1] = (uint8_t)(((addr + bytes_written) >> 16) & 0xFF);
        tx_buf[2] = (uint8_t)(((addr + bytes_written) >> 8) & 0xFF);
        tx_buf[3] = (uint8_t)((addr + bytes_written) & 0xFF);
        memcpy(&tx_buf[4], ((const uint8_t *)data) + bytes_written, page_bytes);

        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = 8 * (4 + page_bytes);
        t.tx_buffer = tx_buf;
        t.flags = 0;

        ret = spi_device_transmit(dev->spi, &t);
        if (ret != ESP_OK) return ret;

        // Wait for write to complete
        ret = mt25q_wait_idle(dev);
        if (ret != ESP_OK) return ret;

        bytes_written += page_bytes;
    }

    return ESP_OK;
}

esp_err_t mt25q_erase_sector(mt25q_dev_t *dev, uint32_t sector_addr) {
    esp_err_t ret;

    ret = mt25q_write_enable(dev);
    if (ret != ESP_OK) return ret;

    uint8_t cmd[4] = {
        MT25Q_CMD_SECTOR_ERASE,
        (sector_addr >> 16) & 0xFF,
        (sector_addr >> 8) & 0xFF,
        sector_addr & 0xFF
    };

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * 4;  // Command + 24-bit address (bits)
    t.tx_buffer = cmd;
    t.flags = 0;

    ret = spi_device_transmit(dev->spi, &t);
    if (ret != ESP_OK) return ret;

    return mt25q_wait_idle(dev);
}

esp_err_t mt25q_reset(mt25q_dev_t *dev) {
    esp_err_t ret;
    uint8_t cmd;

    // Send reset enable command
    cmd = MT25Q_CMD_RESET_ENABLE;
    spi_transaction_t t1;
    memset(&t1, 0, sizeof(t1));
    t1.length = 8;
    t1.tx_buffer = &cmd;
    t1.flags = 0;

    ret = spi_device_transmit(dev->spi, &t1);
    if (ret != ESP_OK) return ret;

    // Send reset command
    cmd = MT25Q_CMD_RESET_MEMORY;
    spi_transaction_t t2;
    memset(&t2, 0, sizeof(t2));
    t2.length = 8;
    t2.tx_buffer = &cmd;
    t2.flags = 0;

    ret = spi_device_transmit(dev->spi, &t2);
    if (ret != ESP_OK) return ret;

    // Wait for reset to complete
    vTaskDelay(pdMS_TO_TICKS(1));

    return ESP_OK;
}

esp_err_t mt25q_read_status(mt25q_dev_t *dev, uint8_t *status) {
    esp_err_t ret;
    uint8_t cmd = MT25Q_CMD_READ_STATUS;

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * 2;         // Command + 1 byte status (bits)
    t.tx_buffer = &cmd;
    t.rx_buffer = status;
    t.flags = 0;

    ret = spi_device_transmit(dev->spi, &t);
    return ret;
}

esp_err_t mt25q_wait_idle(mt25q_dev_t *dev) {
    esp_err_t ret;
    uint8_t status;
    int timeout = 100; // 1 second timeout (100 * 10ms)

    do {
        ret = mt25q_read_status(dev, &status);
        if (ret != ESP_OK) return ret;

        if (!(status & MT25Q_STATUS_WIP)) {
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    } while (--timeout > 0);

    return ESP_ERR_TIMEOUT;
}

static esp_err_t mt25q_write_enable(mt25q_dev_t *dev) {
    uint8_t cmd = MT25Q_CMD_WRITE_ENABLE;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &cmd;
    t.flags = 0;
    return spi_device_transmit(dev->spi, &t);
}

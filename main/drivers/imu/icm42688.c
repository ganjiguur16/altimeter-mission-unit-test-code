#include "icm42688.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ICM42688";

// Internal helper functions declarations
static esp_err_t icm42688_write_reg(icm42688_dev_t *dev, uint8_t reg, uint8_t data);
static esp_err_t icm42688_read_reg(icm42688_dev_t *dev, uint8_t reg, uint8_t *data);
static esp_err_t icm42688_read_reg_burst(icm42688_dev_t *dev, uint8_t reg, uint8_t *data, size_t len);

// Initialize device with I2C interface
esp_err_t icm42688_init_i2c(icm42688_dev_t *dev, i2c_master_bus_handle_t bus, uint8_t addr) {
    esp_err_t ret;
    uint8_t whoami;

    // Initialize device structure
    dev->is_spi = false;
    
    // Setup I2C device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 1000000, // 1MHz max for ICM42688
    };

    ret = i2c_master_bus_add_device(bus, &dev_cfg, &dev->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device");
        return ret;
    }

    // Check device ID
    ret = icm42688_read_reg(dev, ICM42688_REG_WHO_AM_I, &whoami);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return ret;
    }

    if (whoami != ICM42688_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "Wrong WHO_AM_I value: expected 0x%x, got 0x%x", 
                 ICM42688_WHO_AM_I_VAL, whoami);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "ICM42688 found on I2C");
    return ESP_OK;
}

// Configure device with specified settings
esp_err_t icm42688_configure(icm42688_dev_t *dev, const icm42688_config_t *config) {
    esp_err_t ret;

    // Store configuration
    dev->config = *config;

    // Software reset
    ret = icm42688_reset(dev);
    if (ret != ESP_OK) {
        return ret;
    }

    // Wait for reset to complete
    vTaskDelay(pdMS_TO_TICKS(1));

    // Configure accelerometer
    ret = icm42688_write_reg(dev, ICM42688_REG_ACCEL_CONFIG0, 
                            config->accel_fs | config->accel_odr);
    if (ret != ESP_OK) return ret;

    // Configure gyroscope
    ret = icm42688_write_reg(dev, ICM42688_REG_GYRO_CONFIG0,
                            config->gyro_fs | config->gyro_odr);
    if (ret != ESP_OK) return ret;

    // Configure power management
    uint8_t pwr_mgmt = ICM42688_PWR_MGMT0_TEMP_DIS_MASK;
    if (config->temp_en) {
        pwr_mgmt &= ~ICM42688_PWR_MGMT0_TEMP_DIS_MASK;
    }
    pwr_mgmt |= ICM42688_PWR_MGMT0_ACCEL_LN | ICM42688_PWR_MGMT0_GYRO_LN;
    
    ret = icm42688_write_reg(dev, ICM42688_REG_PWR_MGMT0, pwr_mgmt);
    if (ret != ESP_OK) return ret;

    // Configure FIFO if enabled
    if (config->fifo_en) {
        // TODO: Add FIFO configuration
    }

    ESP_LOGI(TAG, "ICM42688 configured successfully");
    return ESP_OK;
}

// Read raw sensor data
esp_err_t icm42688_read_raw_data(icm42688_dev_t *dev, icm42688_data_t *data) {
    uint8_t raw[14]; // Temperature + 6 axes with 2 bytes each
    esp_err_t ret;

    ret = icm42688_read_reg_burst(dev, ICM42688_REG_TEMP_DATA1, raw, sizeof(raw));
    if (ret != ESP_OK) return ret;

    // Temperature (2 bytes)
    int16_t temp_raw = (raw[0] << 8) | raw[1];
    data->temp = (temp_raw / 132.48f) + 25.0f;

    // Accelerometer (6 bytes)
    data->accel.x = (raw[2] << 8) | raw[3];
    data->accel.y = (raw[4] << 8) | raw[5];
    data->accel.z = (raw[6] << 8) | raw[7];

    // Gyroscope (6 bytes)
    data->gyro.x = (raw[8] << 8) | raw[9];
    data->gyro.y = (raw[10] << 8) | raw[11];
    data->gyro.z = (raw[12] << 8) | raw[13];

    return ESP_OK;
}

// Reset device
esp_err_t icm42688_reset(icm42688_dev_t *dev) {
    esp_err_t ret;
    uint8_t reg_val;

    // Perform software reset
    ret = icm42688_write_reg(dev, ICM42688_REG_DEVICE_CONFIG, 0x01);
    if (ret != ESP_OK) return ret;

    // Wait for reset to complete (up to 1ms)
    vTaskDelay(pdMS_TO_TICKS(1));

    // Verify reset completed
    ret = icm42688_read_reg(dev, ICM42688_REG_DEVICE_CONFIG, &reg_val);
    if (ret != ESP_OK) return ret;

    if (reg_val & 0x01) {
        ESP_LOGE(TAG, "Reset failed to complete");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

// Internal helper functions implementation
static esp_err_t icm42688_write_reg(icm42688_dev_t *dev, uint8_t reg, uint8_t data) {
    if (dev->is_spi) {
        // TODO: Implement SPI write
        return ESP_ERR_NOT_SUPPORTED;
    } else {
        uint8_t write_buf[2] = {reg, data};
        return i2c_master_transmit(dev->i2c_dev, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);
    }
}

static esp_err_t icm42688_read_reg(icm42688_dev_t *dev, uint8_t reg, uint8_t *data) {
    if (dev->is_spi) {
        // TODO: Implement SPI read
        return ESP_ERR_NOT_SUPPORTED;
    } else {
        return i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, data, 1, 1000 / portTICK_PERIOD_MS);
    }
}

static esp_err_t icm42688_read_reg_burst(icm42688_dev_t *dev, uint8_t reg, uint8_t *data, size_t len) {
    if (dev->is_spi) {
        // TODO: Implement SPI burst read
        return ESP_ERR_NOT_SUPPORTED;
    } else {
        return i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, data, len, 1000 / portTICK_PERIOD_MS);
    }
}
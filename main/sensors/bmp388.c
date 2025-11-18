#include "bmp388.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef CONFIG_I2C_MASTER_FREQUENCY
#define CONFIG_I2C_MASTER_FREQUENCY 100000
#endif

static esp_err_t read_reg8(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *val)
{
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, val, 1, 100 / portTICK_PERIOD_MS);
    return err;
}

esp_err_t bmp388_probe(i2c_master_bus_handle_t bus,
                       uint8_t addr,
                       i2c_master_dev_handle_t *out_dev,
                       uint8_t *out_chip_id)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
    /* CONFIG_I2C_MASTER_FREQUENCY may not be available to IntelliSense
       or when building outside the usual build system. Fall back to
       a safe default (100kHz) when it's not defined. */
#ifdef CONFIG_I2C_MASTER_FREQUENCY
    .scl_speed_hz = CONFIG_I2C_MASTER_FREQUENCY,
#else
    .scl_speed_hz = 100000,
#endif
    };

    i2c_master_dev_handle_t dev = NULL;
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &dev);
    if (err != ESP_OK) return err;

    uint8_t id = 0;
    err = read_reg8(dev, BMP388_REG_CHIP_ID, &id);
    if (err == ESP_OK && id == BMP388_CHIP_ID_VAL) {
        if (out_chip_id) *out_chip_id = id;
        if (out_dev) *out_dev = dev;
        return ESP_OK;
    }

    i2c_master_bus_rm_device(dev);
    return ESP_ERR_NOT_FOUND;
}

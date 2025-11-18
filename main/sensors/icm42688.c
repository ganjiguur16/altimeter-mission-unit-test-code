#include "icm42688.h"
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

esp_err_t icm42688_probe(i2c_master_bus_handle_t bus,
                         uint8_t addr,
                         i2c_master_dev_handle_t *out_dev,
                         uint8_t *out_whoami)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
    /* Prefer project config frequency when available, otherwise 100kHz */
#ifdef CONFIG_I2C_MASTER_FREQUENCY
    .scl_speed_hz = CONFIG_I2C_MASTER_FREQUENCY,
#else
    .scl_speed_hz = 100000,
#endif
    };

    i2c_master_dev_handle_t dev = NULL;
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &dev);
    if (err != ESP_OK) return err;

    uint8_t who = 0;
    err = read_reg8(dev, ICM42688_REG_WHO_AM_I, &who);
    if (err == ESP_OK && who == ICM42688_WHO_AM_I_VAL) {
        if (out_whoami) *out_whoami = who;
        if (out_dev) *out_dev = dev;
        return ESP_OK;
    }

    /* Not our device */
    i2c_master_bus_rm_device(dev);
    return ESP_ERR_NOT_FOUND;
}

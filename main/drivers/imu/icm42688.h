#pragma once
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"

// Reuse existing register definitions from sensors/icm42688.h
#include "../../sensors/icm42688.h"

// Data structures
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} icm42688_axis_t;

typedef struct {
    icm42688_axis_t accel;  // Accelerometer data
    icm42688_axis_t gyro;   // Gyroscope data
    float temp;             // Temperature data
} icm42688_data_t;

typedef struct {
    uint8_t accel_fs;      // Accelerometer full-scale range
    uint8_t gyro_fs;       // Gyroscope full-scale range
    uint8_t accel_odr;     // Accelerometer output data rate
    uint8_t gyro_odr;      // Gyroscope output data rate
    bool temp_en;          // Temperature sensor enable
    bool fifo_en;          // FIFO enable
} icm42688_config_t;

typedef struct {
    union {
        i2c_master_dev_handle_t i2c_dev;
        spi_device_handle_t spi_dev;
    };
    bool is_spi;           // Interface type (true for SPI, false for I2C)
    icm42688_config_t config;
} icm42688_dev_t;

// Default configuration
#define ICM42688_DEFAULT_CONFIG() {\
    .accel_fs = ICM42688_ACCEL_CONFIG0_FS_SEL_16G,\
    .gyro_fs = ICM42688_GYRO_CONFIG0_FS_SEL_2000,\
    .accel_odr = ICM42688_ODR_1KHZ,\
    .gyro_odr = ICM42688_ODR_1KHZ,\
    .temp_en = true,\
    .fifo_en = false\
}

// API Function declarations
esp_err_t icm42688_init_i2c(icm42688_dev_t *dev, i2c_master_bus_handle_t bus, uint8_t addr);
esp_err_t icm42688_init_spi(icm42688_dev_t *dev, spi_host_device_t host, int cs_pin);
esp_err_t icm42688_configure(icm42688_dev_t *dev, const icm42688_config_t *config);
esp_err_t icm42688_set_power_mode(icm42688_dev_t *dev, uint8_t mode);
esp_err_t icm42688_read_raw_data(icm42688_dev_t *dev, icm42688_data_t *data);
esp_err_t icm42688_read_temperature(icm42688_dev_t *dev, float *temp);
esp_err_t icm42688_fifo_read(icm42688_dev_t *dev, icm42688_data_t *data, size_t *samples);
esp_err_t icm42688_self_test(icm42688_dev_t *dev);
esp_err_t icm42688_set_interrupt(icm42688_dev_t *dev, uint8_t int_config);
esp_err_t icm42688_get_interrupt_status(icm42688_dev_t *dev, uint8_t *status);
esp_err_t icm42688_reset(icm42688_dev_t *dev);
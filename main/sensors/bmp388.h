// Minimal BMP388 probe helpers (I2C)
#pragma once
#include "esp_err.h"
#include "driver/i2c_master.h"

// Device identification
#define BMP388_REG_CHIP_ID 0x00
#define BMP388_CHIP_ID_VAL 0x50

// Error register
#define BMP388_REG_ERR 0x02
#define BMP388_ERR_FATAL_MSK UINT8_C(0x01)
#define BMP388_ERR_CMD_MSK   UINT8_C(0x02)
#define BMP388_ERR_CONF_MSK  UINT8_C(0x04)

// Status register
#define BMP388_REG_STATUS 0x03
#define BMP388_STATUS_CMD_RDY_MSK    UINT8_C(0x10)
#define BMP388_STATUS_DRDY_PRESS_MSK UINT8_C(0x20)
#define BMP388_STATUS_DRDY_TEMP_MSK  UINT8_C(0x40)

// Data registers
#define BMP388_REG_DATA  0x04
#define BMP388_REG_EVENT 0x10
#define BMP388_REG_INT_STATUS 0x11

// Configuration registers
#define BMP388_REG_PWR_CTRL 0x1B
#define BMP388_PWR_CTRL_PRESS_EN  UINT8_C(0x01)
#define BMP388_PWR_CTRL_TEMP_EN   UINT8_C(0x02)
#define BMP388_PWR_CTRL_MODE_MSK  UINT8_C(0x30)
#define BMP388_PWR_CTRL_MODE_SLEEP  UINT8_C(0x00)
#define BMP388_PWR_CTRL_MODE_FORCED UINT8_C(0x10)
#define BMP388_PWR_CTRL_MODE_NORMAL UINT8_C(0x30)

#define BMP388_REG_OSR       0x1C  // Oversampling register
#define BMP388_OSR_PRESS_MSK UINT8_C(0x07)
#define BMP388_OSR_TEMP_MSK  UINT8_C(0x38)
#define BMP388_OSR_PRESS_POS 0
#define BMP388_OSR_TEMP_POS  3

#define BMP388_REG_ODR       0x1D  // Output data rate register
#define BMP388_ODR_MSK      UINT8_C(0x1F)

#define BMP388_REG_CONFIG    0x1F
#define BMP388_CONFIG_IIR_FILTER_MSK UINT8_C(0x0E)
#define BMP388_CONFIG_IIR_FILTER_POS 1

// Calibration registers
#define BMP388_REG_CALIB_DATA 0x31
#define BMP388_LEN_CALIB_DATA UINT8_C(21)

// Command register
#define BMP388_REG_CMD       0x7E
#define BMP388_CMD_SOFT_RESET UINT8_C(0xB6)

// Oversampling macros 
#define BMP388_NO_OVERSAMPLING  UINT8_C(0x00)
#define BMP388_OVERSAMPLING_2X  UINT8_C(0x01)
#define BMP388_OVERSAMPLING_4X  UINT8_C(0x02)
#define BMP388_OVERSAMPLING_8X  UINT8_C(0x03)
#define BMP388_OVERSAMPLING_16X UINT8_C(0x04)
#define BMP388_OVERSAMPLING_32X UINT8_C(0x05)

// IIR filter settings
#define BMP388_IIR_FILTER_DISABLE UINT8_C(0x00)
#define BMP388_IIR_FILTER_COEFF_1 UINT8_C(0x01)
#define BMP388_IIR_FILTER_COEFF_3 UINT8_C(0x02)
#define BMP388_IIR_FILTER_COEFF_7 UINT8_C(0x03)
#define BMP388_IIR_FILTER_COEFF_15 UINT8_C(0x04)
#define BMP388_IIR_FILTER_COEFF_31 UINT8_C(0x05)
#define BMP388_IIR_FILTER_COEFF_63 UINT8_C(0x06)
#define BMP388_IIR_FILTER_COEFF_127 UINT8_C(0x07)

// Output data rate settings
#define BMP388_ODR_200_HZ  UINT8_C(0x00)
#define BMP388_ODR_100_HZ  UINT8_C(0x01)
#define BMP388_ODR_50_HZ   UINT8_C(0x02)
#define BMP388_ODR_25_HZ   UINT8_C(0x03)
#define BMP388_ODR_12_5_HZ UINT8_C(0x04)
#define BMP388_ODR_6_25_HZ UINT8_C(0x05)
#define BMP388_ODR_3_1_HZ  UINT8_C(0x06)
#define BMP388_ODR_1_5_HZ  UINT8_C(0x07)
#define BMP388_ODR_0_78_HZ UINT8_C(0x08)
#define BMP388_ODR_0_39_HZ UINT8_C(0x09)
#define BMP388_ODR_0_2_HZ  UINT8_C(0x0A)
#define BMP388_ODR_0_1_HZ  UINT8_C(0x0B)
#define BMP388_ODR_0_05_HZ UINT8_C(0x0C)
#define BMP388_ODR_0_02_HZ UINT8_C(0x0D)
#define BMP388_ODR_0_01_HZ UINT8_C(0x0E)
#define BMP388_ODR_0_006_HZ UINT8_C(0x0F)
#define BMP388_ODR_0_003_HZ UINT8_C(0x10)
#define BMP388_ODR_0_001_HZ UINT8_C(0x11)

// API Function declarations
esp_err_t bmp388_probe(i2c_master_bus_handle_t bus,
                       uint8_t addr,
                       i2c_master_dev_handle_t *out_dev,
                       uint8_t *out_chip_id);

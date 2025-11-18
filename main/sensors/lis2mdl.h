// Minimal LIS2MDL probe helpers (I2C)
#pragma once
#include "esp_err.h"
#include "driver/i2c_master.h"

// Device identification
#define LIS2MDL_REG_WHO_AM_I 0x4F
#define LIS2MDL_WHO_AM_I_VAL 0x40

// Offset registers
#define LIS2MDL_REG_OFFSET_X_L 0x45
#define LIS2MDL_REG_OFFSET_X_H 0x46
#define LIS2MDL_REG_OFFSET_Y_L 0x47
#define LIS2MDL_REG_OFFSET_Y_H 0x48
#define LIS2MDL_REG_OFFSET_Z_L 0x49
#define LIS2MDL_REG_OFFSET_Z_H 0x4A

// Control and configuration registers
#define LIS2MDL_REG_CFG_A 0x60
#define LIS2MDL_CFG_A_TEMP_COMP_EN  UINT8_C(0x80)
#define LIS2MDL_CFG_A_ODR_MASK      UINT8_C(0x0C)
#define LIS2MDL_CFG_A_ODR_10HZ      UINT8_C(0x00)
#define LIS2MDL_CFG_A_ODR_20HZ      UINT8_C(0x04)
#define LIS2MDL_CFG_A_ODR_50HZ      UINT8_C(0x08)
#define LIS2MDL_CFG_A_ODR_100HZ     UINT8_C(0x0C)
#define LIS2MDL_CFG_A_MD_MASK       UINT8_C(0x03)
#define LIS2MDL_CFG_A_MD_CONTINUOUS UINT8_C(0x00)
#define LIS2MDL_CFG_A_MD_SINGLE     UINT8_C(0x01)
#define LIS2MDL_CFG_A_MD_IDLE       UINT8_C(0x03)

#define LIS2MDL_REG_CFG_B 0x61
#define LIS2MDL_CFG_B_OFF_CANC      UINT8_C(0x02)
#define LIS2MDL_CFG_B_INT_MAG       UINT8_C(0x01)
#define LIS2MDL_CFG_B_OFF_CANC_ONE_SHOT UINT8_C(0x10)

#define LIS2MDL_REG_CFG_C 0x62
#define LIS2MDL_CFG_C_BDU           UINT8_C(0x10)
#define LIS2MDL_CFG_C_I2C_DIS       UINT8_C(0x04)
#define LIS2MDL_CFG_C_INT_MAG_PIN   UINT8_C(0x01)

// Interrupt control registers
#define LIS2MDL_REG_INT_CTRL 0x63
#define LIS2MDL_INT_CTRL_IEN        UINT8_C(0x01)
#define LIS2MDL_INT_CTRL_IEL        UINT8_C(0x02)
#define LIS2MDL_INT_CTRL_IEA        UINT8_C(0x04)
#define LIS2MDL_INT_CTRL_ZIEN       UINT8_C(0x20)
#define LIS2MDL_INT_CTRL_YIEN       UINT8_C(0x10)
#define LIS2MDL_INT_CTRL_XIEN       UINT8_C(0x08)

#define LIS2MDL_REG_INT_SOURCE 0x64
#define LIS2MDL_INT_SOURCE_INT      UINT8_C(0x01)
#define LIS2MDL_INT_SOURCE_MROI     UINT8_C(0x02)
#define LIS2MDL_INT_SOURCE_N_TH     UINT8_C(0x04)
#define LIS2MDL_INT_SOURCE_P_TH     UINT8_C(0x08)
#define LIS2MDL_INT_SOURCE_PTH_X    UINT8_C(0x10)
#define LIS2MDL_INT_SOURCE_PTH_Y    UINT8_C(0x20)
#define LIS2MDL_INT_SOURCE_PTH_Z    UINT8_C(0x40)

#define LIS2MDL_REG_INT_THS_L 0x65
#define LIS2MDL_REG_INT_THS_H 0x66

// Status register
#define LIS2MDL_REG_STATUS 0x67
#define LIS2MDL_STATUS_ZYXOR        UINT8_C(0x80)
#define LIS2MDL_STATUS_ZOR          UINT8_C(0x40)
#define LIS2MDL_STATUS_YOR          UINT8_C(0x20)
#define LIS2MDL_STATUS_XOR          UINT8_C(0x10)
#define LIS2MDL_STATUS_ZYXDA        UINT8_C(0x08)
#define LIS2MDL_STATUS_ZDA          UINT8_C(0x04)
#define LIS2MDL_STATUS_YDA          UINT8_C(0x02)
#define LIS2MDL_STATUS_XDA          UINT8_C(0x01)

// Output data registers
#define LIS2MDL_REG_OUTX_L 0x68
#define LIS2MDL_REG_OUTX_H 0x69
#define LIS2MDL_REG_OUTY_L 0x6A
#define LIS2MDL_REG_OUTY_H 0x6B
#define LIS2MDL_REG_OUTZ_L 0x6C
#define LIS2MDL_REG_OUTZ_H 0x6D

// Temperature sensor registers
#define LIS2MDL_REG_TEMP_OUT_L 0x6E
#define LIS2MDL_REG_TEMP_OUT_H 0x6F

// API Function declarations
esp_err_t lis2mdl_probe(i2c_master_bus_handle_t bus,
                        uint8_t addr,
                        i2c_master_dev_handle_t *out_dev,
                        uint8_t *out_whoami);


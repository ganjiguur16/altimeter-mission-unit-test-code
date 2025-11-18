#pragma once
#include "esp_err.h"
#include "driver/spi_master.h"

// Device identification
#define MT25Q_MANUFACTURER_ID    0x20
#define MT25Q_MEMORY_TYPE       0xBA
#define MT25Q_MEMORY_CAPACITY   0x20 // 512Mb

// Command Set
#define MT25Q_CMD_RESET_ENABLE          0x66
#define MT25Q_CMD_RESET_MEMORY          0x99
#define MT25Q_CMD_READ_ID              0x9F
#define MT25Q_CMD_READ                 0x03
#define MT25Q_CMD_FAST_READ           0x0B
#define MT25Q_CMD_WRITE_ENABLE        0x06
#define MT25Q_CMD_WRITE_DISABLE       0x04
#define MT25Q_CMD_READ_STATUS         0x05
#define MT25Q_CMD_WRITE_STATUS        0x01
#define MT25Q_CMD_PAGE_PROGRAM        0x02
#define MT25Q_CMD_SECTOR_ERASE        0x20  // 4KB sector
#define MT25Q_CMD_BLOCK_ERASE_32K    0x52
#define MT25Q_CMD_BLOCK_ERASE_64K    0xD8
#define MT25Q_CMD_CHIP_ERASE         0xC7

// Status Register Bits
#define MT25Q_STATUS_WIP              (1 << 0)  // Write in progress
#define MT25Q_STATUS_WEL              (1 << 1)  // Write enable latch
#define MT25Q_STATUS_BP0              (1 << 2)  // Block protect bits
#define MT25Q_STATUS_BP1              (1 << 3)
#define MT25Q_STATUS_BP2              (1<< 4)
#define MT25Q_STATUS_QE               (1 << 6)  // Quad enable
#define MT25Q_STATUS_SRWD             (1 << 7)  // Status register write protect

// Device Parameters
#define MT25Q_PAGE_SIZE              256
#define MT25Q_SECTOR_SIZE            4096    // 4KB
#define MT25Q_BLOCK32K_SIZE          32768   // 32KB
#define MT25Q_BLOCK64K_SIZE          65536   // 64KB
#define MT25Q_CHIP_SIZE              67108864 // 512Mb = 64MB

typedef struct {
    spi_device_handle_t spi;
    int cs_pin;
} mt25q_dev_t;

// API Function declarations
esp_err_t mt25q_init(mt25q_dev_t *dev, spi_host_device_t host_id, int cs_pin);
esp_err_t mt25q_read_id(mt25q_dev_t *dev, uint8_t *manufacturer, uint8_t *memory_type, uint8_t *capacity);
esp_err_t mt25q_read(mt25q_dev_t *dev, uint32_t addr, void *data, size_t len);
esp_err_t mt25q_write(mt25q_dev_t *dev, uint32_t addr, const void *data, size_t len);
esp_err_t mt25q_erase_sector(mt25q_dev_t *dev, uint32_t sector_addr);
esp_err_t mt25q_erase_block32k(mt25q_dev_t *dev, uint32_t block_addr);
esp_err_t mt25q_erase_block64k(mt25q_dev_t *dev, uint32_t block_addr);
esp_err_t mt25q_erase_chip(mt25q_dev_t *dev);
esp_err_t mt25q_reset(mt25q_dev_t *dev);

// Helper functions
esp_err_t mt25q_read_status(mt25q_dev_t *dev, uint8_t *status);
esp_err_t mt25q_wait_idle(mt25q_dev_t *dev);
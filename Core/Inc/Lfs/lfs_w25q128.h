#ifndef LITTLEFS_W25Q128_H
#define LITTLEFS_W25Q128_H

#include "lfs.h"

// Function Prototypes for integrating W25Q128 with LittleFS

// Read function: Reads data from W25Q128 flash memory.
int w25q128_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);

// Write function: Writes data to W25Q128 flash memory.
int w25q128_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);

// Erase function: Erases a sector in W25Q128 flash memory.
int w25q128_erase(const struct lfs_config *c, lfs_block_t block);

// Sync function: This function is used to sync any dirty blocks.
int w25q128_sync(const struct lfs_config *c);

// Optional utility to check the ID of W25Q128 chip.
uint32_t W25Q128_ReadID(void);

// Utility to initialize the SPI interface and W25Q128 chip.
void W25Q128_Init(void);


#endif // LITTLEFS_W25Q128_H

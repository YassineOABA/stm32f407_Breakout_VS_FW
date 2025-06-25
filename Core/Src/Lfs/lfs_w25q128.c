#include "lfs_w25q128.h"
#include "w25q128jv.h"

extern struct w25q128jv_dev w25q128jv;

// Read function
int w25q128_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) {
    W25Q128JV_ReadData(&w25q128jv, (block * c->block_size) + off, (uint8_t *)buffer, size);
    return 0;
}

// Write function
int w25q128_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) {
    W25Q128JV_PageProgram(&w25q128jv, (block * c->block_size) + off, (uint8_t *)buffer, size);
    return 0;
}

// Erase function
int w25q128_erase(const struct lfs_config *c, lfs_block_t block) {
    W25Q128JV_EraseSector(&w25q128jv, block * c->block_size);
    return 0;
}

// Sync function
int w25q128_sync(const struct lfs_config *c) {
    return 0;  // Not needed for SPI flash
}

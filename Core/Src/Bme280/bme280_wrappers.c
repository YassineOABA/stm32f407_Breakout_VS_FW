/**
 * Copyright (C) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bme280.h"
#include "bme280_wrappers.h"

/******************************************************************************/
/*!                               Macros                                      */

#define BME280_SHUTTLE_ID  UINT8_C(0x33)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to STM32 platform
 */
BME280_INTF_RET_TYPE bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, struct bme280_dev *dev)
{
    return HAL_I2C_Mem_Read(dev->hi2c, (uint16_t)(BME280_I2C_ADDR_PRIM << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, length, HAL_MAX_DELAY);
}

/*!
 * I2C write function map to STM32 platform
 */
BME280_INTF_RET_TYPE bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, struct bme280_dev *dev)
{
    return HAL_I2C_Mem_Write(dev->hi2c, (uint16_t)(BME280_I2C_ADDR_PRIM << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)reg_data, length, HAL_MAX_DELAY);
}

/*!
 * I2C read function map to STM32 platform
 */
BME280_INTF_RET_TYPE bme280_i2c_read_dma(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, struct bme280_dev *dev)
{
    return HAL_I2C_Mem_Read_DMA(dev->hi2c, (uint16_t)(BME280_I2C_ADDR_PRIM << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, length);
}

/*!
 * I2C write function map to STM32 platform
 */
BME280_INTF_RET_TYPE bme280_i2c_write_dma(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, struct bme280_dev *dev)
{
    return HAL_I2C_Mem_Write_DMA(dev->hi2c, (uint16_t)(BME280_I2C_ADDR_PRIM << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)reg_data, length);
}

/*!
 * Delay function map to STM32 platform
 */
void bme280_delay_us(uint32_t period)
{
    HAL_Delay_us(period);
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
uint8_t bme280_interface_init(struct bme280_dev *dev, I2C_HandleTypeDef *hi2c)
{
    uint8_t rslt = BME280_OK;

    if (dev != NULL)
    {
        /* Bus configuration : I2C */
        dev->hi2c       = hi2c;
        dev->read       = bme280_i2c_read;
        dev->write      = bme280_i2c_write;
        dev->read_dma   = bme280_i2c_read_dma;
        dev->write_dma  = bme280_i2c_write_dma;
        dev->delay_us   = bme280_delay_us;
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief Function deinitializes coines platform.
 */
void bme280_deinit(struct bme280_dev *dev)
{
    HAL_I2C_DeInit(dev->hi2c);
}

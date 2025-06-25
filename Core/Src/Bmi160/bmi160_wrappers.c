/**
 * Copyright (C) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bmi160_wrappers.h"
#include "bmi160_defs.h"

/******************************************************************************/
/*!                 Macro definitions                                         */

/*! Macro that defines read write length */
#define READ_WRITE_LEN     UINT8_C(46)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */

/*! Variable that holds the I2C or SPI bus instance */

/*! Structure to hold interface configurations */

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * SPI read function map to STM32 platform
 */
uint8_t bmi160_spi_read(struct bmi160_dev *dev, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    uint8_t rslt = BMI160_OK;
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    dev->delay_ms(1);
    rslt = HAL_SPI_Transmit(dev->hspi, &reg_addr, 1, HAL_MAX_DELAY);
    rslt += HAL_SPI_Receive(dev->hspi, reg_data, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    dev->delay_ms(1);
    return rslt;
}

/*!
 * SPI write function map to STM32 platform
 */
uint8_t bmi160_spi_write(struct bmi160_dev *dev, uint8_t reg_addr, const uint8_t *reg_data, uint16_t len)
{
    uint8_t rslt = BMI160_OK;
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    dev->delay_ms(1);
    rslt = HAL_SPI_Transmit(dev->hspi, &reg_addr, 1, HAL_MAX_DELAY);
    rslt += HAL_SPI_Transmit(dev->hspi, reg_data, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    dev->delay_ms(1);
    return rslt;
}

/*!
 *  @brief Function to initialize the SPI interface.
 */
uint8_t bmi160_interface_init(struct bmi160_dev *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    uint8_t rslt = BMI160_OK;

    if (dev != NULL)
    {
        /* Bus configuration : SPI */
        /* To initialize the user SPI function */
        dev->hspi       = hspi;
        dev->cs_port    = cs_port;
        dev->cs_pin     = cs_pin;
        dev->read       = bmi160_spi_read;
        dev->write      = bmi160_spi_write;
        dev->delay_ms   = HAL_Delay;
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    }
    else
    {
        rslt = BMI160_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief Deinitializes SPI
 *
 *  @return void.
 */
void bmi160_deinit(struct bmi160_dev *bmi)
{
    HAL_SPI_DeInit(bmi->hspi);
}
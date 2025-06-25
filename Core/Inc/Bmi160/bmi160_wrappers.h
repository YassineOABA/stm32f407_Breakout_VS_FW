/**\
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#ifndef _BMI160_WRAPPERS_H
#define _BMI160_WRAPPERS_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#include "bmi160.h"

/******************************************************************************/
/* Structure declarations */
/******************************************************************************/

/******************************************************************************/
/*!                       Function Definitions                                */

/*!
 *  @brief Function for reading the sensor's registers through SPI bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length       : No of bytes to read.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = bmi160_INTF_RET_SUCCESS -> Success
 *  @retval != bmi160_INTF_RET_SUCCESS  -> Failure Info
 *
 */
uint8_t bmi160_spi_read(struct bmi160_dev *dev, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

/*!
 *  @brief Function for writing the sensor's registers through SPI bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data     : Pointer to the data buffer whose data has to be written.
 *  @param[in] length       : No of bytes to write.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = bmi160_INTF_RET_SUCCESS -> Success
 *  @retval != bmi160_INTF_RET_SUCCESS  -> Failure Info
 *
 */
uint8_t bmi160_spi_write(struct bmi160_dev *dev, uint8_t reg_addr, const uint8_t *reg_data, uint16_t len);

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *
 *  @param[in] bmi    : Structure instance of bmi160_dev
 *  @param[in] intf   : Interface selection parameter
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
uint8_t bmi160_interface_init(struct bmi160_dev *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* _BMI160_WRAPPERS_H */

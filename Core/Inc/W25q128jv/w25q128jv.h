#ifndef W25Q128JV_H
#define W25Q128JV_H
#include "stm32f4xx_hal.h"

struct w25q128jv_dev
{
    SPI_HandleTypeDef * hspi;

    /*! CS port */
    GPIO_TypeDef *cs_port;
    /*! CS Pin*/
    uint16_t cs_pin;

    /*! WP port */
    GPIO_TypeDef *wp_port;
    /*! WP Pin*/
    uint16_t wp_pin;
      
    /*! HLD port */
    GPIO_TypeDef *hld_port;
    /*! HLD Pin*/
    uint16_t hld_pin;

    /*! Chip ID*/
    uint8_t jedec_id[3];

    uint8_t unique_id[8];
};

void W25Q128JV_interface_Init(  struct w25q128jv_dev *dev,
                                SPI_HandleTypeDef * hspi,
                                GPIO_TypeDef *cs_port,
                                uint16_t cs_pin,
                                GPIO_TypeDef *wp_port,
                                uint16_t wp_pin,
                                GPIO_TypeDef *hld_port,
                                uint16_t hld_pin);
void W25Q128JV_Init(struct w25q128jv_dev *dev);
void W25Q128JV_WriteEnable(struct w25q128jv_dev *dev);
uint8_t W25Q128JV_ReadStatus1(struct w25q128jv_dev *dev);
uint8_t W25Q128JV_ReadStatus2(struct w25q128jv_dev *dev);
uint8_t W25Q128JV_ReadStatus3(struct w25q128jv_dev *dev);
void W25Q128JV_ReadJedecID(struct w25q128jv_dev *dev);
void W25Q128JV_ReadUniqueID(struct w25q128jv_dev *dev);
void W25Q128JV_ReadData(struct w25q128jv_dev *dev, uint32_t address, uint8_t *buffer, uint16_t length);
void W25Q128JV_PageProgram(struct w25q128jv_dev *dev, uint32_t address, uint8_t *buffer, uint16_t length);
void W25Q128JV_EraseSector(struct w25q128jv_dev *dev, uint32_t address);
void W25Q128JV_EraseChip(struct w25q128jv_dev *dev);

#endif /* W25Q128JV_H */

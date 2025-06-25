#include "w25q128jv.h" // Change according to your STM32 series

#define W25Q128JV_CMD_WRITE_EN                     0x06
#define W25Q128JV_CMD_WRITE_EN_VOLATILE            0x50
#define W25Q128JV_CMD_WRITE_DISABLE                0x04
#define W25Q128JV_CMD_READ_STATUS_1                0x05
#define W25Q128JV_CMD_READ_STATUS_2                0x35
#define W25Q128JV_CMD_READ_STATUS_3                0x15
#define W25Q128JV_CMD_WRITE_STATUS_1               0x01
#define W25Q128JV_CMD_WRITE_STATUS_2               0x31
#define W25Q128JV_CMD_WRITE_STATUS_3               0x11
#define W25Q128JV_CMD_READ                         0x03
#define W25Q128JV_CMD_FAST_READ                    0x0B
#define W25Q128JV_CMD_FAST_READ_DUAL_OUTPUT        0x3B
#define W25Q128JV_CMD_FAST_READ_QUAD_OUTPUT        0x6B
#define W25Q128JV_CMD_FAST_READ_DUAL_IO            0xBB
#define W25Q128JV_CMD_FAST_READ_QUAD_IO            0xEB
#define W25Q128JV_CMD_SET_BURST_WITH_WRAP         0x77
#define W25Q128JV_CMD_PAGE_PROGRAM                0x02
#define W25Q128JV_CMD_QUAD_INPUT_PAGE_PROGRAM     0x32
#define W25Q128JV_CMD_SECTOR_ERASE                0x20
#define W25Q128JV_CMD_BLOCK_ERASE_32KB            0x52
#define W25Q128JV_CMD_BLOCK_ERASE_64KB            0xD8
#define W25Q128JV_CMD_CHIP_ERASE_C7               0xC7
#define W25Q128JV_CMD_CHIP_ERASE_60               0x60
#define W25Q128JV_CMD_ERASE_PROGRAM_SUSPEND       0x75
#define W25Q128JV_CMD_ERASE_PROGRAM_RESUME        0x7A
#define W25Q128JV_CMD_POWER_DOWN                  0xB9
#define W25Q128JV_CMD_RELEASE_POWER_DOWN_ID       0xAB
#define W25Q128JV_CMD_READ_MANUFACTURER_ID        0x90
#define W25Q128JV_CMD_READ_MANUFACTURER_ID_DUAL   0x92
#define W25Q128JV_CMD_READ_MANUFACTURER_ID_QUAD   0x94
#define W25Q128JV_CMD_READ_UNIQUE_ID              0x4B
#define W25Q128JV_CMD_READ_JEDEC_ID               0x9F
#define W25Q128JV_CMD_READ_SFDP_REGISTER          0x5A
#define W25Q128JV_CMD_ERASE_SECURITY_REGISTERS    0x44
#define W25Q128JV_CMD_PROGRAM_SECURITY_REGISTERS  0x42
#define W25Q128JV_CMD_READ_SECURITY_REGISTERS     0x48
#define W25Q128JV_CMD_INDIVIDUAL_BLOCK_LOCK       0x36
#define W25Q128JV_CMD_INDIVIDUAL_BLOCK_UNLOCK     0x39
#define W25Q128JV_CMD_READ_BLOCK_LOCK             0x3D
#define W25Q128JV_CMD_GLOBAL_BLOCK_LOCK           0x7E
#define W25Q128JV_CMD_GLOBAL_BLOCK_UNLOCK         0x98
#define W25Q128JV_CMD_ENABLE_RESET                0x66
#define W25Q128JV_CMD_RESET_DEVICE                0x99

static void W25Q128JV_CS_LOW(struct w25q128jv_dev *dev)
{
    HAL_GPIO_WritePin(dev->cs_port,dev->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
}

static void W25Q128JV_CS_HIGH(struct w25q128jv_dev *dev)
{
    HAL_GPIO_WritePin(dev->cs_port,dev->cs_pin, GPIO_PIN_SET);
    HAL_Delay(1);
}

static void W25Q128JV_HOLD_DISABLE(struct w25q128jv_dev *dev)
{
    HAL_GPIO_WritePin(dev->hld_port, dev->hld_pin, GPIO_PIN_SET);
    HAL_Delay(1);
}

static void W25Q128JV_HOLD_ENABLE(struct w25q128jv_dev *dev)
{
    HAL_GPIO_WritePin(dev->hld_port, dev->hld_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
}

static void W25Q128JV_WP_DISABLE(struct w25q128jv_dev *dev)
{
    HAL_GPIO_WritePin(dev->wp_port, dev->wp_pin, GPIO_PIN_SET);
    HAL_Delay(1);
}

static void W25Q128JV_WP_ENABLE(struct w25q128jv_dev *dev)
{
    HAL_GPIO_WritePin(dev->wp_port, dev->wp_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
}

void W25Q128JV_interface_Init(  struct w25q128jv_dev *dev,
                                SPI_HandleTypeDef * hspi,
                                GPIO_TypeDef *cs_port,
                                uint16_t cs_pin,
                                GPIO_TypeDef *wp_port,
                                uint16_t wp_pin,
                                GPIO_TypeDef *hld_port,
                                uint16_t hld_pin)
{
    dev->hspi       = hspi;
    dev->cs_port    = cs_port;
    dev->cs_pin     = cs_pin;
    dev->wp_port    = wp_port;
    dev->wp_pin     = wp_pin;
    dev->hld_port   = hld_port;
    dev->hld_pin    = hld_pin;

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

void W25Q128JV_Init(struct w25q128jv_dev *dev) 
{
    W25Q128JV_HOLD_DISABLE(dev);
    W25Q128JV_WP_DISABLE(dev);
}

void W25Q128JV_WriteEnable(struct w25q128jv_dev *dev) 
{
    uint8_t cmd = W25Q128JV_CMD_WRITE_EN;
    W25Q128JV_CS_LOW(dev);
    HAL_SPI_Transmit(dev->hspi, &cmd, 1, 100);
    W25Q128JV_CS_HIGH(dev);
}

uint8_t W25Q128JV_ReadStatus1(struct w25q128jv_dev *dev) 
{
    uint8_t cmd = W25Q128JV_CMD_READ_STATUS_1, status;
    W25Q128JV_CS_LOW(dev);
    HAL_SPI_Transmit(dev->hspi, &cmd, 1, 100);
    HAL_SPI_Receive(dev->hspi, &status, 1, 100);
    W25Q128JV_CS_HIGH(dev);
    return status;
}

uint8_t W25Q128JV_ReadStatus2(struct w25q128jv_dev *dev) 
{
    uint8_t cmd = W25Q128JV_CMD_READ_STATUS_2, status;
    W25Q128JV_CS_LOW(dev);
    HAL_SPI_Transmit(dev->hspi, &cmd, 1, 100);
    HAL_SPI_Receive(dev->hspi, &status, 1, 100);
    W25Q128JV_CS_HIGH(dev);
    return status;
}

uint8_t W25Q128JV_ReadStatus3(struct w25q128jv_dev *dev) 
{
    uint8_t cmd = W25Q128JV_CMD_READ_STATUS_3, status;
    W25Q128JV_CS_LOW(dev);
    HAL_SPI_Transmit(dev->hspi, &cmd, 1, 100);
    HAL_SPI_Receive(dev->hspi, &status, 1, 100);
    W25Q128JV_CS_HIGH(dev);
    return status;
}

void W25Q128JV_ReadJedecID(struct w25q128jv_dev *dev)
{
    uint8_t cmd = W25Q128JV_CMD_READ_JEDEC_ID;
    uint8_t rslt = 0;
    W25Q128JV_CS_LOW(dev);
    rslt = HAL_SPI_Transmit(dev->hspi, &cmd, 1, 100);
    rslt += HAL_SPI_Receive(dev->hspi, dev->jedec_id, 3, 100);
    W25Q128JV_CS_HIGH(dev);
}

void W25Q128JV_ReadUniqueID(struct w25q128jv_dev *dev)
{
    uint8_t cmd[5] = {W25Q128JV_CMD_READ_UNIQUE_ID,0xFF,0xFF,0xFF,0xFF};
    uint8_t rslt = 0;
    W25Q128JV_CS_LOW(dev);
    rslt = HAL_SPI_Transmit(dev->hspi, cmd, 5, 100);
    rslt += HAL_SPI_Receive(dev->hspi, dev->unique_id, 8, 100);
    W25Q128JV_CS_HIGH(dev);
}

void W25Q128JV_ReadData(struct w25q128jv_dev *dev, uint32_t address, uint8_t *buffer, uint16_t length)
{
    uint8_t cmd[4] = {W25Q128JV_CMD_READ, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};
    
    W25Q128JV_CS_LOW(dev);
    HAL_SPI_Transmit(dev->hspi, cmd, 4, 100);
    HAL_SPI_Receive(dev->hspi, buffer, length, 1000);
    W25Q128JV_CS_HIGH(dev);
}

void W25Q128JV_PageProgram(struct w25q128jv_dev *dev, uint32_t address, uint8_t *buffer, uint16_t length)
{
    uint8_t cmd[4] = {W25Q128JV_CMD_PAGE_PROGRAM, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};

    W25Q128JV_WriteEnable(dev);
    W25Q128JV_CS_LOW(dev);
    HAL_SPI_Transmit(dev->hspi, cmd, 4, 100);
    HAL_SPI_Transmit(dev->hspi, buffer, length, 1000);
    W25Q128JV_CS_HIGH(dev);
    HAL_Delay(5); // Wait for write completion
}

void W25Q128JV_EraseSector(struct w25q128jv_dev *dev, uint32_t address)
{
    uint8_t cmd[4] = {W25Q128JV_CMD_SECTOR_ERASE, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};
    
    W25Q128JV_WriteEnable(dev);
    W25Q128JV_CS_LOW(dev);
    HAL_SPI_Transmit(dev->hspi, cmd, 4, 100);
    W25Q128JV_CS_HIGH(dev);
    HAL_Delay(300); // Wait for erase completion
}

void W25Q128JV_EraseChip(struct w25q128jv_dev *dev)
{
    uint8_t cmd = W25Q128JV_CMD_CHIP_ERASE_C7;

    W25Q128JV_WriteEnable(dev);
    W25Q128JV_CS_LOW(dev);
    HAL_SPI_Transmit(dev->hspi, &cmd, 1, 100);
    W25Q128JV_CS_HIGH(dev);
    HAL_Delay(2000); // Wait for full chip erase
}

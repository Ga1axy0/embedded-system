#include "stm32f10x.h"
#include "stm32f10x_spi.h"

#define SPI_FLASH_CS_PIN        GPIO_Pin_3
#define SPI_FLASH_CS_PORT       GPIOC

#define SPI_COMMAND_WRITE_ENABLE    0x06
#define SPI_COMMAND_WRITE_DISABLE   0x04
#define SPI_COMMAND_READ_STATUS     0x05
#define SPI_COMMAND_READ_DATA       0x03
#define SPI_COMMAND_PAGE_PROGRAM    0x02


void SPI_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_SetBits(GPIOC, GPIO_Pin_3);

  
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; 
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    SPI_Init(SPI2, &SPI_InitStructure);
    SPI_Cmd(SPI2, ENABLE);
}
void SPI_Flash_CS_High() { GPIO_SetBits(SPI_FLASH_CS_PORT, SPI_FLASH_CS_PIN); }
void SPI_Flash_CS_Low()  { GPIO_ResetBits(SPI_FLASH_CS_PORT, SPI_FLASH_CS_PIN); }



uint8_t SPI2_SendByte(uint8_t data)
{

    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI2, data);


    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
    return SPI_I2S_ReceiveData(SPI2);
}

void SPI_Flash_ReadID(uint8_t *id)
{
    SPI_Flash_CS_Low();             
    SPI2_SendByte(0x9F);            
    id[0] = SPI2_SendByte(0xFF);    
    id[1] = SPI2_SendByte(0xFF);    
    id[2] = SPI2_SendByte(0xFF);    
    SPI_Flash_CS_High();            
}

void SPI_Flash_WriteEnable(void)
{
    SPI_Flash_CS_Low();
    SPI2_SendByte(SPI_COMMAND_WRITE_ENABLE);
    SPI_Flash_CS_High();
}

void SPI_Flash_WaitForWriteEnd(void)
{
    uint8_t flashstatus = 0;
    SPI_Flash_CS_Low();
    SPI2_SendByte(SPI_COMMAND_READ_STATUS);
    do
    {
        flashstatus = SPI2_SendByte(0xFF);
    }
    while ((flashstatus & 0x01) == SET);
    SPI_Flash_CS_High();
}

void SPI_Flash_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    SPI_Flash_WriteEnable();
    
    SPI_Flash_CS_Low();
    SPI2_SendByte(SPI_COMMAND_PAGE_PROGRAM);
    SPI2_SendByte((WriteAddr & 0xFF0000) >> 16);
    SPI2_SendByte((WriteAddr & 0xFF00) >> 8);
    SPI2_SendByte(WriteAddr & 0xFF);

    while (NumByteToWrite--)
    {
        SPI2_SendByte(*pBuffer);
        pBuffer++;
    }
    SPI_Flash_CS_High();

    SPI_Flash_WaitForWriteEnd();
}

void SPI_Flash_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
    SPI_Flash_CS_Low();
    SPI2_SendByte(SPI_COMMAND_READ_DATA);
    SPI2_SendByte((ReadAddr & 0xFF0000) >> 16);
    SPI2_SendByte((ReadAddr & 0xFF00) >> 8);
    SPI2_SendByte(ReadAddr & 0xFF);

    while (NumByteToRead--)
    {
        *pBuffer = SPI2_SendByte(0xFF);
        pBuffer++;
    }
    SPI_Flash_CS_High();
}


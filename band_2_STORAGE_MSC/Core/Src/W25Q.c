#include "W25Q.h"

extern SPI_HandleTypeDef hspi1;

#define FLASH_CS_GPIO_Port   GPIOA
#define FLASH_CS_Pin         GPIO_PIN_4

#define CMD_RDID        0x9F
#define CMD_WREN        0x06
#define CMD_RDSR1       0x05
#define CMD_READ        0x03
#define CMD_PP          0x02
#define CMD_SE_4K       0x20
#define CMD_EN4B        0xB7        // Enable 4-byte address mode

static inline void CS_L(void){ HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET); }
static inline void CS_H(void){ HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET); }

static void SPI_Tx(const uint8_t *buf, uint16_t len){ HAL_SPI_Transmit(&hspi1,(uint8_t*)buf,len,HAL_MAX_DELAY); }
static void SPI_Rx(uint8_t *buf, uint16_t len){ HAL_SPI_Receive(&hspi1,buf,len,HAL_MAX_DELAY); }

static uint8_t ReadSR1(void){
    uint8_t cmd[2]={CMD_RDSR1,0xFF},rx[2];
    CS_L(); HAL_SPI_TransmitReceive(&hspi1,cmd,rx,2,HAL_MAX_DELAY); CS_H();
    return rx[1];
}

static void WaitBusy(void){
    uint32_t t0=HAL_GetTick();
    while(ReadSR1()&0x01){
        if(HAL_GetTick()-t0>3000) break;
        HAL_Delay(1);
    }
}

static void WriteEnable(void){ uint8_t c=CMD_WREN; CS_L(); SPI_Tx(&c,1); CS_H(); }





void W25Q01_Init(void){
    uint8_t cmd=CMD_EN4B;
    CS_L(); SPI_Tx(&cmd,1); CS_H();   // 进入4字节地址模式
}

void W25Q01_SectorErase4K(uint32_t addr){
    uint8_t cmd[5]={CMD_SE_4K,(addr>>24),(addr>>16),(addr>>8),addr};
    WriteEnable();
    CS_L(); SPI_Tx(cmd,5); CS_H();
    WaitBusy();
}

void W25Q01_PageProgram(uint32_t addr,const uint8_t *data,uint16_t len){
    if(len>256) len=256;
    uint8_t cmd[5]={CMD_PP,(addr>>24),(addr>>16),(addr>>8),addr};
    WriteEnable();
    CS_L(); SPI_Tx(cmd,5); SPI_Tx(data,len); CS_H();
    WaitBusy();
}

void W25Q01_ReadData(uint32_t addr,uint8_t *buf,uint32_t len){
    uint8_t cmd[5]={CMD_READ,(addr>>24),(addr>>16),(addr>>8),addr};
    CS_L(); SPI_Tx(cmd,5); SPI_Rx(buf,len); CS_H();
}


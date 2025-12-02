#ifndef __W25Q_H__
#define __W25Q_H__

#include "main.h"
#include "spi.h"
#include <string.h>
#include <stdio.h>

void W25Q01_Init(void);
void W25Q01_ReadData(uint32_t addr, uint8_t *buf, uint32_t len);
void W25Q01_ProgramImageToZero(const uint8_t *image, uint32_t image_size);
void Flash_WriteImageToZero(const uint8_t *img, uint32_t img_len);

void W25Q01_SectorErase4K(uint32_t addr);
void W25Q01_PageProgram(uint32_t addr, const uint8_t *data, uint16_t len);

#endif
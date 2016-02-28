#ifndef _HALPROXY_H_
#define _HALPROXY_H_

#include <stm32f10x.h>
#define TS_IRQ_PRIORITY 15

void LCD_HAL_Init();
void LCD_HAL_Reset();
void LCD_HAL_Light(int level);
void LCD_HAL_SetRegister(u16 reg);
void LCD_HAL_WriteData(u16 data);
u16 LCD_HAL_ReadData();
u16 LCD_HAL_ReadRegister(u16 reg);

SPI_TypeDef* TS_HAL_Init();
void TS_HAL_CS();
void TS_HAL_DCS();
uint32_t TS_HAL_Sense();

/******************************************************
REIMPLEMENTS FUNCTION IN THIS FILE FOR YOUR LCD
******************************************************/
#define LCD_XSIZE          (320)
#define LCD_YSIZE          (240)


#endif //_HALPROXY_H_

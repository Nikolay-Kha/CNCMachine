#include "halProxy.h"
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>

/******************************************************
REIMPLEMENTS FUNCTION IN THIS FILE FOR YOUR LCD DRIVER
******************************************************/
#include "../hal/fsmc.h"

void LCD_HAL_Init() {
	HAL_FSMC_Init();
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	GPIO_InitTypeDef  gpio;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Pin = GPIO_Pin_13; // Light
	GPIO_Init(GPIOD, &gpio);
	gpio.GPIO_Pin = GPIO_Pin_1 ; // Reset
	GPIO_Init(GPIOE, &gpio);
}

void LCD_HAL_Reset() {
	int i;
	GPIO_ResetBits(GPIOE, GPIO_Pin_1);
	for(i=50000;i>0;i--);
	GPIO_SetBits(GPIOE, GPIO_Pin_1);
	for(i=50000;i>0;i--);
}

void LCD_HAL_Light(int level) {
	if(level)
		GPIOD->BRR = GPIO_Pin_13;
	else
		GPIOD->BSRR = GPIO_Pin_13;
}

inline void LCD_HAL_SetRegister(u16 reg) {
	HAL_FSMC_SetRegister(reg);
}

inline void LCD_HAL_WriteData(u16 data) {
	HAL_FSMC_WriteData(data);
}

inline u16 LCD_HAL_ReadData() {
	return HAL_FSMC_ReadData();
}

inline u16 LCD_HAL_ReadRegister(u16 reg) {
	HAL_FSMC_SetRegister(reg);
	return HAL_FSMC_ReadData();
}

/******************************************************
REIMPLEMENTS FUNCTION IN THIS FILE FOR YOUR TOUCH DRIVER
******************************************************/
#include "tsc2046.h"

SPI_TypeDef*  TS_HAL_Init() {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef  gpio;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio.GPIO_Pin = GPIO_Pin_6; // Touch sense
	GPIO_Init(GPIOB, &gpio);
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Pin = GPIO_Pin_7 ; // CS
	GPIO_Init(GPIOB, &gpio);
	TS_HAL_DCS();

	// touch itteruption
	RCC->APB2ENR|= RCC_APB2ENR_AFIOEN;
	AFIO->EXTICR [6>>0x02] |= AFIO_EXTICR2_EXTI6_PB;
	EXTI->IMR |= EXTI_IMR_MR6;
	EXTI->FTSR |= EXTI_FTSR_TR6;
	EXTI->RTSR |= EXTI_RTSR_TR6;
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_SetPriority(EXTI9_5_IRQn, TS_IRQ_PRIORITY);

	return SPI1;
}

void EXTI9_5_IRQHandler (void) { // touch itteruption handler
	if (EXTI->PR & (1<<6)) // Прерывание от EXTI6?
	{
		EXTI->PR |= (1<<6);
		TS_TouchIrq();
	}
}

inline void TS_HAL_CS() {
	GPIO_ResetBits(GPIOB,GPIO_Pin_7);
}

inline void TS_HAL_DCS() {
	 GPIO_SetBits(GPIOB,GPIO_Pin_7);
}

uint32_t TS_HAL_Sense() {
	return (GPIOB->IDR & GPIO_Pin_6);
}


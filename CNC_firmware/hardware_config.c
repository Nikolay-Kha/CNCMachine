#include "hardware_config.h"

uint32_t getRcc(GPIO_TypeDef* GPIOx) {
	if(GPIOx==GPIOC)
		return RCC_APB2Periph_GPIOC;
	if(GPIOx==GPIOD)
		return RCC_APB2Periph_GPIOD;
	if(GPIOx==GPIOA)
		return RCC_APB2Periph_GPIOA;
	if(GPIOx==GPIOE)
		return RCC_APB2Periph_GPIOE;
	if(GPIOx==GPIOB)
		return RCC_APB2Periph_GPIOB;
	if(GPIOx==GPIOG)
		return RCC_APB2Periph_GPIOG;
	if(GPIOx==GPIOF)
		return RCC_APB2Periph_GPIOF;
	return 0;
}

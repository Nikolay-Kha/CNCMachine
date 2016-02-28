#include "beeper.h"
#include "hardware_config.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"


int BEEPER_COUNTER;

void beep_init()
{
	RCC_APB2PeriphClockCmd(getRcc(BEEPER_PORT), ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = BEEPER_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( BEEPER_PORT , &GPIO_InitStructure);
	BEEPER_PORT->BRR = GPIO_InitStructure.GPIO_Pin;


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
	TIM_Cmd(TIM4, DISABLE);

	NVIC_EnableIRQ(TIM4_IRQn);
	NVIC_SetPriority(TIM4_IRQn, 15);
}

void TIM4_IRQHandler()
{
	BEEPER_COUNTER--;
	if(BEEPER_COUNTER%2==0)
		BEEPER_PORT->BRR = BEEPER_PIN;
	else
		BEEPER_PORT->BSRR = BEEPER_PIN;
	if(BEEPER_COUNTER>0)
		TIM4->CR1 |= TIM_CR1_CEN|TIM_CR1_OPM;
	else
		TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
	TIM4->SR = (uint16_t)~TIM_IT_Update;
}

void beep(int durationms, int times)
{
	TIM4->CR1 &= (uint16_t)~(TIM_CR1_CEN|TIM_CR1_OPM);
	TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
	TIM4->SR = (uint16_t)~TIM_IT_Update;

	if(times>7000)
		times = 7000;
	BEEPER_COUNTER = times*2-1;
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	TIM_TimeBaseInitTypeDef base_timer;
	TIM_TimeBaseStructInit(&base_timer);
	base_timer.TIM_Period = durationms*4-1;
	base_timer.TIM_Prescaler =  ((clocks.SYSCLK_Frequency/1000))/4 - 1;
	TIM_TimeBaseInit(TIM4, &base_timer);
	BEEPER_PORT->BSRR = BEEPER_PIN;
	TIM4->SR = (uint16_t)~TIM_IT_Update;
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	TIM4->CR1 |= TIM_CR1_CEN|TIM_CR1_OPM;
}

void beep_one()
{
	beep(600, 1);
}

void beep_triple()
{
	beep(400, 3);
}

void beep_stop()
{
	TIM4->CR1 &= (uint16_t)~(TIM_CR1_CEN|TIM_CR1_OPM);
	TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
	TIM4->SR = (uint16_t)~TIM_IT_Update;
	BEEPER_PORT->BRR = BEEPER_PIN;
}

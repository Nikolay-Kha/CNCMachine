#include "mcore.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include <math.h>

#define SPINLDE_OFF_DC 0.0f
#define SPINDLE_RPM_FACTOR 650000
float spindleDutyCicle = SPINLDE_OFF_DC;
#define SPINDLE_MIN_WORKING_DUTY 0.0935f

// spindle sensor - PB15

STATUSES spindle_runr(int rpm)
{
	return spindle_runp(((float)rpm*100.0f)/(float)SPINDLE_MAX_RPM);
}

STATUSES spindle_runp(float percent)
{
	int result = STATUS_OK;
	if(percent==SPINDLE_PROBE_MODE) {
		spindleDutyCicle = SPINDLE_MIN_WORKING_DUTY;
	} else {
		float duty = percent/100.0f;
		if(duty<=0.0f){
			spindle_stop();
			return STATUS_ERROR_INVALID_PARAMETER;
		}
		if(duty!=SPINDLE_PROBE_MODE && duty<((float)SPINDLE_MIN_RPM/(float)SPINDLE_MAX_RPM)) {
			duty=(float)SPINDLE_MIN_RPM/(float)SPINDLE_MAX_RPM;
			result = STATUS_WARNING_OUT_OF_SPEED;
		}
		if(duty>1.0f){
			duty=1.0f;
			result = STATUS_WARNING_OUT_OF_SPEED;
		}
		spindleDutyCicle = duty;
	}

	RCC_APB2PeriphClockCmd(getRcc(SPINDLE_PORT), ENABLE); //  SPINDLE_PORT
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = SPINDLE_PWM_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPINDLE_PORT,&GPIO_InitStructure);

	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	TIM_TimeBaseInitTypeDef spindle_timer;
	TIM_TimeBaseStructInit(&spindle_timer);
	/* Делитель учитывается как TIM_Prescaler + 1, поэтому отнимаем 1 */
	spindle_timer.TIM_Period = 1000;
	spindle_timer.TIM_Prescaler = (clocks.SYSCLK_Frequency/TACTS_PER_SEC/spindle_timer.TIM_Period) - 1;
	spindle_timer.TIM_ClockDivision = 0;
	spindle_timer.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM8, &spindle_timer);
	TIM_OCInitTypeDef oc_init;
	TIM_OCStructInit(&oc_init);
	oc_init.TIM_OCMode = TIM_OCMode_PWM1;   // работаем в режиме ШИМ ( PWM )
	oc_init.TIM_OutputState = TIM_OutputState_Enable;
	oc_init.TIM_Pulse = spindle_timer.TIM_Period*spindleDutyCicle+1.0f;   // скважность
	oc_init.TIM_OCPolarity = TIM_OCPolarity_High;  // положительная полярность
	TIM_OC1Init(TIM8,&oc_init);   /// заносим данные в канал
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM8, ENABLE);
	TIM_CtrlPWMOutputs(TIM8, ENABLE);
	TIM_Cmd(TIM8, ENABLE);
	return result;
}

STATUSES spindle_stop()
{
	spindleDutyCicle = SPINLDE_OFF_DC;
	TIM_CtrlPWMOutputs(TIM8, DISABLE);
	TIM_Cmd(TIM8, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, DISABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = SPINDLE_PWM_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPINDLE_PORT,&GPIO_InitStructure);
	SPINDLE_PORT->BRR = SPINDLE_PWM_PIN;
	return STATUS_OK;
}

int spindle_get_rpm()
{
	if(spindleDutyCicle == SPINLDE_OFF_DC)
		return SPINDLE_OFF;
	return SPINDLE_RPM_FACTOR/private_spindle_result;
}

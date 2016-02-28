#include <stm32f10x.h>
#include <stm32f10x_conf.h>

#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include "tsc2046.h"
#include "ili93xx.h"
#include "mcore.h"
#include "cncgui.h"
#include "gcode.h"
#include "beeper.h"
#include "hardware_config.h"
#include "veeprom/veeprom.h"
#include "usbhost.h"

int mButtonFilter = 0;

void EXTI2_IRQHandler(void) {
	if (EXTI->PR & (1<<2)) // Прерывание от EXTI2?
	{
		GUI_drawCloceContextMenu();
		CONNECTED cstatus = gGetStatus();
		action_estop();
		TSInteruptCallibration();
		if(GUI_GetCurrentScreen()==&changeToolScreen || GUI_GetCurrentScreen()==&pauseScreen || (GUI_GetCurrentScreen()==&connectedScreen && cstatus==CONNECTED_FALSE))
			CNC_GUI_AxisScreen();
		if(GUI_GetCurrentScreen()==&fileManagerScreen && uh_MSState()==USB_MS_DISCONNECTED)
			CNC_GUI_AxisScreen();
		NVIC_DisableIRQ(EXTI2_IRQn);
		EXTI->PR |= (1<<2);
		mButtonFilter = 1;
	}
}

int main(void) {
	axis_init();
	CNC_GUI_Init();
	gInit();
	beep_init();
	uh_init();

	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	const int buttonFilterValue = clocks.SYSCLK_Frequency/720000;
	const int ledValue = clocks.SYSCLK_Frequency/7200;


	GPIO_InitTypeDef GPIO_InitStructure;
	//  инициализируем кнопку EMERGENCY STOP - D02
	RCC_APB2PeriphClockCmd(getRcc(ESTOPBUTTONPORT), ENABLE);
	GPIO_InitStructure.GPIO_Pin = ESTOPBUTTONPIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( ESTOPBUTTONPORT , &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_AFIOEN , ENABLE);
	AFIO->EXTICR [2>>0x02] |= AFIO_EXTICR1_EXTI2_PD;
	EXTI->IMR |= EXTI_IMR_MR2; //Прерывания от ноги разрешены
	EXTI->FTSR |= EXTI_FTSR_TR2; //Прерывания по спадающему фронту
	NVIC_EnableIRQ(EXTI2_IRQn);
	NVIC_SetPriority(EXTI2_IRQn, 2);

	// on board LED для отладки и визуального контроля работы. 0 - что проц вообще работает и загрузку, 2 - если тупит в циклах движения движков
    RCC_APB2PeriphClockCmd(getRcc(DEBUGLED_PORT) , ENABLE);
    GPIO_InitStructure.GPIO_Pin = CPULEDPIN  | CPUOVERUSAGELEDPIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( DEBUGLED_PORT , &GPIO_InitStructure);
    DEBUGLED_PORT->BSRR = CPULEDPIN  | CPUOVERUSAGELEDPIN;

    if(TSState()==TS_Pressed)
    	action_calibratets();
    else if( VEEPROM_GetData(TOUCHSCREEN_DATA_VEEPROM_PAGE, TSGetCalibrationDataPtr(), sizeof(TS_CALIBRATION_DATA))==0 )
    	action_calibratets();

    //action_calibrate(GUI_FALSE);
    CNC_GUI_AxisScreen();

    u32 counter = 0;
    int heatersCalibration = 0;

    while (1) {
    	if(mButtonFilter) {
    		if((ESTOPBUTTONPORT->IDR & ESTOPBUTTONPIN)!=0) {
    			mButtonFilter++;
    			if(mButtonFilter>buttonFilterValue) {
    				mButtonFilter = 0;
    				EXTI->PR |= (1<<2);
    				NVIC_EnableIRQ(EXTI2_IRQn);
    			}
    		} else {
    			mButtonFilter = 1;
    		}
    	}
    	gRun();
    	updateCheck();
    	if(heaters_wait_status()==HW_CALIBRATION) {
    		heatersCalibration = 1;
    	} else if(heatersCalibration) {
    		if(heaters_lastCalibrationStatus()!=STATUS_ERROR_ESTOP)
    			beep_one();
    		if(GUI_GetCurrentScreen()==&calibarationScreen)
    			CNC_GUI_AxisScreen();
    		heatersCalibration = 0;
    	}
    	counter++;
    	if(counter%ledValue==0) // светодиодик для отладки - должен моргать - чем реже моргает - тем сильнее забит проц
    		DEBUGLED_PORT->ODR ^= CPULEDPIN;
    }

    return 0;
}

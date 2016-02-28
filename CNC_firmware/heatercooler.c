#include "mcore.h"
#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_adc.h>
#include "stm32f10x_tim.h"
#include <math.h>
#include "hardware_config.h"
#include "pid.h"
#include "veeprom/veeprom.h"

#define FILTER_WEIGHT 0.02f
#define MIN_EXTRUDER_TEMP_FORMOVEMENT 150
#define DELTA_TEMPERATRE_TABLE_FIX 3
#define DELTA_TEMPERATRE_EXTRUDER_FIX 3

HEATER_WAIT heatersWaiter = HW_NONE;
int tableTargetTemp = HEATER_OFF;
int extruderTargetTemp = HEATER_OFF;
float tableMeasuredTemp = 0;
float extruderMeasuredTemp = 0;
int tableFilterTemp = 0;
int extruderFilterTemp = 0;
#define TIMER_ITTERATION_COUNT 4999
int tableHeaterPeriod = 0;
int extruderHeaterPeriod = 0;

#define CALIBRATION_TABLE_POINT1 60
#define CALIBRATION_TABLE_POINT2 80
#define CALIBRATION_EXTRUDER_POINT1 150
#define CALIBRATION_EXTRUDER_POINT2 230
#define CALIBRATION_HEAT_PERCENT 0.1f

typedef struct {
	PID tablePID;
	PID extruderPID;
} CALIBRATION_DATA;

CALIBRATION_DATA calibrationData;

PID *tablePID = &(calibrationData.tablePID);
PID *extruderPID = &(calibrationData.extruderPID);

const uint16_t INTERPOLATION_MATRIX_TABLE[] = TABLE_SENSOR_MATRIX_VALUES;
const int INTERPOLATION_MATRIX_TABLE_SIZE = (sizeof INTERPOLATION_MATRIX_TABLE)/2;
const uint16_t INTERPOLATION_MATRIX_EXTRUDER[] = EXTRUDER_SENSOR_MATRIX_VALUES;
const int INTERPOLATION_MATRIX_EXTRUDER_SIZE = (sizeof INTERPOLATION_MATRIX_EXTRUDER)/2;

int tableNCCounter = 0;
int extruderNCCounter = 0;
#define NC_DETECT_COUNT 5
#define NC_DETECT_COUNT_DELTA_MUL 3
#define NC_DETECT_LEVEL 3900

#define TIMER_MEASURE_FREQUENCY 64// Hz
#define ADC_NUMBER_OF_ITERATIONS 16
#define REACH_TIME 10 //sec
int ADCdata_table[ADC_NUMBER_OF_ITERATIONS];
int ADCdata_Pos = 0;
int ADCdata_extruder[ADC_NUMBER_OF_ITERATIONS];
int calibrationCounter;

int calibrationTableTime;
int calibrationExtruderTime;
float calibrationTableMaxTemperature;
float calibrationTableMinTemperature;
int calibrationTableStartTime;
float calibrationExtruderMaxTemperature;
float calibrationExtruderMinTemperature;
int calibrationExtruderStartTime;
int calibrationTableCycle;
int calibrationExtruderCycle;
int calibrationTableFirstPeak;
int calibrationExtruderFirstPeak;
#define CALIBRATION_HISTERESIS_DEGREE 0.5f
int calibrationDataRead = 0;
STATUSES calibrationCurrentStatus = STATUS_OK;

#define LASER_COOLER_TIME 10*TIMER_MEASURE_FREQUENCY // 10 sec
int laserCoolerTimer = LASER_COOLER_TIME;

int calcADCAverageValue(int m[ADC_NUMBER_OF_ITERATIONS])
{
	int i, j, f, tmp;
	for (i = ADC_NUMBER_OF_ITERATIONS-1; i > 0; i--) //сортируем замеры пузырьковым методом
	{
		f = 0;
		for (j = 0; j < i; j++)
		{
			if (m[j] > m[j + 1])
			{
				tmp = m[j];
				m[j] = m[j + 1];
				m[j + 1] = tmp;
				f = 1;
			}
		}
		if (f == 0) break;
	}

	tmp = 0;
	for(i=ADC_NUMBER_OF_ITERATIONS/4; i<ADC_NUMBER_OF_ITERATIONS-ADC_NUMBER_OF_ITERATIONS/4; i++)
		tmp +=m [i];
	return tmp/(ADC_NUMBER_OF_ITERATIONS/2);
}

int fanAndMeasure() {
	// extruder cooling fan
	if(extruderTargetTemp!=HEATER_OFF || extruderMeasuredTemp>(float)(EXTRUDER_COOLING_TEMP+DELTA_TEMPERATRE_EXTRUDER_FIX))
		COOLING_PORT->BSRR = COOLING_PIN;
	else if(extruderMeasuredTemp<(float)EXTRUDER_COOLING_TEMP && laserCoolerTimer>=LASER_COOLER_TIME)
		COOLING_PORT->BRR = COOLING_PIN;
	if((extruderMeasuredTemp+0.5f)>=(float)MIN_EXTRUDER_TEMP_FORMOVEMENT)
		AXIS_E_DRIVER_ENABLE_PORT->BSRR = AXIS_E_DRIVER_ENABLE_PIN;
	else
		AXIS_E_DRIVER_ENABLE_PORT->BRR = AXIS_E_DRIVER_ENABLE_PIN;


	ADCdata_table[ADCdata_Pos] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
	ADCdata_extruder[ADCdata_Pos] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
	ADCdata_Pos++;
	if(ADCdata_Pos>=ADC_NUMBER_OF_ITERATIONS) {
		ADCdata_Pos = 0;

		int resTable = calcADCAverageValue(ADCdata_table);
		int resExtruder = calcADCAverageValue(ADCdata_extruder);

		// temperature measure
		if(tableFilterTemp)
			tableFilterTemp = resTable*FILTER_WEIGHT + (1.0f-FILTER_WEIGHT)*tableFilterTemp;
		else
			tableFilterTemp = resTable;
		if(resTable>=NC_DETECT_LEVEL) {
			if (tableNCCounter<=NC_DETECT_COUNT*NC_DETECT_COUNT_DELTA_MUL)
				tableNCCounter++;
		}else if(tableNCCounter>-NC_DETECT_COUNT*NC_DETECT_COUNT_DELTA_MUL) {
			tableNCCounter--;
		}

		if(extruderFilterTemp)
			extruderFilterTemp = resExtruder*FILTER_WEIGHT + (1.0f-FILTER_WEIGHT)*extruderFilterTemp;
		else
			extruderFilterTemp = resExtruder;
		if(resExtruder>=NC_DETECT_LEVEL) {
			if (extruderNCCounter<=NC_DETECT_COUNT*NC_DETECT_COUNT_DELTA_MUL)
				extruderNCCounter++;
		}else if(extruderNCCounter>-NC_DETECT_COUNT*NC_DETECT_COUNT_DELTA_MUL) {
			extruderNCCounter--;
		}

		int i;
		if(tableNCCounter>NC_DETECT_COUNT) {
			tableMeasuredTemp = (float)HEATER_NOT_CONNECTED_TEMP;
			tableFilterTemp = 0;
		} else {
			for(i=1; i<INTERPOLATION_MATRIX_TABLE_SIZE; i++) {
				if( tableFilterTemp>INTERPOLATION_MATRIX_TABLE[i] ) {
					tableMeasuredTemp = (float)(i*10) - (float)(tableFilterTemp-INTERPOLATION_MATRIX_TABLE[i])*10.0f/(float)(INTERPOLATION_MATRIX_TABLE[i-1]-INTERPOLATION_MATRIX_TABLE[i]);
					break;
				}
			}
		}

		if(extruderNCCounter>NC_DETECT_COUNT) {
			extruderMeasuredTemp = (float)HEATER_NOT_CONNECTED_TEMP;
			extruderFilterTemp = 0;
		} else {
			for(i=1; i<INTERPOLATION_MATRIX_EXTRUDER_SIZE; i++) {
				if(extruderFilterTemp>INTERPOLATION_MATRIX_EXTRUDER[i]) {
					extruderMeasuredTemp = (float)(i*10) - (float)(extruderFilterTemp-INTERPOLATION_MATRIX_EXTRUDER[i])*10.0f/(float)(INTERPOLATION_MATRIX_EXTRUDER[i-1]-INTERPOLATION_MATRIX_EXTRUDER[i]);
					break;
				}
			}
		}
		return 1;
	}
	return 0;
}

void private_heater_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);

	RCC_APB2PeriphClockCmd(getRcc(HEATERS_PORT), ENABLE);
	GPIO_InitStructure.GPIO_Pin = EXTRUDER_HEATER_PIN | TABLE_HEATER_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( HEATERS_PORT , &GPIO_InitStructure);
	HEATERS_PORT->BRR = GPIO_InitStructure.GPIO_Pin;
	
	RCC_APB2PeriphClockCmd(getRcc(COOLING_PORT), ENABLE);
	GPIO_InitStructure.GPIO_Pin = COOLING_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( HEATERS_PORT , &GPIO_InitStructure);
	HEATERS_PORT->BRR = GPIO_InitStructure.GPIO_Pin;

	RCC_APB2PeriphClockCmd(getRcc(AXIS_E_DRIVER_ENABLE_PORT), ENABLE);
	GPIO_InitStructure.GPIO_Pin = AXIS_E_DRIVER_ENABLE_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( AXIS_E_DRIVER_ENABLE_PORT , &GPIO_InitStructure);
	AXIS_E_DRIVER_ENABLE_PORT->BRR = AXIS_E_DRIVER_ENABLE_PIN;

	// таймер http://robocraft.ru/blog/ARM/722.html
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseInitTypeDef base_timer;
	TIM_TimeBaseStructInit(&base_timer);
	base_timer.TIM_Period = TIMER_ITTERATION_COUNT;
	base_timer.TIM_Prescaler =  (clocks.SYSCLK_Frequency/(base_timer.TIM_Period+1))/TIMER_MEASURE_FREQUENCY - 1;
	TIM_TimeBaseInit(TIM3, &base_timer);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM3, DISABLE);

	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_SetPriority(TIM3_IRQn, 15);

	// http://easystm32.ru/for-beginners/20-adc-in-stm32-part-2
	// http://www.badprog.com/electronics-stm32-using-the-adc-peripheral-with-a-potentiometer
	// http://doc.open-bldc.org/adc_8c_source.html
	//Включаем порт А
	RCC_APB2PeriphClockCmd((getRcc(TEMPERATURES_SENSORS_PORT)) , ENABLE);
	GPIO_InitStructure.GPIO_Pin = TABLE_SENSOR_PIN | EXTRUDER_SENSOR_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(TEMPERATURES_SENSORS_PORT, &GPIO_InitStructure);
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_ADC1EN, ENABLE); //Включаем тактирование АЦП

	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Mode = ADC_Mode_InjecSimult_SlowInterl;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 2;
	ADC_Init(ADC1, &ADC_InitStructure);
	//Задаем номер канала, длительность выборки
	ADC_InjectedSequencerLengthConfig(ADC1, 2);
	ADC_InjectedChannelConfig(ADC1, TABLE_SENSOR_CHANNEL, 1, ADC_SampleTime_239Cycles5);
	ADC_InjectedChannelConfig(ADC1, EXTRUDER_SENSOR_CHANNEL, 2, ADC_SampleTime_239Cycles5);
	ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);
	ADC_AutoInjectedConvCmd(ADC1, ENABLE); // опрашивать непрерывно

	ADC_Cmd(ADC1, ENABLE);//Теперь включаем АЦП
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1) == SET) ;
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1) == SET) ;

	ADC_SoftwareStartInjectedConvCmd(ADC1, ENABLE);
	while (ADC_GetSoftwareStartInjectedConvCmdStatus(ADC1)== SET); //ждем пока первое преобразование завершится

	//Теперь можно читать результат
	fanAndMeasure();
	TIM_Cmd(TIM3, ENABLE);
	calibrationDataRead = VEEPROM_GetData(HEATERS_DATA_VEEPROM_PAGE, &calibrationData, sizeof (CALIBRATION_DATA));
	if(!calibrationDataRead)
		calibrationCurrentStatus = STATUS_ERROR_NOT_CALIBRATED;
	PID_Init(tablePID, 0, TIMER_ITTERATION_COUNT, DELTA_TEMPERATRE_TABLE_FIX);
	PID_Init(extruderPID, 0, TIMER_ITTERATION_COUNT, DELTA_TEMPERATRE_EXTRUDER_FIX);
}

STATUSES heaters_calibrate_till(int tableTemperature, int extruderTemperature)
{
	int cip = heaters_wait_status()==HW_CALIBRATION && calibrationTableTime && calibrationExtruderTime;
	if(!cip) {
		if( heater_get_table_temperature()!=HEATER_OFF && heater_get_extruder_temperature()!=HEATER_OFF )
			return STATUS_BUSY;
		if( tableTemperature > MAX_TABLE_TEMPERATURE-MAX_TABLE_TEMPERATURE*CALIBRATION_HEAT_PERCENT || extruderTemperature > MAX_EXTRUDER_TEMPERATURE-MAX_EXTRUDER_TEMPERATURE*CALIBRATION_HEAT_PERCENT)
			return STATUS_ERROR_OUT_OF_TEMPERATURERANGE;
	}
	extruderTargetTemp = extruderTemperature;
	tableTargetTemp =  tableTemperature;
	calibrationCounter = 0;
	calibrationTableMaxTemperature = tableTargetTemp;
	calibrationTableMinTemperature = tableTargetTemp;
	calibrationTableStartTime = 0;
	calibrationTableCycle = 0;
	calibrationExtruderMaxTemperature = extruderTargetTemp;
	calibrationExtruderMinTemperature = extruderTargetTemp;
	calibrationExtruderStartTime = 0;
	calibrationExtruderCycle = 0;
	calibrationTableTime = 0;
	calibrationExtruderTime = 0;
	calibrationTableFirstPeak = 0;
	calibrationExtruderFirstPeak = 0;

	if(cip)
		return STATUS_OK;
	STATUSES res = heaters_wait(HW_CALIBRATION);
	if(res!=STATUS_OK) {
		extruderTargetTemp = HEATER_OFF;
		tableTargetTemp = HEATER_OFF;
		HEATERS_PORT->BRR = TABLE_HEATER_PIN | EXTRUDER_HEATER_PIN;
	}
	return res;
}

typedef enum {
	ShutdownMin,
	ShutdownMax,
	Regular
} NEXTSTEP;
NEXTSTEP mNextItteration = Regular;

void TIM3_IRQHandler()
{
	TIM3->ARR = TIMER_ITTERATION_COUNT;
	TIM3->SR = (uint16_t)~TIM_IT_Update;
	if(mNextItteration == Regular) {
		if(laser_isWorking()) {
			laserCoolerTimer = 0;
		} else {
			if(laserCoolerTimer<=LASER_COOLER_TIME)
				laserCoolerTimer++;
		}

		if(heatersWaiter==HW_CALIBRATION) {
			if(fanAndMeasure()) {
				if(fabsf(tableMeasuredTemp-(float)tableTargetTemp) > CALIBRATION_HISTERESIS_DEGREE) {
					if(calibrationTableTime==0 && tableMeasuredTemp>(float)tableTargetTemp) {
						if(calibrationTableStartTime==0)
							calibrationTableStartTime = calibrationCounter;
						if(calibrationTableFirstPeak && calibrationTableMaxTemperature<tableMeasuredTemp)
							calibrationTableMaxTemperature = tableMeasuredTemp;
						if(calibrationTableCycle)
							calibrationTableTime = calibrationCounter - calibrationTableStartTime;
						HEATERS_PORT->BRR = TABLE_HEATER_PIN;
					} else {
						if(calibrationTableStartTime) {
							if(calibrationTableFirstPeak==0) {
								calibrationTableFirstPeak = 1;
								calibrationTableStartTime = 0;
							} else {
								if(calibrationTableMinTemperature>tableMeasuredTemp)
									calibrationTableMinTemperature = tableMeasuredTemp;
								calibrationTableCycle = 1;
							}
						}
						if(calibrationTableTime==0) {
							HEATERS_PORT->BSRR = TABLE_HEATER_PIN;
						}
					}
				}

				if(calibrationExtruderTime==0 && fabsf(extruderMeasuredTemp-(float)extruderTargetTemp) > CALIBRATION_HISTERESIS_DEGREE) {
					if(extruderMeasuredTemp>(float)extruderTargetTemp) {
						if(calibrationExtruderStartTime==0)
							calibrationExtruderStartTime = calibrationCounter;
						if(calibrationExtruderFirstPeak && calibrationExtruderMaxTemperature<extruderMeasuredTemp)
							calibrationExtruderMaxTemperature = extruderMeasuredTemp;
						if(calibrationExtruderCycle)
							calibrationExtruderTime = calibrationCounter - calibrationExtruderStartTime;
						HEATERS_PORT->BRR = EXTRUDER_HEATER_PIN;
					} else {
						if(calibrationExtruderStartTime) {
							if(calibrationExtruderFirstPeak==0) {
								calibrationExtruderFirstPeak = 1;
								calibrationExtruderStartTime = 0;
							} else {
								if(calibrationExtruderMinTemperature>extruderMeasuredTemp)
									calibrationExtruderMinTemperature = extruderMeasuredTemp;
								calibrationExtruderCycle = 1;
							}
						}
						if(calibrationExtruderTime==0) {
							HEATERS_PORT->BSRR = EXTRUDER_HEATER_PIN;
						}
					}
				}

				if(calibrationTableTime && calibrationExtruderTime) {
					PID_CalibrationData(tablePID, (float)CALIBRATION_TABLE_POINT1, (float)tableTargetTemp, calibrationTableMaxTemperature-calibrationTableMinTemperature, (float)calibrationTableTime);
					PID_CalibrationData(extruderPID, (float)CALIBRATION_EXTRUDER_POINT1, (float)extruderTargetTemp, calibrationExtruderMaxTemperature-calibrationExtruderMinTemperature, (float)calibrationExtruderTime);
					if(tableTargetTemp==CALIBRATION_TABLE_POINT1) {
						heaters_calibrate_till(CALIBRATION_TABLE_POINT2, CALIBRATION_EXTRUDER_POINT2);
					} else {
						VEEPROM_SetData(HEATERS_DATA_VEEPROM_PAGE, &calibrationData, sizeof (CALIBRATION_DATA));
						calibrationDataRead = 1;
						calibrationCurrentStatus = STATUS_OK;
						tableTargetTemp = HEATER_OFF;
						extruderTargetTemp = HEATER_OFF;
						heatersWaiter = HW_NONE;
					}
				}

				calibrationCounter++;
			}
			TIM3->ARR = TIMER_ITTERATION_COUNT;
			mNextItteration = Regular;
		} else {

			if(fanAndMeasure()) {
				if(tableMeasuredTemp==(float)HEATER_NOT_CONNECTED_TEMP) {
					tableTargetTemp = HEATER_OFF;
				} else if (tableTargetTemp!=HEATER_OFF) {
					tableHeaterPeriod = PID_UpdatePID(tablePID, (float)tableTargetTemp-tableMeasuredTemp, tableMeasuredTemp);
				}

				if(extruderMeasuredTemp==(float)HEATER_NOT_CONNECTED_TEMP) {
					extruderTargetTemp = HEATER_OFF;
				} else if (extruderTargetTemp!=HEATER_OFF) {
					extruderHeaterPeriod = PID_UpdatePID(extruderPID, (float)extruderTargetTemp-extruderMeasuredTemp, extruderMeasuredTemp);
				}
			}

			// turning on heaters and schedule turning off
			int min = TIMER_ITTERATION_COUNT;
			if(tableHeaterPeriod && tableTargetTemp!=HEATER_OFF) {
				if(tableHeaterPeriod<min)
					min = tableHeaterPeriod;
				HEATERS_PORT->BSRR = TABLE_HEATER_PIN;
			} else {
				HEATERS_PORT->BRR = TABLE_HEATER_PIN;
			}
			if(extruderHeaterPeriod && extruderTargetTemp!=HEATER_OFF) {
				if(extruderHeaterPeriod<min)
					min = extruderHeaterPeriod;
				HEATERS_PORT->BSRR = EXTRUDER_HEATER_PIN;
			} else {
				HEATERS_PORT->BRR = EXTRUDER_HEATER_PIN;
			}
			TIM3->ARR = min;
			mNextItteration = (min==TIMER_ITTERATION_COUNT)?Regular:ShutdownMin;

			if(heatersWaiter==HW_TABLE) {
				if(heater_table_reached()==STATUS_OK)
					heatersWaiter=HW_NONE;
			} else if(heatersWaiter==HW_EXTRUDER)  {
				if(heater_extruder_reached()==STATUS_OK)
					heatersWaiter=HW_NONE;
			} else if(heatersWaiter==HW_TABLEEXTRUDER)  {
				if(heater_extruder_reached()==STATUS_OK && heater_table_reached()==STATUS_OK)
					heatersWaiter=HW_NONE;
			}

		}
// and off heaters
	} else	if(mNextItteration == ShutdownMin) {
		TIM3->ARR = TIMER_ITTERATION_COUNT;
		mNextItteration = Regular;
		if(tableHeaterPeriod && tableHeaterPeriod<TIMER_ITTERATION_COUNT && (tableHeaterPeriod<=extruderHeaterPeriod || extruderHeaterPeriod==0)) {
			HEATERS_PORT->BRR = TABLE_HEATER_PIN;
			if(extruderHeaterPeriod>tableHeaterPeriod && extruderHeaterPeriod<TIMER_ITTERATION_COUNT) {
				TIM3->ARR = extruderHeaterPeriod - tableHeaterPeriod;
				mNextItteration = ShutdownMax;
			} else {
				TIM3->ARR = TIMER_ITTERATION_COUNT - tableHeaterPeriod;
				mNextItteration = Regular;
			}
		}
		if(extruderHeaterPeriod && extruderHeaterPeriod<TIMER_ITTERATION_COUNT  && (extruderHeaterPeriod<=tableHeaterPeriod || tableHeaterPeriod==0)) {
			HEATERS_PORT->BRR = EXTRUDER_HEATER_PIN;
			if(tableHeaterPeriod>extruderHeaterPeriod && tableHeaterPeriod<TIMER_ITTERATION_COUNT) {
				TIM3->ARR = tableHeaterPeriod - extruderHeaterPeriod;
				mNextItteration = ShutdownMax;
			} else {
				TIM3->ARR = TIMER_ITTERATION_COUNT - extruderHeaterPeriod;
				mNextItteration = Regular;
			}
		}
	} else if(mNextItteration == ShutdownMax) {
		TIM3->ARR = TIMER_ITTERATION_COUNT;
		if(tableHeaterPeriod && tableHeaterPeriod<TIMER_ITTERATION_COUNT  && tableHeaterPeriod>=extruderHeaterPeriod) {
			HEATERS_PORT->BRR = TABLE_HEATER_PIN;
			TIM3->ARR = TIMER_ITTERATION_COUNT - tableHeaterPeriod;
		}
		if(extruderHeaterPeriod && extruderHeaterPeriod<TIMER_ITTERATION_COUNT && extruderHeaterPeriod>=tableHeaterPeriod) {
			HEATERS_PORT->BRR = EXTRUDER_HEATER_PIN;
			TIM3->ARR = TIMER_ITTERATION_COUNT - extruderHeaterPeriod;
		}
		mNextItteration = Regular;
	}
}

STATUSES heater_table_heat(int temperature)
{
	if(calibrationDataRead==0)
		return STATUS_ERROR_NOT_CALIBRATED;
	if(heatersWaiter==HW_CALIBRATION)
		return STATUS_BUSY;
	if(temperature<=HEATER_NOT_CONNECTED_TEMP)
		return heater_table_heat_off();
	if(temperature>MAX_TABLE_TEMPERATURE)
		return STATUS_ERROR_OUT_OF_TEMPERATURERANGE;
	if(tableMeasuredTemp<=(float)HEATER_NOT_CONNECTED_TEMP)
		return STATUS_ERROR_HEATER_NOT_CONNECTED;

	PID_PreparePID(tablePID, tableMeasuredTemp, (float)temperature);
	tableTargetTemp = temperature;
	if(temperature-(int)tableMeasuredTemp>DELTA_TEMPERATRE_TABLE_FIX)
		tableHeaterPeriod = TIMER_ITTERATION_COUNT/2;
	else
		tableHeaterPeriod = 0;
	if((int)tableMeasuredTemp<temperature)
		HEATERS_PORT->BSRR = TABLE_HEATER_PIN;
	return STATUS_OK;
}

int heater_get_table_temperature()
{
	return tableTargetTemp;
}

STATUSES heater_table_heat_off()
{
	if(heatersWaiter==HW_CALIBRATION) {
		calibrationCurrentStatus = STATUS_ERROR_ESTOP;
		heatersWaiter = HW_NONE;
		HEATERS_PORT->BRR = EXTRUDER_HEATER_PIN;
	}
	tableTargetTemp = HEATER_OFF;
	HEATERS_PORT->BRR = TABLE_HEATER_PIN;
	PID_ResetPID(tablePID);
	return STATUS_OK;
}

int  heater_measured_table_temp()
{
	return (int)roundf(tableMeasuredTemp);
}

STATUSES heater_extruder_heat(int temperature)
{
	if(calibrationDataRead==0)
		return STATUS_ERROR_NOT_CALIBRATED;
	if(heatersWaiter==HW_CALIBRATION)
		return STATUS_BUSY;
	if(temperature<=HEATER_NOT_CONNECTED_TEMP)
		return heater_extruder_heat_off();
	if(temperature>MAX_EXTRUDER_TEMPERATURE)
		return STATUS_ERROR_OUT_OF_TEMPERATURERANGE;
	if(extruderMeasuredTemp<=(float)HEATER_NOT_CONNECTED_TEMP)
		return STATUS_ERROR_HEATER_NOT_CONNECTED;

	PID_PreparePID(extruderPID, extruderMeasuredTemp, (float)temperature);
	extruderTargetTemp = temperature;
	if(temperature-(int)extruderMeasuredTemp>DELTA_TEMPERATRE_EXTRUDER_FIX)
			extruderHeaterPeriod= TIMER_ITTERATION_COUNT/2;
		else
			extruderHeaterPeriod = 0;
	if(temperature>(int)extruderMeasuredTemp)
		HEATERS_PORT->BSRR = EXTRUDER_HEATER_PIN;
	return STATUS_OK;
}

int heater_get_extruder_temperature()
{
	return extruderTargetTemp;
}

STATUSES heater_extruder_heat_off()
{
	if(heatersWaiter==HW_CALIBRATION) {
		calibrationCurrentStatus = STATUS_ERROR_ESTOP;
		heatersWaiter = HW_NONE;
		HEATERS_PORT->BRR = TABLE_HEATER_PIN;
	}
	extruderTargetTemp = HEATER_OFF;
	HEATERS_PORT->BRR = EXTRUDER_HEATER_PIN;
	PID_ResetPID(extruderPID);
	return STATUS_OK;
}

int  heater_measured_extruder_temp()
{
	return (int)roundf(extruderMeasuredTemp);
}

STATUSES heater_table_reached()
{
	if(tableTargetTemp==HEATER_OFF)
		return STATUS_OK;
	if(tablePID->lockTime<TIMER_MEASURE_FREQUENCY*REACH_TIME/ADC_NUMBER_OF_ITERATIONS)
		return STATUS_BUSY;
	return STATUS_OK;
}

STATUSES heater_extruder_reached()
{
	if(extruderTargetTemp==HEATER_OFF)
		return STATUS_OK;
	if(extruderPID->lockTime<TIMER_MEASURE_FREQUENCY*REACH_TIME/ADC_NUMBER_OF_ITERATIONS)
		return STATUS_BUSY;
	return STATUS_OK;
}

STATUSES heaters_wait(HEATER_WAIT what) {
	if(what==HW_NONE) {
		heatersWaiter = HW_NONE;
	} else if( what == HW_TABLE) {
		if(heatersWaiter == HW_TABLE || heatersWaiter == HW_TABLEEXTRUDER)
			return STATUS_OK;
		if(heater_get_table_temperature()==HEATER_OFF)
			return STATUS_ERROR_HEATER_NOT_TURNED_ON;
		if(heater_measured_table_temp()==HEATER_NOT_CONNECTED_TEMP)
			return STATUS_ERROR_HEATER_NOT_CONNECTED;
		if(heater_table_reached()==STATUS_BUSY) {
			if(heatersWaiter==HW_EXTRUDER)
				heatersWaiter = HW_TABLEEXTRUDER;
			else if (heatersWaiter!=HW_TABLEEXTRUDER)
				heatersWaiter = HW_TABLE;
		}
	} else if( what == HW_EXTRUDER) {
		if(heatersWaiter == HW_EXTRUDER || heatersWaiter == HW_TABLEEXTRUDER)
			return STATUS_OK;
		if(heater_get_extruder_temperature()==HEATER_OFF)
			return STATUS_ERROR_HEATER_NOT_TURNED_ON;
		if(heater_measured_extruder_temp()==HEATER_NOT_CONNECTED_TEMP)
			return STATUS_ERROR_HEATER_NOT_CONNECTED;
		if(heater_extruder_reached()==STATUS_BUSY) {
			if(heatersWaiter==HW_TABLE)
				heatersWaiter = HW_TABLEEXTRUDER;
			else if (heatersWaiter!=HW_TABLEEXTRUDER)
				heatersWaiter = HW_EXTRUDER;
		}
	} else if(what == HW_TABLEEXTRUDER) {
		if(heater_get_table_temperature()==HEATER_OFF && heater_get_extruder_temperature()==HEATER_OFF)
			return STATUS_ERROR_HEATER_NOT_TURNED_ON;
		if(heater_measured_table_temp()==HEATER_NOT_CONNECTED_TEMP && heater_measured_extruder_temp()==HEATER_NOT_CONNECTED_TEMP)
			return STATUS_ERROR_HEATER_NOT_CONNECTED;
		if(heater_extruder_reached()==STATUS_BUSY || heater_table_reached()==STATUS_BUSY) {
			heatersWaiter = HW_TABLEEXTRUDER;
		}
	} else if(what == HW_CALIBRATION) {
		if(heater_measured_table_temp()==HEATER_NOT_CONNECTED_TEMP && heater_measured_extruder_temp()==HEATER_NOT_CONNECTED_TEMP)
			return STATUS_ERROR_HEATER_NOT_CONNECTED;
		heatersWaiter = HW_CALIBRATION;
	}
	return STATUS_OK;
}

HEATER_WAIT heaters_wait_status() {
	return heatersWaiter;
}

STATUSES heater_extruder_ready() {
	return ( (COOLING_PORT->ODR & COOLING_PIN) && (AXIS_E_DRIVER_ENABLE_PORT->ODR & AXIS_E_DRIVER_ENABLE_PIN) && (extruderMeasuredTemp+0.5f)>=(float)MIN_EXTRUDER_TEMP_FORMOVEMENT && extruderTargetTemp!=HEATER_OFF)?STATUS_OK:STATUS_ERROR_EXTRUDER_NOT_HEATED;
}

STATUSES heaters_calibrate()
{
	calibrationCurrentStatus = STATUS_BUSY;
	return heaters_calibrate_till(CALIBRATION_TABLE_POINT1, CALIBRATION_EXTRUDER_POINT1);
}

STATUSES heaters_lastCalibrationStatus() {
	if(calibrationCurrentStatus != STATUS_BUSY && calibrationDataRead == 0)
		return STATUS_ERROR_NOT_CALIBRATED;
	return calibrationCurrentStatus;
}


#define STM32_CLOCK_HZ 72000000UL
#define STM32_CYCLES_PER_LOOP 8 // This will need tweaking or calculating, uses for delay, 8 for stm32f103zet6
void inline delay_us(const uint32_t us)
{
	uint32_t takts= us * STM32_CLOCK_HZ / 1000000 / STM32_CYCLES_PER_LOOP;

    asm volatile(" mov r0, %[takts] \n\t"
             "1: subs r0, #1 \n\t"
             " bhi 1b \n\t"
             :
             : [takts] "r" (takts)
             : "r0");
}

void cooler_laserActivate() {
	laserCoolerTimer = 0;
	if( !(COOLING_PORT->ODR & COOLING_PIN)) { // драйвер лазера питается по тем же проводам, что и охлаждение. Холдим все на небольшое время, что бы дать зарядиться конденсаторам в драйвере
		COOLING_PORT->BSRR = COOLING_PIN;
		delay_us(100*1000); // 100 ms
	}
}


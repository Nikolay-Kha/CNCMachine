#include "mcore.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include <math.h>

u32 WPULSE;
u32 WBRPULSE;
char mCalibarationInProgress = 0; // когда включена калибровка
char mToolChangingInProgress = 0; // когда вызвана смена инструмента
char mPauseInProgress = 0; // когда вызвана пауза
char mParkInProgress = 0; // поэтапная парковка бошки
float mToolChangeOldZPos = 0.0f;

#define INTERPOLATION_DONE -10000


typedef struct{
	int ValueInterpolation; // интерполяция записывает сюда свои насчитанные значения
	int ValueZero; // значение куда будем писать координату до начала интерполяции
	float ValueCoeficient; // коэфициент для интерполяции
	int Position; // текущая позиция в импульсах
	s32 sensorError; // счетчик тактов для отсечения погрешности датчиков нуля
	const u32 TABLE_SIZE; // размер стола по этой оси в импульсах
	const u16 STEP_PIN;
	const u16 DIR_PIN;
	const u16 SENSE_PIN;
	const u32 PULSES_PER_MM; // импульсов на милиметр
	const float MAX_VELOCITY; //pulse per tact not more than 1
	const float MAX_ACCELERATION; // pulses per tact^2
	const int SENSOR_POSITION; // 1 - в начале стола, -1 - в конце стола, сенсоры должны стоять чтобы движки вращаясь с отпущенным DIR шли к сенсорам
	const int INVERTED; // 1 - не ивертирована, -1 - инвертирована, меняет направление движения
} _AXIS;


#define SENSOR_ERROR_COUNT 400 // 1.0 mm - ошибка определения нуля датчиками, на сколь можно позволить уйти осям назад после срабатывания датчика
#define PULSES_PER_MM_EXTRUDERMOTOR 96

_AXIS X_AXIS = {INTERPOLATION_DONE, 0, 0.0f, 0, 0, AXIS_X_TABLE_SIZE_MM*PULSES_PER_MM_X, AXIS_X_STEPPIN, AXIS_X_DIRPIN, AXIS_XSENSOR_PIN, PULSES_PER_MM_X, (float)(X_MAXVELOCITY/60*PULSES_PER_MM_X)/(float)TACTS_PER_SEC, (float)X_ACCELERATION/(float)TACTS_PER_SEC/(float)TACTS_PER_SEC, AXIS_XSENSOR_POSITION, AXIS_XINVERTED};
_AXIS Y_AXIS = {INTERPOLATION_DONE, 0, 0.0f, 0, 0, AXIS_Y_TABLE_SIZE_MM*PULSES_PER_MM_Y, AXIS_Y_STEPPIN, AXIS_Y_DIRPIN, AXIS_YSENSOR_PIN, PULSES_PER_MM_Y,(float)(Y_MAXVELOCITY/60*PULSES_PER_MM_Y)/(float)TACTS_PER_SEC, (float)Y_ACCELERATION/(float)TACTS_PER_SEC/(float)TACTS_PER_SEC, AXIS_YSENSOR_POSITION, AXIS_YINVERTED};
_AXIS Z_AXIS = {INTERPOLATION_DONE, 0, 0.0f, 0, 0, AXIS_Z_TABLE_SIZE_MM*PULSES_PER_MM_Z, AXIS_Z_STEPPIN, AXIS_Z_DIRPIN, AXIS_ZSENSOR_PIN, PULSES_PER_MM_Z, (float)(Z_MAXVELOCITY/60*PULSES_PER_MM_Z)/(float)TACTS_PER_SEC, (float)Z_ACCELERATION/(float)TACTS_PER_SEC/(float)TACTS_PER_SEC, AXIS_ZSENSOR_POSITION, AXIS_ZINVERTED};
_AXIS E_AXIS = {INTERPOLATION_DONE, 0, 0.0f, 0, 0, 0, AXIS_E_STEPPIN, AXIS_E_DIRPIN, 0, PULSES_PER_MM_E, (float)(E_MAXVELOCITY/60*PULSES_PER_MM_E)/(float)TACTS_PER_SEC, (float)E_ACCELERATION/(float)TACTS_PER_SEC/(float)TACTS_PER_SEC, AXIS_ESENSOR_POSITION, AXIS_EINVERTED};

// AAXISTODO
_AXIS A_AXIS = {INTERPOLATION_DONE, 0, 0.0f, 0, 0, 0, AXIS_A_STEPPIN, AXIS_A_DIRPIN, 0, PULSES_PER_MM_A, (float)(A_MAXVELOCITY/60*PULSES_PER_MM_A)/(float)TACTS_PER_SEC, (float)A_ACCELERATION/(float)TACTS_PER_SEC/(float)TACTS_PER_SEC, AXIS_ASENSOR_POSITION, AXIS_AINVERTED};

unsigned int private_spindle_result = 0;
unsigned int spindleCounter = 0;
unsigned int spindleACounter = 0;
#define SPINDLE_IDLE_VALUE 6500
#define SPINDLE_FILTER_VALUE 3 // антидребезг контактов геркона
#define SPINDLE_PROBE_CHANGE_VALUE 20
#define SPINDLE_PROBE_WAIT_TIME TACTS_PER_SEC*5
#define AXIS_ZPROBE_SPEED_DIV 28 // каждая ось со скоростью ~400
uint16_t LASER_MAX_PERIOD;
uint16_t LASER_MIN_PERIOD;
uint16_t runningLaserPower = 0;
uint32_t forceLaserTime = 0;

void axis_init()
{
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	// порты
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(getRcc(AXIS_PORT) , ENABLE);
	RCC_APB2PeriphClockCmd(getRcc(SPINDLE_PORT) | RCC_APB2Periph_AFIO , ENABLE);
	RCC_APB2PeriphClockCmd(getRcc(AXIS_PORT_SENSE) , ENABLE);
	RCC_APB2PeriphClockCmd(getRcc(LASER_PORT) , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_AFIOEN , ENABLE);

	GPIO_InitStructure.GPIO_Pin = X_AXIS.STEP_PIN | X_AXIS.DIR_PIN | Y_AXIS.STEP_PIN | Y_AXIS.DIR_PIN | Z_AXIS.STEP_PIN | Z_AXIS.DIR_PIN | E_AXIS.STEP_PIN | E_AXIS.DIR_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( AXIS_PORT , &GPIO_InitStructure);
	AXIS_PORT->BRR = GPIO_InitStructure.GPIO_Pin;

	GPIO_InitStructure.GPIO_Pin = X_AXIS.SENSE_PIN| Y_AXIS.SENSE_PIN | Z_AXIS.SENSE_PIN | SPINDLE_SENS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( AXIS_PORT_SENSE , &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LASER_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( LASER_PORT , &GPIO_InitStructure);
	LASER_PORT->BRR = GPIO_InitStructure.GPIO_Pin;

	spindle_stop(); // init spindle port to 0

	// таймер http://robocraft.ru/blog/ARM/722.html
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	/* Инициализируем базовый таймер: делитель, период.
	   * Другие параметры структуры TIM_TimeBaseInitTypeDef
	   * не имеют смысла для базовых таймеров.
	   */
	TIM_TimeBaseInitTypeDef base_timer;
	TIM_TimeBaseStructInit(&base_timer);
	/* Делитель учитывается как TIM_Prescaler + 1, поэтому отнимаем 1 */
	base_timer.TIM_Period = 99;
	base_timer.TIM_Prescaler = (clocks.SYSCLK_Frequency/TACTS_PER_SEC/(base_timer.TIM_Period+1)) - 1; // при 72 мгц  -> 1.20 mhz
	TIM_TimeBaseInit(TIM7, &base_timer);

	  /* Разрешаем прерывание по обновлению (в данном случае -
	   * по переполнению) счётчика таймера TIM7.
	   */
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	  /* Включаем таймер */
	TIM_Cmd(TIM7, ENABLE);

	  /* Разрешаем обработку прерывания по переполнению счётчика
	   * таймера TIM7.
	   */
	NVIC_EnableIRQ(TIM7_IRQn);
	NVIC_SetPriority(TIM7_IRQn, 0);


	// таймер длительности импульса движка http://about-stm32.narod.ru/delay.html
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	TIM_TimeBaseInitTypeDef base_timer6;
	TIM_TimeBaseStructInit(&base_timer6);
	base_timer6.TIM_Period = PULSE_LENGHT_uS-1;
	base_timer6.TIM_Prescaler =  (clocks.SYSCLK_Frequency/1000000/(base_timer6.TIM_Period+1)) - 1;
	TIM_TimeBaseInit(TIM6, &base_timer6);
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM6, DISABLE);
	//TIM6->CR1 |= TIM_CR1_CEN|TIM_CR1_OPM;	//Запускаем таймер записью бита CEN и устанавливаем режим Одного прохода установкой бита OPM

	// таймер выключения лазера http://about-stm32.narod.ru/delay.html
	LASER_MAX_PERIOD = clocks.SYSCLK_Frequency/TACTS_PER_SEC;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	TIM_TimeBaseInitTypeDef base_timer1;
	TIM_TimeBaseStructInit(&base_timer1);
	base_timer1.TIM_Period = LASER_MAX_PERIOD - 1;
	base_timer1.TIM_Prescaler =   0;
	TIM_TimeBaseInit(TIM1, &base_timer1);
	LASER_MIN_PERIOD = LASER_MIN_PULSE_LENGTH_CPU_TACTS/(base_timer1.TIM_Prescaler+1)/LASER_MIN_PULSE_LENGTH_LOW_ACCURACITY_DIV;

	TIM_OCInitTypeDef oc_init;
	TIM_OCStructInit(&oc_init);
	oc_init.TIM_OCMode = TIM_OCMode_PWM1;   // работаем в режиме ШИМ ( PWM )
	oc_init.TIM_OutputState = TIM_OutputState_Enable;
	oc_init.TIM_Pulse = base_timer1.TIM_Period/2-1;   // скважность
	oc_init.TIM_OCPolarity = TIM_OCPolarity_High;  // положительная полярность
	TIM_OC4Init(TIM1,&oc_init);   /// заносим данные в канал
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, DISABLE);
	TIM_Cmd(TIM1, DISABLE);

	WBRPULSE = X_AXIS.STEP_PIN | Y_AXIS.STEP_PIN | Z_AXIS.STEP_PIN;
	NVIC_EnableIRQ(TIM6_IRQn);
	NVIC_SetPriority(TIM6_IRQn, 0);

	private_heater_init();
}

// функция может быть любой, главное, чтобы повороты внутри нее не превысили ускорение осей - проверять/строить так чтобы не превысили при постановке задания
typedef int (*InterpolationFunction)(u32 t);

InterpolationFunction mInterpolationFunction=0;
u32 mInterpolationRealTime = 0; /// сколько времени нужно на интерполирование - в абсолютной величине
u32 mAccelerationTime; // время на ускорение - в абсолютном времени
u32 mBrakeTime; // время на торможение - в абсолютном времени
float mTimeAccelerationCoef, mTimeBrakeCoef;

u32 timeAccelerationFinished;
u32 timeLinearFinished;
float velocityCoeficient = 1.0f;
static inline u32 timeWithAcceleration(u32 t) {
	if(t<=mAccelerationTime) { // ускоряем время до получения времени 1 к 1
		timeAccelerationFinished = roundf(mTimeAccelerationCoef*t*t);
		timeLinearFinished = timeAccelerationFinished;
		velocityCoeficient = mAccelerationTime?((float)t/(float)mAccelerationTime):1.0f;
		return timeAccelerationFinished;
	} else if(t>mInterpolationRealTime) { // если время вышло
		velocityCoeficient = 0.0f;
		return INTERPOLATION_DONE;
	} else  {
		const int delta = mInterpolationRealTime-mBrakeTime;
		if(t>delta) { // замедляем вермя
			const int del = t-delta;
			velocityCoeficient = 1.0f - (mBrakeTime?((float)del/(float)mBrakeTime):0.0f);
			return timeLinearFinished + roundf((float)(mBrakeTime*2 -del)*mTimeBrakeCoef*(float)del);
		} else { // отдача в реальном времени
			timeLinearFinished = (t - mAccelerationTime) + timeAccelerationFinished;
			velocityCoeficient = 1.0f;
			return timeLinearFinished;
	}
	}
}

STATUSES park_all(){
	return axis_move_to_linear((X_AXIS.SENSOR_POSITION==-1)?(float)AXIS_X_TABLE_SIZE_MM:0.0f, (Y_AXIS.SENSOR_POSITION==-1)?(float)AXIS_Y_TABLE_SIZE_MM:0.0f, (Z_AXIS.SENSOR_POSITION==-1)?(float)AXIS_Z_TABLE_SIZE_MM:0.0f, axis_getEPos(), axis_getAPos(), AXIS_MAX_SPEED, 0.0f);
}

int linearInterpolation(u32 t) {
	if(t==INTERPOLATION_DONE) {
		X_AXIS.ValueInterpolation = INTERPOLATION_DONE;
		Y_AXIS.ValueInterpolation = INTERPOLATION_DONE;
		Z_AXIS.ValueInterpolation = INTERPOLATION_DONE;
		E_AXIS.ValueInterpolation = INTERPOLATION_DONE;

		if(mParkInProgress) {
			mParkInProgress = 0;
			mInterpolationFunction = 0;
			if(park_all()==STATUS_OK)
				return 1;
			else
				return 0;
		}
		return 0;
	}
	X_AXIS.ValueInterpolation = X_AXIS.ValueZero + roundf((float)t * X_AXIS.ValueCoeficient);
	Y_AXIS.ValueInterpolation = Y_AXIS.ValueZero + roundf((float)t * Y_AXIS.ValueCoeficient);
	Z_AXIS.ValueInterpolation = Z_AXIS.ValueZero + roundf((float)t * Z_AXIS.ValueCoeficient);
	E_AXIS.ValueInterpolation = E_AXIS.ValueZero + roundf((float)t * E_AXIS.ValueCoeficient);
	return 1;
}

InterpolationPlane roundPlane;
// RoundRadius должны так же содержать знак в какую сторону направлена ось на станке, поэтому их три
float xRoundRadius;
float yRoundRadius;
float zRoundRadius;
float roundLinearCoeficient;

float roundFinishPosX;
float roundFinishPosY;
float roundFinishPosZ;
float roundFinishPosE;
float roundFinishPosA;
float roundFinishSpeed;
float roundFinishLaserPower;
float roundAlphaMin; // стартовый угол
float roundAlphaCoef; // так чтобы при умножении на конечное время было равно конечному углу
//ValueZero - центр окружности
inline void nativeroundInterpolation(float t, float a) {
	float cosa;// = cosf(a); too much time
	float sina;// = sinf(a);

	// fast sine cosine - http://lab.polygonal.de/?p=205 ---------------------------------------
	if (a < -3.14159265f) { // -6PI...6PI
		a += 6.28318531f;
		if (a < -3.14159265f) {
			a += 6.28318531f;
			if (a < -3.14159265f)
				a += 6.28318531f;
		}
	}else if (a >  3.14159265f) {
		a -= 6.28318531f;
		if (a >  3.14159265f) {
			a -= 6.28318531f;
			if (a >  3.14159265f)
				a -= 6.28318531f;
		}
	}
	//always wrap input angle to -PI..PI
	//compute sine
	if (a < 0)	{
		sina = 1.27323954f * a + 0.405284735f * a * a;
		if (sina < 0)  // можно убрать здесь и далее, тогда точность снизиться, но будет быстрее считаться
			sina = 0.225f * (sina *-sina - sina) + sina;
		else
			sina = 0.225f * (sina * sina - sina) + sina;
	} else {
		sina = 1.27323954f * a - 0.405284735f * a * a;
		if (sina < 0)
			sina = 0.225f * (sina *-sina - sina) + sina;
		else
			sina = 0.225f * (sina * sina - sina) + sina;
	}

	//compute cosine: sin(x + PI/2) = cos(x)
	a += 1.57079632f;
	if (a >  3.14159265f)
		a -= 6.28318531f;
	if (a < 0)	{
		cosa = 1.27323954f * a + 0.405284735f * a * a;
		if (cosa < 0)
			cosa = 0.225f * (cosa *-cosa - cosa) + cosa;
		else
			cosa = 0.225f * (cosa * cosa - cosa) + cosa;
	} else 	{
		cosa = 1.27323954f * a - 0.405284735f * a * a;
		if (cosa < 0)
			cosa = 0.225f * (cosa *-cosa - cosa) + cosa;
		else
			cosa = 0.225f * (cosa * cosa - cosa) + cosa;
	}
	// end sin cos -----------------------------------------------------------------------------

	switch(roundPlane) {
	case PLANE_XY:
		X_AXIS.ValueInterpolation = X_AXIS.ValueZero + xRoundRadius*cosa;
		Y_AXIS.ValueInterpolation = Y_AXIS.ValueZero + yRoundRadius*sina;
		Z_AXIS.ValueInterpolation = Z_AXIS.ValueZero + t*roundLinearCoeficient;
		break;
	case PLANE_YZ:
		X_AXIS.ValueInterpolation = X_AXIS.ValueZero + t*roundLinearCoeficient;
		Y_AXIS.ValueInterpolation = Y_AXIS.ValueZero + yRoundRadius*cosa;
		Z_AXIS.ValueInterpolation = Z_AXIS.ValueZero + zRoundRadius*sina;
		break;
	case PLANE_ZX:
		X_AXIS.ValueInterpolation = X_AXIS.ValueZero + xRoundRadius*sina;
		Y_AXIS.ValueInterpolation = Y_AXIS.ValueZero + t*roundLinearCoeficient;
		Z_AXIS.ValueInterpolation = Z_AXIS.ValueZero + zRoundRadius*cosa;
		break;
	}
	E_AXIS.ValueInterpolation = E_AXIS.ValueZero + roundf((float)t * E_AXIS.ValueCoeficient);
}
int roundInterpolation(u32 t) {
	if(t==INTERPOLATION_DONE) {
		X_AXIS.ValueInterpolation = INTERPOLATION_DONE;
		Y_AXIS.ValueInterpolation = INTERPOLATION_DONE;
		Z_AXIS.ValueInterpolation = INTERPOLATION_DONE;
		E_AXIS.ValueInterpolation = INTERPOLATION_DONE;
		if(mAccelerationTime==0) // нажатие е-стоп, только он может сбросить ускорение на ноль во время задания
			return 0;
		// доводка, т.к. конечная точка не обязательно будет на сечений
		mInterpolationFunction = 0;
		if(axis_move_to_linear(roundFinishPosX, roundFinishPosY, roundFinishPosZ, roundFinishPosE, roundFinishPosA, roundFinishSpeed, roundFinishLaserPower)==STATUS_OK)
			return 1;
		else
			return 0;
	}
	float a = roundAlphaMin+t*roundAlphaCoef;
	nativeroundInterpolation(t, a);
	return 1;
}


u32 TIMER_COUNTER;

int pauseInterpolation(u32 t){
	if (TIMER_COUNTER>mInterpolationRealTime)
		return 0;
	return 1;
}

int lastProbeRPM = 0;
float lastProbeFoundZ = MAXFLOAT;
int probeInterpolation(u32 t){
	if (TIMER_COUNTER>mInterpolationRealTime) { // если долго ничего не произошло
		spindle_stop();
		Z_AXIS.ValueInterpolation = INTERPOLATION_DONE;
		lastProbeFoundZ = MAXFLOAT;
		return 0;
	} else if(TIMER_COUNTER==0) {// включаем шпиндиль
		spindle_runp(100.0f);
	} else if(TIMER_COUNTER==TACTS_PER_SEC*0.05f) {// 50 ms и переводим шпиндиль в режим очень слабого вращения
		spindle_runp(SPINDLE_PROBE_MODE);
	} else if(TIMER_COUNTER==SPINDLE_PROBE_WAIT_TIME-1) {
		lastProbeRPM = spindle_get_rpm();
	} else if(TIMER_COUNTER>=SPINDLE_PROBE_WAIT_TIME) {// после раскрутки начинаем проверять обороты шпинделя и опускаем головку
		int rpm = spindle_get_rpm();
		if(rpm<SPINDLE_PROBE_CHANGE_VALUE || fabsf(lastProbeRPM-rpm)>SPINDLE_PROBE_CHANGE_VALUE) {
			spindle_stop();
			Z_AXIS.ValueInterpolation = INTERPOLATION_DONE;
			lastProbeFoundZ = Z_AXIS.ValueCoeficient;
			return 0;
		}
		Z_AXIS.ValueInterpolation = Z_AXIS.ValueZero + roundf((float)(TIMER_COUNTER-SPINDLE_PROBE_WAIT_TIME) * Z_AXIS.ValueCoeficient);
	}
	return 1;
}

static inline void axis_hanlde(_AXIS *A)
{
	if(A->ValueInterpolation!=INTERPOLATION_DONE) {
		if (A->ValueInterpolation>A->Position){
			AXIS_PORT->BRR = A->DIR_PIN;// направление
			A->Position++;
			WPULSE |= A->STEP_PIN; // импульс
		} else if (A->ValueInterpolation<A->Position) {
			AXIS_PORT->BSRR = A->DIR_PIN;// направление
			A->Position--;
			WPULSE |= A->STEP_PIN; // импульс
		}
	}
}

static inline u8 checkSensor(_AXIS *A) {
	if( (AXIS_PORT_SENSE->IDR & A->SENSE_PIN)==0) {
		if(A->sensorError==0)
			A->sensorError = A->Position;
		if(mInterpolationFunction!=0 ) {
			if(A->sensorError-A->Position>=SENSOR_ERROR_COUNT || mCalibarationInProgress) {
				if(mCalibarationInProgress) {
					A->Position = 0;
					A->ValueZero = 0;
					A->sensorError = 0;
				}
				if(A->ValueCoeficient<0.0f) {
					A->ValueInterpolation = INTERPOLATION_DONE;
					return 1;
				}
			}
		}
	} else {
		A->sensorError = 0;
	}
	return 0;
}

void TIM6_IRQHandler() // дергается вслед за TIM6_IRQHandler с задержкой PULSE_LENGHT_uS
{
	__disable_irq();
	TIM6->SR = (uint16_t)~TIM_IT_Update;
	// проверяем датчики не коснулись ли и если надо продливаем импульс на полную остановку
	if(checkSensor(&X_AXIS)) WBRPULSE &= ~X_AXIS.STEP_PIN;
	if(checkSensor(&Y_AXIS)) WBRPULSE &= ~Y_AXIS.STEP_PIN;
	if(checkSensor(&Z_AXIS)) WBRPULSE &= ~Z_AXIS.STEP_PIN;

	AXIS_PORT->BRR = WBRPULSE;
	__enable_irq();
}

void TIM7_IRQHandler() // дергается 12000 раз в секунду, т.к 400 импульсов на милиметр, максимальная скорость 1800 мм/мин = 30 мм/сек = 12000 импульсов/сек
{
	__disable_irq();
//SysTick->LOAD  = SysTick_LOAD_RELOAD_Msk - 1;
//SysTick->VAL = 0;
//SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

   /* Очищаем бит обрабатываемого прерывания */
	TIM7->SR = (uint16_t)~TIM_IT_Update;
	WPULSE = 0;
	WBRPULSE = X_AXIS.STEP_PIN | Y_AXIS.STEP_PIN | Z_AXIS.STEP_PIN | E_AXIS.STEP_PIN;

	if(mInterpolationFunction) {
		// проверяем датчики не коснулись и если прижат - отменяем движение
		checkSensor(&X_AXIS);
		checkSensor(&Y_AXIS);
		checkSensor(&Z_AXIS);

		axis_hanlde(&X_AXIS);
		axis_hanlde(&Y_AXIS);
		axis_hanlde(&Z_AXIS);
		axis_hanlde(&E_AXIS);

		TIMER_COUNTER++;
		if(mInterpolationFunction(timeWithAcceleration(TIMER_COUNTER))==0) {
			WBRPULSE = 0;
			mInterpolationFunction = 0;
			if(forceLaserTime==0) {
				TIM_CtrlPWMOutputs(TIM1, DISABLE);
				TIM_Cmd(TIM1, DISABLE);
				runningLaserPower = 0;
			}
		}

		AXIS_PORT->BSRR = WPULSE; // импульс
	}

	// запуск таймера на выключение импульса
	TIM6->CR1 |= TIM_CR1_CEN|TIM_CR1_OPM;

	// замер оборотов шпинделя
	if(AXIS_PORT_SENSE->IDR & SPINDLE_SENS_PIN) {
		spindleCounter++;
		spindleACounter = 0;
		if(spindleCounter==SPINDLE_IDLE_VALUE)
			private_spindle_result = 0;
	} else {
		spindleACounter++;
		if(spindleCounter && spindleACounter>SPINDLE_FILTER_VALUE) { // filter антидребезг контактов геркона
			if(mInterpolationFunction==&probeInterpolation) {
				if(private_spindle_result==0 || TIMER_COUNTER>=SPINDLE_PROBE_WAIT_TIME )
					private_spindle_result = spindleCounter;
				else
					private_spindle_result = spindleCounter*0.9f+private_spindle_result*0.1f;
			} else {
				if(private_spindle_result==0)
					private_spindle_result = spindleCounter;
				else
					private_spindle_result = spindleCounter*0.3f+private_spindle_result*0.7f;
			}
			spindleCounter = 0;
		}
		if(spindleACounter==SPINDLE_IDLE_VALUE)
			private_spindle_result = 0;
	}

	// управление лазером
	if(runningLaserPower) {
		if(forceLaserTime) {
			forceLaserTime--;
			if(forceLaserTime==0) {
				TIM_CtrlPWMOutputs(TIM1, DISABLE);
				TIM_Cmd(TIM1, DISABLE);
				runningLaserPower = 0;
			}
			TIM1->CCR4 = runningLaserPower;
		} else {
			const uint16_t t = velocityCoeficient*(float)runningLaserPower;
			TIM1->CCR4 = (t>=LASER_MIN_PERIOD)?t:LASER_MIN_PERIOD;
		}
	} else {
		TIM1->CCR4 = 0;
	}

	__enable_irq();
//int time = SysTick->LOAD - SysTick->VAL;

if(TIM7->SR & TIM_IT_Update) // отладка - если тупит - светодиод будет помигивать, не должен моргать вообще
	DEBUGLED_PORT->BRR = CPUOVERUSAGELEDPIN;
else
	DEBUGLED_PORT->BSRR = CPUOVERUSAGELEDPIN;
}

STATUSES axis_check_table(int x, int y, int z) { // all in pulses
	if(x<0)
		return STATUS_ERROR_OUT_OF_TABLE;
	if(x>X_AXIS.TABLE_SIZE)
		return STATUS_ERROR_OUT_OF_TABLE;
	if(y<0)
		return STATUS_ERROR_OUT_OF_TABLE;
	if(y>Y_AXIS.TABLE_SIZE)
		return STATUS_ERROR_OUT_OF_TABLE;
	if(z<0)
		return STATUS_ERROR_OUT_OF_TABLE;
	if(z>Z_AXIS.TABLE_SIZE)
		return STATUS_ERROR_OUT_OF_TABLE;
	return STATUS_OK;
}


STATUSES init_laser(float laserPower_mW, int isRun){
	if(fabsf(laserPower_mW)==0.0f) {
		if(isRun)
			runningLaserPower = 0;
		return STATUS_OK;
	}
	if(laserPower_mW<0.0f) {
		if(isRun)
			runningLaserPower = 0;
		return STATUS_ERROR_OUT_OF_POWER;
	}
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);

	float lp = laserPower_mW/(float)LASER_OPTICAL_POWER_mW*(float)(clocks.SYSCLK_Frequency/TACTS_PER_SEC);
	if(roundf(lp)<1.0f)
		return STATUS_ERROR_OUT_OF_POWER;
	unsigned int k = 1;
	float dk = 1.0f;
	if(lp<LASER_MIN_PULSE_LENGTH_CPU_TACTS) {
		// подбираем частоту
		dk = (float)LASER_MIN_PULSE_LENGTH_CPU_TACTS/lp;
		if((float)TACTS_PER_SEC/dk<(float)LASER_MIN_PWM_FREQUENCY_HZ)
			return STATUS_ERROR_OUT_OF_POWER;

		if(dk*clocks.SYSCLK_Frequency/TACTS_PER_SEC>60000) { // 60000 - максимальное значение счетчика
			k = ceilf(dk/60000.0f*(float)(clocks.SYSCLK_Frequency/TACTS_PER_SEC));
			dk = dk/(float)k;
		}
	}
	if(!isRun)
		return STATUS_OK;
	cooler_laserActivate();
	TIM1->PSC = k - 1;
	const int mp = LASER_MIN_PULSE_LENGTH_CPU_TACTS*k/dk/LASER_MIN_PULSE_LENGTH_LOW_ACCURACITY_DIV;
	LASER_MIN_PERIOD = mp?mp:1;

	runningLaserPower = roundf(dk*lp);

	LASER_MAX_PERIOD = dk*clocks.SYSCLK_Frequency/TACTS_PER_SEC;
	TIM1->ARR = LASER_MAX_PERIOD - 1;

	TIM1->CNT = 0;
	TIM1->CCR4 = 0;
	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	return STATUS_OK;
}

STATUSES laser_force(float laserPower, int ms) {
	if(ms<0)
		return STATUS_ERROR_INVALID_PARAMETER;
	if(ms==0)
		return STATUS_OK;
	if(ms>2147483647/TACTS_PER_SEC*1000)
		return STATUS_ERROR_INVALID_PARAMETER;

	NVIC_DisableIRQ(TIM7_IRQn);
	STATUSES result = init_laser(laserPower, 1);
	if(result!=STATUS_OK) {
		NVIC_EnableIRQ(TIM7_IRQn);
		return result;
	}
	forceLaserTime = roundf((float)ms*((float)TACTS_PER_SEC/1000.0f));
	TIM7->SR &= ~((uint16_t)TIM_IT_Update);
	NVIC_EnableIRQ(TIM7_IRQn);
	return STATUS_OK;
}

void laser_force_stop()
{
	NVIC_DisableIRQ(TIM7_IRQn);
	if(forceLaserTime) {
		TIM_CtrlPWMOutputs(TIM1, DISABLE);
		TIM_Cmd(TIM1, DISABLE);
	}
	runningLaserPower = 0;
	forceLaserTime = 0;
	NVIC_EnableIRQ(TIM7_IRQn);
}

STATUSES axis_move_linear(float dx, float dy, float dz, float de, float da, float speed, float laserPower) // mm and mm/min
{
	if(fabsf(da)!=0) //AAXISTODO, еще назабыть проверить хардварное ограниечение - невозможность использовать А и Е одновременно
		return STATUS_UNIMPLEMENTED;
	STATUSES result = axis_status();
	if(result!=STATUS_OK)
		return result;
	result = init_laser(laserPower, 0);
	if(result!=STATUS_OK)
		return result;
	if(speed<AXIS_MIN_SPEED)
		return STATUS_ERROR_INVALID_PARAMETER;


	NVIC_DisableIRQ(TIM7_IRQn);

	if(fabsf(de)!=0 && heater_extruder_ready()!=STATUS_OK) {
		NVIC_EnableIRQ(TIM7_IRQn);
		return STATUS_ERROR_EXTRUDER_NOT_HEATED;
	}

	dx *= X_AXIS.SENSOR_POSITION * X_AXIS.INVERTED;
	dy *= Y_AXIS.SENSOR_POSITION * Y_AXIS.INVERTED;
	dz *= Z_AXIS.SENSOR_POSITION * Z_AXIS.INVERTED;
	de *= E_AXIS.INVERTED;

	int continious = 0;
	if(dx==-AXIS_CONTINIOUS) {
		dx = -(float)X_AXIS.Position/(float)X_AXIS.PULSES_PER_MM;
		continious = 1;	}
	if(dx==AXIS_CONTINIOUS) {
		dx =(float)(X_AXIS.TABLE_SIZE-X_AXIS.Position)/(float)X_AXIS.PULSES_PER_MM;
		continious = 1;	}
	if(dy==-AXIS_CONTINIOUS) {
		dy = -(float)Y_AXIS.Position/(float)Y_AXIS.PULSES_PER_MM;
		continious = 1;	}
	if(dy==AXIS_CONTINIOUS) {
		dy = (float)(Y_AXIS.TABLE_SIZE-Y_AXIS.Position)/(float)Y_AXIS.PULSES_PER_MM;
		continious = 1;	}
	if(dz==-AXIS_CONTINIOUS) {
		dz = -(float)Z_AXIS.Position/(float)Z_AXIS.PULSES_PER_MM;
		continious = 1;	}
	if(dz==AXIS_CONTINIOUS) {
		dz = (float)(Z_AXIS.TABLE_SIZE-Z_AXIS.Position)/(float)Z_AXIS.PULSES_PER_MM;
		continious = 1;	}
	// для E AXIS_CONTINIOUS не будем менять, пускай крутит на указанное значение, катушка все равно наверно короче :)
	if(continious && (de==AXIS_CONTINIOUS || de==-AXIS_CONTINIOUS)) { // а такой пируэт смысла вообще не имеет
		NVIC_EnableIRQ(TIM7_IRQn);
		return STATUS_ERROR_INVALID_PARAMETER;
	}

	if(fabsf(dx)==0.0f && fabsf(dy)==0.0f && fabsf(dz)==0.0f && fabsf(de)==0.0f) { // может быть -0.0f
		NVIC_EnableIRQ(TIM7_IRQn);
		return STATUS_OK;
	}

	s32 timeTacts = 0; // количество тактов для выполнения интерполяции
	if(speed != AXIS_MAX_SPEED && speed != AXIS_MAX_SPEED_LOW_ACCELERATION) {
		timeTacts = ceilf(sqrtf(dx*dx+dy*dy+dz*dz)/(speed/60.0f)*(float)TACTS_PER_SEC);
		if(timeTacts==0 && fabsf(de)!=0.0f)
			timeTacts = fabsf(de)/(speed/60.0f)*(float)TACTS_PER_SEC;
	}


	int xWishPulses = roundf((float)X_AXIS.PULSES_PER_MM*dx);
	int yWishPulses = roundf((float)Y_AXIS.PULSES_PER_MM*dy);
	int zWishPulses = roundf((float)Z_AXIS.PULSES_PER_MM*dz);
	int eWishPulses = roundf((float)E_AXIS.PULSES_PER_MM*de);
	if(continious) { // чтобы под заданным углом двигал, а не под углом на сколько осталось стола по какой-то оси.
		int min = X_AXIS.PULSES_PER_MM*X_AXIS.TABLE_SIZE+Y_AXIS.PULSES_PER_MM*Y_AXIS.TABLE_SIZE+Z_AXIS.PULSES_PER_MM*Z_AXIS.TABLE_SIZE; // просто одно из максимально возможных
		if(xWishPulses!=0 && fabsf(xWishPulses)<min) min = fabsf(xWishPulses);
		if(yWishPulses!=0 && fabsf(yWishPulses)<min) min = fabsf(yWishPulses);
		if(zWishPulses!=0 && fabsf(zWishPulses)<min) min = fabsf(zWishPulses);
		// одно из них не ноль, т.к. выше dx dy dz проверялись
		if(xWishPulses!=0) xWishPulses = copysignf(min,xWishPulses);
		if(yWishPulses!=0) yWishPulses = copysignf(min,yWishPulses);
		if(zWishPulses!=0) zWishPulses = copysignf(min,zWishPulses);
	}

	if(axis_check_table(X_AXIS.Position+xWishPulses, Y_AXIS.Position+yWishPulses, Z_AXIS.Position+zWishPulses)==STATUS_ERROR_OUT_OF_TABLE)
	{
		NVIC_EnableIRQ(TIM7_IRQn);
		return STATUS_ERROR_OUT_OF_TABLE;
	}

	if(fabsf(xWishPulses/X_AXIS.MAX_VELOCITY)>timeTacts) {
		if(speed != AXIS_MAX_SPEED && speed != AXIS_MAX_SPEED_LOW_ACCELERATION)
			result = STATUS_WARNING_OUT_OF_SPEED;
		timeTacts = fabsf((float)xWishPulses/X_AXIS.MAX_VELOCITY);
	}
	if(fabsf(yWishPulses/Y_AXIS.MAX_VELOCITY)>timeTacts) {
		if(speed != AXIS_MAX_SPEED && speed != AXIS_MAX_SPEED_LOW_ACCELERATION)
			result = STATUS_WARNING_OUT_OF_SPEED;
		timeTacts = fabsf((float)yWishPulses/Y_AXIS.MAX_VELOCITY);
	}
	if(fabsf(zWishPulses/Z_AXIS.MAX_VELOCITY)>timeTacts) {
		if(speed != AXIS_MAX_SPEED && speed != AXIS_MAX_SPEED_LOW_ACCELERATION)
			result = STATUS_WARNING_OUT_OF_SPEED;
		timeTacts = fabsf((float)zWishPulses/Z_AXIS.MAX_VELOCITY);
	}
	if(fabsf(eWishPulses/E_AXIS.MAX_VELOCITY)>timeTacts) {
		if(speed != AXIS_MAX_SPEED && speed != AXIS_MAX_SPEED_LOW_ACCELERATION)
			result = STATUS_WARNING_OUT_OF_SPEED;
		timeTacts = fabsf((float)eWishPulses/E_AXIS.MAX_VELOCITY);
	}

	u32 accelerationTime;
	u32 brakeTime;
	brakeTime = fabsf(xWishPulses)/(float)timeTacts/X_AXIS.MAX_ACCELERATION; // количество импульсов для останова
	float timeBrakeCoef = X_AXIS.MAX_ACCELERATION/X_AXIS.MAX_VELOCITY/2.0f;
	// для разгона
	u32 tmp;
	if( (tmp=fabsf(yWishPulses)/(float)timeTacts/Y_AXIS.MAX_ACCELERATION) > brakeTime ) {
		brakeTime = tmp;
		timeBrakeCoef = Y_AXIS.MAX_ACCELERATION/Y_AXIS.MAX_VELOCITY/2.0f;
	}
	if( (tmp=fabsf(zWishPulses)/(float)timeTacts/Z_AXIS.MAX_ACCELERATION) > brakeTime ) {
		brakeTime = tmp;
		timeBrakeCoef = Z_AXIS.MAX_ACCELERATION/Z_AXIS.MAX_VELOCITY/2.0f;
	}
	if( (tmp=fabsf(eWishPulses)/(float)timeTacts/E_AXIS.MAX_ACCELERATION) > brakeTime ) {
		brakeTime = tmp;
		timeBrakeCoef = E_AXIS.MAX_ACCELERATION/E_AXIS.MAX_VELOCITY/2.0f;
	}
	float timeAccelerationCoef;

	u32 accelerationkoef = 1;
	if(speed!=AXIS_MAX_SPEED_LOW_ACCELERATION) {
		accelerationTime = brakeTime;
		timeAccelerationCoef = timeBrakeCoef;
	} else {
		accelerationkoef = 10;
		accelerationTime = brakeTime*accelerationkoef;
		timeAccelerationCoef = timeBrakeCoef/accelerationkoef;
	}

	// если не успеем разогнаться до заданной скорости
	// т.к. при ускорении до заданной скорости проденный путь вдвое меньше, чем при прямолийненом движений с достигнeтой скоростью заодно и то же время, будем делить/умножать на 2
	u32 linearTime;
	if(accelerationTime/2+brakeTime/2>timeTacts) {
		int fbr = ceilf((float)timeTacts/(1.0f+(float)accelerationkoef));
		int facc = timeTacts - fbr;
		brakeTime = roundf(sqrt((float)fbr/timeBrakeCoef));
		accelerationTime = roundf(sqrt((float)facc/timeAccelerationCoef));
		linearTime = 0;
	} else {
		linearTime = timeTacts - (float)brakeTime*timeBrakeCoef*(float)brakeTime - (float)accelerationTime*timeAccelerationCoef*(float)accelerationTime;
	}

	// загрузка задания
	mInterpolationFunction = linearInterpolation;
	mAccelerationTime = accelerationTime;
	mBrakeTime = brakeTime;
	mInterpolationRealTime = accelerationTime + brakeTime + linearTime;
	mTimeAccelerationCoef = timeAccelerationCoef;
	mTimeBrakeCoef = timeBrakeCoef;
	X_AXIS.ValueZero = X_AXIS.Position;
	Y_AXIS.ValueZero = Y_AXIS.Position;
	Z_AXIS.ValueZero = Z_AXIS.Position;
	E_AXIS.ValueZero = E_AXIS.Position;
	if(timeTacts) {
		X_AXIS.ValueCoeficient = (float)xWishPulses/(float)timeTacts;
		Y_AXIS.ValueCoeficient = (float)yWishPulses/(float)timeTacts;
		Z_AXIS.ValueCoeficient = (float)zWishPulses/(float)timeTacts;
		E_AXIS.ValueCoeficient = (float)eWishPulses/(float)timeTacts;
	} else {
		X_AXIS.ValueCoeficient = 0;
		Y_AXIS.ValueCoeficient = 0;
		Z_AXIS.ValueCoeficient = 0;
		E_AXIS.ValueCoeficient = 0;
	}

	if(laserPower) {
		if(forceLaserTime)
			laser_force_stop();
		init_laser(laserPower, 1);
	}

	// нулим
	TIMER_COUNTER = 0;
	timeAccelerationFinished = 0;
	timeLinearFinished = 0;
	mInterpolationFunction(0);
	lastProbeFoundZ = MAXFLOAT;

	TIM7->SR &= ~((uint16_t)TIM_IT_Update);
	NVIC_EnableIRQ(TIM7_IRQn);

	return result;
}

STATUSES axis_move_to_linear(float x, float y, float z, float e, float a, float speed, float laserPower)
{
	return axis_move_linear(x-axis_getXPos(), y-axis_getYPos(), z-axis_getZPos(), e-axis_getEPos(), a-axis_getAPos(), speed, laserPower);
}

STATUSES axis_round(float dx, float dy, float dz, float de, float da, float ri, float rj, float rk, float speed, InterpolationDirection direction, InterpolationPlane plane, float laserPower)
{
	return axis_round_to(dx+axis_getXPos(), dy+axis_getYPos(), dz+axis_getZPos(), de+axis_getEPos(), da+axis_getAPos(), ri, rj, rk, speed, direction, plane, laserPower);
}

float angle10(float x, float y) {
	float res = acosf(x/sqrtf(x*x+y*y));
	if(y<0.0f)
		return 2*M_PI - res;
	return res;
}

float angle10Delta(float x, float y, float a0, InterpolationDirection direction) {
	float delta = angle10(x, y)  - a0;
	if ( delta<0.0f ||  (delta==0.0f && direction==ROUND_CCW) )
		delta += 2*M_PI;
	if(direction==ROUND_CW)
		return delta - 2*M_PI;
	return delta;
}

//float dot (double x0, double y0, double x1, double y1) {
//	const double p = x0*x1+y0*y1;
//	const double sqrt1 = sqrt( x0*x0 + y0*y0 );
//	const double sqrt2 = sqrt( x1*x1 + y1*y1 );
//	const double v = p/sqrt1/sqrt2;
//	if(v>=1.0L && v<1.00000001L) // т.к. может возникнуть результат 1.0000...01 при абсолютно нормальных входных данных
//		return 0.0L;
//	if(v<=-1.0L && v>-1.00000001L)
//		return M_PI;
//	return acos( v );
//}

STATUSES axis_round_to(float tx, float ty, float tz, float te, float ta, float ri, float rj, float rk, float speed, InterpolationDirection direction, InterpolationPlane plane, float laserPower)
{
	if(ta!=axis_getAPos()) //AAXISTODO, еще назабыть проверить хардварное ограниечение - невозможность использовать А и Е одновременно
		return STATUS_UNIMPLEMENTED;
	STATUSES result = axis_status();
	if(result!=STATUS_OK)
		return result;
	if( (fabsf(ri)==0.0f && fabsf(rj)==0.0f && fabsf(rk)==0.0f) || speed==AXIS_MAX_SPEED_LOW_ACCELERATION || speed<AXIS_MIN_SPEED)
		return STATUS_ERROR_INVALID_PARAMETER;
	result = init_laser(laserPower, 0);
	if(result!=STATUS_OK)
		return result;

	NVIC_DisableIRQ(TIM7_IRQn);
	if(te!=axis_getEPos() && heater_extruder_ready()!=STATUS_OK) {
		NVIC_EnableIRQ(TIM7_IRQn);
		return STATUS_ERROR_EXTRUDER_NOT_HEATED;
	}

	// исходные данные, заодно переводим в импульсы в нативную системе координат
	int x0 = X_AXIS.Position;
	int y0 = Y_AXIS.Position;
	int z0 = Z_AXIS.Position;
	int x1 = roundf( tx * (float)X_AXIS.INVERTED*(float)X_AXIS.PULSES_PER_MM);
	if(X_AXIS.SENSOR_POSITION<0) x1 = X_AXIS.TABLE_SIZE - x1;
	int y1 = roundf( ty * (float)Y_AXIS.INVERTED*(float)Y_AXIS.PULSES_PER_MM);
	if(Y_AXIS.SENSOR_POSITION<0) y1 = Y_AXIS.TABLE_SIZE - y1;
	int z1 = roundf( tz * (float)Z_AXIS.INVERTED*(float)Z_AXIS.PULSES_PER_MM);
	if(Z_AXIS.SENSOR_POSITION<0) z1 =Z_AXIS.TABLE_SIZE - z1;
	if(axis_check_table(x1, y1, z1)) {
		NVIC_EnableIRQ(TIM7_IRQn);
		return STATUS_ERROR_OUT_OF_TABLE;
	}
	double rip = roundf( ri * (float)X_AXIS.SENSOR_POSITION * (float)X_AXIS.INVERTED*(float)X_AXIS.PULSES_PER_MM);
	double rjp = roundf( rj * (float)Y_AXIS.SENSOR_POSITION * (float)Y_AXIS.INVERTED*(float)Y_AXIS.PULSES_PER_MM);
	double rkp = roundf( rk * (float)Z_AXIS.SENSOR_POSITION * (float)Z_AXIS.INVERTED*(float)Z_AXIS.PULSES_PER_MM);
	int cx = x0 + rip;
	int cy = y0 + rjp;
	int cz = z0 + rkp;
	// вычисляем параметры интерполяций
	float radius = 0.0f;
	float alphastart = 0.0f;
	float alphadelta = 0.0f;
	double linearpath = 0.0L;
	float linearpathMM = 0.0f;
	float radiusMM = 0.0f;
	_AXIS *axisRound1 = 0;
	_AXIS *axisRound2 = 0;
	_AXIS *axisLinear = 0;
	switch(plane) {
	case PLANE_XY:
	{
		radius = sqrtf(rip*rip+rjp*rjp);
		radiusMM = sqrtf(rip/(double)X_AXIS.PULSES_PER_MM*rip/(double)X_AXIS.PULSES_PER_MM+rjp/(double)Y_AXIS.PULSES_PER_MM*rjp/(double)Y_AXIS.PULSES_PER_MM);
		alphastart = angle10(-ri, -rj);
		float dx = ((int)((tx-axis_getXPos())*(float)X_AXIS.PULSES_PER_MM))/(float)X_AXIS.PULSES_PER_MM;
		float dy = ((int)((ty-axis_getYPos())*(float)Y_AXIS.PULSES_PER_MM))/(float)Y_AXIS.PULSES_PER_MM;
		alphadelta = angle10Delta(dx - ri, dy - rj, alphastart, direction);
		linearpath = z1-z0;
		linearpathMM = linearpath/(float)Z_AXIS.PULSES_PER_MM;
		axisRound1 = &X_AXIS;
		axisRound2 = &Y_AXIS;
		axisLinear = &Z_AXIS;
		X_AXIS.ValueZero = cx;
		Y_AXIS.ValueZero = cy;
		Z_AXIS.ValueZero = Z_AXIS.Position;
	}
		break;
	case PLANE_YZ:
	{
		radius = sqrtf(rjp*rjp+rkp*rkp);
		radiusMM = sqrtf(rjp/(double)Y_AXIS.PULSES_PER_MM*rjp/(double)Y_AXIS.PULSES_PER_MM+rkp/(double)Z_AXIS.PULSES_PER_MM*rkp/(double)Z_AXIS.PULSES_PER_MM);
		alphastart = angle10(-rj, -rk);
		float dy = ((int)((ty-axis_getYPos())*(float)Y_AXIS.PULSES_PER_MM))/(float)Y_AXIS.PULSES_PER_MM;
		float dz = ((int)((tz-axis_getZPos())*(float)Z_AXIS.PULSES_PER_MM))/(float)Z_AXIS.PULSES_PER_MM;
		alphadelta = angle10Delta(dy - rj, dz - rk, alphastart, direction);
		linearpath = x1-x0;
		linearpathMM = linearpath/(float)X_AXIS.PULSES_PER_MM;
		axisRound1 = &Y_AXIS;
		axisRound2 = &Z_AXIS;
		axisLinear = &X_AXIS;
		X_AXIS.ValueZero = X_AXIS.Position;
		Y_AXIS.ValueZero = cy;
		Z_AXIS.ValueZero = cz;
	}
		break;
	case PLANE_ZX:
	{
		radius = sqrtf(rkp*rkp+rip*rip);
		radiusMM = sqrtf(rkp/(double)Z_AXIS.PULSES_PER_MM*rkp/(double)Z_AXIS.PULSES_PER_MM+rip/(double)X_AXIS.PULSES_PER_MM*rip/(double)X_AXIS.PULSES_PER_MM);
		alphastart = angle10(-rk, -ri);
		float dz = ((int)((tz-axis_getZPos())*(float)Z_AXIS.PULSES_PER_MM))/(float)Z_AXIS.PULSES_PER_MM;
		float dx = ((int)((tx-axis_getXPos())*(float)X_AXIS.PULSES_PER_MM))/(float)X_AXIS.PULSES_PER_MM;
		alphadelta = angle10Delta(dz - rk, dx - ri, alphastart, direction);
		linearpath = y1-y0;
		linearpathMM = linearpath/(float)Y_AXIS.PULSES_PER_MM;
		axisRound1 = &Z_AXIS;
		axisRound2 = &X_AXIS;
		axisLinear = &Y_AXIS;
		X_AXIS.ValueZero = cx;
		Y_AXIS.ValueZero = Y_AXIS.Position;
		Z_AXIS.ValueZero = cz;
	}
		break;
	}
	// проверяем углы
	if(radius == 0.0f || isnan(alphastart) || isnan(alphadelta) || alphadelta==0.0L || alphadelta==-0.0L) {
		NVIC_EnableIRQ(TIM7_IRQn);
		return STATUS_ERROR_INVALID_PARAMETER;
	}

	// загружаем интерполяцию
	roundPlane =  plane;
	xRoundRadius = radius * X_AXIS.SENSOR_POSITION * X_AXIS.INVERTED;
	yRoundRadius = radius * Y_AXIS.SENSOR_POSITION * Y_AXIS.INVERTED;
	zRoundRadius = radius * Z_AXIS.SENSOR_POSITION * Z_AXIS.INVERTED;
	roundLinearCoeficient = 0.0f;
	// с помощью самой интерполяции проверяем крйние точки траекторий
	int l, p = ceilf(fabsf(alphadelta)*2.0f/M_PI);
	float testAngleValue = ceilf(alphastart*2.0f/M_PI)*M_PI/2.0f + copysignf(M_PI/2.0f, alphadelta);
	for(l=1; l<=p; l++) {
		if(l==p)
			nativeroundInterpolation(0.0f, alphastart+alphadelta);
		else
			nativeroundInterpolation(0.0f, testAngleValue);
		if(axis_check_table(X_AXIS.ValueInterpolation, Y_AXIS.ValueInterpolation, Z_AXIS.ValueInterpolation)) {
			NVIC_EnableIRQ(TIM7_IRQn);
			return STATUS_ERROR_OUT_OF_TABLE;
		}
		testAngleValue += copysignf(M_PI/2.0f, alphadelta);
	}

	int epath = (float)E_AXIS.INVERTED * roundf( te * (float)E_AXIS.PULSES_PER_MM - E_AXIS.Position);

	// проверяем ускорение при движение по оркужности и корректируем скорость
	// и считаем время для ускорения
	double planepathMM = radiusMM*fabsf(alphadelta);
	u32 timeTacts = 0;
	if(speed!=AXIS_MAX_SPEED)
		timeTacts = ceilf(sqrt(planepathMM*planepathMM+linearpathMM*linearpathMM)/(speed/60.0f)*(float)TACTS_PER_SEC);
	float plainSpeedMMS = planepathMM/(float)timeTacts*(float)TACTS_PER_SEC;
	u32 accelerationTime = 0; // количество импульсов для останова
	double timeCoef = 0;
	double tmp;
	if( (tmp =sqrt(axisRound1->MAX_ACCELERATION*M_PI*radius/2.0f)*axisRound1->PULSES_PER_MM/(float)TACTS_PER_SEC) > plainSpeedMMS)  {
		if(speed != AXIS_MAX_SPEED)
			result = STATUS_WARNING_OUT_OF_SPEED;
		plainSpeedMMS = tmp;
		timeTacts = ceilf(planepathMM/plainSpeedMMS*(float)TACTS_PER_SEC);
	}
	if( (tmp =sqrt(axisRound2->MAX_ACCELERATION*M_PI*radius/2.0f)*axisRound2->PULSES_PER_MM/(float)TACTS_PER_SEC) > plainSpeedMMS)  {
		if(speed != AXIS_MAX_SPEED)
			result = STATUS_WARNING_OUT_OF_SPEED;
		plainSpeedMMS = tmp;
		timeTacts = ceilf(planepathMM/plainSpeedMMS*(float)TACTS_PER_SEC);
	}

	if( (tmp = round(fabsf(linearpathMM)*(float)axisLinear->PULSES_PER_MM/(float)axisLinear->MAX_VELOCITY)) > timeTacts ) {
		if(speed != AXIS_MAX_SPEED)
			result = STATUS_WARNING_OUT_OF_SPEED;
		timeTacts = ceilf(tmp);
	}
	if( (tmp = round(fabsf(epath)/(float)E_AXIS.MAX_VELOCITY)) > timeTacts ) {
		if(speed != AXIS_MAX_SPEED)
			result = STATUS_WARNING_OUT_OF_SPEED;
		timeTacts = ceilf(tmp);
	}

	if( (tmp =axisRound1->MAX_VELOCITY/axisRound1->PULSES_PER_MM*(float)TACTS_PER_SEC) < plainSpeedMMS) {
		if(speed != AXIS_MAX_SPEED)
			result = STATUS_WARNING_OUT_OF_SPEED;
		plainSpeedMMS = tmp;
		timeTacts = ceilf(planepathMM/plainSpeedMMS*(float)TACTS_PER_SEC);
	}
	if( (tmp =axisRound2->MAX_VELOCITY/axisRound2->PULSES_PER_MM*(float)TACTS_PER_SEC) < plainSpeedMMS) {
		if(speed != AXIS_MAX_SPEED)
			result = STATUS_WARNING_OUT_OF_SPEED;
		plainSpeedMMS = tmp;
		timeTacts = ceilf(planepathMM/plainSpeedMMS*(float)TACTS_PER_SEC);
	}

	if( (tmp = round(plainSpeedMMS*(float)axisRound1->PULSES_PER_MM/(float)TACTS_PER_SEC/axisRound1->MAX_ACCELERATION)) > accelerationTime ) {
		accelerationTime = tmp;
		timeCoef = axisRound1->MAX_ACCELERATION/axisRound1->MAX_VELOCITY/2.0f;
	}
	if( (tmp = round(plainSpeedMMS*(float)axisRound2->PULSES_PER_MM/(float)TACTS_PER_SEC/axisRound2->MAX_ACCELERATION)) > accelerationTime ) {
		accelerationTime = tmp;
		timeCoef = axisRound2->MAX_ACCELERATION/axisRound2->MAX_VELOCITY/2.0f;
	}
	if( (tmp = linearpath/(float)timeTacts/axisLinear->MAX_ACCELERATION) > accelerationTime ) {
		accelerationTime = tmp;
		timeCoef = axisLinear->MAX_ACCELERATION/axisLinear->MAX_VELOCITY/2.0f;
	}
	if( (tmp = epath/(float)timeTacts/E_AXIS.MAX_ACCELERATION) > accelerationTime ) {
		accelerationTime = tmp;
		timeCoef = E_AXIS.MAX_ACCELERATION/E_AXIS.MAX_VELOCITY/2.0f;
	}

	// проверяем успеем ли разогнать
	u32 linearTime;
	if(accelerationTime>timeTacts) {
		double fbr = (double)timeTacts/2.0f;
		accelerationTime = round(sqrt((double)fbr/timeCoef));
		linearTime = 0;
	} else {
		linearTime = timeTacts - 2.0f*round((double)accelerationTime*timeCoef*(double)accelerationTime); //  /2.0f*2.0f;
	}

	// загрузка задания
	mInterpolationFunction = roundInterpolation;
	mAccelerationTime = accelerationTime;
	mBrakeTime = accelerationTime;
	mInterpolationRealTime = accelerationTime*2 + linearTime;
	mTimeAccelerationCoef = timeCoef;
	mTimeBrakeCoef = timeCoef;

	roundAlphaMin = alphastart;
	roundAlphaCoef = alphadelta/(float)timeTacts;
	roundLinearCoeficient = linearpath/(float)timeTacts;

	E_AXIS.ValueZero = E_AXIS.Position;
	E_AXIS.ValueCoeficient = (float)epath/(float)timeTacts;

	roundFinishPosX = tx;
	roundFinishPosY = ty;
	roundFinishPosZ = tz;
	roundFinishPosE = te;
	roundFinishPosA = ta;
	roundFinishSpeed = speed;
	roundFinishLaserPower = laserPower;

	if(laserPower) {
		if(forceLaserTime)
			laser_force_stop();
		init_laser(laserPower, 1);
	}

	// нулим
	TIMER_COUNTER = 0;
	timeAccelerationFinished = 0;
	timeLinearFinished = 0;
	mInterpolationFunction(0);
	lastProbeFoundZ = MAXFLOAT;

	TIM7->SR &= ~((uint16_t)TIM_IT_Update);
	NVIC_EnableIRQ(TIM7_IRQn);

	return result;
}

STATUSES axis_park()
{
	float zpos = (Z_AXIS.SENSOR_POSITION==-1)?(float)AXIS_Z_TABLE_SIZE_MM:0.0f;
	STATUSES res;
	if(axis_getZPos()==zpos) {
		res = park_all();
	} else {
		mParkInProgress = 1;
		res = axis_move_to_linear(axis_getXPos(), axis_getYPos(), zpos, axis_getEPos(), axis_getAPos(), AXIS_MAX_SPEED, 0.0f);
		if(res!=STATUS_OK)
			mParkInProgress = 0;
	}
	return res;
}

STATUSES axis_pause(int ms, float laserPower)
{
	STATUSES result = axis_status();
	if(result!=STATUS_OK)
		return result;
	if(ms<0)
		return STATUS_ERROR_INVALID_PARAMETER;
	if(ms==0)
		return STATUS_OK;
	result = init_laser(laserPower, 0);
	if(result!=STATUS_OK)
		return result;
	NVIC_DisableIRQ(TIM7_IRQn);
	X_AXIS.ValueInterpolation = INTERPOLATION_DONE;
	Y_AXIS.ValueInterpolation = INTERPOLATION_DONE;
	Z_AXIS.ValueInterpolation = INTERPOLATION_DONE;
	E_AXIS.ValueInterpolation = INTERPOLATION_DONE;

	mAccelerationTime = 0;
	mBrakeTime = 0;
	mInterpolationRealTime = TACTS_PER_SEC*(float)ms/1000.0f;
	mInterpolationFunction = pauseInterpolation;

	if(laserPower) {
		if(forceLaserTime)
			laser_force_stop();
		init_laser(laserPower, 1);
	}

	// нулим
	TIMER_COUNTER = 0;

	TIM7->SR &= ~((uint16_t)TIM_IT_Update);
	NVIC_EnableIRQ(TIM7_IRQn);

	return STATUS_OK;
}

void axis_stop()
{
	if(mInterpolationFunction!=0) {
		if(forceLaserTime == 0) {
			runningLaserPower = 0;
			TIM_CtrlPWMOutputs(TIM1, DISABLE);
			TIM_Cmd(TIM1, DISABLE);
		}
		if(mInterpolationFunction==pauseInterpolation){
			mInterpolationFunction = 0;
			return;
		}
		if(mInterpolationFunction==probeInterpolation){
			mInterpolationFunction = 0;
			spindle_stop();
			return;
		}
		if(TIMER_COUNTER>mInterpolationRealTime-mBrakeTime) { // если уже замедляем
			return;
		} else {
			NVIC_DisableIRQ(TIM7_IRQn);
			float accDone = (float)TIMER_COUNTER/(float)mAccelerationTime;
			if(accDone>1.0f) accDone = 1.0f;
			mBrakeTime = accDone*(float)mBrakeTime;
			mAccelerationTime = 0; // помимо основного назначения(заставить заходить в функцию торможения), обязательно для круговой интерполяции, чтобы дать знать что доволдить не надо
			mInterpolationRealTime = TIMER_COUNTER+mBrakeTime;
			NVIC_EnableIRQ(TIM7_IRQn);
		}
	}
}

STATUSES axis_status()
{
	if(mInterpolationFunction!=0 || mToolChangingInProgress!=0 || mPauseInProgress!=0 || heaters_wait_status()!=HW_NONE)
		return STATUS_BUSY;
	return STATUS_OK;
}


STATUSES axis_isparked()
{
	if( (AXIS_PORT_SENSE->IDR & (X_AXIS.SENSE_PIN|Y_AXIS.SENSE_PIN|Z_AXIS.SENSE_PIN))==0 && X_AXIS.Position==0 && Y_AXIS.Position==0 && Z_AXIS.Position==0)
		return STATUS_OK;
	else
		return STATUS_NOT_ZERO;
}

STATUSES axis_movefromsensors()
{
	return axis_move_linear(2.0f, -2.0f, -2.0f, 0.0f, 0.0f, AXIS_MAX_SPEED, 0.0f);
}

STATUSES axis_foundzero()
{
	int max = X_AXIS.TABLE_SIZE*2; // чтобы двигатели шли с одинаковой скоростью при линейной интерполяции
	if(Y_AXIS.TABLE_SIZE*2>max) max = Y_AXIS.TABLE_SIZE*2;
	if(Z_AXIS.TABLE_SIZE*2>max) max = Z_AXIS.TABLE_SIZE*2;
	X_AXIS.Position = max;
	Y_AXIS.Position = max;
	Z_AXIS.Position = max;
	mCalibarationInProgress = 1;
	axis_move_linear(-AXIS_CONTINIOUS*X_AXIS.SENSOR_POSITION*X_AXIS.INVERTED, -AXIS_CONTINIOUS*Y_AXIS.SENSOR_POSITION*Y_AXIS.INVERTED, -AXIS_CONTINIOUS*Z_AXIS.SENSOR_POSITION*Z_AXIS.INVERTED, 0.0f, 0.0f, AXIS_CALLIBRATION_SPEED, 0.0f);
	while ( axis_isparked()==STATUS_NOT_ZERO) {
		if(mInterpolationFunction==0) {
			mCalibarationInProgress = 0;
			mInterpolationFunction = 0;
			return STATUS_ERROR_OUT_OF_TABLE;
		}
	}
	mInterpolationFunction = 0;
	mCalibarationInProgress = 0;
	return STATUS_OK;
}

float axis_getXPos(){
	if(X_AXIS.SENSOR_POSITION==-1)
		return (float)((X_AXIS.TABLE_SIZE-X_AXIS.Position)*X_AXIS.INVERTED)/(float)X_AXIS.PULSES_PER_MM;
	else
		return (float)(X_AXIS.Position*X_AXIS.INVERTED)/(float)X_AXIS.PULSES_PER_MM;
}

float axis_getYPos(){
	if(Y_AXIS.SENSOR_POSITION==-1)
		return (float)((Y_AXIS.TABLE_SIZE-Y_AXIS.Position)*Y_AXIS.INVERTED)/(float)Y_AXIS.PULSES_PER_MM;
	else
		return (float)(Y_AXIS.Position*Y_AXIS.INVERTED)/(float)Y_AXIS.PULSES_PER_MM;
}

float axis_getZPos(){
	if(Z_AXIS.SENSOR_POSITION==-1)
		return (float)((Z_AXIS.TABLE_SIZE-Z_AXIS.Position)*Z_AXIS.INVERTED)/(float)Z_AXIS.PULSES_PER_MM;
	else
		return (float)(Z_AXIS.Position*Z_AXIS.INVERTED)/(float)Z_AXIS.PULSES_PER_MM;
}

float axis_getEPos(){
	return (float)(E_AXIS.Position*E_AXIS.INVERTED)/(float)E_AXIS.PULSES_PER_MM;
}

float axis_getAPos(){
	return (float)(A_AXIS.Position*A_AXIS.INVERTED)/(float)A_AXIS.PULSES_PER_MM;
}

void axis_setEPos(float e){
	E_AXIS.ValueInterpolation = INTERPOLATION_DONE;
	E_AXIS.Position = roundf(e*(float)E_AXIS.PULSES_PER_MM*(float)E_AXIS.INVERTED);
}

void axis_setAPos(float a){
	// AAXISTODO - при обращение с гуев надо переключаться на эту ось, чтобы драйвер включил ток удержания для оси
	// а в гуях надо установку нуля прям влоб сюда звать
	A_AXIS.ValueInterpolation = INTERPOLATION_DONE;
	A_AXIS.Position = roundf(a*(float)A_AXIS.PULSES_PER_MM*(float)A_AXIS.INVERTED);
}

STATUSES axis_changeTool() {
	mToolChangeOldZPos = axis_getZPos();
	STATUSES res = axis_move_linear(0.0f, 0.0f, -mToolChangeOldZPos, 0.0f, 0.0f, AXIS_MAX_SPEED, 0.0f);
	if(res==STATUS_OK) {
		spindle_stop();
		mToolChangingInProgress = 1;
	}
	return res;
}

STATUSES axis_toolChanged() {
	if(mInterpolationFunction==0) {
		if(fabsf(axis_getZPos() - mToolChangeOldZPos)>0.0f)
			axis_zprobe(mToolChangeOldZPos);
		mToolChangingInProgress = 0;
		return STATUS_OK;
	}
	return STATUS_BUSY;
}

STATUSES axis_stopPause() {
	mPauseInProgress = 1;
	return STATUS_OK;
}
STATUSES axis_stopResume() {
	mPauseInProgress = 0;
	return STATUS_OK;
}

void axis_estop() {
	mToolChangingInProgress = 0;
	mParkInProgress = 0;
	mPauseInProgress = 0;
	laser_force_stop();
	axis_stop();
	spindle_stop();
	heater_extruder_heat_off();
	heater_table_heat_off();
}

STATUSES axis_zprobe_to(float z) {
		return axis_zprobe(z-axis_getZPos());
}

STATUSES axis_zprobe(float dz) {
	if(fabsf(dz)==0.0f)
		return STATUS_OK;
	STATUSES result = axis_status();
	if(mToolChangingInProgress==1 && mInterpolationFunction==0)
		result = STATUS_OK;
	if(result!=STATUS_OK)
		return result;

	NVIC_DisableIRQ(TIM7_IRQn);
	dz *= Z_AXIS.SENSOR_POSITION * Z_AXIS.INVERTED;
	if(dz==-AXIS_CONTINIOUS)
		dz = -(float)Z_AXIS.Position/(float)Z_AXIS.PULSES_PER_MM;
	if(dz==AXIS_CONTINIOUS)
		dz = (float)(Z_AXIS.TABLE_SIZE-Z_AXIS.Position)/(float)Z_AXIS.PULSES_PER_MM;
	int zWishPulses = roundf((float)Z_AXIS.PULSES_PER_MM*dz);
	if(axis_check_table(X_AXIS.Position, Y_AXIS.Position, Z_AXIS.Position+zWishPulses)==STATUS_ERROR_OUT_OF_TABLE)
	{
		NVIC_EnableIRQ(TIM7_IRQn);
		return STATUS_ERROR_OUT_OF_TABLE;
	}
	if(lastProbeFoundZ!=MAXFLOAT) {
		if(lastProbeFoundZ*(float)zWishPulses>0.0f) {// если в туже сторону хотим снова попровать - говорим ок.
			NVIC_EnableIRQ(TIM7_IRQn);
			return STATUS_OK;
		}
	}
	spindle_stop();


	X_AXIS.ValueInterpolation = INTERPOLATION_DONE;
	Y_AXIS.ValueInterpolation = INTERPOLATION_DONE;
	Z_AXIS.ValueInterpolation = INTERPOLATION_DONE;
	E_AXIS.ValueInterpolation = INTERPOLATION_DONE;

	Z_AXIS.ValueZero = Z_AXIS.Position;
	Z_AXIS.ValueCoeficient = copysignf(Z_AXIS.MAX_VELOCITY/AXIS_ZPROBE_SPEED_DIV*Z_AXIS.SENSOR_POSITION * Z_AXIS.INVERTED, (float)zWishPulses);

	mInterpolationRealTime = SPINDLE_PROBE_WAIT_TIME + zWishPulses/Z_AXIS.ValueCoeficient; // максимальное время после которого отключаем поиск
	mInterpolationFunction = probeInterpolation;

	private_spindle_result = 0; // чтобы фильтр обнулить

	// нулим
	TIMER_COUNTER = 0;
	TIM7->SR &= ~((uint16_t)TIM_IT_Update);
	NVIC_EnableIRQ(TIM7_IRQn);

	return STATUS_OK;
}

int laser_isWorking() {
	return runningLaserPower;
}

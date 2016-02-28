// http://roboforum.ru/wiki/%D0%9F%D0%B5%D1%80%D0%B5%D0%B2%D0%BE%D0%B4_%D1%81%D1%82%D0%B0%D1%82%D1%8C%D0%B8_%22%D0%9F%D1%80%D0%BE%D1%81%D1%82%D0%BE_%D0%BE_%D0%9F%D0%98%D0%94-%D0%B0%D0%BB%D0%B3%D0%BE%D1%80%D0%B8%D1%82%D0%BC%D0%B0%D1%85%22

#include "pid.h"
#define MAX_PINT 2147483647
// TODO make them const
volatile float OUT_FILTER_WEIGHT = 0.4f;
volatile float D_FILTER_WEIGHT = 0.4f;
volatile float P_GAIN_KOEFICIENT = 1.5f;
volatile float I_TIME_KOEFICIENT = 0.5f;
volatile float D_TIME_KOEFICIENT = 0.125f;
volatile float I_ME_KOEFICIENT = 1.0f;

int PID_UpdatePID(PID * pid, float error, float value)
{
	float pTerm, dTerm, iTerm, res;

	if((error>=0 &&error<pid->lockValue) || (error<0 && -error<pid->lockValue)) {
		pid->lock = 1;
		if(pid->lockTime<MAX_PINT)
			pid->lockTime++;
	} else {
		pid->lockTime = 0;
	}

	pTerm = error;    // расчет порциального

	if((error>=0 && error<pid->iMaxError) || (error<0 && -error<pid->iMaxError)) { // расчитываем интегральное значение и огрнаничиваем его
		pid->iState += error;
		if (pid->iState > pid->iMax)
			pid->iState = pid->iMax;
		else if (pid->iState < pid->iMin)
			pid->iState = pid->iMin;
		iTerm = pid->iState/pid->iTime;
	} else {
		pid->iState = 0.0f;
		iTerm = 0.0f;
	}

	pid->dFilter = pid->dFilter*(1.0f-D_FILTER_WEIGHT)+(pid->lastValue - value)*D_FILTER_WEIGHT; // расчитываем диффренциальное значение
	dTerm = pid->dTime * pid->dFilter;
	pid->lastValue = value;

	res = pid->pGain * (pTerm + iTerm + dTerm)* pid->resultScale;
	pid->outFilter = (float)pid->outFilter*(1.0f-OUT_FILTER_WEIGHT)+res*OUT_FILTER_WEIGHT;
	if(pid->outFilter >pid->maxResultValue)
		pid->outFilter = pid->maxResultValue;
	if(pid->outFilter<pid->minResultValue)
		pid->outFilter = pid->minResultValue;
	return pid->outFilter;
}


void PID_PreparePID(PID* pid, float currentValue, float targetValue)
{
	pid->lastValue = currentValue;
	pid->lock = 0;
	pid->lockTime = 0;

	float time = pid->refTime - pid->refTimeKoef*(pid->refValue-targetValue);
	float amplitude =  pid->refAmplitude - pid->refAmplitudeKoef*(pid->refValue-targetValue);
	pid->iMaxError = I_ME_KOEFICIENT*amplitude;
	pid->pGain = P_GAIN_KOEFICIENT/amplitude;
	pid->iTime = I_TIME_KOEFICIENT*time;
	pid->dTime = D_TIME_KOEFICIENT*time;
	pid->iMax =  pid->iTime/pid->pGain;
	pid->iMin = -pid->iMax;
}

void PID_ResetPID(PID* pid)
{
	pid->iState = 0;
	pid->dFilter = 0.0f;
	pid->outFilter = 0;
}

void PID_Init(PID * pid, int minValue, int maxValue, int lockValue)
{
	PID_ResetPID(pid);
	pid->maxResultValue = maxValue;
	pid->minResultValue = minValue;
	pid->resultScale = (-minValue>maxValue)?(-minValue):maxValue;
	pid->lockValue = lockValue;
}

void PID_CalibrationData(PID* pid, float refValue, float value, float amplitude, float time)
{
	if(refValue==value) {
		pid->refValue = refValue;
		pid->refAmplitude = amplitude;
		pid->refTime = time;
	} else {
		pid->refAmplitudeKoef = (pid->refAmplitude - amplitude)/(pid->refValue-value);
		pid->refTimeKoef = (pid->refTime - time)/(pid->refValue-value);
	}
}

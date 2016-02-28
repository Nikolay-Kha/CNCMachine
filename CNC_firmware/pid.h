#ifndef __PID_H_
#define __PID_H_

typedef struct {
	float pGain;        // пропорциональный коэфициент
	float iTime;        // время интегрирования
	float dTime;         // время диффренцирования
	float iMax;    // минимум
	float iMin;// и максимум для интегрального значения
	float iMaxError;

	float refValue;
	float refAmplitude;
	float refTime;
	float refAmplitudeKoef;
	float refTimeKoef;

	int resultScale; // на что домножать результат
	int minResultValue;	// ограничение для выходного значения с минимума
	int maxResultValue; // ограничение для выходного значения с максимума
	int lockValue; // при какой рахнице темеперату включать интегральную часть

	float lastValue;                  // последнее значение
	float iState;                  // интегральное значение
	int lock; 				// чтобы не набирать интегральную составляющую во время разогрева
	int lockTime;
	float dFilter;
	int outFilter;
} PID;

void PID_Init(PID * pid, int minValue, int maxValue, int lockValue);
int PID_UpdatePID(PID * pid, float error, float value);
void PID_PreparePID(PID* pid, float currentValue, float targetValue);
void PID_ResetPID(PID* pid);
void PID_CalibrationData(PID* pid, float refValue, float value, float amplitude, float time);

#endif // __PID_H_

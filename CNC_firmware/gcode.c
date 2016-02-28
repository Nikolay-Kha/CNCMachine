#include "gcode.h"
#include "serialport.h"
#include "mcore.h"
#include "cncprotocol.h"
#include "cncgui.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "beeper.h"
#include "statuses.h"
#include "usbhost.h"

#define G_MAX_COMMANDS_IN_LINE 20
float gReadX, gReadY, gReadZ ,gReadP, gReadF, gReadS, gReadA, gReadD, gReadL, gReadI, gReadJ, gReadK, gReadE, gReadC;
InterpolationPlane gPlane;
int gReadG[G_MAX_COMMANDS_IN_LINE], gGpointer=0;
#define M_COMMANDS_OFFSET 100
#define G_NO_COMMAND -1
#define G_BAD_COMMAND -2
#define G_NOT_SPECIFED -99999.0f
int gLastG03 = G_NO_COMMAND;
int statusCounter=0;
int storedIndex = 0;


float gDeltaX[6], gDeltaY[6], gDeltaZ[6], gDeltaE[6], gDeltaA[6];
char gPendingFlag = 0;
MOVEPAT mMovePat = MOVE_ABSOLUTE;
MOVEPAT mEMovePat = MOVE_RELATIVE;
UNITS mUnit = UNITS_METRIC;
char mEMovePatSet = 0;
int gEstopOccured = 0;

char m06PauseTextBuff[255];
LINEFROM gLineFrom = LINEFROM_NONE;
int gErrorWhileProcessingDetected = 0;

char* gLastCommandPtr;
#define RESPONSE_BUF_SIZE 254
char gResponse[RESPONSE_BUF_SIZE+2] = "\0";
int gResponsePointer = 0;
int gIsStoreC = 0;
#define M_CSTORE_COMMAND 20

void serial_port_recived(char *string)
{
	gParseLine(string, LINEFROM_USB, 0);
}

void sendstring(char *str, LINEFROM lineFrom, int index) {
	if(lineFrom==LINEFROM_USB)
		serialposr_sendstring_sync(str);
	else if(lineFrom==LINEFROM_KEYBOARD)
		uh_response(str, index);
	else if(lineFrom==LINEFROM_MASSSTORAGE) {
		while(*str && gResponsePointer<RESPONSE_BUF_SIZE)
			gResponse[gResponsePointer++]=*str++;
		if(gResponsePointer<RESPONSE_BUF_SIZE)
		gResponse[gResponsePointer++] = 0;
	}
}

void sendlineend(LINEFROM lineFrom, int index) {
	if(lineFrom==LINEFROM_USB)
		serialposr_sendstring_sync("\n");
	else if(lineFrom==LINEFROM_KEYBOARD)
		uh_response("\n", index);
	else if(lineFrom==LINEFROM_MASSSTORAGE) {
		if(gErrorWhileProcessingDetected) {
			CNC_GUI_MSerrorScreen(gLastCommandPtr, gResponse);
			uh_MSNextLineBreak();
		} else {
			uh_MSNextLine();
		}

	}
}

void sendlinewithend(char *str, LINEFROM lineFrom, int index) {
	sendstring(str, lineFrom, index);
	sendlineend(lineFrom, index);
}

void gDefArguments()
{
    int tmp;
    for(tmp=0; tmp<G_MAX_COMMANDS_IN_LINE; tmp++) {
        gReadG[tmp]=G_NO_COMMAND;
    }
    gGpointer=0;

	gReadX = G_NOT_SPECIFED;
	gReadY = G_NOT_SPECIFED;
	gReadZ = G_NOT_SPECIFED;
	gReadP = G_NOT_SPECIFED;
	gReadE = G_NOT_SPECIFED;
	gReadA = G_NOT_SPECIFED;
	if(gIsStoreC==0)
		gReadC = 0.0f;
}

void greset() {
	gDefArguments();
	gLastG03 = G_NO_COMMAND;
	mMovePat = MOVE_ABSOLUTE;
	mEMovePat = MOVE_RELATIVE;
	gReadF = 300.0f;
	gReadS = 3000.0f;
	gReadD = 0.0f;
	gReadL = 0.0f;
	gReadI = 0.0f;
	gReadJ = 0.0f;
	gReadK = 0.0f;
	gReadC = 0.0f;
	gIsStoreC = 0;
	int i;
	for(i=0; i<6; i++) {
		gDeltaX[i] = 0.0f;
		gDeltaY[i] = 0.0f;
		gDeltaZ[i] = 0.0f;
		gDeltaE[i] = 0.0f;
		gDeltaA[i] = 0.0f;
	}
	mEMovePatSet = 0;
	axis_setEPos(0.0f);
	axis_setAPos(0.0f);
	raxis_set_relative_zero_2(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	gPlane = PLANE_XY;
}

void gResetTimer() {
	TIM2->CNT = 0;
	TIM2->CR1 |= TIM_CR1_CEN|TIM_CR1_OPM;
}

void gInit()
{
	greset();
	serialport_init();

	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseInitTypeDef base_timer;
	TIM_TimeBaseStructInit(&base_timer);
	base_timer.TIM_Period = 9999; // 5 sec timeout
	base_timer.TIM_Prescaler =  5*(clocks.SYSCLK_Frequency/(base_timer.TIM_Period+1)) - 1;
	TIM_TimeBaseInit(TIM2, &base_timer);
	TIM_Cmd(TIM2, DISABLE);

}

char *readArgi(char *ptr, int * result){
    int res = 0;
    int sign = 1;
    char found = 0;
    while(1) {
        if(*ptr == '+'){
            sign = 1;
        } else if (*ptr == '-') {
            sign = -1;
        } else if(*ptr>=0x30 && *ptr<=0x39) {
            unsigned char v = *ptr-0x30;
            res*=10;
            res+=v;
            found = 1;
        } else {
        	if(found)
        		*result = res*sign;
            return ptr;
        }
        ptr++;
    }
}

char *readArgf(char *ptr, float * result, CONVERTUNIT cu){
	double res = 0.0L;
    double sign = 1.0L;
    double fract = 1.0L;
    char found = 0;
    while(1) {
        if(*ptr == '+'){
            sign = 1.0L;
        } else if (*ptr == '-') {
            sign = -1.0L;
        } else if(*ptr == '.' || *ptr ==',') {
            fract = 0.1L;
        } else if(*ptr>=0x30 && *ptr<=0x39) {
            unsigned char v = *ptr-0x30;
            if(fract==1.0L) {
                res*=10.0L;
                res+=v;
            } else {
                res+=(double)v*fract;
                fract = fract/10.0L;
            }
            found = 1;
        } else {
        	if(found) {
        		*result = (double)res*sign;
        		if(cu==CU_INC_TO_MM)
        			*result *= 25.4f;
        	}
            return ptr;
        }
        ptr++;
    }
}

void printStatus(STATUSES satus) {
	if(statusCounter>0){
		sendstring(" ", gLineFrom, storedIndex);
	}
	statusCounter++;
	gErrorWhileProcessingDetected = statuses_isErrorStatus(satus);
	sendstring(statuses_getStatusText(satus), gLineFrom, storedIndex);
}

void gParseLine(char *linein, LINEFROM lineFrom, int index)
{
	int i;
	for(i=0; i< sizeof CNCPROTOCOL_ASK; i++) {
		if(linein[i]!=CNCPROTOCOL_ASK[i])
			break;
		else if ((i+1)==sizeof CNCPROTOCOL_ASK){
			sendlinewithend(CNCPROTOCOL_ANSWER, lineFrom, index);
			return;
		}
	}

	if( ((linein[0]=='s'||linein[0]=='S') && (linein[1]=='t'||linein[1]=='T') && (linein[2]=='o'||linein[2]=='O') && (linein[3]=='p'||linein[3]=='P') && (linein[4]=='\r'||linein[4]=='\n'||linein[4]==0)) || \
			((linein[0]=='m'||linein[0]=='M') && linein[1]=='1' && linein[2]=='1' && linein[3]=='2' && (linein[4]=='\r'||linein[4]=='\n'||linein[4]==0)) ) { // STOP or M112
		int prepf = gPendingFlag;
		gEStop();
		STATUSES as = axis_status();
		axis_estop();
		if(prepf==0) {
			if(as==STATUS_OK)
				sendlinewithend(statuses_getStatusText(STATUS_OK), lineFrom, index);
			else
				sendlinewithend(statuses_getStatusText(STATUS_ERROR_ESTOP), lineFrom, index);
		}
		return;
	}

	char *line = linein;
	if(gPendingFlag || gReadG[gGpointer]!=G_NO_COMMAND) {
		sendlinewithend(statuses_getStatusText(STATUS_BUSY), lineFrom, index);
		return;
	}
	gLineFrom = lineFrom;
	gErrorWhileProcessingDetected = 0;
	gLastCommandPtr = linein;
	gResponse[0] = 0;
	gResponsePointer = 0;
	storedIndex = index;
	if(line[0]=='%') {
		sendlinewithend(statuses_getStatusText(STATUS_OK), lineFrom, index);
		return;
	}

    gDefArguments();
    int isCReaded = 0;

    while (*line && *line!='\r' && *line!='\n') {
        switch(*line) {
        case '(': // комментарии
            while(*line!=')') {
            	line++;
            	if(*line==0 || *line=='\r' || *line=='\n')
            		goto gofromwhile;
            }
            break;
        case ';':  // комментарии
        	goto gofromwhile;
        case 'X':	//Координата точки траектории по оси X	G0 X100 Y0 Z0
        case 'x':
            line = readArgf(line+1, &gReadX, (mUnit==UNITS_IMPERIAL)?CU_INC_TO_MM:CU_NOT_CONVERT)-1;
            break;
        case 'Y':	//Координата точки траектории по оси Y	G0 X0 Y100 Z0
        case 'y':
            line = readArgf(line+1, &gReadY, (mUnit==UNITS_IMPERIAL)?CU_INC_TO_MM:CU_NOT_CONVERT)-1;
            break;
        case 'Z':	//Координата точки траектории по оси Z	G0 X0 Y0 Z100
        case 'z':
            line = readArgf(line+1, &gReadZ, (mUnit==UNITS_IMPERIAL)?CU_INC_TO_MM:CU_NOT_CONVERT)-1;
            break;
        case 'E':	// Подача проволки экструдера
        case 'e':
        	line = readArgf(line+1, &gReadE, (mUnit==UNITS_IMPERIAL)?CU_INC_TO_MM:CU_NOT_CONVERT)-1;
        	break;
        case 'C':	// Подача проволки экструдера
        case 'c':
        	isCReaded = 1;
        	line = readArgf(line+1, &gReadC, CU_NOT_CONVERT)-1;
        	break;
        case 'P':	//Параметр команды	G04 P101
        case 'p':
            line = readArgf(line+1, &gReadP, CU_NOT_CONVERT)-1;
            break;
        case 'F':	//Скорость рабочей подачи. G1 G91 X10 F100
        case 'f':
            line = readArgf(line+1, &gReadF, CU_NOT_CONVERT)-1;
            break;
        case 'S':	//Скорость вращения шпинделя	S3000 M3
        case 's':
            line = readArgf(line+1, &gReadS, CU_NOT_CONVERT)-1;
            break;
        case 'A':	// 4ая ось, вращение
        case 'a':
            line = readArgf(line+1, &gReadA, CU_NOT_CONVERT)-1;
            break;
        case 'D':	//Параметр коррекции выбранного инструмента	G1 G41 D1 X10. F150.
        case 'd':
            line = readArgf(line+1, &gReadD, (mUnit==UNITS_IMPERIAL)?CU_INC_TO_MM:CU_NOT_CONVERT)-1;
            break;
        case 'I':	//Параметр дуги при круговой интерполяции. Инкрементальное расстояние от начальной точки до центра дуги по оси X.	G03 X10 Y10 I0 J0 F10
        case 'i':
            line = readArgf(line+1, &gReadI, (mUnit==UNITS_IMPERIAL)?CU_INC_TO_MM:CU_NOT_CONVERT)-1;
            break;
        case 'J':	//Параметр дуги при круговой интерполяции. Инкрементальное расстояние от начальной точки до центра дуги по оси Y.	G03 X10 Y10 I0 J0 F10
        case 'j':
            line = readArgf(line+1, &gReadJ, (mUnit==UNITS_IMPERIAL)?CU_INC_TO_MM:CU_NOT_CONVERT)-1;
            break;
        case 'K':	//Параметр дуги при круговой интерполяции. Инкрементальное расстояние от начальной точки до центра дуги по оси Z.	G03 X10 Y10 I0 K0 F10
        case 'k':
            line = readArgf(line+1, &gReadK, (mUnit==UNITS_IMPERIAL)?CU_INC_TO_MM:CU_NOT_CONVERT)-1;
            break;
        case 'L':	//Вызов подпрограммы с данной меткой
        case 'l':
            line = readArgf(line+1, &gReadL, CU_NOT_CONVERT)-1;
            break;
        case 'g':
        case 'G':
        {
        	if(gGpointer>=G_MAX_COMMANDS_IN_LINE) {
        		sendlinewithend(CNCPROTOCOL_LINE_TOO_LONG, lineFrom, index);
        		gDefArguments();
        		gErrorWhileProcessingDetected = 1;
        		return;
        	}
        	char *ptr = line;
            line = readArgi(ptr+1, &gReadG[gGpointer])-1;
            if(line!=ptr) {
            	if(gReadG[gGpointer]<0 || gReadG[gGpointer]>=M_COMMANDS_OFFSET)
            		gReadG[gGpointer] = G_BAD_COMMAND;
				if(gReadG[gGpointer]>=0 && gReadG[gGpointer]<=3)
					gLastG03 = gReadG[gGpointer];
				gGpointer++;
            }
        }
            break;
        case 'm':
        case 'M':
        {
        	if(gGpointer>=G_MAX_COMMANDS_IN_LINE) {
        		sendlinewithend(CNCPROTOCOL_LINE_TOO_LONG, lineFrom, index);
        		gDefArguments();
        		gErrorWhileProcessingDetected = 1;
        		return;
        	}
        	char *ptr = line;
            line = readArgi(ptr+1, &gReadG[gGpointer])-1;
            if(line!=ptr) {
				if(gReadG[gGpointer]==6 || gReadG[gGpointer]==25 || gReadG[gGpointer]==0 || gReadG[gGpointer]==1 || gReadG[gGpointer]==112) { // смена инструмента, паузы - показываем на экране всю строку, здесь сохраняем ее
					int p = 0;
					char *from = linein;
					while (*from && *from!='\r' && *from!='\n' && p<(sizeof  m06PauseTextBuff - 1) ) {
						m06PauseTextBuff[p] = *from++;
						p++;
					}
					m06PauseTextBuff[p] = 0;
				}
				if(gReadG[gGpointer]<0)
					gReadG[gGpointer] = G_BAD_COMMAND;
				else
					gReadG[gGpointer] += M_COMMANDS_OFFSET;
				gGpointer++;
            }
        }
            break;
        }

        line++;
    }
    gofromwhile:
    if(gGpointer==0) {
    	if(gReadX!=G_NOT_SPECIFED || gReadY!=G_NOT_SPECIFED || gReadZ!=G_NOT_SPECIFED) {
    		gReadG[gGpointer] = gLastG03;
    		gGpointer++;
    	}
    	if(isCReaded && gIsStoreC) {
    		gReadG[gGpointer] = M_COMMANDS_OFFSET+M_CSTORE_COMMAND; // set c
    		gGpointer++;
    	}
    }
    statusCounter = 0;
    gGpointer = 0;
    if(gReadG[gGpointer]==G_NO_COMMAND) {
    	sendlinewithend(CNCPROTOCOL_NOTG_CODE, lineFrom, index);
    } else {
    	if(gEstopOccured) {
    		if((TIM2->CR1&TIM_CR1_CEN)!=0) {
    			gDefArguments();
    			gPendingFlag=0;
    			printStatus(STATUS_ERROR_ESTOP);
    			sendlineend(lineFrom, index);
    			gErrorWhileProcessingDetected = 1;
    			TIM2->CR1 &= ~TIM_CR1_CEN;
    			gEstopOccured = 0;
    		}
    		gEstopOccured = 0;
    	}
    	gResetTimer();
    }
}

void gpending(STATUSES res) {
	if(res!=STATUS_OK) {
		printStatus(res);
	} else {
		gPendingFlag = 1;
	}
}


void gRun() {
	if(axis_status()!=STATUS_OK)
		return;
	int gCache = gReadG[gGpointer]; // copy, because interruption could happened during handling

	if(gPendingFlag) {
		printStatus(STATUS_OK);
		gResetTimer();
		gPendingFlag=0;
		if(gCache==G_NO_COMMAND) {
			sendlineend(gLineFrom, storedIndex);
		}
	}

	float todox=0.0f,todoy=0.0f,todoz=0.0f,todoe=0.0f,todoa=0.0f;

	if(gCache>=0 && gCache<=3) { // только для комманд интерполяции
		if(mMovePat==MOVE_ABSOLUTE) {
			todox = (gReadX==G_NOT_SPECIFED)?raxis_getXPos():gReadX;
			todoy = (gReadY==G_NOT_SPECIFED)?raxis_getYPos():gReadY;
			todoz = (gReadZ==G_NOT_SPECIFED)?raxis_getZPos():gReadZ;
			if(mEMovePat==MOVE_ABSOLUTE)
				todoe = (gReadE==G_NOT_SPECIFED)?raxis_getEPos():gReadE;
			else
				todoe = raxis_getEPos() + ((gReadE==G_NOT_SPECIFED)?0.0f:gReadE);
			todoa = (gReadA==G_NOT_SPECIFED)?raxis_getAPos():gReadA;
		} else {
			todox = (gReadX==G_NOT_SPECIFED)?0.0f:gReadX;
			todoy = (gReadY==G_NOT_SPECIFED)?0.0f:gReadY;
			todoz = (gReadZ==G_NOT_SPECIFED)?0.0f:gReadZ;
			if(mEMovePat==MOVE_ABSOLUTE)
				todoe = (gReadE==G_NOT_SPECIFED)?0.0f:(gReadE-raxis_getEPos());
			else
				todoe = (gReadE==G_NOT_SPECIFED)?0.0f:gReadE;
			todoa = (gReadA==G_NOT_SPECIFED)?0.0f:gReadA;
		}
	}

    if(gCache!=G_NO_COMMAND) {
    	switch(gCache) {
    	case 0: // быстрое перемещение
    		if(mMovePat==MOVE_ABSOLUTE)
    			gpending(raxis_move_to_linear(todox ,todoy, todoz, todoe, todoa, AXIS_MAX_SPEED, 0.0f));
    		else
    			gpending(axis_move_linear(todox, todoy, todoz, todoe, todoa, AXIS_MAX_SPEED, 0.0f));
    		break;
    	case 1: // линейная интерполяция
    		if(mMovePat==MOVE_ABSOLUTE)
    			gpending(raxis_move_to_linear(todox, todoy, todoz, todoe, todoa, gReadF, gReadC));
    		else
    			gpending(axis_move_linear(todox, todoy, todoz, todoe, todoa, gReadF, gReadC));
    		break;
    	case 2: // Круговая интерполяция по часовой стрелке
    		if(mMovePat==MOVE_ABSOLUTE)
    			gpending(raxis_round_to(todox, todoy, todoz, todoe, todoa, gReadI, gReadJ, gReadK, gReadF, ROUND_CW, gPlane, gReadC));
    		else
    			gpending(axis_round(todox, todoy, todoz, todoe, todoa, gReadI, gReadJ, gReadK, gReadF, ROUND_CW, gPlane, gReadC));
    		break;
    	case 3: // Круговая интерполяция против часовой стрелки
    		if(mMovePat==MOVE_ABSOLUTE)
    			gpending(raxis_round_to(todox, todoy, todoz, todoe, todoa, gReadI, gReadJ, gReadK, gReadF, ROUND_CCW, gPlane, gReadC));
    		else
    			gpending(axis_round(todox, todoy, todoz, todoe, todoa, gReadI, gReadJ, gReadK, gReadF, ROUND_CCW, gPlane, gReadC));
    		break;
    	case 4: // Задержка выполнения программы, способ задания величины задержки зависит от реализации системы управления
    		if(gReadP==G_NOT_SPECIFED)
    			printStatus(STATUS_ERROR_INVALID_PARAMETER);
    		else
    			gpending(axis_pause(gReadP, gReadC));
    		break;
    	case 17: // Выбор рабочей плоскости X-Y
    		gPlane = PLANE_XY;
    		gReadK = 0.0f; // нулим заданный по данной оси радиус, если он ранее был задан, чтобы получилась как раз нужная интерполяция
    		printStatus(STATUS_OK);
    		break;
    	case 18: // Выбор рабочей плоскости Z-X
    		gPlane = PLANE_ZX;
    		gReadJ = 0.0f;
    		printStatus(STATUS_OK);
    		break;
    	case 19: // Выбор рабочей плоскости Y-Z
    		gPlane = PLANE_YZ;
    		gReadI = 0.0f;
    		printStatus(STATUS_OK);
    		break;
    	case 20: // Режим работы в дюймовой системе
    		mUnit = UNITS_IMPERIAL;
    		printStatus(STATUS_OK);
    		break;
    	case 21: // Режим работы в метрической системе
    		mUnit = UNITS_METRIC;
    		printStatus(STATUS_OK);
    		break;
    	case 28: // парковка головки
    		gpending(axis_park());
    		break;
    	case 30: // поиск z
    		if(gReadZ==G_NOT_SPECIFED) {
    			gpending(axis_zprobe(-AXIS_CONTINIOUS));
    		} else {
    			if(mMovePat==MOVE_ABSOLUTE)
    				gpending(raxis_zprobe_to(gReadZ));
    			else
    				gpending(axis_zprobe(gReadZ));
    		}
    		break;
    	case 53: // Отключить смещение начала системы координат станка
    		raxis_set_relative_zero_2(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    		printStatus(STATUS_OK);
    		break;
    	case 54: //Переключиться на заданную оператором систему координат
    	case 55:
    	case 56:
    	case 57:
    	case 58:
    	case 59: // 6 слотов
    	{
    		int i = gCache-54;
    		gDeltaX[i] = (gReadX==G_NOT_SPECIFED)?gDeltaX[i]:(raxis_getXPos()+raxis_get_zero_2_x()-gReadX);
			gDeltaY[i] = (gReadY==G_NOT_SPECIFED)?gDeltaY[i]:(raxis_getYPos()+raxis_get_zero_2_y()-gReadY);
			gDeltaZ[i] = (gReadZ==G_NOT_SPECIFED)?gDeltaZ[i]:(raxis_getZPos()+raxis_get_zero_2_z()-gReadZ);
			gDeltaE[i] = (gReadE==G_NOT_SPECIFED)?gDeltaE[i]:(raxis_getEPos()+raxis_get_zero_2_e()-gReadE);
			gDeltaA[i] = (gReadA==G_NOT_SPECIFED)?gDeltaA[i]:(raxis_getAPos()+raxis_get_zero_2_a()-gReadA);
    		raxis_set_relative_zero_2(gDeltaX[i], gDeltaY[i], gDeltaZ[i], gDeltaE[i], gDeltaA[i]);
    		printStatus(STATUS_OK);
    		break;
    	}
    	case 90:
    		mMovePat=MOVE_ABSOLUTE;
    		if(mEMovePatSet==0)
    			mEMovePat=MOVE_ABSOLUTE;
    		printStatus(STATUS_OK);
    		break;
    	case 91:
			mMovePat=MOVE_RELATIVE;
			if(mEMovePatSet==0)
				mEMovePat=MOVE_RELATIVE;
			printStatus(STATUS_OK);
			break;
    	case 92: // установка абсолютных накопительных положений
    	{
    		if(gReadX==G_NOT_SPECIFED && gReadY==G_NOT_SPECIFED && gReadZ==G_NOT_SPECIFED && gReadE==G_NOT_SPECIFED && gReadA==G_NOT_SPECIFED) {
    			raxis_set_relative_zero_2(raxis_getXPos()+raxis_get_zero_2_x(), raxis_getYPos()+raxis_get_zero_2_y(), raxis_getZPos()+raxis_get_zero_2_z(), raxis_getEPos()+raxis_get_zero_2_e(), raxis_getAPos()+raxis_get_zero_2_a());
    		} else {
				raxis_set_relative_zero_2(gReadX==G_NOT_SPECIFED?raxis_get_zero_2_x():(raxis_getXPos()+raxis_get_zero_2_x()-gReadX), gReadY==G_NOT_SPECIFED?raxis_get_zero_2_y():(raxis_getYPos()+raxis_get_zero_2_y()-gReadY), \
						gReadZ==G_NOT_SPECIFED?raxis_get_zero_2_z():(raxis_getZPos()+raxis_get_zero_2_z()-gReadZ), gReadE==G_NOT_SPECIFED?raxis_get_zero_2_e():(raxis_getEPos()+raxis_get_zero_2_e()-gReadE), \
								gReadA==G_NOT_SPECIFED?raxis_get_zero_2_a():(raxis_getAPos()+raxis_get_zero_2_a()-gReadA) );
    		}
    		printStatus(STATUS_OK);
    		break;
    	}
    	/*
    	case 6: // параболическая интерполяция
    	case 8: // разгон
    	case 9: // торможение
    	case 33: // нарезание резьбы
    	case 34: // нарезание резьбы с увеличивающимся шагом
    	case 35: // нарезание резьбы с уменьшающимся шагом
    	case 40: // Отмена компенсации радиуса инструмента
    	case 41: // Компенсировать радиус инструмента слева от траектории
    	case 42: // Компенсировать радиус инструмента справа от траектории
    	case 43: // Компенсировать длину инструмента положительно
    	case 44: // Компенсировать длину инструмента отрицательно
    	case 45: // Коррекция на положение инструмента
    	case 46: // Коррекция на положение инструмента
    	case 47: // Коррекция на положение инструмента
    	case 48: // Коррекция на положение инструмента
    	case 49: // Отмена компенсации длины инструмента
    	case 50: // Коррекция на положение инструмента
    	case 51: // Коррекция на положение инструмента
    	case 52: // Коррекция на положение инструмента
    	case 63: // нарезание резьбы метчиком
    	case 93: // скорость подачи функции, обратной времени
    	case 94: // подача в минуту
    	case 95: // подача на оборот
    	case 96: // постоянная скорость резки
    	case 97: // обороты в минуту
    		break;*/
    	// по ГОСТу
    	// 5, 7, 10..16, 20..32, 36..39, 60..62, 64..79, 98..99 - не определенно
    	// 80..89 постоянные циклы, 9 слотов, 80 - отмена

    	case M_COMMANDS_OFFSET+0: // Приостановить работу станка до нажатия кнопки «старт» на пульте управления, так называемый «безусловный технологический останов»
    	case M_COMMANDS_OFFSET+1: // Приостановить работу станка до нажатия кнопки «старт», если включён режим подтверждения останова
    	case M_COMMANDS_OFFSET+112: // Emergency Stop
    		heater_extruder_heat_off();
    		heater_table_heat_off();
    		spindle_stop();
    		gpending(axis_stopPause());
    	    CNC_GUI_PauseScreen(m06PauseTextBuff);
    	    beep_triple();
    		break;
    	case M_COMMANDS_OFFSET+2: // Конец программы, без сброса модальных функций
			spindle_stop();
			greset();
			printStatus(STATUS_OK);
			break;
    	case M_COMMANDS_OFFSET+3:	// Начать вращение шпинделя по часовой стрелке
    	case M_COMMANDS_OFFSET+13:	// Включить охлаждение и вращение шпинделя по часовой стрелке
			printStatus(spindle_runr(gReadS));
			break;
    	case M_COMMANDS_OFFSET+5:	// Остановить вращение шпинделя
			printStatus(spindle_stop());
			break;
    	case M_COMMANDS_OFFSET+6: // Сменить инструмент
    	case M_COMMANDS_OFFSET+25: // Замена инструмента вручную
    		gpending(axis_changeTool());
    		CNC_GUI_ChangeToolScreen(m06PauseTextBuff);
    		beep_triple();
    		break;
    	case M_COMMANDS_OFFSET+M_CSTORE_COMMAND:
    		gIsStoreC = 1;
    		printStatus(STATUS_OK);
    		break;
    	case M_COMMANDS_OFFSET+M_CSTORE_COMMAND+1:
    		gIsStoreC = 0;
    		printStatus(STATUS_OK);
    		break;
    	case M_COMMANDS_OFFSET+30:	// Конец программы, со сбросом модальных функций
			heater_extruder_heat_off();
			heater_table_heat_off();
			spindle_stop();
			greset();
			printStatus(STATUS_OK);
			beep_one();
			break;
    	case M_COMMANDS_OFFSET+82:
    		mEMovePat = MOVE_ABSOLUTE;
    		mEMovePatSet = 1;
    		printStatus(STATUS_OK);
    		break;
    	case M_COMMANDS_OFFSET+83:
    		mEMovePat = MOVE_RELATIVE;
    		mEMovePatSet = 1;
    		printStatus(STATUS_OK);
    		break;
    	case M_COMMANDS_OFFSET+104: //Set Extruder Temperature
    		printStatus(heater_extruder_heat(gReadS));
    		break;
    	case M_COMMANDS_OFFSET+109: // Set Extruder Temperature and Wait
    	{
    		STATUSES res = heater_extruder_heat(gReadS);
    		if(res!=STATUS_OK || gReadS<=HEATER_NOT_CONNECTED_TEMP)
    			printStatus(res);
    		else
    			gpending(heaters_wait(HW_EXTRUDER));
    		break;
    	}
    	case M_COMMANDS_OFFSET+116: // Wait for all temperatures
    		gpending(heaters_wait(HW_TABLEEXTRUDER));
    		break;
    	case M_COMMANDS_OFFSET+140: // Bed Temperature (Fast)
			printStatus(heater_table_heat(gReadS));
			break;
    	case M_COMMANDS_OFFSET+190: // Wait for bed temperature to reach target temp
    	{
    		STATUSES res = heater_table_heat(gReadS);
    		if(res!=STATUS_OK || gReadS<=HEATER_NOT_CONNECTED_TEMP)
    			printStatus(res);
    		else
    			gpending(heaters_wait(HW_TABLE));
    		break;
    	}
    	case M_COMMANDS_OFFSET+300: //Play beep sound
    		if(gReadP==G_NOT_SPECIFED)
    			beep(500, 1);
    		else
    			beep(gReadP, 1);
    		printStatus(STATUS_OK);
    		break;

    	case M_COMMANDS_OFFSET+1001: //Calibrate heaters
    		gpending(heaters_calibrate());
    	    break;
    	case M_COMMANDS_OFFSET+1002: //Calibrate touch screen
    	    if(action_calibratets())
    	    	printStatus(STATUS_OK);
    	    else
    	    	printStatus(STATUS_ERROR_ESTOP);
    	    break;
    	/************************* M коды ***********************/
    	// В ГОСТе большинство из этого не определено
    	/*
    	case M_COMMANDS_OFFSET+4:	// Начать вращение шпинделя против часовой стрелки
    	case M_COMMANDS_OFFSET+7:	// Включить дополнительное охлаждение
    	case M_COMMANDS_OFFSET+8:	// Включить основное охлаждение.
    	case M_COMMANDS_OFFSET+9:	// Выключить охлаждение
    	case M_COMMANDS_OFFSET+14:	// Включить охлаждение и вращение шпинделя против часовой стрелки
    	case M_COMMANDS_OFFSET+17:	// Конец подпрограммы
    	 */	// Замена инструмента вручную

    	case G_BAD_COMMAND:
    	default:
    		printStatus(STATUS_UNIMPLEMENTED);
    	}
    	if(!gPendingFlag && gReadG[gGpointer+1]==G_NO_COMMAND) { // +1 because interupption could happebd before next line
    	    		sendlineend(gLineFrom, storedIndex);
    	    		gResetTimer();
    	}
    	gGpointer++;
    }
}

void gEStop()
{
	if(gPendingFlag) {
		gDefArguments();
		gPendingFlag=0;
		printStatus(STATUS_ERROR_ESTOP);
		sendlineend(gLineFrom, storedIndex);
		TIM2->CR1 &= ~TIM_CR1_CEN;
	} else {
		gResetTimer();
		gEstopOccured = 1;
	}
}

CONNECTED gGetStatus()
{
	if(gLineFrom==LINEFROM_KEYBOARD)
		return CONNECTED_FALSE;
	if(gPendingFlag || ((TIM2->CR1&TIM_CR1_CEN)!=0 && gEstopOccured==0) )
		return CONNECTED_TRUE;
	return CONNECTED_FALSE;
}

void gFinish()
{
	TIM2->CR1 &= ~TIM_CR1_CEN;
}

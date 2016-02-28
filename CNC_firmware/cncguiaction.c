#include "cncgui.h"
#include "mcore.h"
#include "lang.h"
#include <math.h>
#include <stdio.h>
#include "gcode.h"
#include "tsc2046.h"
#include "veeprom/veeprom.h"
#include "beeper.h"

void action_calibrate(GUI_BOOL isRecalibration)
{
	if(isRecalibration==GUI_TRUE && axis_isparked()==STATUS_OK) {
		if(axis_movefromsensors()==STATUS_OK) {
			CNC_GUI_CalibarationScreen();
			while(axis_status()!=STATUS_OK);
			if(axis_isparked()!=STATUS_NOT_ZERO)
				CNC_GUI_AxisScreen();
		}
	}
	if(axis_isparked()==STATUS_NOT_ZERO || heaters_lastCalibrationStatus()==STATUS_ERROR_NOT_CALIBRATED) {
		CNC_GUI_CalibarationScreen();
		if(axis_isparked()==STATUS_NOT_ZERO)
			axis_foundzero();
		if(heaters_lastCalibrationStatus()==STATUS_ERROR_NOT_CALIBRATED) {
			heaters_calibrate();
			while(heaters_wait_status()==HW_CALIBRATION);
		}
		CNC_GUI_AxisScreen();
	}
}

int action_calibratets()
{
	int res = TSCalibrate();
	if(res)
		VEEPROM_SetData(TOUCHSCREEN_DATA_VEEPROM_PAGE, TSGetCalibrationDataPtr(), sizeof(TS_CALIBRATION_DATA));
	return res;
}

void action_spindle()
{
	if(headBtnOn.value == GUI_FALSE) {
		spindle_stop();
	} else {
		spindle_runr(CNC_GUI_RPMBYSLIDER);
	}
}

void action_move(float mx, float my, float mz, GUITouchEventType type, GUI_BOOL isClicked)
{
	if(gGetStatus()==CONNECTED_TRUE)
		return;
	if(axisBtnLaser.value == GUI_TRUE)
		laser_force(GUI_LASER_POWER, GUI_LASER_TIME);
	if(type!=GUITS_Up && axisStepComboBox.value!=CB_CONTINIOUSINDEX)
		return;
	float speed = AXIS_MAX_SPEED;
	float step;
	switch(axisStepComboBox.value){
	case CB_CONTINIOUSINDEX:
		if(isClicked==GUI_FALSE) {
			axis_stop();
			return;
		}
		if(axisBtnLaser.value == GUI_TRUE)
			laser_force(GUI_LASER_POWER, GUI_LASER_TIME_LONG);
		axis_move_linear(AXIS_CONTINIOUS*mx, AXIS_CONTINIOUS*my, AXIS_CONTINIOUS*mz, 0.0f, 0.0f, AXIS_MAX_SPEED_LOW_ACCELERATION, 0.0f);
		return;
	case 7:
		step = 10.0f;
		break;
	case 6:
		step = 5.0f;
		break;
	case 5:
		step = 1.0f;
		break;
	case 4:
		step = 0.5f;
		break;
	case 3:
		step = 0.1f;
		break;
	case 2:
		step = 0.05f;
		break;
	case 1:
		step = 0.01f;
		break;
	case 0:
		step = 0.0025f;
		break;
	default:
		step = 1.0f;
	}
	float x = axis_getXPos()+mx*step;
	float y = axis_getYPos()+my*step;
	float z = axis_getZPos()+mz*step;
	if(axisBtnAlign.value==GUI_TRUE) {
		if(mx!=0) {
			x = truncf(axis_getXPos()/step)*step+mx*step;
			if(fabsf(x-axis_getXPos())>step+0.00001f)
				x -= mx*step;
		}
		if(my!=0) {
			y = truncf(axis_getYPos()/step)*step+my*step;
			if(fabsf(y-axis_getYPos())>step+0.00001f)
				y -= my*step;
		}
		if(mz!=0) {
			z = truncf(axis_getZPos()/step)*step+mz*step;
			if(fabsf(z-axis_getZPos())>step+0.00001f)
				z -= mz*step;
		}
	}
	axis_move_to_linear(x,y,z,axis_getEPos(),axis_getAPos(),speed, 0.0f);

}


void action_move_set()
{
	if(axisBtnSet0.value==GUI_TRUE)
		axis_move_to_linear(raxis_get_zero_1_x(), raxis_get_zero_1_y(), raxis_get_zero_1_z(), axis_getEPos(), axis_getAPos(), AXIS_MAX_SPEED, 0.0f);
	else
		axis_move_to_linear(move2XDef, move2YDef, move2ZDef, axis_getEPos(), axis_getAPos(), AXIS_MAX_SPEED, 0.0f);
}

void action_setz0()
{
	raxis_lock_relative_zero_1z();
	axisBtnSet0.value=GUI_TRUE;
	GUI_drawView(&axisBtnSet0);
	snprintf(move2SetText, CNC_GUI_TEXTS_LENGTH*3, STR_MOVE2SET, (double)raxis_get_zero_1_x(), (double)raxis_get_zero_1_y(), (double)raxis_get_zero_1_z());
	GUI_drawView(&axisBtnMove2Set);
}

void action_set0()
{
	if(axisBtnSet0.value==GUI_TRUE){
		raxis_lock_relative_zero_1();
		snprintf(move2SetText, CNC_GUI_TEXTS_LENGTH*3, STR_MOVE2SET, (double)raxis_get_zero_1_x(), (double)raxis_get_zero_1_y(), (double)raxis_get_zero_1_z());
	} else{
		raxis_free_relative_zero_1();
		printDefMove2Set();
	}
	GUI_drawView(&axisBtnMove2Set);
}

void action_estop()
{
	gEStop();
	axis_estop();
	beep_stop();
}

void action_extruder()
{
	if(headBtnHeatExtruder.value == GUI_FALSE) {
		heater_extruder_heat_off();
	} else {
		if(heater_extruder_heat(headExtruderComboBox.value*10+100)!=STATUS_OK) {
			headBtnHeatExtruder.value = GUI_FALSE;
			GUI_drawView(&headBtnHeatExtruder);
		}
	}
}

void action_table()
{
	if(headBtnHeatTable.value == GUI_FALSE) {
		heater_table_heat_off();
	} else {
		if(heater_table_heat(headTableComboBox.value*5+40)!=STATUS_OK) {
			headBtnHeatTable.value = GUI_FALSE;
			GUI_drawView(&headBtnHeatTable);
		}
	}
}

void action_laser()
{
	float power = GUI_LASER_POWER;
	if(headLaserComboBox.value==0)
		power = 0.5f;
	else if(headLaserComboBox.value==1)
		power = 1.0f;
	else if(headLaserComboBox.value==2)
		power = 5.0f;
	else if(headLaserComboBox.value==3)
		power = 10.0f;
	else if(headLaserComboBox.value==4)
		power = 25.0f;
	else if(headLaserComboBox.value==5)
		power = 50.0f;
	else if(headLaserComboBox.value==6)
		power = 75.0f;
	else if(headLaserComboBox.value==7)
		power = 100.0f;
	else if(headLaserComboBox.value==8)
		power = 150.0f;
	else if(headLaserComboBox.value==9)
		power = 200.0f;
	else if(headLaserComboBox.value==10)
		power = 250.0f;
	else if(headLaserComboBox.value==11)
		power = 300.0f;
	axis_pause(GUI_LASER_TIME_LONG, power);
}

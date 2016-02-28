#include "cncgui.h"
#include "mcore.h"
#include "lang.h"
#include "gcode.h"
#include "beeper.h"
#include "usbhost.h"
#include <stdio.h>


#define AXISBTNWIDTH 45
#define AXISBTNHEIGHT 33

char spindleText[CNC_GUI_TEXTS_LENGTH] ="\0";
char tableText[CNC_GUI_TEXTS_LENGTH] ="\0";
char extruderText[CNC_GUI_TEXTS_LENGTH] ="\0";
char xText[CNC_GUI_TEXTS_LENGTH] ="\0";
char yText[CNC_GUI_TEXTS_LENGTH] = "\0";
char zText[CNC_GUI_TEXTS_LENGTH] = "\0";
char move2SetText[CNC_GUI_TEXTS_LENGTH*3] = "\0";
char fileManageFileNames[FILESMANAGER_FILE_COUNT][CNC_GUI_TEXTS_LENGTH];
char fileManagerPath[CNC_GUI_SHOW_PATH_LENGTH] = "\0";
int fileManagerStartIndex = 0;
View *lastClickedItem = 0;

void printDefMove2Set(){
	snprintf(move2SetText, CNC_GUI_TEXTS_LENGTH*3, STR_MOVE2SET, (double)move2XDef, (double)move2YDef, (double)move2ZDef);
}

void CNC_GUI_Init()
{
	GUI_InitHardware();
	CNC_GUI_logoScreen();

	GUI_initScreen(&calibarationScreen,Green);
	GUI_initLabel(&calibarationLabel1, &calibarationScreen, STR_CALIBRATION,&lucidaConsole_18pt, Black, 0, 104,LCD_XSIZE,GUI_CONTENT,AlignCenter);
	GUI_initLabel(&calibarationLabel2, &calibarationScreen, STR_PLSWAIT,&lucidaConsole_18pt, Black, 0, 104+lucidaConsole_18pt.charHeight,LCD_XSIZE,GUI_CONTENT,AlignCenter);

//	GUI_initScreen(&mainScreen,White);
//	GUI_initButton(&mainTestButton,&mainScreen,"Button",&lucidaConsole_18pt,Grey,GUI_TRUE,10,10,GUI_CONTENT, GUI_CONTENT,AlignCenter);
//	mainTestButton.value = GUI_FALSE;
//	GUI_initLabel(&mainTestLabel,&mainScreen, "Label",&lucidaConsole_18pt, Blue, 50, 50,GUI_CONTENT, GUI_CONTENT,AlignCenter);
//	GUI_initSlider(&mainTestSlider, &mainScreen, Green, 50,100, GUI_CONTENT,GUI_CONTENT);
//	GUI_initComboBox(&mainTestComboBox, &mainScreen, "text1\0text2\0text3\0text4\0\0", &lucidaConsole_12pt, Red, 50, 170, GUI_CONTENT, GUI_CONTENT, AlignCenter);

	GUI_initScreen(&axisScreen, White);
	GUI_initButton(&axisBtnAxis, &axisScreen, STR_AXIS, &lucidaConsole_18pt, Green, GUI_FALSE, 256, 0, 64, 36, AlignCenter);
	GUI_initButton(&axisBtnHead, &axisScreen, STR_HEAD, &lucidaConsole_18pt, LightGrey, GUI_FALSE, 256, 36, 64, 36, AlignCenter);
	GUI_initButton(&axisBtnPark, &axisScreen, STR_PARK, &lucidaConsole_12pt, LightGrey, GUI_FALSE, 0, 204, 80, 36, AlignCenter);
	GUI_initButton(&axisBtnCalibrate, &axisScreen, STR_CALIB, &lucidaConsole_12pt, LightGrey, GUI_FALSE, 80, 204, 80, 36, AlignCenter);
	GUI_initButton(&axisBtnSet0, &axisScreen, STR_SET0, &lucidaConsole_12pt, Magenta, GUI_TRUE,  160, 204, 80, 36, AlignCenter);
	GUI_initButton(&axisBtnMove2Set, &axisScreen, move2SetText, &lucidaConsole_6pt, LightGrey, GUI_FALSE,  240, 204, 80, 36, AlignLeftTop);
	GUI_initButton(&axisBtnXInc, &axisScreen, ">", &lucidaConsole_18pt, LightGrey, GUI_FALSE, AXISBTNWIDTH*2, AXISBTNHEIGHT, AXISBTNWIDTH, AXISBTNHEIGHT, AlignCenter);
	GUI_initButton(&axisBtnXDec, &axisScreen, "<", &lucidaConsole_18pt, LightGrey, GUI_FALSE, 0, AXISBTNHEIGHT, AXISBTNWIDTH, AXISBTNHEIGHT, AlignCenter);
	GUI_initButton(&axisBtnYDec, &axisScreen, "v", &lucidaConsole_18pt, LightGrey, GUI_FALSE, AXISBTNWIDTH, AXISBTNHEIGHT*2, AXISBTNWIDTH, AXISBTNHEIGHT, AlignCenter);
	GUI_initButton(&axisBtnYInc, &axisScreen, "^", &lucidaConsole_18pt, LightGrey, GUI_FALSE, AXISBTNWIDTH, 0, AXISBTNWIDTH, AXISBTNHEIGHT, AlignCenter);
	GUI_initButton(&axisBtnZDec, &axisScreen, "v", &lucidaConsole_18pt, LightGrey, GUI_FALSE, AXISBTNWIDTH, AXISBTNHEIGHT*4, AXISBTNWIDTH, AXISBTNHEIGHT, AlignCenter);
	GUI_initButton(&axisBtnZInc, &axisScreen, "^", &lucidaConsole_18pt, LightGrey, GUI_FALSE, AXISBTNWIDTH, AXISBTNHEIGHT*3, AXISBTNWIDTH, AXISBTNHEIGHT, AlignCenter);
	GUI_initButton(&axisBtnXIncYDec, &axisScreen, "\\", &lucidaConsole_18pt, LightGrey, GUI_FALSE, AXISBTNWIDTH*2, AXISBTNHEIGHT*2, AXISBTNWIDTH, AXISBTNHEIGHT, AlignCenter);
	GUI_initButton(&axisBtnXDecYDec, &axisScreen, "/", &lucidaConsole_18pt, LightGrey, GUI_FALSE, 0, AXISBTNHEIGHT*2, AXISBTNWIDTH, AXISBTNHEIGHT, AlignCenter);
	GUI_initButton(&axisBtnXIncYInc, &axisScreen, "/", &lucidaConsole_18pt, LightGrey, GUI_FALSE, AXISBTNWIDTH*2, 0, AXISBTNWIDTH, AXISBTNHEIGHT, AlignCenter);
	GUI_initButton(&axisBtnXDecYInc, &axisScreen, "\\", &lucidaConsole_18pt, LightGrey, GUI_FALSE, 0, 0, AXISBTNWIDTH, AXISBTNHEIGHT, AlignCenter);
	GUI_initComboBox(&axisStepComboBox, &axisScreen, STR_AXISSTEP, &lucidaConsole_18pt, Black, 0, AXISBTNHEIGHT*5+4, 180, GUI_CONTENT, AlignCenter);
	axisStepComboBox.value = CB_CONTINIOUSINDEX;
	GUI_initLabel(&axisLabelX, &axisScreen, xText,&lucidaConsole_18pt, Blue, AXISBTNWIDTH*3+6, AXISBTNHEIGHT*3, 137, GUI_CONTENT, AlignLeftVCenter);
	GUI_initLabel(&axisLabelY, &axisScreen, yText,&lucidaConsole_18pt, Blue, AXISBTNWIDTH*3+6, AXISBTNHEIGHT*3+24, 137, GUI_CONTENT, AlignLeftVCenter);
	GUI_initLabel(&axisLabelZ, &axisScreen, zText,&lucidaConsole_18pt, Blue, AXISBTNWIDTH*3+6, AXISBTNHEIGHT*3+48, 137, GUI_CONTENT, AlignLeftVCenter);
	GUI_initButton(&axisBtnAlign, &axisScreen, STR_ALIGN, &lucidaConsole_12pt, Magenta, GUI_TRUE,  190, AXISBTNHEIGHT*5+6, 50, GUI_CONTENT, AlignCenter);
	GUI_initButton(&axisBtnSetZ0, &axisScreen, STR_SETZ0, &lucidaConsole_12pt, Yellow, GUI_FALSE,  240, AXISBTNHEIGHT*5+6, 80, GUI_CONTENT, AlignCenter);
	GUI_initButton(&axisBtnEStop, &axisScreen, STR_ESTOP, &lucidaConsole_18pt, Red, GUI_FALSE,  AXISBTNWIDTH*3+AXISBTNWIDTH/2, AXISBTNHEIGHT/2, 90, GUI_CONTENT, AlignCenter);
	GUI_initLabel(&axisLabelXMM, &axisScreen, STR_MM,&lucidaConsole_18pt, Blue, 280, AXISBTNHEIGHT*3, GUI_CONTENT, GUI_CONTENT, AlignLeftVCenter);
	GUI_initLabel(&axisLabelYMM, &axisScreen, STR_MM,&lucidaConsole_18pt, Blue, 280, AXISBTNHEIGHT*3+24, GUI_CONTENT, GUI_CONTENT, AlignLeftVCenter);
	GUI_initLabel(&axisLabelZMM, &axisScreen, STR_MM,&lucidaConsole_18pt, Blue, 280, AXISBTNHEIGHT*3+48, GUI_CONTENT, GUI_CONTENT, AlignLeftVCenter);
	GUI_initButton(&axisBtnZProbe, &axisScreen, "V", &lucidaConsole_18pt, Cyan, GUI_FALSE, 0, AXISBTNHEIGHT*4, AXISBTNWIDTH, AXISBTNHEIGHT, AlignCenter);
	GUI_initButton(&axisBtnLaser, &axisScreen, STR_LASER, &lucidaConsole_18pt, Violet, GUI_TRUE, AXISBTNWIDTH*3+AXISBTNWIDTH/2, AXISBTNHEIGHT*2, 90, AXISBTNHEIGHT, AlignCenter);

	GUI_initContextMenuItem(&axisBtnCalibrate, &callibrateCntMnuItemAxis, STR_AXIS, &lucidaConsole_12pt, LightGrey);
	GUI_initContextMenuItem(&axisBtnCalibrate, &callibrateCntMnuItemScreen, STR_SCREEN, &lucidaConsole_12pt, LightGrey);
	GUI_initContextMenuItem(&axisBtnCalibrate, &callibrateCntMnuItemHeaters, STR_HEATERS, &lucidaConsole_12pt, LightGrey);

	GUI_initScreen(&headScreen, White);
	GUI_initButton(&headBtnAxis, &headScreen, STR_AXIS, &lucidaConsole_18pt, LightGrey, GUI_FALSE, 256, 0, 64, 36, AlignCenter);
	GUI_initButton(&headBtnHead, &headScreen, STR_HEAD, &lucidaConsole_18pt, Green, GUI_FALSE, 256, 36, 64, 36, AlignCenter);
	GUI_initButton(&headBtnOn, &headScreen,STR_ON,&lucidaConsole_18pt,Red,GUI_TRUE,212, 8, 40, 36, AlignCenter);
	GUI_initSlider(&headSliderRPM, &headScreen, Blue, 4, 58, 248, GUI_CONTENT);
	headSliderRPM.value = GUI_SLIDER_MAX_VALUE/5; // 20%
	GUI_initLabel(&headLabelRPM, &headScreen, spindleText, &lucidaConsole_18pt, Blue, 4, 8 ,208, 36, AlignLeftVCenter);
	GUI_initLabel(&headLabelTableTemp, &headScreen, tableText, &lucidaConsole_18pt, Blue, 4, 100, 161, 36, AlignLeftVCenter);
	GUI_initLabel(&headLabelExtruderTemp, &headScreen, extruderText, &lucidaConsole_18pt, Blue, 4, 136, 161, 36, AlignLeftVCenter);
	GUI_initButton(&headBtnHeatExtruder, &headScreen, STR_HEAT, &lucidaConsole_18pt, Red, GUI_TRUE, 256, 133, 64, 36, AlignCenter);
	GUI_initButton(&headBtnHeatTable, &headScreen, STR_HEAT, &lucidaConsole_18pt, Red, GUI_TRUE, 256, 97, 64, 36, AlignCenter);
	GUI_initButton(&headBtnFilamentOut, &headScreen, "v", &lucidaConsole_18pt, LightGrey, GUI_FALSE, LCD_XSIZE-AXISBTNWIDTH*2, LCD_YSIZE-AXISBTNHEIGHT*2-4, AXISBTNWIDTH, AXISBTNHEIGHT, AlignCenter);
	GUI_initButton(&headBtnFilamentIn, &headScreen, "^", &lucidaConsole_18pt, LightGrey, GUI_FALSE, LCD_XSIZE-AXISBTNWIDTH, LCD_YSIZE-AXISBTNHEIGHT*2-4, AXISBTNWIDTH, AXISBTNHEIGHT, AlignCenter);
	GUI_initLabel(&headLabelFilament, &headScreen, STR_FILAMENTNH, &lucidaConsole_18pt, Black, 4, LCD_YSIZE-AXISBTNHEIGHT*2-4, LCD_XSIZE-4-AXISBTNWIDTH*2, AXISBTNHEIGHT, AlignLeftVCenter);
	GUI_initComboBox(&headTableComboBox, &headScreen, STR_TABLETEMPSTEP, &lucidaConsole_18pt, Black, 165, 100, 90, GUI_CONTENT, AlignCenter);
	headTableComboBox.value = 6;
	GUI_initComboBox(&headExtruderComboBox, &headScreen, STR_EXTRUDERTEMPSTEP, &lucidaConsole_18pt, Black, 165, 136, 90, GUI_CONTENT, AlignCenter);
	headExtruderComboBox.value = 8;
	GUI_initButton(&headLaserBtn, &headScreen, STR_LASER, &lucidaConsole_18pt, Violet, GUI_FALSE, 4, LCD_YSIZE-AXISBTNHEIGHT-4, 90, AXISBTNHEIGHT, AlignCenter);
	GUI_initComboBox(&headLaserComboBox, &headScreen, STR_LASERPOWER, &lucidaConsole_18pt, Violet, 120, LCD_YSIZE-AXISBTNHEIGHT-4, 144, GUI_CONTENT, AlignCenter);
	headLaserComboBox.value = 2;

	GUI_initScreen(&changeToolScreen, Yellow);
	GUI_initLabel(&changeToolLabel, &changeToolScreen, STR_CHANGETOOL,&lucidaConsole_18pt, Black, 4, 4, LCD_XSIZE,GUI_CONTENT, AlignCenter);
	GUI_initLabel(&changeToolLabelString, &changeToolScreen, "",&lucidaConsole_12pt, Black, 4, 44, 144,GUI_CONTENT, AlignLeftTop);
	GUI_initButton(&changeToolBtnContinue, &changeToolScreen, STR_CONTINUE, &lucidaConsole_18pt, Green, GUI_FALSE,  24, 188, 124, GUI_CONTENT, AlignCenter);
	GUI_initButton(&changeToolBtnStop, &changeToolScreen, STR_ESTOP, &lucidaConsole_18pt, Red, GUI_FALSE,  184, 188, 124, GUI_CONTENT, AlignCenter);

	GUI_initScreen(&pauseScreen, Yellow);
	GUI_initLabel(&pauseLabel, &pauseScreen, STR_PAUSE,&lucidaConsole_18pt, Black, 4, 4, LCD_XSIZE,GUI_CONTENT, AlignCenter);
	GUI_initLabel(&pauseLabelString, &pauseScreen, "",&lucidaConsole_12pt, Black, 4, 44, 144,GUI_CONTENT, AlignLeftTop);
	GUI_initButton(&pauseBtnContinue, &pauseScreen, STR_CONTINUE, &lucidaConsole_18pt, Green, GUI_FALSE,  24, 188, 124, GUI_CONTENT, AlignCenter);
	GUI_initButton(&pauseBtnStop, &pauseScreen, STR_ESTOP, &lucidaConsole_18pt, Red, GUI_FALSE,  184, 188, 124, GUI_CONTENT, AlignCenter);

	GUI_initScreen(&connectedScreen, White);
	GUI_initLabel(&connectedLabel, &connectedScreen, STR_CONNECTED,&lucidaConsole_6pt, Red, 304, 4, 16, GUI_CONTENT, AlignCenter);
	GUI_initLabel(&connectedSpindleSpeed, &connectedScreen, STR_SPINDLEOFF,&lucidaConsole_12pt, Black, 58, 204, 90, 16, AlignCenter);
	GUI_initLabel(&connectedLabelX, &connectedScreen, xText,&lucidaConsole_12pt, Black, 58, 222, 88, 16, AlignCenter);
	GUI_initLabel(&connectedLabelY, &connectedScreen, yText,&lucidaConsole_12pt, Black, 146, 222, 88, 16, AlignCenter);
	GUI_initLabel(&connectedLabelZ, &connectedScreen, zText,&lucidaConsole_12pt, Black, 234, 222, 86, 16, AlignCenter);
	GUI_initButton(&connectedBtn, &connectedScreen, STR_ESTOP, &lucidaConsole_12pt, Red, GUI_FALSE,  2, 204, 56, 36, AlignCenter);
	GUI_initLabel(&connectedTableTemp, &connectedScreen, tableText,&lucidaConsole_12pt, Black, 150, 204, 84, 16, AlignCenter);
	GUI_initLabel(&connectedExtruderTemp, &connectedScreen, extruderText,&lucidaConsole_12pt, Black, 236, 204, 84, 16, AlignCenter);

	GUI_initScreen(&fileManagerScreen, White);
	GUI_initLabel(&fileManagerPathLabel, &fileManagerScreen, fileManagerPath, &lucidaConsole_6pt, Black, 2, 0, 183, 34, AlignCenter);
	GUI_initButton(&fileManagerUpBtn, &fileManagerScreen, STR_UP, &lucidaConsole_12pt, Cyan, GUI_FALSE,  185, 0, 45, 34, AlignCenter);
	GUI_initButton(&fileManagerScrollDownBtn, &fileManagerScreen, "<", &lucidaConsole_12pt, Cyan, GUI_FALSE,  230, 0, 45, 34, AlignCenter);
	GUI_initButton(&fileManagerScrollUpBtn, &fileManagerScreen, ">", &lucidaConsole_12pt, Cyan, GUI_FALSE,  275, 0, 45, 34, AlignCenter);
	int i;
	for(i=0; i<FILESMANAGER_FILE_COUNT; i++) {
		int d = i/6, o = i%6;
		fileManageFileNames[i][0] = 'T'; fileManageFileNames[i][1] = 'e'; fileManageFileNames[i][2] = 's'; fileManageFileNames[i][3] = 't'; fileManageFileNames[i][4] = 0;
		GUI_initButton(&fileManagerFileBtn[i], &fileManagerScreen, fileManageFileNames[i], &lucidaConsole_6pt, CNC_GUI_FILE_COLOR, GUI_FALSE,  d+d*106, 36+o*34, 106, 34, AlignCenter);
		if(i==0) {
			GUI_initContextMenuItem(&fileManagerFileBtn[i], &fileManagerCntMnuOk, STR_RUN, &lucidaConsole_12pt, LightGrey);
			GUI_initContextMenuItem(&fileManagerFileBtn[i], &fileManagerCntMnuCancel, STR_CANCEL, &lucidaConsole_12pt, LightGrey);
		} else {
			fileManagerFileBtn[i].contextMenu = fileManagerFileBtn[0].contextMenu;
		}
	}

	GUI_initScreen(&errorScreen, Red);
	GUI_initLabel(&errorLabelError, &errorScreen, STR_ERROR, &lucidaConsole_18pt, Black, 0, 64,LCD_XSIZE,GUI_CONTENT,AlignCenter);
	GUI_initLabel(&errorLabelCommand, &errorScreen, "", &lucidaConsole_6pt, Black, 0, 110,LCD_XSIZE,GUI_CONTENT,AlignLeftVCenter);
	GUI_initLabel(&errorLabelResponse, &errorScreen, "", &lucidaConsole_6pt, Black, 0, 126,LCD_XSIZE,GUI_CONTENT,AlignLeftVCenter);
	GUI_initButton(&errorBtnClose, &errorScreen, STR_CLOSE,&lucidaConsole_18pt, Green, GUI_FALSE, 40, 155,LCD_XSIZE-80,GUI_CONTENT,AlignCenter);

	printDefMove2Set();
}

void CNC_GUI_logoScreen()
{
	LCD_Test();
	LCD_WriteString(20,166,(u8*)"CNC firmware designed", &lucidaConsole_18pt,Black, Transparent);
	LCD_WriteString(100,190,(u8*)"by LuCiFer", &lucidaConsole_18pt,Black, Transparent);
	LCD_WriteString(100,214,(u8*)"***2014***", &lucidaConsole_18pt, Black, Transparent);
	int i;
	for(i=4000000; i>0; i--);
}

void CNC_GUI_CalibarationScreen()
{
	GUI_drawScreen(&calibarationScreen);
}

void CNC_GUI_AxisScreen()
{
	//GUI_drawScreen(&mainScreen);
	if(uh_MSState()==USB_MS_CONNECTED)
		GUI_drawScreen(&fileManagerScreen);
	else
		GUI_drawScreen(&axisScreen);
}

void CNC_GUI_HeadScreen()
{
	GUI_drawScreen(&headScreen);
}

void CNC_GUI_PauseScreen(char *str)
{
	pauseLabelString.text = str;
	GUI_drawScreen(&pauseScreen);
}

void CNC_GUI_ChangeToolScreen(char *str)
{
	changeToolLabelString.text = str;
	GUI_drawScreen(&changeToolScreen);
}

void CNC_GUI_ConnectedScreen()
{
	if(GUI_GetCurrentScreen()==&connectedScreen)
		return;
	connectedCursorCl00 = connectedScreen.bgColor;
	connectedCursorCl10 = connectedScreen.bgColor;
	connectedCursorCl01 = connectedScreen.bgColor;
	connectedCursorCl11 = connectedScreen.bgColor;
	GUI_drawScreen(&connectedScreen);
	updateXYZReset();
}

void CNC_GUI_MSerrorScreen(char *command, char *responce)
{
	errorLabelCommand.text = command;
	GUI_drawView(&errorLabelCommand);
	errorLabelResponse.text = responce;
	GUI_drawView(&errorLabelResponse);
	GUI_drawScreen(&errorScreen);
}

void FM_hideAllControls()
{
	fileManagerUpBtn.visible = GUI_FALSE;
	GUI_drawView(&fileManagerUpBtn);
	fileManagerScrollDownBtn.visible = GUI_FALSE;
	GUI_drawView(&fileManagerScrollDownBtn);
	fileManagerScrollUpBtn.visible = GUI_FALSE;
	GUI_drawView(&fileManagerScrollUpBtn);
}

void guiEvent(View *view, GUITouchEventType type)
{
	if(view == &headBtnAxis) {
		if(type==GUITS_Up)
			CNC_GUI_AxisScreen();
	} else if (view == &axisBtnHead) {
		if(type==GUITS_Up)
			CNC_GUI_HeadScreen();
	} else if (view == &axisBtnCalibrate) {
		if(type==GUITS_Up)
			action_calibrate(GUI_TRUE);
	} else if (view == &callibrateCntMnuItemAxis) {
		action_calibrate(GUI_TRUE);
	} else if (view == &callibrateCntMnuItemScreen) {
		action_calibratets();
	} else if (view == &callibrateCntMnuItemHeaters) {
		if(heaters_calibrate()==STATUS_OK)
			CNC_GUI_CalibarationScreen();
	} else 	if( (type==GUITS_Up && view==&headBtnOn) || view==&headSliderRPM){
		action_spindle();
	} else if (view==&axisBtnPark) {
		if(type==GUITS_Up)
			axis_park();
	} else if (view==&axisBtnSet0) {
		if(type==GUITS_Up) {
			action_set0();
		}
	} else if (view==&axisBtnSetZ0) {
		if(type==GUITS_Up) {
			action_setz0();
		}
	} else if (view==&axisBtnMove2Set) {
		if(type==GUITS_Up){
			action_move_set();
		}
	} else if (view==&axisBtnEStop) {
		if(type==GUITS_Up){
			action_estop();
		}
	} else if (view==&connectedBtn) {
		if(type==GUITS_Up){
			if(gGetStatus()==CONNECTED_TRUE)
				action_estop();
			else
				CNC_GUI_AxisScreen();
		}
	} else if (view==&changeToolBtnStop || view==&pauseBtnStop) {
		if(type==GUITS_Up){
			action_estop();
			beep_stop();
			CNC_GUI_AxisScreen();
		}
	}else if (view==&changeToolBtnContinue) {
		if(type==GUITS_Up){
			beep_stop();
			if(axis_toolChanged()==STATUS_OK)
				CNC_GUI_ConnectedScreen();
		}
	} else if (view==&pauseBtnContinue) {
		if(type==GUITS_Up){
			beep_stop();
			if(axis_stopResume()==STATUS_OK)
				CNC_GUI_ConnectedScreen();
		}
	} else if (view==&headBtnFilamentIn) {
		if(headBtnFilamentIn.isClicked==GUI_FALSE){
			axis_stop();
		} else {
			axis_move_linear(0.0f, 0.0f, 0.0f,-AXIS_CONTINIOUS, 0.0f, AXIS_MAX_SPEED_LOW_ACCELERATION, 0.0f);
		}
	} else if (view==&headBtnFilamentOut) {
		if(headBtnFilamentOut.isClicked==GUI_FALSE){
			axis_stop();
		} else {
			axis_move_linear(0.0f, 0.0f, 0.0f, AXIS_CONTINIOUS, 0.0f, AXIS_MAX_SPEED_LOW_ACCELERATION, 0.0f);
		}
	} else if (view==&axisBtnZProbe) {
		if(type==GUITS_Up){
			axis_zprobe(-AXIS_CONTINIOUS);
		}
	} else if (view==&headLaserBtn) {
		if(view->isClicked==GUI_FALSE) {
			axis_stop();
		} else {
			action_laser();
		}
	} else if (view==&axisBtnLaser) {
		if(type==GUITS_Up){
			if(view->value == GUI_FALSE) {
				laser_force_stop();
			} else {
				laser_force(GUI_LASER_POWER, GUI_LASER_TIME);
			}
		}
	} else if (view==&axisBtnXInc) {
		action_move(1.0f,0.0f,0.0f, type, view->isClicked);
	} else if (view==&axisBtnYInc) {
		action_move(0.0f,1.0f,0.0f, type, view->isClicked);
	} else if (view==&axisBtnZInc) {
		action_move(0.0f,0.0f,1.0f, type, view->isClicked);
	}else if (view==&axisBtnXDec) {
		action_move(-1.0f,0.0f,0.0f, type, view->isClicked);
	} else if (view==&axisBtnYDec) {
		action_move(0.0f,-1.0f,0.0f, type, view->isClicked);
	} else if (view==&axisBtnZDec) {
		action_move(0.0f,0.0f,-1.0f, type, view->isClicked);
	} else if (view==&axisBtnXIncYInc) {
		action_move(1.0f,1.0f,0.0f, type, view->isClicked);
	} else if (view==&axisBtnXIncYDec) {
		action_move(1.0f,-1.0f,0.0f, type, view->isClicked);
	} else if (view==&axisBtnXDecYInc) {
		action_move(-1.0f,1.0f,0.0f, type, view->isClicked);
	} else if (view==&axisBtnXDecYDec) {
		action_move(-1.0f,-1.0f,0.0f, type, view->isClicked);
	} else if ( (view==&headBtnHeatExtruder && type==GUITS_Up) || view==&headExtruderComboBox) {
		action_extruder();
	} else if ( (view==&headBtnHeatTable && type==GUITS_Up) || view==&headTableComboBox) {
		action_table();
	} else if (view==&errorBtnClose) {
		if(type==GUITS_Up) {
			CNC_GUI_AxisScreen();
		}
	} else if (view==&fileManagerUpBtn) {
		if(type==GUITS_Up) {
			FM_hideAllControls();
			uh_MSCHDir("..");
		}
	} else if (view==&fileManagerScrollDownBtn) {
		if(type==GUITS_Up) {
			FM_hideAllControls();
			fileManagerStartIndex -= FILESMANAGER_FILE_COUNT;
			uh_MSDir();
		}
	} else if (view==&fileManagerScrollUpBtn) {
		if(type==GUITS_Up) {
			FM_hideAllControls();
			fileManagerStartIndex += FILESMANAGER_FILE_COUNT;
			uh_MSDir();
		}
	} else if (view == &fileManagerCntMnuOk) {
		uh_MSType(lastClickedItem->text);
	} else if(GUI_GetCurrentScreen()==&fileManagerScreen) {
			int i;
			for(i=0; i<FILESMANAGER_FILE_COUNT; i++) {
				if(view==&fileManagerFileBtn[i]) {
					if(fileManagerFileBtn[i].color==CNC_GUI_DIR_COLOR) {
						if(type==GUITS_Up)
							uh_MSCHDir(fileManagerFileBtn[i].text);
					} else {
						lastClickedItem = view;
						if(type==GUITS_Up)
							GUI_drawContextMenu(view);
					}
					break;
				}
		}
	}

//	if(view == &mainTestButton || view == &mainTestSlider)
//	{
//		if(mainTestButton.value == GUI_TRUE)
//			spindle_runp(mainTestSlider.value*100.0f/GUI_SLIDER_MAX_VALUE);
//		else
//			spindle_stop();
//	}
}

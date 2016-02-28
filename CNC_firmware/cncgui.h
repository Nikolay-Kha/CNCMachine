#ifndef _CNC_GUI_H
#define _CNC_GUI_H

#include "guilib.h"
#include "mcore.h"
Screen calibarationScreen;
View calibarationLabel1;
View calibarationLabel2;

//Screen mainScreen;
//View mainTestButton;
//View mainTestLabel;
//View mainTestSlider;
//View mainTestComboBox;

Screen axisScreen;
View axisBtnAxis;
View axisBtnHead;
View axisBtnCalibrate;
View axisBtnPark;
View axisBtnSet0;
View axisBtnMove2Set;
View axisBtnXInc;
View axisBtnXDec;
View axisBtnYInc;
View axisBtnYDec;
View axisBtnZInc;
View axisBtnZDec;
View axisBtnXIncYInc;
View axisBtnXDecYInc;
View axisBtnXIncYDec;
View axisBtnXDecYDec;
View axisStepComboBox;
View axisLabelX;
View axisLabelY;
View axisLabelZ;
View axisBtnAlign;
View axisBtnSetZ0;
View axisBtnEStop;
View axisLabelXMM;
View axisLabelYMM;
View axisLabelZMM;
View axisBtnZProbe;
View axisBtnLaser;

View callibrateCntMnuItemAxis;
View callibrateCntMnuItemScreen;
View callibrateCntMnuItemHeaters;

Screen headScreen;
View headBtnAxis;
View headBtnHead;
View headBtnOn;
View headSliderRPM;
View headLabelRPM;
View headLabelTableTemp;
View headLabelExtruderTemp;
View headBtnHeatExtruder;
View headBtnHeatTable;
View headBtnFilamentIn;
View headBtnFilamentOut;
View headLabelFilament;
View headTableComboBox;
View headExtruderComboBox;
View headLaserBtn;
View headLaserComboBox;

Screen changeToolScreen;
View changeToolLabel;
View changeToolLabelString;
View changeToolBtnContinue;
View changeToolBtnStop;

Screen pauseScreen;
View pauseLabel;
View pauseLabelString;
View pauseBtnContinue;
View pauseBtnStop;

Screen connectedScreen;
View connectedLabel;
View connectedLabelX;
View connectedLabelY;
View connectedLabelZ;
View connectedBtn;
View connectedSpindleSpeed;
View connectedExtruderTemp;
View connectedTableTemp;

u16 connectedCursorCl00;
u16 connectedCursorCl10;
u16 connectedCursorCl01;
u16 connectedCursorCl11;

Screen fileManagerScreen;
View fileManagerPathLabel;
View fileManagerUpBtn;
View fileManagerScrollUpBtn;
View fileManagerScrollDownBtn;
#define FILESMANAGER_FILE_COUNT 18
View fileManagerFileBtn[FILESMANAGER_FILE_COUNT];
extern int fileManagerStartIndex;
View fileManagerCntMnuOk;
View fileManagerCntMnuCancel;

Screen errorScreen;
View errorLabelError;
View errorLabelCommand;
View errorLabelResponse;
View errorBtnClose;


#define CNC_GUI_SHOW_PATH_LENGTH 21
#define CNC_GUI_FILE_COLOR LightGrey
#define CNC_GUI_DIR_COLOR Yellow
#define CNC_GUI_TEXTS_LENGTH 32
extern char spindleText[];
extern char tableText[];
extern char extruderText[];
extern char xText[];
extern char yText[];
extern char zText[];
extern char move2SetText[];
extern char fileManageFileNames[FILESMANAGER_FILE_COUNT][CNC_GUI_TEXTS_LENGTH];
extern char fileManagerPath[];

#define move2XDef 0.0f
#define move2YDef 0.0f
#define move2ZDef 0.0f

#define GUI_LASER_POWER 1.0f
#define GUI_LASER_TIME 10000
#define GUI_LASER_TIME_LONG 60000

#define CNC_GUI_RPMBYSLIDER (int)(SPINDLE_MIN_RPM+headSliderRPM.value*(SPINDLE_MAX_RPM-SPINDLE_MIN_RPM)/GUI_SLIDER_MAX_VALUE)

void CNC_GUI_Init();

void CNC_GUI_logoScreen();

void CNC_GUI_CalibarationScreen();

void CNC_GUI_AxisScreen();

void CNC_GUI_HeadScreen();

void CNC_GUI_ChangeToolScreen(char *str);

void CNC_GUI_PauseScreen(char *str);

void CNC_GUI_ConnectedScreen();

void CNC_GUI_MSerrorScreen(char *command, char *responce);


void action_calibrate(GUI_BOOL isRecalibration);
int action_calibratets();
void action_spindle();
void action_move(float mx, float my, float mz, GUITouchEventType type,  GUI_BOOL isClicked);
void action_move_set();
void action_set0();
void action_setz0();
void action_estop();
void action_extruder();
void action_table();
void action_laser();

void updateCheck();
void updateXYZReset();

//private
void printDefMove2Set();

#endif // _CNC_GUI_H

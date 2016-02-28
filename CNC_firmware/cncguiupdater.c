#include "mcore.h"
#include "cncgui.h"
#include "lang.h"
#include "stm32f10x.h"
#include "gcode.h"
#include "usbhost.h"
#include <stdio.h>
#include <math.h>

u32 lastSpindleRPM = SPINDLE_MAX_RPM*2;
u32 lastSpindleRPMSlider = GUI_SLIDER_MAX_VALUE*2;
int lastTableTemp = HEATER_OFF*2;
int lastExtruderTemp = HEATER_OFF*2;
float lastXPos=-1.0f;
float lastYPos=-1.0f;
float lastZPos=-1.0f;
CONNECTED lastConnected = CONNECTED_FALSE;
char spindleConnectedText[CNC_GUI_TEXTS_LENGTH] = "\0";
#define XYZ_REDRAW_DIV 20
int xReDrawCounter = 0;
int yReDrawCounter = 0;
int zReDrawCounter = 0;
char keyboradInit = 0;

void updateXYZReset() {
	lastXPos=-1.0f;
	lastYPos=-1.0f;
	lastZPos=-1.0f;
}

void updateCheck()
{
	if(uh_MSState()==USB_MS_CONNECTED) {
		if(GUI_GetCurrentScreen()!=&errorScreen && GUI_GetCurrentScreen()!=&connectedScreen && GUI_GetCurrentScreen()!=&fileManagerScreen && gGetStatus()==CONNECTED_FALSE)
			CNC_GUI_AxisScreen();
	}
	//////////////// \/ Head Screen \/ ////////////////////////////////////////////////
	int checkValue = spindle_get_rpm();
	if(checkValue!=SPINDLE_OFF) {
		if(lastSpindleRPM!=checkValue) {
			lastSpindleRPMSlider = GUI_SLIDER_MAX_VALUE*2;
			lastSpindleRPM = checkValue;
			headLabelRPM.color = Red;
			connectedSpindleSpeed.color = Red;
			snprintf(spindleText,CNC_GUI_TEXTS_LENGTH, STR_SPINDLE_RPM, checkValue);
			GUI_drawView(&headLabelRPM);
			snprintf(spindleConnectedText,CNC_GUI_TEXTS_LENGTH, STR_SPINDLE_RPMSHORT, checkValue);
			connectedSpindleSpeed.text = spindleConnectedText;
			GUI_drawView(&connectedSpindleSpeed);
		}
	} else {
		if((void*)connectedSpindleSpeed.text!=(void*)STR_SPINDLEOFF) {
			connectedSpindleSpeed.color = Black;
			connectedSpindleSpeed.text=STR_SPINDLEOFF;
			GUI_drawView(&connectedSpindleSpeed);
		}
		if(lastSpindleRPMSlider!=headSliderRPM.value) {
			lastSpindleRPMSlider = headSliderRPM.value;
			lastSpindleRPM = SPINDLE_MAX_RPM*2;
			headLabelRPM.color = Blue;
			snprintf(spindleText,CNC_GUI_TEXTS_LENGTH, STR_SPINDLE_RPM, CNC_GUI_RPMBYSLIDER);
			GUI_drawView(&headLabelRPM);
		}
	}

	checkValue = (checkValue!=SPINDLE_OFF)?GUI_TRUE:GUI_FALSE;
	if(headBtnOn.value!=checkValue) {
		headBtnOn.value = checkValue;
		GUI_drawView(&headBtnOn);
	}

	checkValue = heater_measured_table_temp();
	if(checkValue!=lastTableTemp) {
		lastTableTemp = checkValue;
		if(checkValue!=HEATER_NOT_CONNECTED_TEMP) {
			snprintf(tableText, CNC_GUI_TEXTS_LENGTH, STR_TABLETEMP, checkValue);
		} else {
			headLabelTableTemp.color = Black;
			connectedTableTemp.color = Black;
			snprintf(tableText, CNC_GUI_TEXTS_LENGTH, STR_TABLETEMPNC);
		}
		GUI_drawView(&headLabelTableTemp);
		GUI_drawView(&connectedTableTemp);
	}

	if(checkValue!=HEATER_NOT_CONNECTED_TEMP) {
		if(heater_get_table_temperature()==HEATER_OFF && headBtnHeatTable.value != GUI_FALSE) {
			headBtnHeatTable.value = GUI_FALSE;
			GUI_drawView(&headBtnHeatTable);
		}
		if(heater_get_table_temperature()!=HEATER_OFF && headBtnHeatTable.value != GUI_TRUE) {
			headBtnHeatTable.value = GUI_TRUE;
			GUI_drawView(&headBtnHeatTable);
		}
		if(heater_get_table_temperature()==HEATER_OFF && headLabelTableTemp.color!=Blue) {
			headLabelTableTemp.color=Blue;
			connectedTableTemp.color=Black;
			GUI_drawView(&headLabelTableTemp);
			GUI_drawView(&connectedTableTemp);
		}

		if(heater_get_table_temperature()!=HEATER_OFF && headLabelTableTemp.color!=Red) {
			headLabelTableTemp.color=Red;
			connectedTableTemp.color=Red;
			GUI_drawView(&headLabelTableTemp);
			GUI_drawView(&connectedTableTemp);
		}
	} else if(headBtnHeatTable.value != GUI_FALSE) {
		headBtnHeatTable.value = GUI_FALSE;
		GUI_drawView(&headBtnHeatTable);
	}

	checkValue = heater_measured_extruder_temp();
	if(checkValue!=lastExtruderTemp) {
		lastExtruderTemp = checkValue;
		if(checkValue!=HEATER_NOT_CONNECTED_TEMP) {
			snprintf(extruderText, CNC_GUI_TEXTS_LENGTH, STR_EXTRUDERTEMP, checkValue);
		} else {
			headLabelExtruderTemp.color = Black;
			connectedExtruderTemp.color = Black;
			snprintf(extruderText, CNC_GUI_TEXTS_LENGTH, STR_EXTRUDERTEMPNC);
		}
		GUI_drawView(&headLabelExtruderTemp);
		GUI_drawView(&connectedExtruderTemp);
	}

	if(checkValue!=HEATER_NOT_CONNECTED_TEMP) {
		if(heater_get_extruder_temperature()==HEATER_OFF && headBtnHeatExtruder.value != GUI_FALSE) {
			headBtnHeatExtruder.value = GUI_FALSE;
			GUI_drawView(&headBtnHeatExtruder);
		}
		if(heater_get_extruder_temperature()!=HEATER_OFF && headBtnHeatExtruder.value != GUI_TRUE) {
			headBtnHeatExtruder.value = GUI_TRUE;
			GUI_drawView(&headBtnHeatExtruder);
		}
		if(heater_get_extruder_temperature()==HEATER_OFF && headLabelExtruderTemp.color!=Blue) {
			headLabelExtruderTemp.color=Blue;
			connectedExtruderTemp.color=Black;
			GUI_drawView(&headLabelExtruderTemp);
			GUI_drawView(&connectedExtruderTemp);
		}

		if(heater_get_extruder_temperature()!=HEATER_OFF && headLabelExtruderTemp.color!=Red) {
			headLabelExtruderTemp.color=Red;
			connectedExtruderTemp.color=Red;
			GUI_drawView(&headLabelExtruderTemp);
			GUI_drawView(&connectedExtruderTemp);
		}
	} else if(headBtnHeatExtruder.value != GUI_FALSE) {
		headBtnHeatExtruder.value = GUI_FALSE;
		GUI_drawView(&headBtnHeatExtruder);
	}

	if(heater_extruder_ready()==STATUS_OK && headLabelFilament.color!=Blue) {
		headLabelFilament.color=Blue;
		headLabelFilament.text = STR_FILAMENT;
		GUI_drawView(&headLabelFilament);
	}

	if(heater_extruder_ready()!=STATUS_OK && headLabelFilament.color!=Black) {
		headLabelFilament.color=Black;
		headLabelFilament.text = STR_FILAMENTNH;
		GUI_drawView(&headLabelFilament);
	}

	//////////////// /\ Head Screen /\ ////////////////////////////////////////////////

	float testXValue = axis_getXPos();
	float testYValue = axis_getYPos();
	float testZValue = axis_getZPos();
	// draw preview on screen
	if(gGetStatus()==CONNECTED_TRUE && GUI_GetCurrentScreen() == &connectedScreen && (lastXPos!=testXValue || lastYPos!=testYValue)) {
		// x = 300 - axisy + 2
		// y = 200 - axisx + 2
		// очень платформозависимо
		const u16 screenLastX = 300 - lastYPos + 2;
		const u16 screenLastY = 200 - lastXPos + 2;
		const u16 screenX = 300 - testYValue + 2;
		const u16 screenY = 200 - testXValue + 2;
		if(lastXPos>=0.0f && lastYPos>=0.0f) {
			LCD_SetPoint(screenLastX-1, screenLastY-1, connectedCursorCl00);
			LCD_SetPoint(screenLastX-1, screenLastY+1, connectedCursorCl01);
			LCD_SetPoint(screenLastX+1, screenLastY-1, connectedCursorCl10);
			LCD_SetPoint(screenLastX+1, screenLastY+1, connectedCursorCl11);

			GUI_LCD_Line(screenLastX, screenLastY, screenX, screenY, Black);
		}
		connectedCursorCl00 = LCD_GetPoint(screenX-1, screenY-1);
		connectedCursorCl01 = LCD_GetPoint(screenX-1, screenY+1);
		connectedCursorCl10 = LCD_GetPoint(screenX+1, screenY-1);
		connectedCursorCl11 = LCD_GetPoint(screenX+1, screenY+1);
		LCD_SetPoint(screenX-1, screenY-1, Red);
		LCD_SetPoint(screenX-1, screenY+1, Red);
		LCD_SetPoint(screenX+1, screenY-1, Red);
		LCD_SetPoint(screenX+1, screenY+1, Red);
		LCD_SetPoint(screenX, screenY, Red);
	}
	//////////////// \/ Axis Screen \/ ////////////////////////////////////////////////
	if(lastXPos!=testXValue) {
		lastXPos = testXValue;
		snprintf(xText,CNC_GUI_TEXTS_LENGTH, STR_XPOS, (double)testXValue);
		if(xReDrawCounter==0)
			xReDrawCounter = XYZ_REDRAW_DIV;
	}
	if(lastYPos!=testYValue) {
		lastYPos = testYValue;
		snprintf(yText,CNC_GUI_TEXTS_LENGTH, STR_YPOS, (double)testYValue);
		if(yReDrawCounter==0)
			yReDrawCounter = XYZ_REDRAW_DIV;
	}
	if(lastZPos!=testZValue) {
		lastZPos = testZValue;
		snprintf(zText,CNC_GUI_TEXTS_LENGTH, STR_ZPOS, (double)testZValue);
		if(zReDrawCounter==0)
			zReDrawCounter = XYZ_REDRAW_DIV;
	}
	if(xReDrawCounter>0) {
		xReDrawCounter--;
		if(xReDrawCounter==0) {
			GUI_drawView(&axisLabelX);
			GUI_drawView(&connectedLabelX);
		}
	}
	if(yReDrawCounter>0) {
		yReDrawCounter--;
		if(yReDrawCounter==0) {
			GUI_drawView(&axisLabelY);
			GUI_drawView(&connectedLabelY);
		}
	}
	if(zReDrawCounter>0) {
		zReDrawCounter--;
		if(zReDrawCounter==0) {
			GUI_drawView(&axisLabelZ);
			GUI_drawView(&connectedLabelZ);
		}
	}
	//////////////// /\ Axis Screen /\ ////////////////////////////////////////////////

	//////////////// \/ Connected Screen, also  inside other screens \/ ////////////////////////////////////////////////
	CONNECTED connect = gGetStatus();
	if(connect!=lastConnected) {
		lastConnected = connect;
		if(connect==CONNECTED_TRUE) {
			if(axisBtnLaser.value == GUI_TRUE) {
				axisBtnLaser.value = GUI_FALSE;
				laser_force_stop();
			}
			connectedLabel.text = STR_CONNECTED;
			connectedBtn.color = Red;
			connectedBtn.text = STR_ESTOP;
			GUI_drawView(&connectedBtn);
			connectedLabel.color = Red;
			GUI_drawView(&connectedLabel);
			if(GUI_GetCurrentScreen()!=&changeToolScreen && GUI_GetCurrentScreen()!=&pauseScreen){
				if(GUI_GetCurrentScreen()==&connectedScreen) {
					if(keyboradInit) {
						GUI_LCD_Rectangle(0,0,LCD_XSIZE, 200,connectedScreen.bgColor,connectedScreen.bgColor);
						keyboradInit = 0;
					}
				}else {
					CNC_GUI_ConnectedScreen();
				}
			}
		} else {
			connectedLabel.text = STR_CONNECTED;
			connectedBtn.color = Green;
			connectedBtn.text = STR_CLOSE;
			GUI_drawView(&connectedBtn);
			connectedLabel.color = Grey;
			GUI_drawView(&connectedLabel);
		}
	}
	if(GUI_GetCurrentScreen()!=&connectedScreen)
		keyboradInit = 0;

	//////////////// /\ Connected Screen /\ ////////////////////////////////////////////////
}

char *cutWidth(char *s, const FONT_INFO *font)
{
	int i = 0;
	while(s[i]) i++;
	int w = 0;
	while(i>0 ) {
		i--;
		w  += font->fontCharInfo[s[i]-font->startCharecter].charWidth + FONTS_FontMargin(font);
		if(w>=LCD_XSIZE) {
			i++;
			break;
		}
	}
	return &s[i];
}

void uh_input_updated()
{
	const FONT_INFO *font =  &lucidaConsole_12pt;
	connectedLabel.text = STR_KEYBOARD;
	connectedLabel.color = Red;
	connectedBtn.color = Green;
	connectedBtn.text = STR_CLOSE;
	if(GUI_GetCurrentScreen()!=&connectedScreen || keyboradInit==0) {
		keyboradInit = 1;
		CNC_GUI_ConnectedScreen();
		uh_line_updated(UPDATE_ALL_LINES);
	}
	// очень платформозависимо
	GUI_LCD_Rectangle(0, 200-16, LCD_XSIZE, font->charHeight, connectedScreen.bgColor,connectedScreen.bgColor);
	GUI_LCD_Print(0, 200-16, cutWidth(uh_getCurrentLine(), font), font, Black);
}

void uh_line_updated(int num)
{
	const FONT_INFO *font =  &lucidaConsole_12pt;
	const int zeroLineWidth = 200-32;

	if(GUI_GetCurrentScreen()!=&connectedScreen || keyboradInit==0) {
		keyboradInit = 1;
		CNC_GUI_ConnectedScreen();
		uh_input_updated();
		uh_line_updated(UPDATE_ALL_LINES);
		return;
	}

	if(num==UPDATE_ALL_LINES) {
		int i;
		for(i=0; i<=zeroLineWidth/font->charHeight; i++)
			uh_line_updated(i);
		return;
	}
	if(num<0 || zeroLineWidth-num*font->charHeight<0)
		return;

	// очень платформозависимо
	GUI_LCD_Rectangle(0, zeroLineWidth-num*font->charHeight, LCD_XSIZE, font->charHeight, connectedScreen.bgColor,connectedScreen.bgColor);
	GUI_LCD_Print(0, zeroLineWidth-num*font->charHeight, cutWidth(uh_getLine(num), font), font, (num%2==0)?Blue:Black);
	if(num==10)
		GUI_drawView(&connectedLabel);
}

void uh_path_updated()
{
	char *a = uh_MSPath();
	int i = 0;
	while(*a && i<CNC_GUI_SHOW_PATH_LENGTH-1) {
		fileManagerPath[i++] = *a++;
	}
	fileManagerPath[i] = 0;
	fileManagerStartIndex = 0;
}

void uh_file_info_recieved(char *info, int index)
{
	if(index==fileManagerStartIndex+FILESMANAGER_FILE_COUNT) {
		fileManagerScrollUpBtn.visible = GUI_TRUE;
		return;
	}
	if(index<fileManagerStartIndex || index>=fileManagerStartIndex+FILESMANAGER_FILE_COUNT)
		return;
	if(fileManagerStartIndex==index && index) {
		fileManagerScrollDownBtn.visible = GUI_TRUE;
	}

	index = index-fileManagerStartIndex;
	char *to = fileManageFileNames[index];
	while(*info!=0 && *info!=' ') {
		*to++ = *info++;
	}
	*to = 0;
	while(*info) {
		if(info[0]=='D' && info[1]=='I' && info[2]=='R') {
			fileManagerFileBtn[index].color = CNC_GUI_DIR_COLOR;
			break;
		}
		info++;
	}
	fileManagerFileBtn[index].visible = GUI_TRUE;
}

void uh_file_transfer_done()
{
	int i;
	GUI_drawView(&fileManagerPathLabel);
	GUI_drawView(&fileManagerScrollDownBtn);
	GUI_drawView(&fileManagerScrollUpBtn);
	if(fileManagerPathLabel.text[0]=='\\' && fileManagerPathLabel.text[1]==0)
		fileManagerUpBtn.visible = GUI_FALSE;
	else
		fileManagerUpBtn.visible = GUI_TRUE;
	GUI_drawView(&fileManagerUpBtn);
	for(i=0; i<FILESMANAGER_FILE_COUNT; i++)
			GUI_drawView(&fileManagerFileBtn[i]);
}

void uh_file_transfer_prepare()
{
	int i=0;
	for(i=0; i<FILESMANAGER_FILE_COUNT; i++) {
		fileManagerFileBtn[i].color = CNC_GUI_FILE_COLOR;
		fileManagerFileBtn[i].visible = GUI_FALSE;
	}

	fileManagerScrollDownBtn.visible = GUI_FALSE;
	fileManagerScrollUpBtn.visible = GUI_FALSE;
	fileManagerUpBtn.visible = GUI_FALSE;

	GUI_drawView(&fileManagerScrollDownBtn);
	GUI_drawView(&fileManagerScrollUpBtn);
	GUI_drawView(&fileManagerUpBtn);
	for(i=0; i<FILESMANAGER_FILE_COUNT; i++)
		GUI_drawView(&fileManagerFileBtn[i]);
}

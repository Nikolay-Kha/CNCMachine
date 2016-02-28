#ifndef _LCD_PROXY_H
#define _LCD_PROXY_H
/******************************************************
REIMPLEMENTS FUNCTION IN THIS FILE FOR YOUR LCD DRIVER
******************************************************/
#include "ili93xx.h"

#define GUI_SCREEN_WIDTH LCD_XSIZE
#define GUI_SCREEN_HEIGHT LCD_YSIZE
void GUI_LCD_Clear(u16 color);
void GUI_LCD_Rectangle(u16 x, u16 y, u16 width, u16 height, u16 color, u16 fillColor);
void GUI_LCD_Line(u16 x0, u16 y0, u16 x1, u16 y1,u16 color);
void GUI_LCD_Print(u16 x, u16 y, const char *text, const FONT_INFO *font, u16 color);
void GUI_TS_LCD_InitHardware();
/******************************************************
CHOOSE FONT THAT YOU NEED
******************************************************/
#include "lucida6.h"
#include "lucida12.h"
#include "lucida18.h"
/******************************************************
REIMPLEMENTS FUNCTION IN THIS FILE FOR YOUR TOUCH DRIVER
******************************************************/
#include "tsc2046.h"

typedef TouchEventType GUITouchEventType;
#define GUITS_Down TS_Down
#define GUITS_Up TS_Up
#define GUITS_Move TS_Move
#define GUITS_ContextMenu TS_LongPress
#define GUITS_DrawCalibration TS_WaitTouchPointForCalibration
#define GUITS_FinishCalibration TS_CalibrationDone
typedef TouchEventReturn GUITouchEventReturn;
#define GUITS_NotInteresting TS_NotInteresting
#define GUITS_InterestingUpOnly TS_InterestingUpOnly
#define GUITS_InterestingMovementAndUp TS_InterestingMovementAndUp

#endif // _LCD_PROXY_H

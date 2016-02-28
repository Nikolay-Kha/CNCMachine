#include "lcdProxy.h"
#include "touches.h"
/******************************************************
REIMPLEMENTS FUNCTION IN THIS FILE FOR YOUR LCD DRIVER
******************************************************/

void GUI_LCD_Clear(u16 color) {
	LCD_Clear(color);
}

void GUI_LCD_Rectangle(u16 x, u16 y, u16 width, u16 height, u16 color, u16 fillColor){
	LCD_Rectangle(x, y, x+width-1, y+height-1, color, fillColor);
}

void GUI_LCD_Line(u16 x0, u16 y0, u16 x1, u16 y1,u16 color){
	LCD_Line(x0,y0,x1,y1,color);
}

void GUI_LCD_Print(u16 x, u16 y, const char *text,const FONT_INFO *font, u16 color) {
	LCD_WriteString(x, y, (u8*)text, font, color,Transparent);
}

void GUI_TS_LCD_InitHardware() {
	LCD_Initializtion();
	TS_Init();
}
/******************************************************
REIMPLEMENTS FUNCTION IN THIS FILE FOR YOUR TOUCH DRIVER
******************************************************/

#include "tsc2046.h"
TouchEventReturn touchEvent(u16 x, u16 y, TouchEventType type)
{
	return GUI_HandleTouch(x, y, type);
}

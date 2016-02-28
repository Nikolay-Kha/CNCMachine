#ifndef G_CODE_H
#define G_CODE_H

typedef enum {
	LINEFROM_NONE,
	LINEFROM_USB,
	LINEFROM_KEYBOARD,
	LINEFROM_MASSSTORAGE
} LINEFROM;

typedef enum {
	MOVE_ABSOLUTE = 0,
	MOVE_RELATIVE
} MOVEPAT;

typedef enum {
	CU_NOT_CONVERT = 0,
	CU_INC_TO_MM
} CONVERTUNIT;

typedef enum {
	UNITS_METRIC = 0,
	UNITS_IMPERIAL
} UNITS;

typedef enum {
	CONNECTED_FALSE =0,
	CONNECTED_TRUE
} CONNECTED;

   void gParseLine(char *line, LINEFROM lineFrom, int index);
   void gRun();
   void gInit();
   void gEStop();
   void gFinish();
   CONNECTED gGetStatus();

#endif // G_CODE_H



/******************************************************************************

 * FileName:        SD Data Logger.c
 * Dependencies:    project requires File System library
 * Processor:       PIC24F
 * Compiler:        C30 v2.01/C32 v0.00.18
 * Company:         Microchip Technology, Inc.

Software License Agreement

The software supplied herewith by Microchip Technology Incorporated
(the �Company? for its PICmicro?Microcontroller is intended and
supplied to you, the Company�s customer, for use solely and
exclusively on Microchip PICmicro Microcontroller products. The
software is owned by the Company and/or its supplier, and is
protected under applicable copyright laws. All rights are reserved.
Any use in violation of the foregoing restrictions may subject the
user to criminal sanctions under applicable laws, as well as to
civil liability for the breach of the terms and conditions of this
license.

THIS SOFTWARE IS PROVIDED IN AN �AS IS?CONDITION. NO WARRANTIES,
WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

 *******************************************************************************/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "GenericTypeDefs.h"
#include "HardwareProfile.h"
#include "FSIO.h"
#include "usb.h"
#include "usb_host_msd.h"
#include "usb_host_msd_scsi.h"
#include "uart2.h"
#include "hidmain.h"
#include "../../CNC_firmware/usbHostProtocol.h"
#if defined( __PIC32MX__ )
#include <plib.h>
#endif


// *****************************************************************************
// *****************************************************************************
// Constants
// *****************************************************************************
// *****************************************************************************

#if defined( __C30__)
#define ADC_READING_POTENTIOMETER       ADC1BUF1
#define ADC_READING_TEMPERATURE         ADC1BUF0
#define ADC_READING_VBUS                ADC1BUF2
#define SCAN_MASK                   0x0130      // Mask for AN4, AN5, and AN8
#elif defined( __PIC32MX__ )
#define ADC_READING_POTENTIOMETER   ADC1BUF0
#define ADC_READING_TEMPERATURE     ADC1BUF1
#define ADC_READING_VBUS            ADC1BUF2
#define SCAN_MASK               0x0114      // Mask for AN2, AN5 and AN8
#else
#error Unsupported Processor
#endif
#define COPY_BUFFER_SIZE                512         // Increase to improve copy efficiency
#define MAX_ALLOWED_CURRENT             500         // Maximum power we can supply in mA
#define MAX_BUFFERED_COMMANDS           8           // Must be a power of 2
#define MAX_COMMAND_LENGTH              50
#define MAX_LOG_BUFFER_SIZE             512
#define NUM_LOG_BUFFERS                 2
#define VERSION                         "v1.0"

// RTCC Format Correlation Constants
// The 16-bit and 32-bit architectures use two different formats for the RTCC.
// These constants can be used to access the byte in a four-byte date or time
// array.  Note: No constant is given for the weekday, because the weekday is
// stored in a different byte.  This is done to flag the user to take special
// care when writing code that utilizes the weekday.

#if defined( __C30__ )
#define DEFAULT_YEARS               0x0010
#define DEFAULT_MONTH_DAY           0x0701
#define DEFAULT_WEEKDAY_HOURS       0x0710
#define DEFAULT_MINUTES_SECONDS     0x1000

#define INDEX_HOURS                 2
#define INDEX_MINUTES               1
#define INDEX_SECONDS               0
#define INDEX_YEAR                  2
#define INDEX_MONTH                 1
#define INDEX_DAY                   0
#elif defined (__PIC32MX__ )
#define PIC32MX_TIMER2_INTERRUPT    0x00000100
#define PIC32MX_TIMER3_INTERRUPT    0x00001000

#define DEFAULT_DATE                0x07081503
#define DEFAULT_TIME                0x04210000

#define INDEX_HOURS                 3
#define INDEX_MINUTES               2
#define INDEX_SECONDS               1
#define INDEX_YEAR                  3
#define INDEX_MONTH                 2
#define INDEX_DAY                   1
#else
#error Cannot set up RTCC constants
#endif

// We are taking Timer 3 for an acquisition timer.

// NOTE - The datasheet doesn't state this, but the timer does get reset to 0
// after a period register match.  So we don't have to worry about resetting
// the timer manually.

#define STOP_TIMER_IN_IDLE_MODE         0x2000
#define TIMER_SOURCE_INTERNAL           0x0000
#define TIMER_ON                        0x8000
#define GATED_TIME_DISABLED             0x0000
#define TIMER_16BIT_MODE                0x0000

#if defined( __C30__ )
#define TIMER_PRESCALER_1               0x0000
#define TIMER_PRESCALER_8               0x0010
#define TIMER_PRESCALER_64              0x0020
#define TIMER_PRESCALER_256             0x0030
#define TIMER_INTERRUPT_PRIORITY        0x0002
#elif defined( __PIC32MX__ )
#define TIMER_PRESCALER_1               0x0000
#define TIMER_PRESCALER_2               0x0010
#define TIMER_PRESCALER_4               0x0020
#define TIMER_PRESCALER_8               0x0030
#define TIMER_PRESCALER_16              0x0040
#define TIMER_PRESCALER_32              0x0050
#define TIMER_PRESCALER_64              0x0060
#define TIMER_PRESCALER_256             0x0070
#else
#error No timer constants
#endif


// *****************************************************************************
// *****************************************************************************
// Data Structures
// *****************************************************************************
// *****************************************************************************

typedef struct _COMMAND {
    char buffer[MAX_COMMAND_LENGTH];
    BYTE index;
    BYTE command;
    BYTE escFirstChar;

    struct {
        BYTE reading : 1;
        BYTE escNeedFirstChar : 1;
        BYTE escNeedSecondChar : 1;
    };
} COMMAND;

typedef enum _COMMANDS {
    COMMAND_NO_COMMAND,
    COMMAND_ATTRIB,
    COMMAND_CD,
    COMMAND_COPY,
    COMMAND_DATE,
    COMMAND_DEL,
    COMMAND_DIR,
    COMMAND_HELP,
    COMMAND_MD,
    COMMAND_RD,
    COMMAND_REN,
    COMMAND_TIME,
    COMMAND_TYPE,
    COMMAND_STATE,
    COMMAND_UNKNOWN
} COMMANDS;

typedef struct _LOG_DATA {
    BYTE buffer[MAX_LOG_BUFFER_SIZE];
    WORD index;
    BOOL bufferFull;
} LOG_DATA;

typedef struct _LOGGER_STATUS {
#if defined( __PIC32MX__ )
    // PIC32 does not have atomic bit-field set/clear instructions, so
    // flags that are modified in both an ISR and main-line code must be
    // separate variables or we risk inadvertantly changing other flags
    // in the same byte.
    BOOL mediaPresent;

    union {
        BYTE value;

        struct {
            BYTE readingPotentiometer : 1;
            BYTE readingTemperature : 1;
        };
    };
#else

    union {
        BYTE value;

        struct {
            BYTE mediaPresent : 1;
            BYTE readingPotentiometer : 1;
            BYTE readingTemperature : 1;
        };
    };
#endif
} LOGGER_STATUS;

typedef struct _OLD_COMMANDS {
    char lines[MAX_BUFFERED_COMMANDS][MAX_COMMAND_LENGTH];
    BYTE oldest;
    BYTE newest;
    BYTE showing;
} OLD_COMMANDS;


#if defined( __C30__ )

// PIC24 RTCC Structure

typedef union {

    struct {
        unsigned char mday; // BCD codification for day of the month, 01-31
        unsigned char mon; // BCD codification for month, 01-12
        unsigned char year; // BCD codification for years, 00-99
        unsigned char reserved; // reserved for future use. should be 0
    }; // field access
    unsigned char b[4]; // byte access
    unsigned short w[2]; // 16 bits access
    unsigned long l; // 32 bits access
} PIC24_RTCC_DATE;

// PIC24 RTCC Structure

typedef union {

    struct {
        unsigned char sec; // BCD codification for seconds, 00-59
        unsigned char min; // BCD codification for minutes, 00-59
        unsigned char hour; // BCD codification for hours, 00-24
        unsigned char weekday; // BCD codification for day of the week, 00-06
    }; // field access
    unsigned char b[4]; // byte access
    unsigned short w[2]; // 16 bits access
    unsigned long l; // 32 bits access
} PIC24_RTCC_TIME;

#endif

typedef struct _VOLUME_INFO {
    char label[12];
    BYTE valid;
} VOLUME_INFO;


// *****************************************************************************
// *****************************************************************************
// Internal Function Prototypes
// *****************************************************************************
// *****************************************************************************

BYTE GetCommand(void);
DWORD GetCurrentTick(void);
void GetOneWord(char *buffer);
DWORD GetUserDate(void);
DWORD GetUserTime(void);
void InitializeAnalogMonitor(void);
void InitializeClock(void);
void InitializeCommand(void);
void MonitorMedia(void);
void MonitorUser(void);
void PrintFileInformation(SearchRec searchRecord);
void RedoCommandPrompt(void);
void ReplaceCommandLine(void);
void InitializeUSBMiniA(void);

#if defined( __C30__ )
DWORD PIC24RTCCGetTime(void);
DWORD PIC24RTCCGetDate(void);
void PIC24RTCCSetTime(WORD weekDay_hours, WORD minutes_seconds);
void PIC24RTCCSetDate(WORD xx_year, WORD month_day);
void UnlockRTCC(void);
#endif


// *****************************************************************************
// *****************************************************************************
// Macros
// *****************************************************************************
// *****************************************************************************

#define IsNum(c)            ((('0' <= c) && (c <= '9')) ? TRUE : FALSE)
#define UpperCase(c)        (('a'<= c) && (c <= 'z') ? c - 0x20 : c)
#define SkipWhiteSpace()    { while (commandInfo.buffer[commandInfo.index] == ' ') commandInfo.index++; }


// *****************************************************************************
// *****************************************************************************
// Global Variables
// *****************************************************************************
// *****************************************************************************

OLD_COMMANDS commandBuffer;
COMMAND commandInfo;
volatile BYTE logBufferReading;
BYTE logBufferWriting;
LOGGER_STATUS loggerStatus;
volatile DWORD currentTick;
volatile LOG_DATA logData[NUM_LOG_BUFFERS];
VOLUME_INFO volume;

#if defined(__C30__)
PIC24_RTCC_DATE currentDate;
PIC24_RTCC_TIME currentTime;
PIC24_RTCC_TIME previousTime;
#elif defined( __PIC32MX__ )
rtccDate currentDate;
rtccTime currentTime;
rtccTime previousTime;
#else
#error No time structure defined
#endif

// *****************************************************************************
// *****************************************************************************
// Configuration Bits
// *****************************************************************************
// *****************************************************************************
#if defined (__PIC24F__)
#define PLL_96MHZ_OFF   0xFFFF
#define PLL_96MHZ_ON    0xF7FF
_CONFIG2(FCKSM_CSECMD&IESO_OFF & PLL_96MHZ_ON & PLLDIV_DIV5 & FNOSC_PRIPLL & POSCMOD_HS) // Primary HS OSC with PLL, USBPLL /3
_CONFIG1(JTAGEN_OFF & ICS_PGx2 & FWDTEN_OFF) // JTAG off, watchdog timer off

#elif defined (__PIC32MX__)
// Define the device configuration
//  System Clock = 60 MHz,  Peripherial Bus = 8 MHz
//  Primary Osc w/PLL (XT+,HS+,EC+PLL)
//  Input Divider    2x Divider
//  Multiplier      15x Multiplier
//  WDT disabled
//  Other options are don't care
#pragma config UPLLEN   = ON            // USB PLL Enabled
#pragma config FPLLMUL  = MUL_15        // PLL Multiplier
#pragma config UPLLIDIV = DIV_5         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_5         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer 
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select
#pragma config DEBUG    = ON            // Debugger Enabled

#else
#error Cannot set up configuration bits.
#endif

void msTask() {
    FSFILE *filePointer1;
    FSFILE *filePointer2;
    char oneChar;
    char param1[MAX_COMMAND_LENGTH];
    char param2[MAX_COMMAND_LENGTH];
    unsigned char setAttribute, checkAttribute;
    SearchRec searchRecord;
    MonitorMedia();
    MonitorUser();

    if (GetCommand()) {
        SkipWhiteSpace();
        switch (commandInfo.command) {
            case COMMAND_NO_COMMAND:
                break;

            case COMMAND_ATTRIB:
                if (!loggerStatus.mediaPresent) {
                    UART2PrintString("\r\n");
                    UART2PrintString(UHP_ERROR);
                    UART2PrintString("No media present.\r\n");
                    break;
                }

                checkAttribute = ATTR_READ_ONLY | ATTR_HIDDEN | ATTR_SYSTEM | ATTR_ARCHIVE;
                setAttribute = 0x00;

                GetOneWord(param1);

                while ((param1[0] == '+' || param1[0] == '-') && setAttribute != 0xFF) {
                    switch (param1[1]) {
                        case 'R':
                            if (((checkAttribute & ATTR_READ_ONLY) == 0) || (param1[2] != 0)) {
                                UART2PrintString("\r\n");
                                UART2PrintString(UHP_ERROR);
                                UART2PrintString("Invalid switch - ");
                                UART2PutChar(param1[0]);
                                UART2PrintString("R\r\n");
                                setAttribute = 0xFF;
                            } else {
                                checkAttribute &= ~ATTR_READ_ONLY;
                                if (param1[0] == '+')
                                    setAttribute |= ATTR_READ_ONLY;
                            }
                            break;
                        case 'A':
                            if (((checkAttribute & ATTR_ARCHIVE) == 0) || (param1[2] != 0)) {
                                UART2PrintString("\r\n");
                                UART2PrintString(UHP_ERROR);
                                UART2PrintString("Invalid switch - ");
                                UART2PutChar(param1[0]);
                                UART2PrintString("A\r\n");
                                setAttribute = 0xFF;
                            } else {
                                checkAttribute &= ~ATTR_ARCHIVE;
                                if (param1[0] == '+')
                                    setAttribute |= ATTR_ARCHIVE;
                            }
                            break;
                        case 'H':
                            if (((checkAttribute & ATTR_HIDDEN) == 0) || (param1[2] != 0)) {
                                UART2PrintString("\r\n");
                                UART2PrintString(UHP_ERROR);
                                UART2PrintString("Invalid switch - ");
                                UART2PutChar(param1[0]);
                                UART2PrintString("H\r\n");
                                setAttribute = 0xFF;
                            } else {
                                checkAttribute &= ~ATTR_HIDDEN;
                                if (param1[0] == '+')
                                    setAttribute |= ATTR_HIDDEN;
                            }
                            break;
                        case 'S':
                            if (((checkAttribute & ATTR_SYSTEM) == 0) || (param1[2] != 0)) {
                                UART2PrintString("\r\n");
                                UART2PrintString(UHP_ERROR);
                                UART2PrintString("Invalid switch - ");
                                UART2PutChar(param1[0]);
                                UART2PrintString("S\r\n");
                                setAttribute = 0xFF;
                            } else {
                                checkAttribute &= ~ATTR_SYSTEM;
                                if (param1[0] == '+')
                                    setAttribute |= ATTR_SYSTEM;
                            }
                            break;
                        default:
                            UART2PrintString("\r\n");
                            UART2PrintString(UHP_ERROR);
                            UART2PrintString("Invalid switch - ");
                            UART2PutChar(param1[0]);
                            UART2PutChar(param1[1]);
                            UART2PrintString("\r\n");
                            setAttribute = 0xFF;
                            break;
                    }
                    GetOneWord(param1);
                }

                if (param1[0] == 0) {
                    UART2PrintString("\r\n");
                    UART2PrintString(UHP_ERROR);
                    UART2PrintString("Please enter a target file or directory.\r\n");
                    break;
                }

                if (setAttribute != 0xFF) {
                    if (!FindFirst(param1, ATTR_MASK & ~ATTR_VOLUME, &searchRecord)) {
                        if ((setAttribute == 0x00) && (checkAttribute == 0x27)) {
                            char buffer[50];
                            sprintf(buffer, "       %.42s", searchRecord.filename);
                            if (searchRecord.attributes & ATTR_READ_ONLY) {
                                buffer[0] = 'R';
                            }
                            if (searchRecord.attributes & ATTR_HIDDEN) {
                                buffer[1] = 'H';
                            }
                            if (searchRecord.attributes & ATTR_SYSTEM) {
                                buffer[2] = 'S';
                            }
                            if (searchRecord.attributes & ATTR_ARCHIVE) {
                                buffer[3] = 'A';
                            }
                            UART2PrintString(buffer);
                            UART2PrintString("\r\n");
                        } else {
                            if ((filePointer2 = FSfopen(param1, "r")) == NULL) {
                                UART2PrintString("\r\n");
                                UART2PrintString(UHP_ERROR);
                                UART2PrintString("Error opening file\r\n");
                                break;
                            }

                            if (checkAttribute & ATTR_READ_ONLY)
                                setAttribute |= (searchRecord.attributes & ATTR_READ_ONLY);
                            if (checkAttribute & ATTR_ARCHIVE)
                                setAttribute |= (searchRecord.attributes & ATTR_ARCHIVE);
                            if (checkAttribute & ATTR_SYSTEM)
                                setAttribute |= (searchRecord.attributes & ATTR_SYSTEM);
                            if (checkAttribute & ATTR_HIDDEN)
                                setAttribute |= (searchRecord.attributes & ATTR_HIDDEN);
                            if (FSattrib(filePointer2, setAttribute)) {
                                UART2PrintString("\r\n");
                                UART2PrintString(UHP_ERROR);
                                UART2PrintString("Could not set attributes\r\n");
                            } else {
                                if (FSfclose(filePointer2)) {
                                    UART2PrintString("\r\n");
                                    UART2PrintString(UHP_ERROR);
                                    UART2PrintString("Could not set attributes\r\n");
                                }
                            }
                        }
                    }
                }

                break;

            case COMMAND_CD:
                if (!loggerStatus.mediaPresent) {
                    UART2PrintString("\r\n");
                    UART2PrintString(UHP_ERROR);
                    UART2PrintString("No media present.\r\n");
                    break;
                }

                if (commandInfo.buffer[commandInfo.index] == 0) {
                    UART2PrintString("\r\n");
                    UART2PrintString(UHP_ERROR);
                    UART2PrintString("Parameter required\r\n");
                } else {
                    if (FSchdir(&(commandInfo.buffer[commandInfo.index]))) {
                        UART2PrintString("\r\n");
                        UART2PrintString(UHP_ERROR);
                        UART2PrintString("Error performing command\r\n");
                    }
                }
                break;

            case COMMAND_COPY:
                if (!loggerStatus.mediaPresent) {
                    UART2PrintString("\r\n");
                    UART2PrintString(UHP_ERROR);
                    UART2PrintString("No media present.\r\n");
                    break;
                }

                GetOneWord(param1);
                GetOneWord(param2);
                if ((param1[0] == 0) || (param2[0] == 0)) {
                    UART2PrintString("\r\n");
                    UART2PrintString(UHP_ERROR);
                    UART2PrintString("Two parameters required\r\n");
                } else {
                    if (FindFirst(param1,
                            ATTR_DIRECTORY | ATTR_ARCHIVE | ATTR_READ_ONLY | ATTR_HIDDEN, &searchRecord)) {
                        UART2PrintString("\r\n");
                        UART2PrintString(UHP_ERROR);
                        UART2PrintString("File not found\r\n");
                    } else if ((filePointer1 = FSfopen(param1, "r")) == NULL) {
                        UART2PrintString("\r\n");
                        UART2PrintString(UHP_ERROR);
                        UART2PrintString("Error opening file\r\n");
                    } else {
                        if ((filePointer2 = FSfopen(param2, "w")) == NULL) {
                            UART2PrintString("\r\n");
                            UART2PrintString(UHP_ERROR);
                            UART2PrintString("Error creating file\r\n");
                        } else {
                            size_t charsRead;
                            BYTE readBuffer[COPY_BUFFER_SIZE];

                            while (!FSfeof(filePointer1)) {
                                charsRead = FSfread(readBuffer, 1, COPY_BUFFER_SIZE, filePointer1);
                                if (charsRead) {
                                    if (FSfwrite(readBuffer, 1, charsRead, filePointer2) != charsRead) {
                                        UART2PrintString("\r\n");
                                        UART2PrintString(UHP_ERROR);
                                        UART2PrintString("Error writing file\r\n");
                                        break;
                                    }
                                }
                            }
                            FSfclose(filePointer2);
                        }
                        FSfclose(filePointer1);
                    }
                }
                break;

            case COMMAND_DATE:
                if (commandInfo.buffer[commandInfo.index] == 0) {
                    UART2PrintString("Current date: ");

#if defined( __C30__)
                    currentDate.l = PIC24RTCCGetDate();
#elif defined( __PIC32MX__ )
                    currentDate.l = RtccGetDate();
#else
#error Need DATE implementation.
#endif

                    UART2PutChar('2');
                    UART2PutChar('0');
                    UART2PutHex(currentDate.year);
                    UART2PutChar('-');
                    UART2PutHex(currentDate.mon);
                    UART2PutChar('-');
                    UART2PutHex(currentDate.mday);
                    UART2PrintString("\r\n");
                } else {
                    // Set the current date.
                    DWORD_VAL date;

                    date.Val = GetUserDate();

                    if (date.Val) {
#if defined( __C30__)
                        PIC24RTCCSetDate(date.w[1], date.w[0]);
#elif defined( __PIC32MX__ )
                        RtccSetDate(date.Val);
                        RtccEnable();
#else
#error Need DATE implementation.
#endif
                    } else {
                        UART2PrintString("\r\n");
                        UART2PrintString(UHP_ERROR);
                        UART2PrintString("Invalid date specified\r\n");
                    }
                }
                break;

            case COMMAND_DEL:
                if (!loggerStatus.mediaPresent) {
                    UART2PrintString("\r\n");
                    UART2PrintString(UHP_ERROR);
                    UART2PrintString("No media present.\r\n");
                    break;
                }

                if (commandInfo.buffer[commandInfo.index] == 0) {
                    UART2PrintString("\r\n");
                    UART2PrintString(UHP_ERROR);
                    UART2PrintString("Parameter required\r\n");
                } else {
                    if (FindFirst(&(commandInfo.buffer[commandInfo.index]),
                            ATTR_DIRECTORY | ATTR_ARCHIVE | ATTR_READ_ONLY | ATTR_HIDDEN, &searchRecord)) {
                        UART2PrintString("\r\n");
                        UART2PrintString(UHP_ERROR);
                        UART2PrintString("File not found\r\n");
                    } else {
                        if (FSremove(&(commandInfo.buffer[commandInfo.index]))) {
                            UART2PrintString("\r\n");
                            UART2PrintString(UHP_ERROR);
                            UART2PrintString("Error performing command\r\n");
                        }
                    }
                }
                break;

            case COMMAND_DIR:
                if (!loggerStatus.mediaPresent) {
                    UART2PrintString("\r\n");
                    UART2PrintString(UHP_ERROR);
                    UART2PrintString("No media present.\r\n");
                    break;
                }

                if (commandInfo.buffer[commandInfo.index] == 0) {
                    strcpy(param1, "*.*");
                } else {
                    strcpy(param1, &(commandInfo.buffer[commandInfo.index]));
                }
                if (!FindFirst(param1, ATTR_DIRECTORY | ATTR_ARCHIVE | ATTR_READ_ONLY | ATTR_HIDDEN, &searchRecord)) {
                    PrintFileInformation(searchRecord);
                    while (!FindNext(&searchRecord)) {
                        PrintFileInformation(searchRecord);
                    }
                } else {
                    UART2PrintString("\r\n");
                    UART2PrintString(UHP_ERROR);
                    UART2PrintString("No files found.\r\n");
                }
                break;

            case COMMAND_HELP:
                UART2PrintString("\r\nSure Electronics USB To UART ");
                UART2PrintString(VERSION);
                UART2PrintString("\r\nModifed for CNC by LuCiFer");
                UART2PrintString("\r\n\r\nAvailable commands:\r\n");
                UART2PrintString("    ATTRIB <+|-><R|S|H|A> <name> - change attributes specified\r\n");
                UART2PrintString("    CD <name>                    - change directory\r\n");
                UART2PrintString("    COPY <file1> <file2>         - copy [file1] to [file2]\r\n");
                UART2PrintString("    DATE [yyyy-mm-dd]            - display or set the date\r\n");
                UART2PrintString("    DEL <file>                   - delete file, current directory only\r\n");
                UART2PrintString("    DIR [file]                   - display directory\r\n");
                UART2PrintString("    HELP or ?                    - display help\r\n");
                UART2PrintString("    MD <name>                    - make directory\r\n");
                UART2PrintString("    RD <name>                    - remove directory\r\n");
                UART2PrintString("    REN <file1> <file2>          - rename [file1] to [file2]\r\n");
                UART2PrintString("    TIME [hh:mm:ss]              - display or set the time (24 hr format)\r\n");
                UART2PrintString("    TYPE <file>                  - print out contents of file\r\n");
                break;
            case COMMAND_STATE:
                if(HID_isConnected()==DEVICE_KEYBOARD) {
                    UART2PrintString("\r\n");
                    UART2PrintString(UHP_KEYBOARD_CONNECTED);
                    UART2PrintString("\r\n");
                } else if(HID_isConnected()==DEVICE_MOUSE) {
                    UART2PrintString("\r\n");
                    UART2PrintString(UHP_MOUSE_CONNECTED);
                    UART2PrintString("\r\n");
                } else if(loggerStatus.mediaPresent) {
                    char buffer[50];
                    buffer[0] = 0;
                    FSgetcwd(buffer, 50);
                    UART2PrintString("\r\n");
                    UART2PrintString(UHP_PATH);
                    UART2PrintString(buffer);
                    UART2PrintString("\r\n");
                } else {
                    UART2PrintString("\r\n");
                    UART2PrintString(UHP_WAITFORDEVICE);
                    UART2PrintString("\r\n");
                }
                break;

            case COMMAND_MD:
                if (!loggerStatus.mediaPresent) {
                    UART2PrintString("\r\n");
                    UART2PrintString(UHP_ERROR);
                    UART2PrintString("No media present.\r\n");
                    break;
                }

                if (commandInfo.buffer[commandInfo.index] == 0) {
                    UART2PrintString("\r\n");
                    UART2PrintString(UHP_ERROR);
                    UART2PrintString("Parameter required\r\n");
                } else {
                    if (FSmkdir(&(commandInfo.buffer[commandInfo.index]))) {
                        UART2PrintString("\r\n");
                        UART2PrintString(UHP_ERROR);
                        UART2PrintString("Error performing command\r\n");
                    }
                }
                break;

            case COMMAND_RD:
                if (!loggerStatus.mediaPresent) {
                    UART2PrintString("\r\n");
                    UART2PrintString(UHP_ERROR);
                    UART2PrintString("No media present.\r\n");
                    break;
                }

                if (commandInfo.buffer[commandInfo.index] == 0) {
                    UART2PrintString("\r\n");
                    UART2PrintString(UHP_ERROR);
                    UART2PrintString("Parameter required\r\n");
                } else {
                    if (FSrmdir(&(commandInfo.buffer[commandInfo.index]), TRUE)) {
                        UART2PrintString("\r\n");
                        UART2PrintString(UHP_ERROR);
                        UART2PrintString("Error performing command\r\n");
                    }
                }
                break;

            case COMMAND_REN:
                if (!loggerStatus.mediaPresent) {
                    UART2PrintString("\r\n");
                    UART2PrintString(UHP_ERROR);
                    UART2PrintString("No media present.\r\n");
                    break;
                }

                GetOneWord(param1);
                GetOneWord(param2);
                if ((param1[0] == 0) || (param2[0] == 0)) {
                    UART2PrintString("\r\n");
                    UART2PrintString(UHP_ERROR);
                    UART2PrintString("Two parameters required\r\n");
                } else {
                    if ((filePointer1 = FSfopen(param1, "r")) == NULL) {
                        UART2PrintString("\r\n");
                        UART2PrintString(UHP_ERROR);
                        UART2PrintString("Cannot find file\r\n");
                    } else {
                        if (FSrename(param2, filePointer1)) {
                            UART2PrintString("\r\n");
                            UART2PrintString(UHP_ERROR);
                            UART2PrintString("Error performing command\r\n");
                        }
                        FSfclose(filePointer1);
                    }
                }
                break;

            case COMMAND_TIME:
                if (commandInfo.buffer[commandInfo.index] == 0) {
                    // Display the current time.
                    UART2PrintString("Current time: ");

#if defined( __C30__)
                    currentTime.l = PIC24RTCCGetTime();
#elif defined( __PIC32MX__ )
                    currentTime.l = RtccGetTime();
#else
#error Need TIME implementation.
#endif

                    UART2PutHex(currentTime.hour);
                    UART2PutChar(':');
                    UART2PutHex(currentTime.min);
                    UART2PutChar(':');
                    UART2PutHex(currentTime.sec);
                    UART2PrintString("\r\n");
                } else {
                    // Set the current date.
                    DWORD_VAL time;

                    time.Val = GetUserTime();

                    if (time.Val) {
#if defined( __C30__)
                        PIC24RTCCSetTime(time.w[1], time.w[0]);
#elif defined( __PIC32MX__ )
                        RtccSetTime(time.Val);
                        RtccEnable();
#else
#error Need TIME implementation.
#endif
                    } else {
                        UART2PrintString("\r\n");
                        UART2PrintString(UHP_ERROR);
                        UART2PrintString("Invalid time specified\r\n");
                    }
                }
                break;

            case COMMAND_TYPE:
                if (!loggerStatus.mediaPresent) {
                    UART2PrintString("\r\n");
                    UART2PrintString(UHP_ERROR);
                    UART2PrintString("No media present.\r\n");
                    break;
                }

                if (commandInfo.buffer[commandInfo.index] == 0) {
                    UART2PrintString(UHP_ERROR);
                    UART2PrintString("Parameter required\r\n");
                } else {
                    if (FindFirst(&(commandInfo.buffer[commandInfo.index]),
                            ATTR_DIRECTORY | ATTR_ARCHIVE | ATTR_READ_ONLY | ATTR_HIDDEN, &searchRecord)) {
                        UART2PrintString("\r\n");
                        UART2PrintString(UHP_ERROR);
                        UART2PrintString("File not found\r\n");
                    } else if ((filePointer1 = FSfopen(&(commandInfo.buffer[commandInfo.index]), "r")) == NULL) {
                        UART2PrintString("\r\n");
                        UART2PrintString(UHP_ERROR);
                        UART2PrintString("Error opening file\r\n");
                    } else {
                        UART2PrintString(UHP_FILELINE);
                        UART2PrintString("\r\n");
                        unsigned short count = 0;
                        char ibreak = 1, tmp;
                        while (!FSfeof(filePointer1)) {
                            // This is just a quick example.  It's not very efficient to
                            // read only one byte at a time!
                            if (FSfread(&oneChar, 1, 1, filePointer1) == 1) {
                                if (count > 500 || oneChar == 0) {
                                    ibreak = 0;
                                    UART2PrintString("\r\n");
                                    UART2PrintString(UHP_ERROR);
                                    UART2PrintString("Not a text file\r\n");
                                    break;
                                }
                                while (U2STAbits.URXDA) tmp = U2RXREG;
                                UART2PutChar(oneChar);
                                if(oneChar != '\n' && oneChar != '\r')
                                    count++;
                                if (oneChar == '\n' && count>0) {
                                    while (!U2STAbits.URXDA);
                                    oneChar = U2RXREG;
                                    if (oneChar == '\r' || oneChar == '\n') {
                                        count = 0;
                                    } else {
                                        ibreak = 0;
                                        break;
                                    }
                                    while (U2STAbits.URXDA) oneChar = U2RXREG;
                                }
                            } else {
                                ibreak = 0;
                                UART2PrintString("\r\n");
                                UART2PrintString(UHP_ERROR);
                                UART2PrintString("Error reading file\r\n");
                                break;
                            }
                        }

                        UART2PrintString("\r\n");
                        if(ibreak) {
                            while (!U2STAbits.URXDA);
                            while (U2STAbits.URXDA) oneChar = U2RXREG;
                        }
                        UART2PrintString(UHP_FILEDONE);
                        UART2PrintString("\r\n");
                        FSfclose(filePointer1);
                    }
                }
                break;

            default:
                UART2PrintString("\r\n");
                UART2PrintString(UHP_ERROR);
                UART2PrintString("Unsupported command\r\n");
                break;
        }
        InitializeCommand();
    }
}
//******************************************************************************
//******************************************************************************
//******************************************************************************
// Main
//******************************************************************************
//******************************************************************************
//******************************************************************************


BOOL bSomethingInserted = FALSE;
BOOL bLastSomethingInserted = FALSE;
#define FCY 32000000UL
#include <libpic30.h>
int main(void) {
    __delay_ms(750);// delay to allow stm32 start before pic
    //int                 value;

#if defined(__C30__)
    OSCCON = 0x3302; // Enable secondary oscillator
    CLKDIV = 0x0000; // Set PLL prescaler (1:1)

#elif defined(__PIC32MX__)

#if defined(RUN_AT_60MHZ)
    // Use OSCCON default
#else
    OSCCONCLR = 0x38000000; //PLLODIV
#if defined(RUN_AT_48MHZ)
    OSCCONSET = 0x08000000; //PLLODIV /2
#elif defined(RUN_AT_24MHZ)
    OSCCONSET = 0x10000000; //PLLODIV /4
#else
#error Cannot set OSCCON
#endif
#endif

    value = SYSTEMConfigWaitStatesAndPB(GetSystemClock());

    // Enable the cache for the best performance
    CheKseg0CacheOn();

    INTEnableSystemMultiVectoredInt();

    value = OSCCON;
    while (!(value & 0x00000020)) {
        value = OSCCON; // Wait for PLL lock to stabilize
    }

    INTEnableInterrupts();

#else
#error Cannot initialize
#endif

    // Config Port to UART2
#if defined (__C30__)
    //Assign U2RX To Pin RP23,RD3
    RPINR19bits.U2RXR = 23;
    //Assign U2TX To Pin RP22,RD2
    RPOR11bits.RP22R = 5;
#endif

    UART2Init();

    // Initialize the RTCC
    // Turn on the secondary oscillator
#if defined( __C30__)
    __asm__("MOV #OSCCON,w1");
    __asm__("MOV.b #0x02, w0");
    __asm__("MOV #0x46, w2");
    __asm__("MOV #0x57, w3");
    __asm__("MOV.b w2, [w1]");
    __asm__("MOV.b w3, [w1]");
    __asm__("MOV.b w0, [w1]");

    PIC24RTCCSetDate(DEFAULT_YEARS, DEFAULT_MONTH_DAY);
    PIC24RTCCSetTime(DEFAULT_WEEKDAY_HOURS, DEFAULT_MINUTES_SECONDS);

    RCFGCAL = 0x8000;
#elif defined( __PIC32MX__)
    RtccInit();
    RtccSetDate(DEFAULT_DATE);
    RtccSetTime(DEFAULT_TIME);
    RtccEnable();
    mRtccWrEnable();
    while (RtccEnable() != RTCC_CLK_ON); // Make sure the RTCC is counting.
    //PutChar('~');
    mRtccWrDisable();
#else
#error No Real Time clock
#endif

    // Turn on Power LED, Red LED, D2
    TRISBbits.TRISB13 = 0;
    _RB13 = 0;

    // Turn on the timer and the A/D converter to monitor inputs.
    InitializeAnalogMonitor();
    InitializeClock();

    commandBuffer.newest = MAX_BUFFERED_COMMANDS;
    commandBuffer.oldest = MAX_BUFFERED_COMMANDS;
    commandBuffer.showing = MAX_BUFFERED_COMMANDS;
    loggerStatus.value = 0;
    volume.valid = FALSE;
    InitializeCommand();
    USBInitialize(0);

    while (1) {
        hidTask();
        msTask();
        bSomethingInserted = loggerStatus.mediaPresent | HID_isConnected() != DEVICE_NOT_CONNECTED_OR_NOT_SUPPORTED;
        if (!bSomethingInserted && bSomethingInserted != bLastSomethingInserted) {
            UART2PrintString("\r\n");
            UART2PrintString(UHP_WAITFORDEVICE);
            UART2PrintString("\r\n");
        }
        bLastSomethingInserted = bSomethingInserted;
    }
}

//******************************************************************************
//******************************************************************************
// Internal Functions
//******************************************************************************
//******************************************************************************

/****************************************************************************
  Function:
    void EraseCommandLine( void )

  Description:
    This function erases the current command line.  It works by sending a
    backspace-space-backspace combination for each character on the line.

  Precondition:
    commandInfo.index must be valid.

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
 ***************************************************************************/

void EraseCommandLine(void) {
    BYTE i;
    BYTE lineLength;

    lineLength = commandInfo.index;
    for (i = 0; i < lineLength; i++) {
        UART2PutChar(0x08);
        UART2PutChar(' ');
        UART2PutChar(0x08);
    }
    commandInfo.index = 0;
}

/****************************************************************************
  Function:
    BYTE GetCommand( void )

  Description:
    This function returns whether or not the user has finished entering a
    command.  If so, then the command entered by the user is determined and
    placed in commandInfo.command.  The command line index
    (commandInfo.index) is set to the first non-space character after the
    command.

  Precondition:
    commandInfo.reading must be valid.

  Parameters:
    None

  Return Values:
    TRUE    - The user has entered a command.  The command is in
                  commandInfo.command.
    FALSE   - The user has not finished entering a command.

  Remarks:
    None
 ***************************************************************************/

BYTE GetCommand(void) {
    char firstWord[MAX_COMMAND_LENGTH];

    if (commandInfo.reading) {
        return FALSE;
    } else {
        commandInfo.index = 0;

        commandInfo.index = 0;
        GetOneWord(firstWord);
        SkipWhiteSpace();

        if (firstWord[0] == 0) {
            commandInfo.command = COMMAND_NO_COMMAND;
            return TRUE;
        }

        if (!strncmp(firstWord, "ATTRIB", 6) && (strlen(firstWord) == 6)) {
            commandInfo.command = COMMAND_ATTRIB;
            return TRUE;
        }
        if (!strncmp(firstWord, "CD", 2) && (strlen(firstWord) == 2)) {
            commandInfo.command = COMMAND_CD;
            return TRUE;
        }
        if (!strncmp(firstWord, "COPY", 4) && (strlen(firstWord) == 4)) {
            commandInfo.command = COMMAND_COPY;
            return TRUE;
        }
        if (!strncmp(firstWord, "DATE", 4) && (strlen(firstWord) == 4)) {
            commandInfo.command = COMMAND_DATE;
            return TRUE;
        }
        if (!strncmp(firstWord, "DEL", 3) && (strlen(firstWord) == 3)) {
            commandInfo.command = COMMAND_DEL;
            return TRUE;
        }
        if (!strncmp(firstWord, "DIR", 3) && (strlen(firstWord) == 3)) {
            commandInfo.command = COMMAND_DIR;
            return TRUE;
        }
        if (!strncmp(firstWord, "STATE", 5) && (strlen(firstWord) == 5)) {
            commandInfo.command = COMMAND_STATE;
            return TRUE;
        }
        if ((!strncmp(firstWord, "HELP", 4) && (strlen(firstWord) == 4)) ||
                (!strncmp(firstWord, "?", 1) && (strlen(firstWord) == 1))) {
            commandInfo.command = COMMAND_HELP;
            return TRUE;
        }
        if (!strncmp(firstWord, "MD", 2) && (strlen(firstWord) == 2)) {
            commandInfo.command = COMMAND_MD;
            return TRUE;
        }
        if (!strncmp(firstWord, "RD", 2) && (strlen(firstWord) == 2)) {
            commandInfo.command = COMMAND_RD;
            return TRUE;
        }
        if (!strncmp(firstWord, "REN", 3) && (strlen(firstWord) == 3)) {
            commandInfo.command = COMMAND_REN;
            return TRUE;
        }
        if (!strncmp(firstWord, "TIME", 4) && (strlen(firstWord) == 4)) {
            commandInfo.command = COMMAND_TIME;
            return TRUE;
        }
        if (!strncmp(firstWord, "TYPE", 4) && (strlen(firstWord) == 4)) {
            commandInfo.command = COMMAND_TYPE;
            return TRUE;
        }
        commandInfo.command = COMMAND_UNKNOWN;
        return TRUE;
    }
}

/****************************************************************************
  Function:
    DWORD GetCurrentTick( void )

  Description:
    This function gets the current tick count in terms of milliseconds.
    Note that the timer is only accurate to MILLISECONDS_PER_TICK ms.

  Precondition:
    None

  Parameters:
    None

  Returns:
    DWORD - Number of milliseconds since the timer was turned on.

  Remarks:

 ***************************************************************************/
DWORD GetCurrentTick(void) {
    return (currentTick * MILLISECONDS_PER_TICK);
}

/****************************************************************************
  Function:
    void GetOneWord( char *buffer )

  Description:
    This function copies the next word in the command line to the specified
    buffer.  Word deliniation is marked by a space character.  The returned
    word is null terminated.

  Precondition:
    commandInfo.buffer and commandInfo.index are valid

  Parameters:
 *buffer - Pointer to where the word is to be stored.

  Returns:
    None

  Remarks:

 ***************************************************************************/
void GetOneWord(char *buffer) {
    SkipWhiteSpace();

    while ((commandInfo.buffer[commandInfo.index] != 0) &&
            (commandInfo.buffer[commandInfo.index] != ' ')) {
        *buffer++ = commandInfo.buffer[commandInfo.index++];
    }
    *buffer = 0;
}

/****************************************************************************
  Function:
    DWORD GetUserDate( void )

  Description:
    This function extracts a user entered date from the command line and
    places it in a DWORD that matches the format required for the RTCC.
    The required format is:
                        YYYY-MM-DD
    where YY is between 2000 and 2099.  If the date is not in a valid format,
    0 is returned.

  Precondition:
    commandInfo.buffer and commandInfo.index are valid, and
    commandInfo.index points to the first character of the date

  Parameters:


  Returns:
    If the project is built for a PIC24F, this function returns a DWORD in
    the format <00><YY><MM><DD>.  If it is built for a PIC32MX, this function
    returns a DWORD in the format <YY><MM><DD><00>.

  Remarks:
    Range checks are not comprehensive.  The day is not qualified based on
    how many days are in the specified month.

    The values from the RTCC are assumed to be in BCD format.

    The two architectures have different formats for the date.  The index
    values are set above accordingly.
 ***************************************************************************/

DWORD GetUserDate(void) {
    DWORD_VAL date;

    date.Val = 0;

    // Get the year.
    if (commandInfo.buffer[commandInfo.index++] != '2') return 0;
    if (commandInfo.buffer[commandInfo.index++] != '0') return 0;

    if (!IsNum(commandInfo.buffer[commandInfo.index])) return 0;
    date.v[INDEX_YEAR] = (commandInfo.buffer[commandInfo.index++] - '0') << 4;
    if (!IsNum(commandInfo.buffer[commandInfo.index])) return 0;
    date.v[INDEX_YEAR] |= (commandInfo.buffer[commandInfo.index++] - '0');

    if (commandInfo.buffer[commandInfo.index++] != '-') return 0;

    // Get the month.
    if (!IsNum(commandInfo.buffer[commandInfo.index])) return 0;
    date.v[INDEX_MONTH] = (commandInfo.buffer[commandInfo.index++] - '0') << 4;
    if (!IsNum(commandInfo.buffer[commandInfo.index])) return 0;
    date.v[INDEX_MONTH] |= (commandInfo.buffer[commandInfo.index++] - '0');
    if (!((0x01 <= date.v[INDEX_MONTH]) && (date.v[INDEX_MONTH] <= 0x12))) return 0;

    if (commandInfo.buffer[commandInfo.index++] != '-') return 0;

    // Get the day.
    if (!IsNum(commandInfo.buffer[commandInfo.index])) return 0;
    date.v[INDEX_DAY] = (commandInfo.buffer[commandInfo.index++] - '0') << 4;
    if (!IsNum(commandInfo.buffer[commandInfo.index])) return 0;
    date.v[INDEX_DAY] |= (commandInfo.buffer[commandInfo.index++] - '0');
    if (!((0x01 <= date.v[INDEX_DAY]) && (date.v[INDEX_DAY] <= 0x31))) return 0;

    return date.Val;
}

/****************************************************************************
  Function:
    DWORD GetUserTime( void )

  Description:
    This function extracts a user entered time from the command line and
    places it in a DWORD that matches the format required for the RTCC.  The
    required format is:
                        HH:MM:SS
    in 24-hour format.  If the time is not in a valid format, 0 is returned.

  Precondition:
    commandInfo.buffer and commandInfo.index are valid;
    commandInfo.index points to the first character of the time

  Parameters:
    None

  Return Values:
    If the project is built for a PIC24F, this function returns a DWORD in
    the format <00><HH><MM><SS>.  If it is built for a PIC32MX, this function
    returns a DWORD in the format <HH><MM><SS><00>.

  Remarks:
    The values from the RTCC are assumed to be in BCD format.

    The two architectures have different formats for the date. The index
    values are set above accordingly.
 ***************************************************************************/

DWORD GetUserTime(void) {
    DWORD_VAL time;

    time.Val = 0;

    // Get the hours.
    if (!IsNum(commandInfo.buffer[commandInfo.index])) return 0;
    time.v[INDEX_HOURS] |= (commandInfo.buffer[commandInfo.index++] - '0') << 4;
    if (!IsNum(commandInfo.buffer[commandInfo.index])) return 0;
    time.v[INDEX_HOURS] |= (commandInfo.buffer[commandInfo.index++] - '0');
    if (time.v[INDEX_HOURS] > 0x23) return 0;

    if (commandInfo.buffer[commandInfo.index++] != ':') return 0;

    // Get the minutes.
    if (!IsNum(commandInfo.buffer[commandInfo.index])) return 0;
    time.v[INDEX_MINUTES] |= (commandInfo.buffer[commandInfo.index++] - '0') << 4;
    if (!IsNum(commandInfo.buffer[commandInfo.index])) return 0;
    time.v[INDEX_MINUTES] |= (commandInfo.buffer[commandInfo.index++] - '0');
    if (time.v[INDEX_MINUTES] > 0x59) return 0;

    if (commandInfo.buffer[commandInfo.index++] != ':') return 0;

    // Get the seconds.
    if (!IsNum(commandInfo.buffer[commandInfo.index])) return 0;
    time.v[INDEX_SECONDS] |= (commandInfo.buffer[commandInfo.index++] - '0') << 4;
    if (!IsNum(commandInfo.buffer[commandInfo.index])) return 0;
    time.v[INDEX_SECONDS] |= (commandInfo.buffer[commandInfo.index++] - '0');
    if (time.v[INDEX_SECONDS] > 0x59) return 0;

    return time.Val;
}

/****************************************************************************
  Function:
     void InitializeClock( void )

  Description:
    This function initializes the tick timer.  It configures and starts the
    timer, and enables the timer interrupt.  We are using Timer 3 so we can
    also trigger an A/D conversion when the timer rolls over.

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
 ***************************************************************************/

void InitializeClock(void) {
    currentTick = 0;

#if defined( __C30__ )
    IPC2bits.T3IP = TIMER_INTERRUPT_PRIORITY;
    IFS0bits.T3IF = 0;

    TMR3 = 0;
    PR3 = TIMER_PERIOD;
    T3CON = TIMER_ON | STOP_TIMER_IN_IDLE_MODE | TIMER_SOURCE_INTERNAL |
            GATED_TIME_DISABLED | TIMER_16BIT_MODE | TIMER_PRESCALER;

    IEC0bits.T3IE = 1;
#elif defined( __PIC32MX__ )
    //OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_8, TIMER_PERIOD);
    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_16, TIMER_PERIOD);

    // set up the timer interrupt with a priority of 2
    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2);
#else
#error Cannot initialize timer.
#endif

    return;
}

/****************************************************************************
  Function:
    void InitializeCommand( void )

  Description:
    This function prints a command prompt and initializes the command line
    information.  If available, the command prompt format is:
                    [Volume label]:[Current directory]>

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
 ***************************************************************************/

void InitializeCommand(void) {
    char buffer[50];

    if (loggerStatus.mediaPresent) {
        buffer[0] = 0;
        //        if (volume.valid)
        //        {
        //            UART2PrintString( volume.label );
        //            UART2PutChar( ':' );
        //        }
        if(FSgetcwd(buffer, 50)!=NULL) {
            UART2PrintString("\r\n");
            UART2PrintString(UHP_PATH);
            UART2PrintString(buffer);
            UART2PrintString("\r\n");
            UART2PrintString(":");
            UART2PrintString(buffer);
        }
    }

    UART2PrintString("> ");

    commandInfo.command = COMMAND_NO_COMMAND;
    commandInfo.index = 0;
    commandInfo.reading = TRUE;

    memset(commandInfo.buffer, 0x00, MAX_COMMAND_LENGTH);
}

/****************************************************************************
  Function:
     void InitializeAnalogMonitor( void )

  Description:
    This function initializes the monitoring of the three analog inputs.
    Since we need to monitor Vbus while doing other operations, we will
    simply scan all required input channels: the temperature sensor (AN4),
    the potentiometer (AN5), and Vbus (AN8).

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
 ***************************************************************************/

void InitializeAnalogMonitor(void) {
    // Set up the A/D converter

    AD1PCFG &= ~SCAN_MASK; // Disable digital input on AN4, AN5, AN8
    AD1CSSL = SCAN_MASK; // Scan AN4, AN5, AN8
    AD1CON3 = 0x1F05; // 31 Tad auto-sample, Tad = 5*Tcy
    AD1CON2 = 0x040C; // Scan inputs, 3 conversions per interrupt, MUX A only
    AD1CON1 = 0x8046; // Turn on, start sampling, convert on Timer 3

    // Enable A/D interrupts

#if defined( __C30__ )
    IEC0bits.AD1IE = 1;
#elif defined( __PIC32MX__ )
    IEC1SET = 0x00000002;
    ConfigIntADC10(ADC_INT_ON | ADC_INT_PRI_2);
#else
#error Cannot turn on ADC interrupt
#endif

    return;
}

/****************************************************************************
  Function:
    void MonitorMedia( void )

  Description:
    This function calls the background tasks necessary to support USB Host
    operation.  Upon initial insertion of the media, it initializes the file
    system support and reads the volume label.  Upon removal of the media,
    the volume label is marked invalid.

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
 ***************************************************************************/

void MonitorMedia(void) {
    BYTE mediaPresentNow;
    BYTE mountTries;
    SearchRec searchRecord;

    USBTasks();
    mediaPresentNow = MDD_MediaDetect();
    if (mediaPresentNow != loggerStatus.mediaPresent) {
        if (mediaPresentNow) {
            mountTries = 10;
            while (!FSInit() && mountTries--);
            if (!mountTries) {
                UART2PrintString("\r\n");
                UART2PrintString(UHP_ERROR);
                UART2PrintString("APP: Could not mount media\r\n");
                loggerStatus.mediaPresent = FALSE;
            } else {
                loggerStatus.mediaPresent = TRUE;

                // Find the volume label.  We need to do this here while we are at the
                // root directory.
                if (!FindFirst("*.*", ATTR_VOLUME, &searchRecord)) {
                    strcpy(volume.label, searchRecord.filename);
                    volume.valid = TRUE;
                }

                RedoCommandPrompt();
            }

            // Turn on USB Storage Power LED, Green LED, D3
            TRISBbits.TRISB14 = 0;
            _RB14 = 0;
        } else {
            // Turn off USB Storage Power LED, Green LED, D3
            TRISBbits.TRISB14 = 1;

            loggerStatus.mediaPresent = FALSE;
            volume.valid = FALSE;

            RedoCommandPrompt();
        }
    }
}

/****************************************************************************
  Function:
    void MonitorUser( void )

  Description:
    This routine monitors command line input from the user.  The
    UART is polled rather than using interrupts.  If the previous
    command line is still being processed, the characters are
    ignored.  All text input is converted to upper case for
    simplification.  When the user presses the Enter or Return key,
    the command line is considered terminated.  Escape sequences are
    captured and checked.  If the up-arrow is pressed, the previous
    buffered command is displayed, until the oldest command is
    reached.  If the down-arrow is pressed, the next buffered
    command is displayed, until the newest command is displayed.
    If the down-arrow is pressed one additional time, the command
    line is erased.

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    Currently, blank entries are added to the old command buffer.
 ***************************************************************************/

void MonitorUser(void) {
    char oneChar;

    if (U2STAbits.URXDA) {
        oneChar = U2RXREG;

        // If we are currently processing a command, throw the character away.
        if (commandInfo.reading) {
            if (commandInfo.escNeedSecondChar) {
                if (commandInfo.escFirstChar == 0x5B) {
                    if (oneChar == 0x41) // Up arrow
                    {
                        if (commandBuffer.showing != commandBuffer.oldest) {
                            if (commandBuffer.showing == MAX_BUFFERED_COMMANDS) {
                                commandBuffer.showing = commandBuffer.newest;
                            } else {
                                commandBuffer.showing = (commandBuffer.showing - 1) & (MAX_BUFFERED_COMMANDS - 1);
                            }
                        }
                        ReplaceCommandLine();
                    } else if (oneChar == 0x42) // Down arrow
                    {
                        if (commandBuffer.showing != MAX_BUFFERED_COMMANDS) {
                            if (commandBuffer.showing != commandBuffer.newest) {
                                commandBuffer.showing = (commandBuffer.showing + 1) & (MAX_BUFFERED_COMMANDS - 1);
                                ReplaceCommandLine();
                            } else {
                                EraseCommandLine();
                                commandBuffer.showing = MAX_BUFFERED_COMMANDS;
                            }
                        } else {
                            EraseCommandLine();
                        }
                    }
                }
                commandInfo.escNeedSecondChar = FALSE;
            } else if (commandInfo.escNeedFirstChar) {
                commandInfo.escFirstChar = oneChar;
                commandInfo.escNeedFirstChar = FALSE;
                commandInfo.escNeedSecondChar = TRUE;
            } else {
                if (oneChar == 0x1B) // ESC - an escape sequence
                {
                    commandInfo.escNeedFirstChar = TRUE;
                } else if (oneChar == 0x18) // Cancel
                {
                    commandInfo.index = 0;
                }else if (oneChar == 0x08) // Backspace
                {
                    if (commandInfo.index > 0) {
                        commandInfo.index--;
                        UART2PutChar(0x08);
                        UART2PutChar(' ');
                        UART2PutChar(0x08);
                    }
                } else if ((oneChar == 0x0D) || (oneChar == 0x0A)) {
                    UART2PrintString("\r\n");
                    commandInfo.buffer[commandInfo.index] = 0; // Null terminate the input command
                    commandInfo.reading = FALSE;
                    commandInfo.escNeedFirstChar = FALSE;
                    commandInfo.escNeedSecondChar = FALSE;

                    // Copy the new command into the command buffer
                    commandBuffer.showing = MAX_BUFFERED_COMMANDS;
                    if (commandBuffer.oldest == MAX_BUFFERED_COMMANDS) {
                        commandBuffer.oldest = 0;
                        commandBuffer.newest = 0;
                    } else {
                        commandBuffer.newest = (commandBuffer.newest + 1) & (MAX_BUFFERED_COMMANDS - 1);
                        if (commandBuffer.newest == commandBuffer.oldest) {
                            commandBuffer.oldest = (commandBuffer.oldest + 1) & (MAX_BUFFERED_COMMANDS - 1);
                        }
                    }
                    strcpy(&(commandBuffer.lines[commandBuffer.newest][0]), commandInfo.buffer);
                } else if ((0x20 <= oneChar) && (oneChar <= 0x7E)) {
                    oneChar = UpperCase(oneChar); // To make later processing simpler
                    if (commandInfo.index < MAX_COMMAND_LENGTH) {
                        commandInfo.buffer[commandInfo.index++] = oneChar;
                    }
                    UART2PutChar(oneChar); // Echo the character
                }
            }
        }
    }
}


/****************************************************************************
  Function:
    DWORD   PIC24RTCCGetDate( void )

  Description:
    This routine reads the date from the RTCC module.

  Precondition:
    The RTCC module has been initialized.


  Parameters:
    None

  Returns:
    DWORD in the format <xx><YY><MM><DD>

  Remarks:
    To catch roll-over, we do two reads.  If the readings match, we return
    that value.  If the two do not match, we read again until we get two
    matching values.

    For the PIC32MX, we use library routines, which return the date in the
    PIC32MX format.
 ***************************************************************************/

#if defined( __C30__ )

DWORD PIC24RTCCGetDate(void) {
    DWORD_VAL date1;
    DWORD_VAL date2;

    do {
        while (RCFGCALbits.RTCSYNC);

        RCFGCALbits.RTCPTR0 = 1;
        RCFGCALbits.RTCPTR1 = 1;
        date1.w[1] = RTCVAL;
        date1.w[0] = RTCVAL;

        RCFGCALbits.RTCPTR0 = 1;
        RCFGCALbits.RTCPTR1 = 1;
        date2.w[1] = RTCVAL;
        date2.w[0] = RTCVAL;

    } while (date1.Val != date2.Val);

    return date1.Val;
}
#endif


/****************************************************************************
  Function:
    DWORD   PIC24RTCCGetTime( void )

  Description:
    This routine reads the time from the RTCC module.

  Precondition:
    The RTCC module has been initialized.

  Parameters:
    None

  Returns:
    DWORD in the format <xx><HH><MM><SS>

  Remarks:
    To catch roll-over, we do two reads.  If the readings match, we return
    that value.  If the two do not match, we read again until we get two
    matching values.

    For the PIC32MX, we use library routines, which return the time in the
    PIC32MX format.
 ***************************************************************************/

#if defined( __C30__ )

DWORD PIC24RTCCGetTime(void) {
    DWORD_VAL time1;
    DWORD_VAL time2;

    do {
        while (RCFGCALbits.RTCSYNC);

        RCFGCALbits.RTCPTR0 = 1;
        RCFGCALbits.RTCPTR1 = 0;
        time1.w[1] = RTCVAL;
        time1.w[0] = RTCVAL;

        RCFGCALbits.RTCPTR0 = 1;
        RCFGCALbits.RTCPTR1 = 0;
        time2.w[1] = RTCVAL;
        time2.w[0] = RTCVAL;

    } while (time1.Val != time2.Val);

    return time1.Val;
}
#endif


/****************************************************************************
  Function:
    void PIC24RTCCSetDate( WORD xx_year, WORD month_day )

  Description:
    This routine sets the RTCC date to the specified value.


  Precondition:
    The RTCC module has been initialized.

  Parameters:
    WORD xx_year    - BCD year in the lower byte
    WORD month_day  - BCD month in the upper byte, BCD day in the lower byte

  Returns:
    None

  Remarks:
    For the PIC32MX, we use library routines.
 ***************************************************************************/

#if defined( __C30__ )

void PIC24RTCCSetDate(WORD xx_year, WORD month_day) {
    UnlockRTCC();
    RCFGCALbits.RTCPTR0 = 1;
    RCFGCALbits.RTCPTR1 = 1;
    RTCVAL = xx_year;
    RTCVAL = month_day;
}
#endif


/****************************************************************************
  Function:
    void PIC24RTCCSetTime( WORD weekDay_hours, WORD minutes_seconds )

  Description:
    This routine sets the RTCC time to the specified value.

  Precondition:
    The RTCC module has been initialized.

  Parameters:
    WORD weekDay_hours      - BCD weekday in the upper byte, BCD hours in the
                                lower byte
    WORD minutes_seconds    - BCD minutes in the upper byte, BCD seconds in
                                the lower byte

  Returns:
    None

  Remarks:
    For the PIC32MX, we use library routines.
 ***************************************************************************/

#if defined( __C30__ )

void PIC24RTCCSetTime(WORD weekDay_hours, WORD minutes_seconds) {
    UnlockRTCC();
    RCFGCALbits.RTCPTR0 = 1;
    RCFGCALbits.RTCPTR1 = 0;
    RTCVAL = weekDay_hours;
    RTCVAL = minutes_seconds;
}
#endif

/****************************************************************************
  Function:
    void PrintFileInformation( SearchRec searchRecord )

  Description:
    This function prints the file information that is contained in
    searchRecord.  Information printed is:
                <date> <time> [<DIR>] [<size>] <name>

  Precondition:
    None

  Parameters:
    SearchRec searchRecord  - File information

  Returns:
    None

  Remarks:
    The timestamp is assumed to be in the following format:
            Date format:    Bits 15-9:  Year (0 = 1980, 127 = 2107)
                            Bits 8-5:   Month (1 = January, 12 = December)
                            Bits 4-0:   Day (1 - 31)

            Time format:    Bits 15-11: Hours (0-23)
                            Bits 10-5:  Minutes (0-59)
                            Bits 4-0:   Seconds/2 (0-29)
 ***************************************************************************/

void PrintFileInformation(SearchRec searchRecord) {
    char buffer[20];

    UART2PrintString(UHP_FILEENTRY);
    // Display the file name.
    UART2PrintString(searchRecord.filename);
    UART2PrintString("             ");

    // Display the file's date/time stamp.
    sprintf(buffer, "%04d-%02d-%02d ", ((((DWORD_VAL) (searchRecord.timestamp)).w[1] & 0xFE00) >> 9) + 1980,
            (((DWORD_VAL) (searchRecord.timestamp)).w[1] & 0x01E0) >> 5,
            ((DWORD_VAL) (searchRecord.timestamp)).w[1] & 0x001F);
    UART2PrintString(buffer);
    sprintf(buffer, "%02d:%02d:%02d ", (((DWORD_VAL) (searchRecord.timestamp)).w[0] & 0xF800) >> 11,
            (((DWORD_VAL) (searchRecord.timestamp)).w[0] & 0x07E0) >> 5,
            (((DWORD_VAL) (searchRecord.timestamp)).w[0] & 0x001F) << 1);
    UART2PrintString(buffer);

    // Display the file size.  If the file is actually a directory, display an indication.
    if (searchRecord.attributes & ATTR_DIRECTORY) {
        UART2PrintString("<DIR>           ");
    } else {
        sprintf(buffer, "     %10ld ", ((DWORD_VAL) (searchRecord.filesize)).Val);
        UART2PrintString(buffer);
    }
    //UART2PrintString("  ");
    //UART2PrintString(getLastLFN());
    UART2PrintString("\r\n");
}

/****************************************************************************
  Function:
    void RedoCommandPrompt( void )

  Description:
    This function is called when the user either removes or inserts media.
    We want to let the user know right away that something has changed, so
    we change the command prompt immediately.  If the user is entering a
    command, we rebuild his command.

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
 ***************************************************************************/
void RedoCommandPrompt(void) {
    int i;
    char buffer[50];

    //    UART2PrintString( "\r\n" );
    //    if (volume.valid)
    //    {
    //        UART2PrintString( volume.label );
    //        //UART2PutChar( ':' );
    //        UART2PrintString( ":\\" );
    //    }
    buffer[0] = 0;
    if(FSgetcwd(buffer, 50)==NULL)
        return;
    UART2PrintString("\r\n");
    UART2PrintString(UHP_PATH);
    UART2PrintString(buffer);
    UART2PrintString("\r\n");
    UART2PrintString(":");
    UART2PrintString(buffer);
    UART2PrintString("> ");

    if (commandInfo.reading) {
        for (i = 0; i < commandInfo.index; i++) {
            UART2PutChar(commandInfo.buffer[i]);
        }
    }
}

/****************************************************************************
  Function:
    void ReplaceCommandLine( void )

  Description:
    This function is called when the user presses the arrow keys to scroll
    through previous commands.  The function erases the current command line
    and replaces it with the previous command indicated by
    commandBuffer.showing.

  Precondition:
    The buffer of old commands is valid.

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
 ***************************************************************************/

void ReplaceCommandLine(void) {
    BYTE i;
    BYTE lineLength;
    char oneChar;

    EraseCommandLine();

    lineLength = strlen(commandBuffer.lines[commandBuffer.showing]);
    for (i = 0; i < lineLength; i++) {
        oneChar = commandBuffer.lines[commandBuffer.showing][i];
        UART2PutChar(oneChar);
        commandInfo.buffer[commandInfo.index++] = oneChar;
    }
}


/****************************************************************************
  Function:
    void UnlockRTCC( void )

  Description:
    This function unlocks the RTCC so we can write a value to it.

  Precondition:
    None

  Parameters:
    None

  Return Values:
    None

  Remarks:
    For the PIC32MX, we use library routines.
 ***************************************************************************/

#define RTCC_INTERRUPT_REGISTER IEC3
#define RTCC_INTERRUPT_VALUE    0x2000

#if defined( __C30__ )

void UnlockRTCC(void) {
    BOOL interruptsWereOn;

    interruptsWereOn = FALSE;
    if ((RTCC_INTERRUPT_REGISTER & RTCC_INTERRUPT_VALUE) == RTCC_INTERRUPT_VALUE) {
        interruptsWereOn = TRUE;
        RTCC_INTERRUPT_REGISTER &= ~RTCC_INTERRUPT_VALUE;
    }

    // Unlock the RTCC module
    __asm__("mov #NVMKEY,W0");
    __asm__("mov #0x55,W1");
    __asm__("mov #0xAA,W2");
    __asm__("mov W1,[W0]");
    __asm__("nop");
    __asm__("mov W2,[W0]");
    __asm__("bset RCFGCAL,#13");
    __asm__("nop");
    __asm__("nop");

    if (interruptsWereOn) {
        RTCC_INTERRUPT_REGISTER |= RTCC_INTERRUPT_VALUE;
    }
}
#endif


/****************************************************************************
  Function:
    void __attribute__((__interrupt__, auto_psv)) _T3Interrupt(void)

  Description:
    Timer ISR, used to update the tick count.  Since we are using Timer 3, we
    can also trigger an A/D conversion off of the timer.

  Precondition:
    None

  Parameters:
    None

  Return Values:
    None

  Remarks:
    None
 ***************************************************************************/
#if defined( __C30__ )
void __attribute__((__interrupt__, auto_psv)) _T3Interrupt(void)
#elif defined( __PIC32MX__ )
void __ISR(_TIMER_3_VECTOR, ipl2) _T3Interrupt(void)
#else
#error Cannot prototype timer interrupt
#endif
{
#if defined( __C30__ )
    if (IFS0bits.T3IF)
#elif defined( __PIC32MX__ )
    if (IFS0 & PIC32MX_TIMER3_INTERRUPT)
#else
#error Cannot check timer interrupt
#endif
    {
        // Clear the interrupt flag
#if defined( __C30__ )
        IFS0bits.T3IF = 0;
#elif defined( __PIC32MX__ )
        mT3ClearIntFlag();
#else
#error Cannot clear timer interrupt.
#endif

        // Count the timer tick
        currentTick++;
    }
}

/****************************************************************************
  Function:
    void __attribute__((__interrupt__, auto_psv)) _ADCInterrupt(void)

  Description:
    ADC ISR, used to read the temperature

  Precondition:
    None

  Parameters:
    None

  Return Values:
    None

  Remarks:
    The ADC interrupt bits are in different registers in the two architectures.

    The conversion itself is triggered by Timer 3.
 ***************************************************************************/
#if defined( __C30__ )
void __attribute__((__interrupt__, auto_psv)) _ADC1Interrupt(void)
#elif defined( __PIC32MX__ )
#pragma interrupt _ADC1Interrupt ipl2 vector 27
void _ADC1Interrupt(void)
#else
#error Cannot prototype ADC interrupt
#endif
{
#if defined( __C30__ )
    if (IFS0bits.AD1IF)
#elif defined( __PIC32MX__ )
    if (IFS1bits.AD1IF)
#else
#error Cannot check ADC interrupt flag
#endif
    {
        // Clear the interrupt flag
#if defined( __C30__ )
        IFS0bits.AD1IF = 0;
#elif defined( __PIC32MX__ )
        IFS1CLR = 0x00000002;
#else
#error Cannot clear ADC interrupt
#endif

        if (loggerStatus.readingTemperature) {
            // Store the temperature reading
            sprintf((char *) &(logData[logBufferReading].buffer[logData[logBufferReading].index]),
                    "%010ld,%05d\r\n", GetCurrentTick(), ADC_READING_TEMPERATURE);
            logData[logBufferReading].index += 18;

            if ((logData[logBufferReading].index + 18) > MAX_LOG_BUFFER_SIZE) {
                // TODO we don't check for buffers wrapping!
                logData[logBufferReading++].bufferFull = TRUE;
                if (logBufferReading >= NUM_LOG_BUFFERS) {
                    logBufferReading = 0;
                }
            }
        }
    }
}


#if defined( __C30__ )

/*******************************************************************************
Function:       void __attribute__((__interrupt__, auto_psv)) _DefaultInterrupt(void)

This is just here to catch any spurious interrupts that we see during debugging.

 *******************************************************************************/
void __attribute__((interrupt, auto_psv)) _DefaultInterrupt(void) {
    //while (1);
}
#endif

//******************************************************************************
//******************************************************************************
// USB Support Functions
//******************************************************************************
//******************************************************************************

/****************************************************************************
  Function:
    BOOL USB_ApplicationEventHandler( BYTE address, USB_EVENT event,
                    void *data, DWORD size )

  Description:
    This routine handles USB events sent from the USB Embedded Host stack.

  Precondition:
    None

  Parameters:
    BYTE address    - Address of the USB device generating the event
    USB_EVENT event - Event that has occurred
    void *data      - Pointer to the data associated with the event
    DWORD size      - Size of the data pointed to by *data

  Return Values:
    TRUE    - The event was handled successfully
    FALSE   - The even was not handled successfully

  Remarks:
    We will default to returning TRUE for unknown events, and let operation
    proceed.  Other applications may wish to return FALSE, since we are not
    really handling the event.
 ***************************************************************************/

BOOL USB_ApplicationEventHandler(BYTE address, USB_EVENT event, void *data, DWORD size) {
    if (HIDUSB_ApplicationEventHandler(address, event, data, size) == TRUE)
        return TRUE;
    switch (event) {
        case EVENT_VBUS_REQUEST_POWER:
            // We will let everything attach.
            return TRUE;
            break;

        case EVENT_VBUS_RELEASE_POWER:
            // We are not monitoring power allocation, so we have
            // nothing to update.
            return TRUE;
            break;

        case EVENT_HUB_ATTACH:
        case EVENT_UNSUPPORTED_DEVICE:
        case EVENT_CANNOT_ENUMERATE:
        case EVENT_CLIENT_INIT_ERROR:
        case EVENT_OUT_OF_MEMORY:
        case EVENT_UNSPECIFIED_ERROR: // This should never occur
            //usbErrorCode = USBHostDeviceStatus(1);

            // Shut down the USB.
            USBHostShutdown();
            //screenState = SCREEN_DISPLAY_FLASH_ERROR;
            return TRUE;
            break;
        default:
            break;
    }

    return TRUE;
}


#include "usbhost.h"
#include "usbHostProtocol.h"
#include "cncgui.h"
#include "guilib.h"
#include "mcore.h"
#include "gcode.h"
#include "serialport.h"

#define HISTORY_LENGTH 50
int lastHid2 = 0;
usbHostKeyboardBuffer kbdBuffer[HISTORY_LENGTH];
int kbdPos = 0;
int kbdCommandPos = 0;
int kbdTakePos = 0;
int kbdResponsePos = 0;
int screenScroll = 0;
#define SCROLL_STEP 3
USB_MS_STATUS uhMSState = USB_MS_DISCONNECTED;
char uhMSPath[255];
#define DIR_NOT_SENDED -1
int uhDirSended = DIR_NOT_SENDED;
char isFileTransfer = 0;

char *readArgHex(char *ptr, int * result){
    int res = 0;
    char found = 0;
    while(1) {
        if(*ptr>=0x30 && *ptr<=0x39) {
            unsigned char v = *ptr-0x30;
            res*=16;
            res+=v;
            found = 1;
        } else if(*ptr>=0x41 && *ptr<=0x46) {
            unsigned char v = *ptr-0x37;
            res*=16;
            res+=v;
            found = 1;
         }else if(*ptr>=0x61 && *ptr<=0x66) {
            unsigned char v = *ptr-0x57;
            res*=16;
            res+=v;
            found = 1;
        } else {
        	if(found)
        		*result = res;
            return ptr;
        }
        ptr++;
    }
}

void uh_initCommand(int n)
{
	int i;
	for(i=0; i < UH_BUF_SIZES; i++ ) {
		kbdBuffer[n].command[i] = 0;
		kbdBuffer[n].response[i] = 0;
	}
	kbdBuffer[n].init = 1;
	kbdPos = n;
	kbdTakePos = n;
	kbdCommandPos = 0;
	kbdResponsePos = 0;
}

char *isMassStoare(char *string) {
	int i;
	for(i=0; i< sizeof (UHP_PATH)-1; i++)
		if(UHP_PATH[i]!=string[i])
			return string;
	return &string[i];
}

char *isUsbEmpty(char *string) {
	int i;
	for(i=0; i< sizeof (UHP_WAITFORDEVICE)-1; i++)
		if(UHP_WAITFORDEVICE[i]!=string[i])
			return string;
	return &string[i];
}

char *isKeyboard(char *string) {
	int i;
	for(i=0; i< sizeof (UHP_KEYBOARD_REPORT)-1; i++)
		if(UHP_KEYBOARD_REPORT[i]!=string[i])
			return string;
	return &string[i];
}

char *isError(char *string) {
	int i;
	for(i=0; i< sizeof (UHP_ERROR)-1; i++)
		if(UHP_ERROR[i]!=string[i])
			return string;
	return &string[i];
}

char *isFileEntry(char *string) {
	int i;
	for(i=0; i< sizeof (UHP_FILEENTRY)-1; i++)
		if(UHP_FILEENTRY[i]!=string[i])
			return string;
	return &string[i];
}

char *isFileLine(char *string) {
	int i;
	for(i=0; i< sizeof (UHP_FILELINE)-1; i++)
		if(UHP_FILELINE[i]!=string[i])
			return string;
	return &string[i];
}

char *isFileDone(char *string) {
	int i;
	for(i=0; i< sizeof (UHP_FILEDONE)-1; i++)
		if(UHP_FILEDONE[i]!=string[i])
			return string;
	return &string[i];
}

void kbdChar(char c) {

	if(kbdCommandPos>=UH_BUF_SIZES-1)
		return;
	if(kbdBuffer[kbdPos].command[kbdCommandPos]) {
		int i;
		char prevChar = c;
		for(i=kbdCommandPos; i<UH_BUF_SIZES-2; i++) {
			char tmp = kbdBuffer[kbdPos].command[i];
			kbdBuffer[kbdPos].command[i]=prevChar;
			prevChar = tmp;
		}
	} else {
		kbdBuffer[kbdPos].command[kbdCommandPos] = c;
	}
	kbdCommandPos++;
	uh_input_updated();
}

char saveBuffer[UH_BUF_SIZES];
void currentLineFromHistory() {
	int i, j = -1;
	for(i=0; i<UH_BUF_SIZES; i++) {
		if(kbdTakePos==kbdPos)
			kbdBuffer[kbdPos].command[i] = saveBuffer[i];
		else
			kbdBuffer[kbdPos].command[i] = kbdBuffer[kbdTakePos].command[i];
		if(j==-1 && kbdBuffer[kbdPos].command[i]==0)
			j = i;
	}
	if(j!=-1)
		kbdCommandPos = j;
	else
		kbdCommandPos = 0;
	uh_input_updated();
}

void saveState() {
	int i=0;
	for(i=0; i<UH_BUF_SIZES; i++) {
		saveBuffer[i] = kbdBuffer[kbdPos].command[i];
	}
}

void hostserial_port_recived(char *string)
{
	if(isFileTransfer==0 && gGetStatus()==CONNECTED_TRUE)
		return;
	char *res = isKeyboard(string);
	if(res!=string) {
		if(GUI_GetCurrentScreen()==&pauseScreen || GUI_GetCurrentScreen()==&changeToolScreen)
			return;
		int hid0 = -1, hid1 = -1, hid2 = -1;
		res = readArgHex(res, &hid0)+1;
		res = readArgHex(res, &hid1)+1;
		res = readArgHex(res, &hid2)+1;

		if(lastHid2>=0x59 && lastHid2<=0x62 && (hid2==0 || lastHid2!=hid2)) // NUM1-NUM0
			action_move(0.0f,0.0f, 0.0f, GUITS_Up, GUI_FALSE);

		if(hid2>=0x3A && hid2<=0x42) {// F1-F9
			if(hid0==0) {
				axisStepComboBox.value = 0x42 - hid2;
				GUI_drawView(&axisStepComboBox);
			} else {
				spindle_runr((hid2-0x39)*1000);
			}
		} else if (hid2==0x43) { // F10
			if(hid0!=0)
				spindle_runr(10000);
		} else if (hid2==0x44) { // F11
			spindle_stop();
		} else if (hid2==0x48) { // Pause-Break
			action_estop();
		} else if(hid2==0x59) { // NUM1
			action_move(-1.0f, -1.0f, 0.0f, GUITS_Up, GUI_TRUE);
		} else if(hid2==0x5A) { // NUM2
			action_move(0.0f, -1.0f, 0.0f, GUITS_Up, GUI_TRUE);
		} else if(hid2==0x5B) { // NUM3
			action_move(1.0f, -1.0f, 0.0f, GUITS_Up, GUI_TRUE);
		} else if(hid2==0x5C) { // NUM4
			action_move(-1.0f, 0.0f, 0.0f, GUITS_Up, GUI_TRUE);
		} else if(hid2==0x5D) { // NUM5
			action_move(0.0f, 0.0f, 1.0f, GUITS_Up, GUI_TRUE);
		} else if(hid2==0x5E) { // NUM6
			action_move(1.0f, 0.0f, 0.0f, GUITS_Up, GUI_TRUE);
		} else if(hid2==0x5F) { // NUM7
			action_move(-1.0f, 1.0f, 0.0f, GUITS_Up, GUI_TRUE);
		} else if(hid2==0x60) { // NUM8
			action_move(0.0f, 1.0f, 0.0f, GUITS_Up, GUI_TRUE);
		} else if(hid2==0x61) { // NUM9
			action_move(1.0f, 1.0f, 0.0f, GUITS_Up, GUI_TRUE);
		} else if(hid2==0x62) { // NUM0
			action_move(0.0f, 0.0f, -1.0f, GUITS_Up, GUI_TRUE);
		} else if(hid2>=0x04 && hid2<=0x1D) {
			if(hid0 & 0x02)
				kbdChar(hid2+0x3d); /* return capital */
			else
				kbdChar(hid2+0x5d); /* return small case */
	    } else if(hid2>=0x1E && hid2<=0x26) { // 1-9
	    	kbdChar(hid2+0x13);
	    } else if(hid2==0x27) {
	    	kbdChar('0');
	    } else if(hid2==0x36 || hid2==0x37) {
	    	kbdChar('.');
	    } else if(hid2==0x2D) {
	    	kbdChar('-');
	    } else if(hid2==0x2E) {
	    	kbdChar('+');
	    } else if(hid2==0x2D) {
	    	kbdChar('-');
	    } else if(hid2==0x2C) {
	    	kbdChar(' ');
	    } else if(hid2==0x29) { // ESC
	    	screenScroll = 0;
	    	uh_line_updated(UPDATE_ALL_LINES);
	    	kbdTakePos = kbdPos;
	    	uh_initCommand(kbdPos);
	    	uh_input_updated();
	    } else if(hid2==0x50) { // <-
	    	if(kbdCommandPos>0) {
	    		kbdCommandPos--;
	    		uh_input_updated();
	    	}
	    } else if(hid2==0x4F) { // ->
	    	if( kbdBuffer[kbdPos].command[kbdCommandPos]!=0 && kbdCommandPos+2 < UH_BUF_SIZES) {
	    		kbdCommandPos++;
	    		uh_input_updated();
	    	}
	    } else if(hid2==0x51) { /* \/ */
	    	if(kbdTakePos==kbdPos)
	    		saveState();
	    	if(kbdTakePos+1==kbdPos || (kbdPos==0 && kbdTakePos==HISTORY_LENGTH-1)) {
	    		kbdTakePos = kbdPos;
	    	} else if(kbdTakePos<HISTORY_LENGTH-1) {
				if(kbdBuffer[kbdTakePos+1].init) {
					kbdTakePos++;
				}
	    	} else {
	    		if(kbdBuffer[0].init) {
	    			kbdTakePos=0;
	    		}
	    	}
	    	currentLineFromHistory();
	    } else if(hid2==0x52) { /* /\ */
	    	if(kbdTakePos==kbdPos)
	    		saveState();
	    	if(kbdTakePos-1==kbdPos || (kbdPos==HISTORY_LENGTH-1 && kbdTakePos==0)) {
				kbdTakePos = kbdPos;
	    	} else if(kbdTakePos>0) {
				if(kbdBuffer[kbdTakePos-1].init) {
					kbdTakePos--;
				}
			} else {
				if(kbdBuffer[HISTORY_LENGTH-1].init) {
					kbdTakePos=HISTORY_LENGTH-1;
				}
			}

	    	currentLineFromHistory();
	    } else if(hid2==0x2A) { // backspace
	    	int i;
	    	if(kbdCommandPos>0) {
	    		for(i=kbdCommandPos; i<UH_BUF_SIZES; i++)
	    			kbdBuffer[kbdPos].command[i-1]=kbdBuffer[kbdPos].command[i];
	    		kbdCommandPos--;
	    	}
	    	uh_input_updated();
	    } else if(hid2==0x4C) { // delete
	    	int i;
	    	for(i=kbdCommandPos; i<UH_BUF_SIZES-1; i++)
	    		kbdBuffer[kbdPos].command[i]=kbdBuffer[kbdPos].command[i+1];
	    	uh_input_updated();
	    } else if(hid2==0x28) { // enter
	    	kbdResponsePos = 0;
	    	int i = kbdPos;
	    	if(kbdPos>=HISTORY_LENGTH-1)
	    		kbdPos = 0;
	    	else
	    		kbdPos++;
	    	gFinish();
	    	gParseLine(kbdBuffer[i].command, LINEFROM_KEYBOARD, i);
	    	screenScroll = 0;
	    	uh_initCommand(kbdPos);
	    	uh_input_updated();
	    	uh_line_updated(UPDATE_ALL_LINES);
	    } else if(hid2==0x2B || hid2==0x35) { // TAB
	    	if(GUI_GetCurrentScreen()==&connectedScreen)
	    		CNC_GUI_AxisScreen();
	    	else
	    		uh_input_updated();
	    } else if(hid2==0x4B)  {// PgUp
	    	int i = screenScroll;
	    	if(screenScroll<-SCROLL_STEP) {
	    		screenScroll+=SCROLL_STEP;
	    	} else {
	    		screenScroll = 0;
			}
	    	if(i!=screenScroll)
	    		uh_line_updated(UPDATE_ALL_LINES);
	    } else if(hid2==0x4E)  {// PgDn
	    	int i = screenScroll;
	    	if(screenScroll>=-HISTORY_LENGTH+3*SCROLL_STEP) {
	    		int p = kbdPos+screenScroll-2*SCROLL_STEP;
	    		if(p<0)
	    			p += HISTORY_LENGTH;
	    		if(kbdBuffer[p].init && p!=kbdPos)
	    			screenScroll-= SCROLL_STEP;
	    	} else {
	    		int p = kbdPos-HISTORY_LENGTH+2*SCROLL_STEP;
	    		if(p<0)
	    			p += HISTORY_LENGTH;
	    		if(kbdBuffer[p].init && p!=kbdPos)
	    			screenScroll = -HISTORY_LENGTH+2*SCROLL_STEP;
			}
	    	if(i!=screenScroll)
	    		uh_line_updated(UPDATE_ALL_LINES);
	    }

		lastHid2 = hid2;
		return;
	} else {
		res = isError(string);
		if(res!=string) {
			isFileTransfer = 0;
			uhDirSended = DIR_NOT_SENDED;
		} else {
			res = isFileEntry(string);
			if(res!=string) {
				if( !((res[0]=='.' && res[1]==' ') || (res[0]=='.' && res[1]=='.' && res[2]==' '))) {
					uh_file_info_recieved(res, uhDirSended);
					uhDirSended++;
				}
			} else {
				res = isMassStoare(string);
				if(res!=string) {
					isFileTransfer = 0;
					uhMSState = USB_MS_CONNECTED;
					char needDir = 0;
					int i = 0;
					while(*res!=0 && *res!='\r' && *res!='\n')
					{
						if(*res!=uhMSPath[i])
							needDir = 1;
						uhMSPath[i++] = *res++;
					}
					if(uhMSPath[i]!=0)
						needDir = 1;
					uhMSPath[i]=0;
					if(needDir) {
						uh_path_updated();
						uh_MSDir();
					} else {
						if(uhDirSended!=DIR_NOT_SENDED)
							uh_file_transfer_done();
						uhDirSended = DIR_NOT_SENDED;
					}
				} else {
					res = isUsbEmpty(string);
					if(res!=string) {
						isFileTransfer = 0;
						uhDirSended = DIR_NOT_SENDED;
						uhMSState = USB_MS_DISCONNECTED;
						uhMSPath[0] = 0;
						uh_path_updated();
						uh_file_transfer_prepare();
						if(GUI_GetCurrentScreen()==&fileManagerScreen)
							CNC_GUI_AxisScreen();
					} else {
						res = isFileLine(string);
						if(res!=string) {
							isFileTransfer = 1;
						} else {
							res = isFileDone(string);
							if(res!=string) {
								isFileTransfer = 0;
								gFinish();
							} else {
								if(isFileTransfer) {
									gParseLine(string, LINEFROM_MASSSTORAGE, 0);
								}
							}
						}
					}
				}
			}
		}
	}
}

void uh_MSDir()
{
	uh_file_transfer_prepare();
	hostserialposr_sendstring_sync("\x18"); // Cancel
	hostserialposr_sendstring_sync("DIR\n");
	uhDirSended = 0;
}

void uh_MSCHDir(const char *dir)
{
	hostserialposr_sendstring_sync("\x18"); // Cancel
	hostserialposr_sendstring_sync("CD ");
	hostserialposr_sendstring_sync(dir);
	hostserialposr_sendstring_sync("\n");
}

void uh_MSType(const char *file)
{
	gFinish();
	hostserialposr_sendstring_sync("\x18"); // Cancel
	hostserialposr_sendstring_sync("TYPE ");
	hostserialposr_sendstring_sync(file);
	hostserialposr_sendstring_sync("\n");
}

void uh_MSNextLine()
{
	hostserialposr_sendstring_sync("\r\n");
}

void uh_MSNextLineBreak()
{
	isFileTransfer = 0;
	hostserialposr_sendstring_sync("N");
}

void uh_init()
{
	int i;
	for (i=0; i<=HISTORY_LENGTH-1; i++)
		kbdBuffer[i].init = 0;
	uh_initCommand(0);
	uhMSPath[0]=0;
}

char returnStringBuff[UH_BUF_SIZES];
char *uh_getCurrentLine()
{
	int i=0;
	while (i<kbdCommandPos) {
		returnStringBuff[i]=kbdBuffer[kbdPos].command[i];
		i++;
	}
	returnStringBuff[i]='_';
	if(kbdBuffer[kbdPos].command[i]==0) {
		returnStringBuff[i+1]=0;
	} else {
		while(kbdBuffer[kbdPos].command[i]) {
			returnStringBuff[i+1] = kbdBuffer[kbdPos].command[i];
			i++;
		}
		returnStringBuff[i+1] = 0;
	}
	return returnStringBuff;
}

char *uh_getLine(int num)
{
	if(num/2>=HISTORY_LENGTH || num<0 || (screenScroll - num/2 - 1)<=-HISTORY_LENGTH)
		return "";
	int pos = kbdPos - num/2 - 1 + screenScroll;
	if(pos<0)
		pos += HISTORY_LENGTH;
	if(pos<0)
		return "";
	if(num%2==0)
		return kbdBuffer[pos].response;
	else
		return kbdBuffer[pos].command;
}

void uh_response(char *str, int index)
{
	while(*str){
		if(*str>=0x20 && kbdResponsePos<UH_BUF_SIZES-2)
			kbdBuffer[index].response[kbdResponsePos++] = *str;
		str++;
	}
	kbdBuffer[index].response[kbdResponsePos] = 0;
	uh_line_updated(2*(kbdPos - index - 1 + screenScroll));
}

USB_MS_STATUS uh_MSState()
{
	return uhMSState;
}

char *uh_MSPath()
{
	return uhMSPath;
}

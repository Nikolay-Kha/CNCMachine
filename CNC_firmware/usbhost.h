#ifndef _USB_HOST_H_
#define _USB_HOST_H_

#define UH_BUF_SIZES 64
#define UPDATE_ALL_LINES -1
typedef struct {
	char command[UH_BUF_SIZES];
	char response[UH_BUF_SIZES];
	char init;
} usbHostKeyboardBuffer;

typedef enum {
	USB_MS_DISCONNECTED,
	USB_MS_CONNECTED
} USB_MS_STATUS;

void uh_init();
char *uh_getCurrentLine();
char *uh_getLine(int num);
void uh_response(char *str, int index);
USB_MS_STATUS uh_MSState();
char *uh_MSPath();
void uh_MSDir();
void uh_MSCHDir(const char *dir);
void uh_MSType(const char *file);
void uh_MSNextLine();
void uh_MSNextLineBreak();

extern void uh_input_updated();
extern void uh_line_updated(int num);
extern void uh_path_updated();
extern void uh_file_info_recieved(char *info, int index);
extern void uh_file_transfer_done();
extern void uh_file_transfer_prepare();

#endif // _USB_HOST_H_

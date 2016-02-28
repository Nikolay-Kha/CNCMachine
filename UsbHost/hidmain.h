/* 
 * File:   hidmain.h
 * Author: LuCiFer
 *
 * Created on 26 Май 2014 г., 23:33
 */

#ifndef HIDMAIN_H
#define	HIDMAIN_H

#include "usb.h"

#ifdef	__cplusplus
extern "C" {
#endif

typedef enum
{
    DEVICE_NOT_CONNECTED_OR_NOT_SUPPORTED,
    DEVICE_MOUSE,
    DEVICE_KEYBOARD
} CONNECT_STATE;

void hidTask (void);
BOOL HIDUSB_ApplicationEventHandler( BYTE address, USB_EVENT event, void *data, DWORD size );
CONNECT_STATE HID_isConnected();

#ifdef	__cplusplus
}
#endif

#endif	/* HIDMAIN_H */


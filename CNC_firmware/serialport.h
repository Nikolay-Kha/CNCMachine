#ifndef __SERIAL_PORT_H
#define __SERIAL_PORT_H

void serialport_init();

void serialposr_sendstring_sync(char *string);
void serialposr_sendstring_async(char *string);
char serialposr_async_done();
void hostserialposr_sendstring_sync(const char *string);

extern void serial_port_recived(char *string);
extern void hostserial_port_recived(char *string);



#endif //__SERIAL_PORT_H

#ifndef SERIALPORT_H
#define SERIALPORT_H
#include <stdio.h>

#if ( defined(WIN64) || defined(_WIN64) || defined(__WIN64__) || defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__) )
#define WINDOWS
typedef void *HANDLE;
#define COM HANDLE
#elif defined(__linux__) || defined(__linux)
#define LINUX
#define COM int
#endif

#define ERROR_WRITE_STRING (char*)"ERROR ---> CAN'T WRITE TO PORT\n"
#define ERROR_READ_STRING (char*)"ERROR ---> CAN'T READ FROM PORT\n"

class SerialPort
{
public:
    static SerialPort *open(char *port);
    ~SerialPort();
    void send(char *text);
    bool sendAndWaitAnswer(char *text, int timeOutMs);
    // private, but due to static thread method it here
    COM getCom();
    bool mTreadFlag;
    void *mThread;
    char lastCharRecieved;
    bool mReadError;
private:
    SerialPort(COM comport);
    COM mCom;
};

extern void SerialPortRecieved(SerialPort *port, char *text);
extern void SerialPortError(SerialPort *port, char *text);

#endif // SERIALPORT_H

#include "serialport.h"
#include <stdio.h>
#ifdef LINUX

#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <pthread.h>

static void * thread_start(void *arg)
{
    SerialPort *port =  (SerialPort *)arg;
    int hCOM = port->getCom();

    const int buffSize = 1024;
    char buff[buffSize];
    while (port->mTreadFlag) {
        int rb = read(hCOM, buff, buffSize);
        if(rb>0) {
            port->mReadError = false;
            buff[rb]='\0';
            SerialPortRecieved(port, buff);
            port->lastCharRecieved = buff[rb-1];
        } else if(rb==0) {
            if(port->mReadError==false)
                SerialPortError(port, ERROR_READ_STRING);
            port->mReadError = true;
            usleep(50000);
        } else {
            port->mReadError = false;
            usleep(50000);
        }
    }
    port->mTreadFlag = true;
    return 0;
}



SerialPort *SerialPort::open(char *port)
{
    COM comp = ::open ( port, O_RDWR| O_NONBLOCK | O_NDELAY );
    if(comp>=0)
    {
       struct termios tio;
       memset(&tio,0,sizeof(tio));
       if ( tcgetattr ( comp, &tio ) != 0 ) {
           close(comp);
           return 0;
       }
       tio.c_iflag=0;
       tio.c_oflag=0;
       tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
       tio.c_lflag=0;
       tio.c_cc[VMIN]=1;
       tio.c_cc[VTIME]=5;

       cfsetospeed(&tio,B115200);            // 115200 baud
       cfsetispeed(&tio,B115200);            // 115200 baud

       if ( tcsetattr ( comp, TCSANOW, &tio ) != 0) {
           close(comp);
           return 0;
       }

       SerialPort *port = new SerialPort(comp);
       pthread_t  thr;
       if(pthread_create(&thr, 0, thread_start, (void *)port)!=0) {
           delete port;
           return 0;
       }
       port->mThread = (void *)thr;

       return port;
    }
    return 0;
}

SerialPort::SerialPort(COM comport)
{
    mCom = comport;
    mTreadFlag = true;
    mReadError = false;
}

SerialPort::~SerialPort()
{
    mTreadFlag = false;
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_nsec += 200000000;
    pthread_timedjoin_np((pthread_t)mThread, 0, &ts);
    if(mTreadFlag==false)
        pthread_cancel((pthread_t)mThread);
    close(mCom);
}

bool SerialPort::sendAndWaitAnswer(char *text, int timeOutMs)
{
    lastCharRecieved = 0;
    send(text);
    for(int i=0; i<timeOutMs/10+1; i++) // 1 secS
    {
        if(lastCharRecieved=='\n' || lastCharRecieved=='\r') {
            return true;
        }
        usleep(10000);
    }
    return false;
}

void SerialPort::send(char *text)
{
    int bw = write(mCom, text, strlen(text));
    if(bw<=0) {
        SerialPortError(this, ERROR_WRITE_STRING);
        return;
    }
    bw = write(mCom, "\n", 1);
    if(bw<=0) {
        SerialPortError(this, ERROR_WRITE_STRING);
        return;
    }
    tcflush( mCom, TCOFLUSH );
}

COM SerialPort::getCom()
{
    return mCom;
}

#endif

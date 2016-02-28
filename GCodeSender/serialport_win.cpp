#include "serialport.h"
#include <stdio.h>
#ifdef WINDOWS
#include <windows.h>

DWORD ThreadProc (LPVOID lpdwThreadParam )
{
    SerialPort *port =  (SerialPort *)lpdwThreadParam;
    HANDLE hCOM = port->getCom();
    DWORD read;
    const int buffSize = 1024;
    char buff[buffSize];
    while (port->mTreadFlag) {
        read = 0;
        if(ReadFile(hCOM, buff, buffSize, &read,0)) {
            if(read>0) {
                port->mReadError = false;
                buff[read]='\0';
                SerialPortRecieved(port, buff);
                port->lastCharRecieved = buff[read-1];
            } else {
                if(GetCommModemStatus(hCOM, &read)==0) {
                    if(port->mReadError==false)
                        SerialPortError(port, ERROR_READ_STRING);
                    port->mReadError = true;
                } else {
                    port->mReadError = false;
                }
                Sleep(50);
            }
        } else {
            port->mReadError = true;
            SerialPortError(port, ERROR_READ_STRING);
            Sleep(50);
        }
    }
    port->mTreadFlag = true;
    return 0;
}

SerialPort *SerialPort::open(char *port)
{
    char namebuff[MAX_PATH];
    snprintf(namebuff, sizeof namebuff, "\\\\.\\%s", port);
    HANDLE hCOM=CreateFileA(namebuff,GENERIC_READ|GENERIC_WRITE,0,NULL,OPEN_EXISTING,0,NULL);
    if (hCOM!=INVALID_HANDLE_VALUE)
    {
        DCB cdcb;
        if( GetCommState(hCOM,&cdcb)==0 ) {
            CloseHandle(hCOM);
            return 0;
        }
        cdcb.BaudRate=CBR_115200;
        cdcb.Parity=NOPARITY;
        cdcb.ByteSize=8;
        cdcb.StopBits=ONESTOPBIT;
        cdcb.EvtChar=13;
        if( SetCommState(hCOM,&cdcb)==0 ) {
            CloseHandle(hCOM);
            return 0;
        }
//        SetCommMask(hCOM,0);
//        SetupComm(hCOM, 10240,10240);
//        PurgeComm(hCOM, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
        COMMTIMEOUTS to;
        to.ReadIntervalTimeout = MAXDWORD;
        to.ReadTotalTimeoutMultiplier = 0;
        to.ReadTotalTimeoutConstant = 0;
        to.WriteTotalTimeoutMultiplier = 0;
        to.WriteTotalTimeoutConstant = 0;
        SetCommTimeouts(hCOM,&to);

        SerialPort *port = new SerialPort(hCOM);
        DWORD dwThreadId;
        if ( (port->mThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)&ThreadProc, (LPVOID)port,  0, &dwThreadId)) == NULL)
        {
            delete port;
            return 0;
        }
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
    WaitForSingleObject((HANDLE)mThread, 200);
    if(mTreadFlag==false)
        TerminateThread((HANDLE)mThread, 0);
    CloseHandle(mCom);
}

void SerialPort::send(char *text)
{
    DWORD nb;
    WriteFile(mCom,text,strlen(text),&nb,0);
    if(nb<=0) {
        SerialPortError(this, ERROR_WRITE_STRING);
        return;
    }
    WriteFile(mCom,"\n",1,&nb,0);
    if(nb<=0) {
        SerialPortError(this, ERROR_WRITE_STRING);
        return;
    }
    FlushFileBuffers(mCom);
}

COM SerialPort::getCom()
{
    return mCom;
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
        Sleep(10);
    }
    return false;
}

#endif

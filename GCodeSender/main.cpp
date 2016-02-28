#include <iostream>
#include <stdio.h>
#include <strings.h>
//#include <stdlib.h>
//#include <signal.h>
#include "../CNC_firmware/cncprotocol.h"

#include "serialport.h"
using namespace std;

#define SP_SCREEN (char*)"-screen"
#define ERRORCODE 1

SerialPort *port = 0;
bool autoDetectInProgress = false;
char answerBuf[32];
int answerBufPointer = 0;
bool errorOccurred = false;

#ifdef WINDOWS
#include <windows.h>
HANDLE console = GetStdHandle(STD_OUTPUT_HANDLE);
int lastXpos = 0;
void printbefore(char *text)
{
    CONSOLE_SCREEN_BUFFER_INFO csbi;
    GetConsoleScreenBufferInfo(console, &csbi);
    COORD CursorPosition;
    CursorPosition.X = lastXpos;
    CursorPosition.Y = csbi.dwCursorPosition.Y-1;
    SetConsoleCursorPosition(console,CursorPosition);
    cout << text;
    int len = strlen(text);
    if(text[len-1]=='\n')
        lastXpos = 0;
    else
        lastXpos += len;
    SetConsoleCursorPosition(console,csbi.dwCursorPosition);
}
#else
#include <string.h>
#endif

int strICcmp(const char *s1, const char *s2)
{
#ifdef WINDOWS
    return strcmpi(s1,s2);
#else
    return strcasecmp(s1,s2);
#endif
}


SerialPort *detectPort(){
    char name[256];
    SerialPort *res;
    for(int i=0; i<10; i++) {
        #ifdef WINDOWS
        sprintf(name, "COM%d", i);
        #else
        sprintf(name, "/dev/ttyUSB%d", i);
        #endif
        res=SerialPort::open(name);
        if( res ) {
            autoDetectInProgress = true;
            answerBufPointer = 0;
            res->sendAndWaitAnswer((char*)CNCPROTOCOL_ASK, 500);
            autoDetectInProgress = false;
            if(strncmp(answerBuf, (char*)CNCPROTOCOL_ANSWER, strlen(CNCPROTOCOL_ANSWER))==0)
            {
                printf("CNC found %s - %s", name, answerBuf);
                return res;
            }
            else
                delete res;
        }
    }
    printf("CNC device not found\r\n");
    return 0;
}

void screen() {
    char buff[256];
    while(true) {
        putchar('>');
        //buff[0]=0;
        if(fgets(buff, sizeof buff, stdin)==0)
            break;
        if(port->mReadError)
            break;
        // check quit or exit
        if ( (((buff[0]=='q' || buff[0]=='Q') && (buff[1]=='u' || buff[1]=='U')) ||\
               ((buff[0]=='e' || buff[0]=='E') && (buff[1]=='x' || buff[1]=='X'))) && \
             (buff[2]=='i' || buff[2]=='I') && (buff[3]=='t' || buff[3]=='T') )
             break;
        putchar('\n');
        if(strlen(buff)>0) {
             port->send(buff);
        }
    }
}

//void abort(int signum)
//{
//   if(port)
//       delete port;
//   exit(signum);
//}

void promtExit() {
    printf("Press Enter to exit\n");
    getchar();
}

int main(int argc, char* argv [])
{
//    signal(SIGABRT,abort);
//    signal(SIGTERM,abort);
//    signal(SIGINT,abort);
    bool askExit = true;
    char *file = 0;
    bool bScreen = false;
    if(argc<2) {
        port = detectPort();
        if(!port) {
            if(askExit) promtExit();
            return ERRORCODE;
        }
        bScreen = true;
    } else if(argc<3) {
        file = argv[1];
        if(strICcmp(file,"?")==0 || strICcmp(file,"/?")==0 || strICcmp(file,"-?")==0 || strICcmp(file,"/h")==0 || strICcmp(file,"-h")==0 || strICcmp(file,"/help")==0 ||strICcmp(file,"-help")==0 || strICcmp(file,"--help")==0 ) {
            printf("Usage: gcodecnc <SerialPort> [%s] Filename\r\n", SP_SCREEN);
            return ERRORCODE;
        }
        port = detectPort();
        if(!port) {
            if(askExit && strICcmp(file, SP_SCREEN)!=0)
                promtExit();
            return ERRORCODE;
        }
    } else if(argc<4) {
        askExit = false;
        file = argv[2];
        port = SerialPort::open(argv[1]);
        if(!port) {
            printf("Error while opennig %s\r\n", argv[1]);
            if(askExit) promtExit();
            return ERRORCODE;
        }
    }
    if(bScreen || strICcmp(file, SP_SCREEN)==0) {
       screen();
       askExit = false;
    } else {
        FILE *fl = fopen(file, "r");
        if(fl) {
            char lineBuffer[1024];
            char describeBuffer[32];
            char t;
            int totalLine = 0;
            int counterLine = 0;
            fpos_t position;
            fgetpos (fl, &position);
            while( (t=fgetc(fl))!=EOF ) if(t=='\n') totalLine++;
            fsetpos (fl, &position);
            while(fgets(lineBuffer, sizeof lineBuffer, fl)) {
                counterLine++;
            repeat:
                sprintf(describeBuffer, "[%d/%d]>", counterLine, totalLine);
                if(strlen(lineBuffer)>1) {
                   if(lineBuffer[0]=='(' || lineBuffer[0]==';') {
                       cout << describeBuffer << lineBuffer;
                        continue;
                   }
                   cout << describeBuffer << lineBuffer << endl;
                   if(!port->sendAndWaitAnswer(lineBuffer, 1800000)) {
                       puts("No answer yet. Make sure machine have done command and press enter for coninue.\n");
                       if(!fgets(lineBuffer, sizeof lineBuffer, stdin))
                           break;
                   }
                   if(port->mReadError)
                       errorOccurred = true;
                   if(errorOccurred){
                       do
                       {
                            cout << "An error has occurred. Continue (c), repeat(r), stop(s)? ";
                            cin >> t;
                       } while( t!='c' &&  t!='s' && t!='r' && t!='C' &&  t!='S' && t!='R');
                       if (std::cin.peek()) std::cin.ignore();
                       errorOccurred = false;
                       if(t=='s' || t=='S') {
                           askExit = false;
                           break;
                       }
                       else if(t=='r' || t=='R')
                           goto repeat;
                   }
                } else {
                    cout << describeBuffer << endl;
                }
            }
            fclose(fl);
            puts("***DONE***\r\n");
        } else {
            printf("Error while openning %s\r\n", file);
            if(askExit) promtExit();
            return 1;
        }
    }

    delete port;
    if(askExit) promtExit();
    return 0;
}

void SerialPortRecieved(SerialPort */*port*/, char *text)
{
    if(autoDetectInProgress) {
        strncat(&answerBuf[answerBufPointer], text, sizeof answerBuf-answerBufPointer-1);
        answerBufPointer += strlen(text);
    } else {
        if(strstr(text, CNCPROTOCOL_ANSWER_ERROR_PATERN))
            errorOccurred = true;
#ifdef WINDOWS
        printbefore(text);
#else
        std::cout.setf( std::ios_base::unitbuf );
        cout << "\e[s";
        cout << "\e[A";
        cout << "\r";
        int i=0;
        while (text[i]!='\r' && text[i]!='\n' && text[i]!=0)
            cout << text[i++];
        cout << "\e[u";
        std::cout.setf( std::ios_base::unitbuf );
#endif
    }
}

void SerialPortError(SerialPort */*port*/, char *text)
{
    cout << ' ';
    puts(text);
}



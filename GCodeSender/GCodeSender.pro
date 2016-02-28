TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
#CONFIG -= qt
win32{
QMAKE_LFLAGS *= -static -static-libgcc
}

TARGET = ../../bin/gcodecnc
INCLUDEPATH += ../CNC_firmware/

SOURCES += main.cpp \
    serialport_win.cpp \
    serialport_linux.cpp

HEADERS += \
    serialport.h \
    ../CNC_firmware/cncprotocol.h


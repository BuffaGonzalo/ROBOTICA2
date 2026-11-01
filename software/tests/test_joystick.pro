QT += core gui widgets network serialport
CONFIG += c++17 console
TARGET = test_joystick
INCLUDEPATH += ..
SOURCES += test_joystick.cpp ../mainwindow.cpp
HEADERS += ../mainwindow.h ../joystickwidget.h ../routeplanwidget.h ../smartdoublespinbox.h
FORMS += ../mainwindow.ui
RESOURCES += ../resources.qrc

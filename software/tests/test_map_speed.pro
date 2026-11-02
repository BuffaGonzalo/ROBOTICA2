QT += core gui widgets network serialport
CONFIG += c++17 console
TARGET = test_map_speed
INCLUDEPATH += ..
SOURCES += test_map_speed.cpp ../mainwindow.cpp
HEADERS += ../mainwindow.h ../routeplanwidget.h ../joystickwidget.h ../smartdoublespinbox.h
FORMS += ../mainwindow.ui
RESOURCES += ../resources.qrc

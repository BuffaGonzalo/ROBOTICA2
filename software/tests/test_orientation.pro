QT += core gui widgets network serialport
CONFIG += c++17 console
TARGET = test_orientation
INCLUDEPATH += ..
SOURCES += test_orientation.cpp ../mainwindow.cpp
HEADERS += ../mainwindow.h ../routeplanwidget.h ../joystickwidget.h ../smartdoublespinbox.h ../../firmware/main/pose_kinematics.h
FORMS += ../mainwindow.ui
RESOURCES += ../resources.qrc

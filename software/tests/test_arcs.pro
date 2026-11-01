QT += core gui widgets network serialport
CONFIG += c++17 console
TARGET = test_arcs
INCLUDEPATH += ..
SOURCES += test_arcs.cpp ../mainwindow.cpp
HEADERS += ../mainwindow.h ../routeplanwidget.h ../joystickwidget.h ../smartdoublespinbox.h ../../firmware/main/pose_kinematics.h
FORMS += ../mainwindow.ui
RESOURCES += ../resources.qrc

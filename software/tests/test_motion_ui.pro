QT += core gui widgets network serialport
CONFIG += c++17 console
TARGET = test_motion_ui
INCLUDEPATH += ..
SOURCES += test_motion_ui.cpp ../mainwindow.cpp
HEADERS += ../mainwindow.h ../smartdoublespinbox.h
FORMS += ../mainwindow.ui
RESOURCES += ../resources.qrc

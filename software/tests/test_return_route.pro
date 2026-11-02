QT += core gui widgets network serialport
CONFIG += c++17 console
TARGET = test_return_route
INCLUDEPATH += ..
SOURCES += test_return_route.cpp ../mainwindow.cpp
HEADERS += ../mainwindow.h ../obstacleplanner.h ../routeplanwidget.h ../joystickwidget.h ../smartdoublespinbox.h
FORMS += ../mainwindow.ui
RESOURCES += ../resources.qrc

QT += core gui widgets network serialport
CONFIG += c++17 console
TARGET = test_route_json
INCLUDEPATH += ..
SOURCES += test_route_json.cpp ../mainwindow.cpp
HEADERS += ../mainwindow.h ../routeplanwidget.h ../smartdoublespinbox.h
FORMS += ../mainwindow.ui
RESOURCES += ../resources.qrc

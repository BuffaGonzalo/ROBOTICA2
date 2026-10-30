#include "mainwindow.h"

#include <QApplication>
#include <QIcon>
#include <QStyleFactory>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    app.setApplicationName("MICRO-Software");
    app.setApplicationDisplayName("MICRO-Software");
    app.setWindowIcon(QIcon(":/resources/icons/app_icon.png"));
    app.setStyle(QStyleFactory::create("Fusion"));

    MainWindow window;
    window.setWindowIcon(QIcon(":/resources/icons/app_icon.png"));
    window.show();
    return app.exec();
}

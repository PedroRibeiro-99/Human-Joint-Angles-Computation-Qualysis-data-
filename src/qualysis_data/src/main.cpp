#include <QCoreApplication>
#include <QApplication>
#include "../include/qualysis_data/mainwindow.h"
#include <iostream>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow app(argc, argv);
    app.show();
    return a.exec();
}

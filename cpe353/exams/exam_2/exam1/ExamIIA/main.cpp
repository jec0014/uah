//
// DO NOT MOFIDY OR SUBMIT THIS FILE
//

#include "mainwindow.h"
#include <QApplication>
#include <QtDebug>

static void PrintStuff(QString s)
{
    qDebug() << s;
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MainWindow w;
    w.setWindowTitle("exam2a");
    w.show();

    QObject::connect(&w, &MainWindow::customSignal, &PrintStuff);

    return a.exec();
}



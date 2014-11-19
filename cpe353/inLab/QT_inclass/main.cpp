#include "mainwindow.h"
#include <QApplication>
#include <QtSql>
#include <QString>

QSqlDatabase db = QSqlDatabase::addDatabase( "QMYSQL" );
QSqlQuery sqlQuery;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    /*
    db.setHostName("pavelow.ece.uah.edu");
    db.setPort(3300);
    db.setDatabaseName("cpe353");
    db.setUserName("cpe353student");
    db.setPassword("123qwerty456");
    */

    bool connected = db.open();

    qDebug() << "Connected to db: " << connected;

    if (connected) {
        sqlQuery.exec("SELECT * FROM vets");
        while (sqlQuery.next())
        {
            qDebug() << sqlQuery.value(0).toString();
        }
    }

    w.show();

    return a.exec();
}

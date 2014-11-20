#include "mainwindow.h"
#include <QApplication>

// Connect to MYSQL
//QSqlDatabase db = QSqlDatabase::addDatabase( "QMYSQL" );

// Connect to SQLite
//QSqlDatabase db = QSqlDatabase::addDatabase( "QSQLITE" );

//QSqlQuery sqlQuery;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    // Connect to MYSQL
    /*
    db.setHostName("pavelow.ece.uah.edu");
    db.setPort(3300);
    db.setDatabaseName("cpe353");
    db.setUserName("cpe353student");
    db.setPassword("123qwerty456");
    */

    // Connect to SQLite
    /*
    db.setDatabaseName( "practice.db" );

    if (!db.open())
    {
        qDebug() << db.lastError();
        qDebug() << "Error: Database not open.";
        return (1);
    }

    if (connected) {
        sqlQuery.exec("SELECT * FROM vehicles");
        while (sqlQuery.next())
        {
            qDebug() << sqlQuery.value(0).toString();
        }
    }

    QSqlQueryModel model;
    model.setQuery("SELECT * FROM vehicles");
    */

    //QTableView * view = new QTableView;
    //view->setModel(&model);
    //view->show();

    /*
    QSqlTableModel* tmodel = new QSqlTableModel;
    */



    //view->setModel(tmodel);
    //view->show();

    w.show();

    return a.exec();
}

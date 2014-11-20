#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSql>
#include <QString>
#include <QTableView>
#include <QDebug>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QSqlDatabase db;
    QSqlQuery sqlQuery;
    QSqlQueryModel qmodel;
    QSqlTableModel * tmodel;
    QTableView tview;

public slots:
    void execQuery();

signals:
    // Nothing, right now

private:
    Ui::MainWindow *ui;

};

#endif // MAINWINDOW_H



#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSql>
#include <QString>
#include <QTableView>
#include <QDebug>
#include <QMenuBar>
#include <QMenu>

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
    QMenu * mainMenu;

public slots:
    void execQuery();

signals:
    // Nothing, right now

private:
    Ui::MainWindow *ui;
    void CreateMenu();
    void CreateActions();
    QAction *insultAction;

};

#endif // MAINWINDOW_H



#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSqlDatabase>
#include <QLabel>
#include <QSql>
#include <QSqlDatabase>
#include <QIntValidator>
#include <QtDebug>
#include <QSqlQuery>
#include <QSqlQueryModel>
#include <QTableView>
#include <QSqlError>
#include <QAction>
#include <QSqlTableModel>


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
    QSqlQuery * dbQuery;
    QAction * dbOpened;
    QTableView * tview;
    QSqlTableModel * tmodel;
    
public slots:
    void openDB();
    void viewTable();
    void createTable();
    void executeQuery();
    void p05();
    void p04();
    
signals:

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H

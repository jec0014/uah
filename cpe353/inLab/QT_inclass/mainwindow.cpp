#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    connect( ui->clearButton, SIGNAL(clicked()), ui->queryEdit, SLOT(clear()) );
    connect( ui->sendButton, SIGNAL(clicked()), this, SLOT(execQuery()) );

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::execQuery()
{
    QSqlQuery newQ;

    qDebug() << "Send button clicked." << Q_FUNC_INFO;

    QString temp;

    temp = ui->queryEdit->text();

    qDebug() << newQ.exec(temp);

    while (newQ.next())
    {
        qDebug() << newQ.value(0).toString();
    }

}

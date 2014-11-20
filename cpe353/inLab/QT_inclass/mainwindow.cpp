#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    db = QSqlDatabase::addDatabase("QSQLITE");
    db.setDatabaseName("practice.db");
    bool connected = db.open();
    qDebug() << "Connected to db: " << connected;

    qmodel.setQuery("SELECT * FROM 'vehicles'");

    ui->tableView->setModel(&qmodel);
    ui->tableView->show();

    /*tmodel.setTable("vehicles");
    tmodel.select();
    tmodel.setEditStrategy(QSqlTableModel::OnRowChange);
    */

    connect( ui->clearButton, SIGNAL(clicked()), ui->queryEdit, SLOT(clear()) );
    connect( ui->sendButton, SIGNAL(clicked()), this, SLOT(execQuery()) );

    //ui->tableView->setModel(&tmodel);

}

MainWindow::~MainWindow()
{
    qDebug() << "Ending the program!";
    delete ui;
}

void MainWindow::execQuery()
{
    QSqlQuery newQ;

    qDebug() << "Send button clicked." << Q_FUNC_INFO;

    QString temp;

    temp = ui->queryEdit->text();

    qDebug() << newQ.exec(temp);

    if (!newQ.isActive()) {
        qDebug() << newQ.lastError();
        return;
    }

    while (newQ.next())
    {
        qDebug() << newQ.value(0).toString();
    }

}

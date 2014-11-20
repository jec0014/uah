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

    //ui->tableWidget->setModel(&qmodel);
    //ui->tableWidget->show();

    /*tmodel.setTable("vehicles");
    tmodel.select();
    tmodel.setEditStrategy(QSqlTableModel::OnRowChange);
    */

    connect( ui->clearButton, SIGNAL(clicked()), ui->queryEdit, SLOT(clear()) );
    connect( ui->sendButton, SIGNAL(clicked()), this, SLOT(execQuery()) );
    
    CreateActions();
    CreateMenu();
    
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

void MainWindow::CreateMenu() {
    //mainMenu = QMenuMicrosoft::menuBar()->addMenu(rt("&etc"));
    
    mainMenu = menuBar()->addMenu(tr("&etc"));
    mainMenu->addAction(insultAction);
    mainMenu->addSeparator();
}

void MainWindow::CreateActions() {
    insultAction = new QAction(tr("this?"), this);
    insultAction->setStatusTip("No.");
    //connect()
}

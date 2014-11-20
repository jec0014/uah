/*
 * No time to debug, or even test for that matter.
 * It's a good thing this place isn't affecting my professional future.
 */

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    
    connect(ui->localButton, SIGNAL(clicked()), this, SLOT(openDB()));
    connect(ui->viewButton, SIGNAL(clicked()), this, SLOT(viewTable()));
    connect(ui->createButton, SIGNAL(clicked()), this, SLOT(createTable()));
    connect(ui->sendButton, SIGNAL(clicked()), this, SLOT(executeQuery()));
    connect(ui->clearButton, SIGNAL(clicked()), ui->queryEdit, SLOT(clear()));
    connect(ui->selectButton, SIGNAL(clicked()), this, SLOT(p05()));
    connect(ui->addButton, SIGNAL(clicked()), this, SLOT(p04()));
    
    dbOpened = new QAction(this);
    dbOpened->setStatusTip("Ready");
    dbOpened->showStatusText();
    
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::openDB()
{
    QString name;
    name = "";
    name.append(ui->dbEdit->text());
    qDebug() << "Database name: " << name;
    
    db = QSqlDatabase::addDatabase("QSQLITE");
    
    db.setDatabaseName(name);
    
    if (!db.open()) {
        qDebug() << "Database FAILED to open.";
        qDebug() << db.lastError();
    } else {
        qDebug() << "Database opened.";
        dbOpened->setStatusTip("SQLite database opened");
        dbOpened->showStatusText();
        dbQuery = new QSqlQuery;
    }
}

void MainWindow::viewTable()
{
    QString table;
    //table = "SELECT * FROM '";
    table.append(ui->viewEdit->text());
    //table.append("'");
    
    //dbQuery.exec(table);
    
    QSqlTableModel * tmodel = new QSqlTableModel;
    tmodel->setTable(table);
    tmodel->select();
    tmodel->setEditStrategy(QSqlTableModel::OnManualSubmit);
    
    tview = new QTableView;
    tview->setModel(tmodel);
    tview->show();
}

void MainWindow::createTable()
{
    QString coll = ui->leftEdit->text();
    QString colr = ui->rightEdit->text();
    QString table = ui->createEdit->text();
    
    QString typel = "";
    QString typer = "";
    
    if (ui->leftBox->currentText() == "INTEGER")
    {
        typel = "int";
    } else {
        typel = "text";
    }
    
    if (ui->rightBox->currentText() == "INTEGER")
    {
        typer = "int";
    } else {
        typer = "text";
    }
    
    QString query = "CREATE TABLE ";
    query.append(table);
    query.append(" ('");
    query.append(coll);
    query.append("' '");
    query.append(typel);
    query.append("', '");
    query.append(colr);
    query.append("' '");
    query.append(typel);
    query.append("')");
    
    
    QSqlQuery newq;
    bool executed = newq.exec(query);
    
    if (executed) {
        dbOpened->setStatusTip("Query run");
        dbOpened->showStatusText();
    } else {
        dbOpened->setStatusTip("Query failed");
        dbOpened->showStatusText();
    }
}

void MainWindow::executeQuery()
{
    QString query = "";
    query.append(ui->queryEdit->text());
    
    QSqlQuery newq;
    bool executed = newq.exec(query);
    
    if (executed) {
        dbOpened->setStatusTip("Query run");
        dbOpened->showStatusText();
    } else {
        dbOpened->setStatusTip("Query failed");
        dbOpened->showStatusText();
    }
    
    while (newq.next())
    {
        qDebug() << newq.value(0).toString();
        
    }
}

void MainWindow::p04()
{
    QString coll = ui->leftValue->text();
    QString colr = ui->rightValue->text();
    QString table = ui->targetEdit->text();
    
    QString query = "INSERT INTO ";
    query.append(table);
    query.append(" VALUES ('");
    query.append(coll);
    query.append("', '");
    query.append(colr);
    query.append("')");
    
    
    QSqlQuery newq;
    bool executed = newq.exec(query);
    
    if (executed) {
        dbOpened->setStatusTip("Query run");
        dbOpened->showStatusText();
    } else {
        dbOpened->setStatusTip("Query failed");
        dbOpened->showStatusText();
    }
}

void MainWindow::p05()
{
    QString coln = ui->colEdit->text();
    QString colv = ui->valEdit->text();
    QString table = ui->selectEdit->text();
    
    QString query = "SELECT * FROM ";
    query.append(table);
    query.append(" WHERE '");
    query.append(coln);
    query.append("' = '");
    query.append(colv);
    query.append("'");
    
    
    QSqlQuery newq;
    bool executed = newq.exec(query);
    
    if (executed) {
        dbOpened->setStatusTip("Query run");
        dbOpened->showStatusText();
    } else {
        dbOpened->setStatusTip("Query failed");
        dbOpened->showStatusText();
    }
    
    while (newq.next())
    {
        qDebug() << newq.value(0).toString();
        
    }
}
















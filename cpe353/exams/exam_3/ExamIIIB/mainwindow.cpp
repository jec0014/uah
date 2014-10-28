#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFile>
#include <QFileDialog>
#include <QTextStream>
#include <QLineEdit>
#include <QSpinBox>
#include <QLabel>
#include <QtDebug>
#include <QEvent>
#include <QMouseEvent>
#include <QHostAddress>
#include <QTimer>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Problem 1
    connect(ui->leftDial, SIGNAL(valueChanged(int)), ui->rightDial, SLOT(setValue(int)));
    connect(ui->leftDial, SIGNAL(valueChanged(int)), ui->rightLabel, SLOT(setNum(int)));

    connect(ui->rightDial, SIGNAL(valueChanged(int)), ui->leftDial, SLOT(setValue(int)));
    connect(ui->rightDial, SIGNAL(valueChanged(int)), ui->leftLabel, SLOT(setNum(int)));

    // Problem 2
    timer = new QTimer(this);
    timer->setInterval(1000);
    timer->start();

    connect(timer, SIGNAL(timeout()), this, SLOT(toggleButton()));
    connect(ui->fastButton, SIGNAL(clicked()), this, SLOT(setTimerFast()));
    connect(ui->slowButton, SIGNAL(clicked()), this, SLOT(setTimerSlow()));

    ui->outputButton->installEventFilter(this);

    // Problem 3
    connect(ui->lineEditQt4, SIGNAL(textChanged(QString)), ui->labelQt4, SLOT(setText(QString)));
    connect(ui->spinBoxQt5, static_cast<void  (QSpinBox::*)(int)>(&QSpinBox::valueChanged), ui->labelQt5, static_cast<void  (QLabel::*)(int)>(&QLabel::setNum));
    ui->spinBoxQt5->setMaximum(9);
    ui->spinBoxQt5->setMinimum(0);
    ui->lineEditQt4->setMaxLength(6);
    ui->lineEditQt4->setInputMask("AAAAAA");
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::toggleButton()
{
    if (ui->outputButton->isChecked())
    {
        ui->outputButton->setChecked(false);
    } else {
        ui->outputButton->setChecked(true);
    }
}

void MainWindow::setTimerFast()
{
    timer->setInterval(100);
}

void MainWindow::setTimerSlow()
{
    timer->setInterval(1000);
}

void MainWindow::setTimerInt(int val)
{
    timer->setInterval(val);
}

bool MainWindow::eventFilter(QObject *watched, QEvent *e)
{
    if (watched == ui->outputButton && e->type() == QEvent::MouseButtonPress)
    {
        return true;
    }
    return QWidget::eventFilter(watched, e);
}






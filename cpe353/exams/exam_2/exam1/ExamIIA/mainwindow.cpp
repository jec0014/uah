#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Problem #1
    ui->changeMeLabel->setText("Chargers");
    ui->changeMeLabel->setFrameShape(QFrame::Box);

    // Problem #2
    connect(ui->sliderQt4, SIGNAL(valueChanged(int)), ui->spinboxQt4, SLOT(setValue(int)));
    connect(ui->spinboxQt4, SIGNAL(valueChanged(int)), ui->sliderQt4, SLOT(setValue(int)));

    // Problem #3
    connect(ui->dialQt5, &QDial::valueChanged, ui->spinboxQt5, &QSpinBox::setValue);
    connect(ui->spinboxQt5, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), ui->dialQt5, &QDial::setValue);

    // Problem #4
    //QObject::connect(&emitButton, &QLineEdit::customSignal, &cutomSignalFunction)

    // Problem #5
    //QToolBar    toolbar;
    //QMenuBar   Exam;
    //toolBarArea.
    //mainToolBar.addWidget(&Exam);



    // Problem #6


}

MainWindow::~MainWindow()
{
    delete ui;
}



#include "mainwindow.h"
#include "ui_mainwindow.h"

/*
 * LCS Project
 *
 * 1. Ability to enter 2 strings of arbitrary length
 * 2. Ability to generate 2 random strings of arbitrary length if user does not wish to supply his or her own string
 * 3. Visually show the tables
 * 4. Ability to execute program step by step - go backward one step, go forward one step etc. Also highlight the step being executed in the algorithm (similar to shown on the notes)
 * 5. Visually backtrack to identify the LCS (similar to shown on the notes)
 *
 */

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

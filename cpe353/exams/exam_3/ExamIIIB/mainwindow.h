#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMenu>
#include <QAction>
#include <QTcpSocket>
#include <QTimer>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    bool eventFilter (QObject *watched, QEvent *e);

protected:

private:
    Ui::MainWindow *ui;
    int updateTimer;
    bool speed;
    QTimer* timer;

private slots:
    void toggleButton();
    void setTimerFast();
    void setTimerSlow();

};

#endif // MAINWINDOW_H

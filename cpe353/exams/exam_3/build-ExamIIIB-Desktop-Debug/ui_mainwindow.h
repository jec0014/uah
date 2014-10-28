/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.3.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDial>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout;
    QTabWidget *tabWidget;
    QWidget *tab_6;
    QVBoxLayout *verticalLayout_3;
    QTextBrowser *textBrowser_2;
    QWidget *tab_2;
    QVBoxLayout *verticalLayout_4;
    QTextBrowser *textBrowser_3;
    QHBoxLayout *horizontalLayout_2;
    QSpacerItem *horizontalSpacer_12;
    QLabel *leftLabel;
    QDial *leftDial;
    QSpacerItem *horizontalSpacer_3;
    QDial *rightDial;
    QLabel *rightLabel;
    QSpacerItem *horizontalSpacer_13;
    QWidget *tab_4;
    QVBoxLayout *verticalLayout_8;
    QTextBrowser *textBrowser_5;
    QHBoxLayout *horizontalLayout_4;
    QSpacerItem *horizontalSpacer_8;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout_6;
    QRadioButton *slowButton;
    QRadioButton *fastButton;
    QSpacerItem *horizontalSpacer_7;
    QRadioButton *outputButton;
    QSpacerItem *horizontalSpacer_11;
    QWidget *tab;
    QVBoxLayout *verticalLayout_2;
    QTextBrowser *textBrowser_8;
    QFrame *line;
    QHBoxLayout *horizontalLayout_9;
    QHBoxLayout *horizontalLayout;
    QLabel *label_2;
    QLineEdit *lineEditQt4;
    QSpacerItem *horizontalSpacer;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_4;
    QLabel *labelQt4;
    QFrame *line_2;
    QHBoxLayout *horizontalLayout_8;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_5;
    QSpinBox *spinBoxQt5;
    QSpacerItem *horizontalSpacer_2;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_6;
    QLabel *labelQt5;
    QWidget *tab_3;
    QVBoxLayout *verticalLayout_5;
    QTextBrowser *textBrowser_4;
    QHBoxLayout *horizontalLayout_3;
    QSpacerItem *horizontalSpacer_4;
    QLabel *label;
    QSpacerItem *horizontalSpacer_5;
    QTextEdit *textEdit;
    QWidget *tab_5;
    QVBoxLayout *verticalLayout_7;
    QTextBrowser *textBrowser_6;
    QHBoxLayout *horizontalLayout_10;
    QSpacerItem *horizontalSpacer_10;
    QLineEdit *sendEdit;
    QSpacerItem *horizontalSpacer_6;
    QPushButton *sendButton;
    QSpacerItem *horizontalSpacer_9;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(632, 558);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        verticalLayout = new QVBoxLayout(centralWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tab_6 = new QWidget();
        tab_6->setObjectName(QStringLiteral("tab_6"));
        verticalLayout_3 = new QVBoxLayout(tab_6);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        textBrowser_2 = new QTextBrowser(tab_6);
        textBrowser_2->setObjectName(QStringLiteral("textBrowser_2"));

        verticalLayout_3->addWidget(textBrowser_2);

        tabWidget->addTab(tab_6, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QStringLiteral("tab_2"));
        verticalLayout_4 = new QVBoxLayout(tab_2);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        textBrowser_3 = new QTextBrowser(tab_2);
        textBrowser_3->setObjectName(QStringLiteral("textBrowser_3"));

        verticalLayout_4->addWidget(textBrowser_3);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalSpacer_12 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_12);

        leftLabel = new QLabel(tab_2);
        leftLabel->setObjectName(QStringLiteral("leftLabel"));

        horizontalLayout_2->addWidget(leftLabel);

        leftDial = new QDial(tab_2);
        leftDial->setObjectName(QStringLiteral("leftDial"));
        leftDial->setMaximum(9);

        horizontalLayout_2->addWidget(leftDial);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_3);

        rightDial = new QDial(tab_2);
        rightDial->setObjectName(QStringLiteral("rightDial"));
        rightDial->setMaximum(9);
        rightDial->setValue(9);

        horizontalLayout_2->addWidget(rightDial);

        rightLabel = new QLabel(tab_2);
        rightLabel->setObjectName(QStringLiteral("rightLabel"));

        horizontalLayout_2->addWidget(rightLabel);

        horizontalSpacer_13 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_13);


        verticalLayout_4->addLayout(horizontalLayout_2);

        tabWidget->addTab(tab_2, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName(QStringLiteral("tab_4"));
        verticalLayout_8 = new QVBoxLayout(tab_4);
        verticalLayout_8->setSpacing(6);
        verticalLayout_8->setContentsMargins(11, 11, 11, 11);
        verticalLayout_8->setObjectName(QStringLiteral("verticalLayout_8"));
        textBrowser_5 = new QTextBrowser(tab_4);
        textBrowser_5->setObjectName(QStringLiteral("textBrowser_5"));

        verticalLayout_8->addWidget(textBrowser_5);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        horizontalSpacer_8 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_8);

        groupBox = new QGroupBox(tab_4);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        verticalLayout_6 = new QVBoxLayout(groupBox);
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setContentsMargins(11, 11, 11, 11);
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        slowButton = new QRadioButton(groupBox);
        slowButton->setObjectName(QStringLiteral("slowButton"));
        slowButton->setChecked(true);

        verticalLayout_6->addWidget(slowButton);

        fastButton = new QRadioButton(groupBox);
        fastButton->setObjectName(QStringLiteral("fastButton"));

        verticalLayout_6->addWidget(fastButton);


        horizontalLayout_4->addWidget(groupBox);

        horizontalSpacer_7 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_7);

        outputButton = new QRadioButton(tab_4);
        outputButton->setObjectName(QStringLiteral("outputButton"));
        outputButton->setCheckable(true);
        outputButton->setChecked(false);

        horizontalLayout_4->addWidget(outputButton);

        horizontalSpacer_11 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_11);


        verticalLayout_8->addLayout(horizontalLayout_4);

        tabWidget->addTab(tab_4, QString());
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        verticalLayout_2 = new QVBoxLayout(tab);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        textBrowser_8 = new QTextBrowser(tab);
        textBrowser_8->setObjectName(QStringLiteral("textBrowser_8"));

        verticalLayout_2->addWidget(textBrowser_8);

        line = new QFrame(tab);
        line->setObjectName(QStringLiteral("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout_2->addWidget(line);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setObjectName(QStringLiteral("horizontalLayout_9"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label_2 = new QLabel(tab);
        label_2->setObjectName(QStringLiteral("label_2"));

        horizontalLayout->addWidget(label_2);

        lineEditQt4 = new QLineEdit(tab);
        lineEditQt4->setObjectName(QStringLiteral("lineEditQt4"));

        horizontalLayout->addWidget(lineEditQt4);


        horizontalLayout_9->addLayout(horizontalLayout);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_9->addItem(horizontalSpacer);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        label_4 = new QLabel(tab);
        label_4->setObjectName(QStringLiteral("label_4"));

        horizontalLayout_5->addWidget(label_4);

        labelQt4 = new QLabel(tab);
        labelQt4->setObjectName(QStringLiteral("labelQt4"));
        labelQt4->setFrameShape(QFrame::Box);

        horizontalLayout_5->addWidget(labelQt4);


        horizontalLayout_9->addLayout(horizontalLayout_5);


        verticalLayout_2->addLayout(horizontalLayout_9);

        line_2 = new QFrame(tab);
        line_2->setObjectName(QStringLiteral("line_2"));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        verticalLayout_2->addWidget(line_2);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setObjectName(QStringLiteral("horizontalLayout_8"));
        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        label_5 = new QLabel(tab);
        label_5->setObjectName(QStringLiteral("label_5"));

        horizontalLayout_6->addWidget(label_5);

        spinBoxQt5 = new QSpinBox(tab);
        spinBoxQt5->setObjectName(QStringLiteral("spinBoxQt5"));

        horizontalLayout_6->addWidget(spinBoxQt5);


        horizontalLayout_8->addLayout(horizontalLayout_6);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_8->addItem(horizontalSpacer_2);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        label_6 = new QLabel(tab);
        label_6->setObjectName(QStringLiteral("label_6"));

        horizontalLayout_7->addWidget(label_6);

        labelQt5 = new QLabel(tab);
        labelQt5->setObjectName(QStringLiteral("labelQt5"));
        labelQt5->setFrameShape(QFrame::Box);
        labelQt5->setAlignment(Qt::AlignCenter);

        horizontalLayout_7->addWidget(labelQt5);


        horizontalLayout_8->addLayout(horizontalLayout_7);


        verticalLayout_2->addLayout(horizontalLayout_8);

        tabWidget->addTab(tab, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QStringLiteral("tab_3"));
        verticalLayout_5 = new QVBoxLayout(tab_3);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        textBrowser_4 = new QTextBrowser(tab_3);
        textBrowser_4->setObjectName(QStringLiteral("textBrowser_4"));

        verticalLayout_5->addWidget(textBrowser_4);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_4);

        label = new QLabel(tab_3);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout_3->addWidget(label);

        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_5);


        verticalLayout_5->addLayout(horizontalLayout_3);

        textEdit = new QTextEdit(tab_3);
        textEdit->setObjectName(QStringLiteral("textEdit"));

        verticalLayout_5->addWidget(textEdit);

        tabWidget->addTab(tab_3, QString());
        tab_5 = new QWidget();
        tab_5->setObjectName(QStringLiteral("tab_5"));
        verticalLayout_7 = new QVBoxLayout(tab_5);
        verticalLayout_7->setSpacing(6);
        verticalLayout_7->setContentsMargins(11, 11, 11, 11);
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        textBrowser_6 = new QTextBrowser(tab_5);
        textBrowser_6->setObjectName(QStringLiteral("textBrowser_6"));

        verticalLayout_7->addWidget(textBrowser_6);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setObjectName(QStringLiteral("horizontalLayout_10"));
        horizontalSpacer_10 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_10->addItem(horizontalSpacer_10);

        sendEdit = new QLineEdit(tab_5);
        sendEdit->setObjectName(QStringLiteral("sendEdit"));

        horizontalLayout_10->addWidget(sendEdit);

        horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_10->addItem(horizontalSpacer_6);

        sendButton = new QPushButton(tab_5);
        sendButton->setObjectName(QStringLiteral("sendButton"));

        horizontalLayout_10->addWidget(sendButton);

        horizontalSpacer_9 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_10->addItem(horizontalSpacer_9);


        verticalLayout_7->addLayout(horizontalLayout_10);

        tabWidget->addTab(tab_5, QString());

        verticalLayout->addWidget(tabWidget);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 632, 25));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(3);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        textBrowser_2->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans'; font-size:10pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">Directions:    No collaboration!!</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">You may u"
                        "se Qt Creator (including Qt Assistant) and the CPE 353 course pdf lecture notes on Angel.  </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#ff0000;\">Use of any other resources will be considered cheating and will result in </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">zero credit (0 points) </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#ff0000;\">and additional penalties.</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">When completing the tasks associated with each problem, you may o"
                        "nly modify the files </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">mainwindow.h</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> and </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">mainwindow.cpp   </span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#ff0000;\">Do NOT modify </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">main.cpp </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weig"
                        "ht:600; color:#ff0000;\">and </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">mainwindow.ui </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#ff0000;\">(the form) or you will receive</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> zero credit (0 points).</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">The text browser object at the top of each tab descibes the desired task associated with that tab -- you may need to scroll down to read the complete description.  </span></p>\n"
"<p style=\"-qt-p"
                        "aragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">Your goal is to make your program's behavior match that of the sample solution!!!</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">See Angel for directions on how to execute a copy of the sample solution.</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-"
                        "top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0617f3;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0617f3;\">When you are finished, upload both </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#ff0000;\">mainwindow.h</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0617f3;\"> and </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#ff0000;\">mainwindow.cpp</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0617f3;\"> as attachments to your Angel dropbox submission.  </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; "
                        "font-weight:600; color:#000000;\">Be sure both files are attached to each submission you make.</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0617f3;\"><br /></p></body></html>", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab_6), QApplication::translate("MainWindow", "Directions", 0));
        textBrowser_3->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans'; font-size:10pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">(20%) Topic:  Interact with form widgets</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">Complete all the following tasks.</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; fon"
                        "t-size:13pt; font-weight:600;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">* Use a Qt/C++ statement(s) such that changes in the </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">leftDial</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> object's value updates the value of the </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">rightDial</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> object</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"><br /></p>\n"
"<p style=\" margin-top:"
                        "0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">* Use a Qt/C++ statement(s) such that changes in the </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">rightDial</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> object's value updates the value of the </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">leftDial</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> object</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-i"
                        "ndent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">* Use a Qt/C++ statement(s) to ensure that the </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">leftLabel</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> and </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">rightLabel</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> values are updated correctly</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#000000;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI"
                        "'; font-size:13pt; font-weight:600; color:#ff0000;\">Do NOT use the Designer tool to modify the form itself or you will receive</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#000000;\"> zero credit (0 points).</span></p></body></html>", 0));
        leftLabel->setText(QApplication::translate("MainWindow", "0", 0));
        rightLabel->setText(QApplication::translate("MainWindow", "9", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("MainWindow", "P01", 0));
        textBrowser_5->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans'; font-size:10pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">(20%) Topic:  Using QTimer objects and event filters</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">Complete all the following tasks.</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Gra"
                        "nde UI'; font-size:13pt; font-weight:600;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">* Add Qt/C++ statement(s) to check/uncheck timer output at slow rate</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">* Add Qt/C++ statement(s) to check/uncheck timer output at fast rate</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucid"
                        "a Grande UI'; font-size:13pt; font-weight:600;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">* Add an </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#ff0000;\">event filter</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> to block direct checking of timer output (must use an event filter to receive credit)</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#000000;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:"
                        "600; color:#ff0000;\">Do NOT use the Designer tool to modify the form itself or you will receive</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#000000;\"> zero credit (0 points).</span></p></body></html>", 0));
        groupBox->setTitle(QApplication::translate("MainWindow", "Rate", 0));
        slowButton->setText(QApplication::translate("MainWindow", "1 sec on, 1 sec off", 0));
        fastButton->setText(QApplication::translate("MainWindow", "0.1 sec on, 0.1 sec 0ff", 0));
        outputButton->setText(QApplication::translate("MainWindow", "Timer Output", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QApplication::translate("MainWindow", "P02", 0));
        textBrowser_8->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans'; font-size:10pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">(20%)  Topic:  Using </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">connect</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> statements as specified below </span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">Complete all th"
                        "e following tasks.</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">* Use a </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">Qt4-style connect</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> statement to link the changing of the </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">lineEditQt4</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> object's value to changing the </span><span style=\" font-family:'.Lucida Grande UI'; font-siz"
                        "e:13pt; font-weight:600; color:#0000ff;\">labelQt4</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> object's value</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">* Use a </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#ff0000;\">Qt5-style connect</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> statement to link the changing of the </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#ff0000;\">spinBoxQt5</span><span style=\" font-family:'.Lucida Grande"
                        " UI'; font-size:13pt; font-weight:600;\"> object's value to changing the </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#ff0000;\">labelQt5</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> object's value</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">* Add Qt/C++ statements to restrict range of </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">spinBoxQt5</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> to 0-9 inclusive [0,9]</span></p>\n"
""
                        "<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">* Add Qt/C++ statements to allow from zero to six letters A-Z and/or a-z as inputs for </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">lineEditQt4   </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#000000;\">(allow uppercase and lowercase letters only)</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#000000"
                        ";\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#ff0000;\">Do NOT use the Designer tool to modify the form itself or you will receive</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#000000;\"> zero credit (0 points).</span></p></body></html>", 0));
        label_2->setText(QApplication::translate("MainWindow", "A", 0));
        label_4->setText(QApplication::translate("MainWindow", "B", 0));
        labelQt4->setText(QString());
        label_5->setText(QApplication::translate("MainWindow", "C", 0));
        label_6->setText(QApplication::translate("MainWindow", "D", 0));
        labelQt5->setText(QApplication::translate("MainWindow", "0", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("MainWindow", "P03", 0));
        textBrowser_4->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans'; font-size:10pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">(20%) Topic:  Using files in Qt </span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">Complete the following tasks.</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt;"
                        " font-weight:600;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">* Use Qt/C++ statement(s) to add a </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">Save File</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> option to a pulldown menu called </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">Exam</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Gran"
                        "de UI'; font-size:13pt; font-weight:600;\">* Add Qt/C++ statement(s) such that triggering of the </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">Save File</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> option will result in the display of a dialog for selecting name of text file (.txt) to save</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">* Use Qt/C++ statement(s) to save all text from the </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">textEdit</span><span style="
                        "\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> object below as </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#ff0000;\">plain text</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> to the selected file.  The text saved must exactly match in the </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">textEdit</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> object for credit.</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#000000;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; fo"
                        "nt-weight:600; color:#ff0000;\">Do NOT use the Designer tool to modify the form itself or you will receive</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#000000;\"> zero credit (0 points).</span></p></body></html>", 0));
        label->setText(QApplication::translate("MainWindow", "Name of Object Below = textEdit", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("MainWindow", "P04", 0));
        textBrowser_6->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans'; font-size:10pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">(20%) Topic:  TCP Sockets</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">Complete all the following tasks.</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; fo"
                        "nt-weight:600;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">* Add Qt/C++ statement(s) such that clicking the </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">sendButton</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> object will trigger sending the text from the </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">sendEdit</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> as a message via TCP port </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">5151</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> of </span><span style=\" font-family:'.Lucida Grande UI'; "
                        "font-size:13pt; font-weight:600; color:#0000ff;\">LocalHost     </span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">* Add Qt/C++ statement(s) such that the </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">sendEdit </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">object is cleared after the text has been sent</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600"
                        ";\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\">* For full credit, this send function must work each time </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">sendButton</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600;\"> is clicked</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#000000;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#000000;\">Hints:</span></p>\n"
"<p style=\" margin-top:0px; m"
                        "argin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#000000;\">Type </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#ff0000;\">nc -l 5151</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#000000;\"> in a terminal window </span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#0000ff;\">before</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#000000;\"> trying to send a message in order to verify the message has been sent (lowercase L, not digit one)</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#000000;\"><br /></p>\n"
"<p style=\" mar"
                        "gin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#ff0000;\">Do NOT use the Designer tool to modify the main form itself or to create the pop-up dialog or you will receive</span><span style=\" font-family:'.Lucida Grande UI'; font-size:13pt; font-weight:600; color:#000000;\"> zero credit (0 points).</span></p></body></html>", 0));
        sendButton->setText(QApplication::translate("MainWindow", "Send", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab_5), QApplication::translate("MainWindow", "P05", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H

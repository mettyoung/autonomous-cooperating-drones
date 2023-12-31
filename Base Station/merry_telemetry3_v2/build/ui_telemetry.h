/********************************************************************************
** Form generated from reading UI file 'telemetry.ui'
**
** Created by: Qt User Interface Compiler version 5.0.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TELEMETRY_H
#define UI_TELEMETRY_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QWidget>
#include "qcustomplot.h"

QT_BEGIN_NAMESPACE

class Ui_Telemetry
{
public:
    QGridLayout *gridLayout;
    QPushButton *btn_land2;
    QLabel *label_2;
    QPushButton *btn_legends;
    QLabel *label;
    QPushButton *btn_start2;
    QLabel *label_6;
    QLabel *lbl_quad2_stat;
    QProgressBar *pb_quad2;
    QProgressBar *pb_quad1;
    QTabWidget *tab_telemetry;
    QWidget *tab_quad1;
    QGridLayout *gridLayout_2;
    QCustomPlot *plot_x1;
    QCustomPlot *plot_height1;
    QCustomPlot *plot_y1;
    QCustomPlot *plot_yaw1;
    QWidget *tab_quad2;
    QGridLayout *gridLayout_3;
    QCustomPlot *plot_x2;
    QCustomPlot *plot_y2;
    QCustomPlot *plot_height2;
    QCustomPlot *plot_yaw2;
    QWidget *tab_control;
    QGridLayout *gridLayout_4;
    QComboBox *comboBox;
    QSpacerItem *verticalSpacer;
    QSpacerItem *horizontalSpacer_4;
    QPushButton *btn_arm;
    QLineEdit *txt_yaw;
    QLineEdit *txt_height;
    QPushButton *btn_height;
    QPushButton *btn_yaw;
    QSpacerItem *horizontalSpacer_2;
    QSpacerItem *verticalSpacer_2;
    QPushButton *btn_start;
    QSpacerItem *horizontalSpacer_3;
    QSpacerItem *horizontalSpacer_5;
    QLabel *label_4;
    QPushButton *btn_land1;
    QPushButton *btn_start1;
    QLabel *label_5;
    QLabel *lbl_quad1_stat;
    QPushButton *btn_arm2;
    QPushButton *btn_reset;
    QPushButton *btn_print;
    QSpacerItem *horizontalSpacer;
    QPushButton *btn_arm1;
    QLabel *label_3;
    QSpacerItem *horizontalSpacer_6;

    void setupUi(QWidget *Telemetry)
    {
        if (Telemetry->objectName().isEmpty())
            Telemetry->setObjectName(QStringLiteral("Telemetry"));
        Telemetry->resize(1106, 726);
        gridLayout = new QGridLayout(Telemetry);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        btn_land2 = new QPushButton(Telemetry);
        btn_land2->setObjectName(QStringLiteral("btn_land2"));

        gridLayout->addWidget(btn_land2, 4, 4, 1, 1);

        label_2 = new QLabel(Telemetry);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setAutoFillBackground(false);

        gridLayout->addWidget(label_2, 1, 2, 1, 1);

        btn_legends = new QPushButton(Telemetry);
        btn_legends->setObjectName(QStringLiteral("btn_legends"));

        gridLayout->addWidget(btn_legends, 4, 8, 1, 1);

        label = new QLabel(Telemetry);
        label->setObjectName(QStringLiteral("label"));

        gridLayout->addWidget(label, 4, 2, 1, 1);

        btn_start2 = new QPushButton(Telemetry);
        btn_start2->setObjectName(QStringLiteral("btn_start2"));

        gridLayout->addWidget(btn_start2, 1, 4, 2, 1);

        label_6 = new QLabel(Telemetry);
        label_6->setObjectName(QStringLiteral("label_6"));

        gridLayout->addWidget(label_6, 4, 5, 1, 1);

        lbl_quad2_stat = new QLabel(Telemetry);
        lbl_quad2_stat->setObjectName(QStringLiteral("lbl_quad2_stat"));

        gridLayout->addWidget(lbl_quad2_stat, 2, 6, 2, 1);

        pb_quad2 = new QProgressBar(Telemetry);
        pb_quad2->setObjectName(QStringLiteral("pb_quad2"));
        pb_quad2->setValue(0);

        gridLayout->addWidget(pb_quad2, 4, 6, 1, 1);

        pb_quad1 = new QProgressBar(Telemetry);
        pb_quad1->setObjectName(QStringLiteral("pb_quad1"));
        pb_quad1->setValue(0);

        gridLayout->addWidget(pb_quad1, 4, 3, 1, 1);

        tab_telemetry = new QTabWidget(Telemetry);
        tab_telemetry->setObjectName(QStringLiteral("tab_telemetry"));
        tab_quad1 = new QWidget();
        tab_quad1->setObjectName(QStringLiteral("tab_quad1"));
        gridLayout_2 = new QGridLayout(tab_quad1);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        plot_x1 = new QCustomPlot(tab_quad1);
        plot_x1->setObjectName(QStringLiteral("plot_x1"));

        gridLayout_2->addWidget(plot_x1, 1, 0, 1, 1);

        plot_height1 = new QCustomPlot(tab_quad1);
        plot_height1->setObjectName(QStringLiteral("plot_height1"));

        gridLayout_2->addWidget(plot_height1, 0, 0, 1, 1);

        plot_y1 = new QCustomPlot(tab_quad1);
        plot_y1->setObjectName(QStringLiteral("plot_y1"));

        gridLayout_2->addWidget(plot_y1, 1, 1, 1, 1);

        plot_yaw1 = new QCustomPlot(tab_quad1);
        plot_yaw1->setObjectName(QStringLiteral("plot_yaw1"));

        gridLayout_2->addWidget(plot_yaw1, 0, 1, 1, 1);

        tab_telemetry->addTab(tab_quad1, QString());
        tab_quad2 = new QWidget();
        tab_quad2->setObjectName(QStringLiteral("tab_quad2"));
        gridLayout_3 = new QGridLayout(tab_quad2);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        plot_x2 = new QCustomPlot(tab_quad2);
        plot_x2->setObjectName(QStringLiteral("plot_x2"));

        gridLayout_3->addWidget(plot_x2, 1, 0, 1, 1);

        plot_y2 = new QCustomPlot(tab_quad2);
        plot_y2->setObjectName(QStringLiteral("plot_y2"));

        gridLayout_3->addWidget(plot_y2, 1, 1, 1, 1);

        plot_height2 = new QCustomPlot(tab_quad2);
        plot_height2->setObjectName(QStringLiteral("plot_height2"));

        gridLayout_3->addWidget(plot_height2, 0, 0, 1, 1);

        plot_yaw2 = new QCustomPlot(tab_quad2);
        plot_yaw2->setObjectName(QStringLiteral("plot_yaw2"));

        gridLayout_3->addWidget(plot_yaw2, 0, 1, 1, 1);

        tab_telemetry->addTab(tab_quad2, QString());
        tab_control = new QWidget();
        tab_control->setObjectName(QStringLiteral("tab_control"));
        gridLayout_4 = new QGridLayout(tab_control);
        gridLayout_4->setSpacing(6);
        gridLayout_4->setContentsMargins(11, 11, 11, 11);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        comboBox = new QComboBox(tab_control);
        comboBox->setObjectName(QStringLiteral("comboBox"));

        gridLayout_4->addWidget(comboBox, 0, 1, 1, 1);

        verticalSpacer = new QSpacerItem(298, 344, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_4->addItem(verticalSpacer, 6, 1, 1, 4);

        horizontalSpacer_4 = new QSpacerItem(38, 138, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_4->addItem(horizontalSpacer_4, 1, 0, 3, 1);

        btn_arm = new QPushButton(tab_control);
        btn_arm->setObjectName(QStringLiteral("btn_arm"));

        gridLayout_4->addWidget(btn_arm, 5, 3, 1, 1);

        txt_yaw = new QLineEdit(tab_control);
        txt_yaw->setObjectName(QStringLiteral("txt_yaw"));

        gridLayout_4->addWidget(txt_yaw, 2, 1, 1, 1);

        txt_height = new QLineEdit(tab_control);
        txt_height->setObjectName(QStringLiteral("txt_height"));

        gridLayout_4->addWidget(txt_height, 1, 1, 1, 1);

        btn_height = new QPushButton(tab_control);
        btn_height->setObjectName(QStringLiteral("btn_height"));

        gridLayout_4->addWidget(btn_height, 1, 3, 1, 1);

        btn_yaw = new QPushButton(tab_control);
        btn_yaw->setObjectName(QStringLiteral("btn_yaw"));

        gridLayout_4->addWidget(btn_yaw, 2, 3, 1, 1);

        horizontalSpacer_2 = new QSpacerItem(678, 485, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_4->addItem(horizontalSpacer_2, 2, 5, 5, 1);

        verticalSpacer_2 = new QSpacerItem(282, 140, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_4->addItem(verticalSpacer_2, 4, 1, 1, 3);

        btn_start = new QPushButton(tab_control);
        btn_start->setObjectName(QStringLiteral("btn_start"));

        gridLayout_4->addWidget(btn_start, 5, 1, 1, 1);

        horizontalSpacer_3 = new QSpacerItem(39, 138, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_4->addItem(horizontalSpacer_3, 1, 2, 3, 1);

        horizontalSpacer_5 = new QSpacerItem(38, 138, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_4->addItem(horizontalSpacer_5, 1, 4, 3, 1);

        tab_telemetry->addTab(tab_control, QString());

        gridLayout->addWidget(tab_telemetry, 0, 0, 1, 9);

        label_4 = new QLabel(Telemetry);
        label_4->setObjectName(QStringLiteral("label_4"));

        gridLayout->addWidget(label_4, 1, 5, 1, 1);

        btn_land1 = new QPushButton(Telemetry);
        btn_land1->setObjectName(QStringLiteral("btn_land1"));

        gridLayout->addWidget(btn_land1, 4, 1, 1, 1);

        btn_start1 = new QPushButton(Telemetry);
        btn_start1->setObjectName(QStringLiteral("btn_start1"));

        gridLayout->addWidget(btn_start1, 1, 1, 2, 1);

        label_5 = new QLabel(Telemetry);
        label_5->setObjectName(QStringLiteral("label_5"));

        gridLayout->addWidget(label_5, 2, 5, 2, 1);

        lbl_quad1_stat = new QLabel(Telemetry);
        lbl_quad1_stat->setObjectName(QStringLiteral("lbl_quad1_stat"));

        gridLayout->addWidget(lbl_quad1_stat, 2, 3, 2, 1);

        btn_arm2 = new QPushButton(Telemetry);
        btn_arm2->setObjectName(QStringLiteral("btn_arm2"));

        gridLayout->addWidget(btn_arm2, 3, 4, 1, 1);

        btn_reset = new QPushButton(Telemetry);
        btn_reset->setObjectName(QStringLiteral("btn_reset"));

        gridLayout->addWidget(btn_reset, 3, 8, 1, 1);

        btn_print = new QPushButton(Telemetry);
        btn_print->setObjectName(QStringLiteral("btn_print"));

        gridLayout->addWidget(btn_print, 1, 8, 1, 1);

        horizontalSpacer = new QSpacerItem(748, 58, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer, 1, 7, 4, 1);

        btn_arm1 = new QPushButton(Telemetry);
        btn_arm1->setObjectName(QStringLiteral("btn_arm1"));

        gridLayout->addWidget(btn_arm1, 3, 1, 1, 1);

        label_3 = new QLabel(Telemetry);
        label_3->setObjectName(QStringLiteral("label_3"));

        gridLayout->addWidget(label_3, 2, 2, 2, 1);

        horizontalSpacer_6 = new QSpacerItem(48, 58, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer_6, 1, 0, 4, 1);

        btn_reset->raise();
        btn_print->raise();
        btn_legends->raise();
        tab_telemetry->raise();
        btn_start1->raise();
        btn_arm1->raise();
        btn_land1->raise();
        btn_land2->raise();
        btn_arm2->raise();
        btn_start2->raise();
        lbl_quad2_stat->raise();
        pb_quad2->raise();
        pb_quad1->raise();
        lbl_quad1_stat->raise();
        label->raise();
        label_3->raise();
        label_5->raise();
        label_6->raise();
        label_2->raise();
        label_4->raise();

        retranslateUi(Telemetry);

        tab_telemetry->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(Telemetry);
    } // setupUi

    void retranslateUi(QWidget *Telemetry)
    {
        Telemetry->setWindowTitle(QApplication::translate("Telemetry", "Telemetry", 0));
        btn_land2->setText(QApplication::translate("Telemetry", "LAND", 0));
        label_2->setText(QApplication::translate("Telemetry", "QUAD1", 0));
        btn_legends->setText(QApplication::translate("Telemetry", "Hide Legends", 0));
        label->setText(QApplication::translate("Telemetry", "Battery", 0));
        btn_start2->setText(QApplication::translate("Telemetry", "START", 0));
        label_6->setText(QApplication::translate("Telemetry", "Battery", 0));
        lbl_quad2_stat->setText(QApplication::translate("Telemetry", "Disconnected!", 0));
        tab_telemetry->setTabText(tab_telemetry->indexOf(tab_quad1), QApplication::translate("Telemetry", "QUAD1", 0));
        tab_telemetry->setTabText(tab_telemetry->indexOf(tab_quad2), QApplication::translate("Telemetry", "QUAD2", 0));
        comboBox->clear();
        comboBox->insertItems(0, QStringList()
         << QApplication::translate("Telemetry", "QUAD1", 0)
         << QApplication::translate("Telemetry", "QUAD2", 0)
        );
        btn_arm->setText(QApplication::translate("Telemetry", "ARM", 0));
        btn_height->setText(QApplication::translate("Telemetry", "Height", 0));
        btn_yaw->setText(QApplication::translate("Telemetry", "Yaw", 0));
        btn_start->setText(QApplication::translate("Telemetry", "START", 0));
        tab_telemetry->setTabText(tab_telemetry->indexOf(tab_control), QApplication::translate("Telemetry", "CONTROL TEST", 0));
        label_4->setText(QApplication::translate("Telemetry", "QUAD2", 0));
        btn_land1->setText(QApplication::translate("Telemetry", "LAND", 0));
        btn_start1->setText(QApplication::translate("Telemetry", "START", 0));
        label_5->setText(QApplication::translate("Telemetry", "Status", 0));
        lbl_quad1_stat->setText(QApplication::translate("Telemetry", "Disconnected!", 0));
        btn_arm2->setText(QApplication::translate("Telemetry", "ARM", 0));
        btn_reset->setText(QApplication::translate("Telemetry", "RESET", 0));
        btn_print->setText(QApplication::translate("Telemetry", "PRINT", 0));
        btn_arm1->setText(QApplication::translate("Telemetry", "ARM", 0));
        label_3->setText(QApplication::translate("Telemetry", "Status", 0));
    } // retranslateUi

};

namespace Ui {
    class Telemetry: public Ui_Telemetry {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TELEMETRY_H

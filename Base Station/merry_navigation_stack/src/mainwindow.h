#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QScrollBar>
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>
#include <QTextStream>
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

public Q_SLOTS:
    void writeToLogLp0(QString);
    void writeToLogLp1(QString);
    void writeToLogMgp0(QString);
    void writeToLogMgp1(QString);
    void writeToDisplayImage(QImage, QString, uint);
    void setCommandQuad0(double pitch, double yaw);
    void setCommandQuad1(double pitch, double yaw);
private Q_SLOTS:
    void on_generateMapBtn_clicked();
    void on_resetBtn_clicked();

    void on_req0Btn_clicked();

    void on_req1Btn_clicked();

    void on_autonomousBtn_clicked();

    void on_saveBtnMgp0_clicked();

    void on_clearBtnMgp0_clicked();

    void on_saveBtnMgp1_clicked();

    void on_clearBtnMgp1_clicked();

    void on_saveBtnLp0_clicked();

    void on_clearBtnLp0_clicked();

    void on_saveBtnLp1_clicked();

    void on_clearBtnLp1_clicked();

Q_SIGNALS:
    void toggleGenerateMap();
    void resetRgbdslamSystem();
    void reqFrontier0();
    void reqFrontier1();
    void toggleAutonomous();

};

#endif // MAINWINDOW_H

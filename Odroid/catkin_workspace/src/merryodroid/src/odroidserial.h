#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <ros/ros.h>
#include <string>
#include <boost/timer.hpp>
#include <QObject>
#include <QTimer>
#include <QDebug>
#include <QApplication>
#include <QString>

class OdroidSerial : public QObject
{
    Q_OBJECT

public:
    // Constructor
    OdroidSerial(char *portName, float rateRead, float rateWrite);
    void closeSerialPort(int serialPort);   //net

private:
    int serialPort;
    uint throttle_;
    char* portName_;

    bool isStarted;
    bool isArmed;
    
    bool isActuallyArmed_;
    //For UART decoding
    QString uartRead;
    float uartFloatYaw, uartFloatHeight, uartFloatBattery, uartFloatBatteryAve;
    int BattCtr;
    float uartFloatHeightOld;
    float GUIuartFloatYaw;
    bool oldAvailable;
    float offsetYaw, offsetHeight;

    // Timer Related
    float dtRead;
    float dtWrite;
    QTimer *timerRead_;
    QTimer *timerWrite_;
    QTimer *timerLand_;

    // UART functions
    int openSerialPort(char* portName);

    // uart writer
    unsigned char buffer[8];
    int tempYaw, tempPitch, tempRoll, tempThrottle;
    unsigned int encodeCommand(unsigned int command);

    // Buffer Channels
    unsigned int yaw;
    unsigned int pitch;
    unsigned int roll;
    unsigned int throttle;
    unsigned int PIDyaw;
    unsigned int PIDpitch;
    unsigned int PIDroll;
    unsigned int PIDthrottle;

  //void readPort();
    unsigned int getEncodedYaw();
    unsigned int getEncodedPitch();
    unsigned int getEncodedRoll();
    unsigned int getEncodedThrottle();

    float land_height_goal;

private Q_SLOTS:
    void send_land();
public Q_SLOTS:
    void startQuad();
    void armQuad();
    void start();
    void uartListener();
    void uartWriter();
    void writeSerialBytes();
    void readPort();
    void storePIDYaw(unsigned int yaw);
    void storePIDPitch(unsigned int pitch);
    void storePIDRoll(unsigned int roll);
    void storePIDThrottle(unsigned int throttle);
    void commandLand();
    void setOffset();

Q_SIGNALS:
    void started(int port);
    // send uart signals for PID
    void PIDstoreActualYaw(float uartFloatYaw);
    void PIDstoreActualPitch(float uartFloatPitch);
    void PIDstoreActualRoll(float uartFloatRoll);
    void PIDstoreActualHeight(float uartFloatHeight);
    // for GUI
    void GUIstoreActualYaw(float uartFloatYaw);
    void GUIstoreActualPitch(float uartFloatPitch);
    void GUIstoreActualRoll(float uartFloatRoll);
    void GUIstoreActualHeight(float uartFloatHeight);
    void GUIstoreActualBattery(float uartFloatBattery);
    void land(float goal);
    void disarm();
};


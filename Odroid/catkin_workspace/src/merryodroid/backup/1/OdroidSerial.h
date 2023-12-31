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

class OdroidSerial : public QObject
{
    Q_OBJECT

public:
    // Constructor
    OdroidSerial(char *portName);
    void closeSerialPort(int serialPort);   //net

private:
    int serialPort;
    uint throttle_;
    char* portName_;

    bool isStarted;
    bool isArmed;

    // For UART encoding
    unsigned int uart0, uart1, temp1, temp2;
    char tempCommand0, tempCommand1;
    char *pwmCommandUART0, *pwmCommandUART1;

    // UART functions
    int openSerialPort(char* portName);         // net
    void writeSerialBytes(int serialPort, const char *data, int numBytes);
    void readPort();

public Q_SLOTS:
    void throttleRecieved(uint throttleVal);
    void startQuad();
    void armQuad();
    void start();
    void uartListener(int port);
    void commandLand();
    void commandWriter(uint pwm);

Q_SIGNALS:
    void started(int port);
};


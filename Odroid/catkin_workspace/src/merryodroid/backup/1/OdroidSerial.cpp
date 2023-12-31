#include "odroidserial.h"
/*
//
// Copyright 2013 Christopher D. McMurrough
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.
//
*/
/***********************************************************************************************************************
FILENAME: serial_comms.cpp
AUTHORS: Christopher D. McMurrough
DESCRIPTION:
Provides an example of communcating with a USB-serial enabled microcontroller
REVISION HISTORY:
01.19.2014 CDM original file creation
01.20.2014 CDM published under GPL
***********************************************************************************************************************/

/***********************************************************************************************************************
int openSerialPort(char* portName)
attempt to open a serial port with the given name, returning -1 on failure
***********************************************************************************************************************/

int OdroidSerial::openSerialPort(char* portName)
{
    // store the file descriptor for the serial port
    int fd;

    // attempt to open the port
    fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);

    // return -1 if we are unable to open the port
    if(fd == -1)
    {
        return -1;
    }
    else
    {
        // create a structure to store the port settings
        struct termios port_settings;

        // get the current port settings
        tcgetattr(fd, &port_settings);

        // set the baud rates
        cfsetispeed(&port_settings, B115200);
        cfsetospeed(&port_settings, B115200);

        // set 8 bits, no parity, no stop bits
        port_settings.c_cflag &= ~PARENB;
        port_settings.c_cflag &= ~CSTOPB;
        port_settings.c_cflag &= ~CSIZE;
        port_settings.c_cflag |= CS8;

        // set raw input mode
        port_settings.c_lflag |= ~ICANON;

        // apply the settings to the port
        tcsetattr(fd, TCSANOW, &port_settings);

        // set the non blocking functionality
        fcntl(fd, F_SETFL, O_NONBLOCK);

        // return the file descriptor
        return fd;
    }
}

/***********************************************************************************************************************
void closeSerialPort(int serialPort)
close the given serial port
***********************************************************************************************************************/

void OdroidSerial::closeSerialPort(int serialPort)
{
    tcflush(serialPort, TCIOFLUSH);
    close(serialPort);
}

/***********************************************************************************************************************

Rap Cornejo

***********************************************************************************************************************/

// Constructor
OdroidSerial::OdroidSerial(char* portName) : portName_(portName), isStarted(0), isArmed(0){}

/****************************************************************
 *PUBLIC SLOTS
 ***************************************************************/

void OdroidSerial::start()
{
    // attempt to open the serial port
    serialPort = openSerialPort(portName_);

    // check if connected
    if(serialPort == -1)
    {
        printf("unable to open serial port %s \n", portName_);
        qApp->exit();
    }
    else
    {
        printf("%s opened successfully!\n", portName_);
        Q_EMIT started(serialPort);
    }
}

void OdroidSerial::throttleRecieved(uint throttleVal)
{
    if(isStarted && isArmed)
    {
        // Add throttle header
        throttle_ = 0x6000 | throttleVal;

        // Send throttle value for uart encoding
        commandWriter(throttle_);
    }
}

void OdroidSerial::startQuad()
{
    printf("OdroidSerial received signal to start quad");
    isStarted = TRUE;
    commandWriter(0x2001);
}

void OdroidSerial::armQuad()
{
    printf("OdroidSerial received signal to arm quad.");
    isArmed = TRUE;
    commandWriter(0x3001);
}

/*
 * Create a new object of this class to be able to use uartListener
 * Put the object into a separate thread since this function runs on a
 * while(1) loop for continuos readings.
 */
void OdroidSerial::uartListener(int port)
{
    serialPort = port;

    while(1)
    {
        // check for received serial data
        readPort();

        // sleep for 20ms (20*1000 microseconds)
        usleep(20*1000);
    }
}

// Encodes received data then sends to the serial port.
void OdroidSerial::commandWriter(uint pwm)
{
    uart0 = 0x00;
    uart1 = 0x00;
    temp1 = 0x00;
    temp2 = 0x00;

    uart0 = (pwm & 0x007F) << 1;
    uart0 = uart0 + 0x0001;
    temp1 = (pwm & 0x0700) << 1;
    temp2 = (pwm & 0x0080) << 1;
    uart1 = (temp1 + temp2 + (pwm & 0xF000)) >> 8;

    qDebug() << "commandWriter() received: " << uart1 << " " << uart0;

    tempCommand0 = uart0;
    tempCommand1 = uart1;
    pwmCommandUART0 = &tempCommand0;
    pwmCommandUART1 = &tempCommand1;

    writeSerialBytes(serialPort, pwmCommandUART0, 2);
}
// Fail safe; in case of disconnection.
void OdroidSerial::commandLand()
{
    qDebug() << "Landing...";
    commandWriter(1750);
//    while(thro < 2000)
//    {
//        pwm = 0x00;
//        thro = 2000;
//        pwm = 0x6000 + thro;

//        commandWriter(pwm);
//        qDebug() << "Throttle: " << thro;

//        QThread::msleep(500);
//    }
}

/****************************************************************
 *PRIVATE FUNCTION
 ***************************************************************/

void OdroidSerial::readPort()
{
    const int bufferSize = 100;
    const char* buff[bufferSize];
    int n;

    // attempt to read bytes from the port
    n = read(serialPort, buff, bufferSize);

    if(n > 0)
        printf("%c\n", buff);

    // print any received bytes to terminal
    if(n > 0)
    {
        //printf("RECEIVED FROM THE SERIAL PORT: ");
        //for(int i = 0; i < n; i++)
        //{
        //    char ch = buff[i];
        //    printf("%c", ch & 0xff);
        //}
        //printf("\n");
    }
}

void OdroidSerial::writeSerialBytes(int serialPort, const char* data, int numBytes)
{
    int writeCheck = write(serialPort, data, numBytes);

    if(writeCheck > 1)
        printf("Data sent\n");
}


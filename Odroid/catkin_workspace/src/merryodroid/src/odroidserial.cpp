#include "odroidserial.h"
#include <fstream>
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
Lorenzo Mapili

***********************************************************************************************************************/

// Constructor
OdroidSerial::OdroidSerial(char* portName, float rateRead, float rateWrite) : portName_(portName), isStarted(0), isArmed(0),
isActuallyArmed_(false)
{
    this->dtRead = 1/rateRead;
    this->dtWrite = 1/rateWrite;
    this->isStarted = false;
    this->isArmed = false;
    this->offsetYaw = 0;
    this->offsetHeight = 0;
    this->oldAvailable = false;
    this->land_height_goal = 0.5;
    this->uartFloatBatteryAve = 0;
    this->BattCtr = 1;
}

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
    qDebug() << "SERIALPORT:" << serialPort << "\n";
    }

    QObject::connect(this, SIGNAL(disarm()), this, SLOT(startQuad()));
}

void OdroidSerial::startQuad()
{
    printf("OdroidSerial received signal to start quad\n");
    isStarted = TRUE;
}

void OdroidSerial::armQuad()
{
    printf("OdroidSerial received signal to arm quad.\n");
    isArmed = true;
    isActuallyArmed_ = !isActuallyArmed_;
}

void OdroidSerial::uartListener()
{
    timerRead_ = new QTimer();
    timerRead_->start((int)(dtRead*1000));
    connect(timerRead_, SIGNAL(timeout()), this, SLOT(readPort()));
}

void OdroidSerial::uartWriter()
{
    timerWrite_ = new QTimer();
    timerWrite_->start((int)(dtWrite*1000));
    connect(timerWrite_, SIGNAL(timeout()), this, SLOT(writeSerialBytes()));
}

void OdroidSerial::storePIDYaw(unsigned int yaw)
{
    PIDyaw = yaw;
    this->yaw = yaw | 0x7000;
    this->yaw = encodeCommand(this->yaw);
}

void OdroidSerial::storePIDPitch(unsigned int pitch)
{
    PIDpitch = pitch;
    this->pitch = pitch | 0x5000;
    this->pitch = encodeCommand(this->pitch);
}

void OdroidSerial::storePIDRoll(unsigned int roll)
{
    PIDroll = roll;
    this->roll = roll | 0x4000;
    this->roll = encodeCommand(this->roll);
}

void OdroidSerial::storePIDThrottle(unsigned int throttle)
{
    PIDthrottle = throttle;
    this->throttle = throttle | 0x6000;
    this->throttle = encodeCommand(this->throttle);
}

unsigned int OdroidSerial::encodeCommand(unsigned int command)
{
    unsigned int temp = 0;
    command = command | 0x8000;
    temp = (command & 0x0780) << 1;
    command = (command & 0xF07F) | temp;
}

unsigned int OdroidSerial::getEncodedYaw()
{
    return yaw;
}
unsigned int OdroidSerial::getEncodedPitch()
{
    return pitch;
}
unsigned int OdroidSerial::getEncodedRoll()
{
    return roll;
}
unsigned int OdroidSerial::getEncodedThrottle()
{
    return throttle;
}

// Land
void OdroidSerial::commandLand()
{
    qDebug() << "Received command to land.";
    if (!isActuallyArmed_)
	return;

    timerLand_ = new QTimer();
    timerLand_->start(500);
    connect(timerLand_, SIGNAL(timeout()), this, SLOT(send_land()));
/*
    ros::Rate r(2.5);
    while(land_height_goal > -0.2)
    {
        land_height_goal -= 0.1;
        qDebug() << "Landing";
        qDebug() << "Height goal: " << land_height_goal;

        Q_EMIT land(land_height_goal);

        if(land_height_goal < -0.2)
	{
            Q_EMIT disarm();
	    qDebug() << "disarming";
  	}

        r.sleep();
    }
*/
}

void OdroidSerial::send_land()
{
    if(land_height_goal > -0.2)
    {
	land_height_goal -= 0.1;
	Q_EMIT land(land_height_goal);

	if(land_height_goal < -0.2)
	{
	    Q_EMIT disarm();
	    qDebug() << "disarming";
	    timerLand_->stop();
	}
    }
}

/****************************************************************
 *PRIVATE SLOT
 ***************************************************************/

void OdroidSerial::setOffset()
{
    offsetYaw = uartFloatYaw;
    offsetHeight = uartFloatHeight;
}

void OdroidSerial::readPort()
{
    const int bufferSize = 500;
    char buff[bufferSize];
    int n;

    //read bytes from the port
    n = read(serialPort, buff, bufferSize);
    buff[n] = 0;
    if(n > 0)
    {
        uartRead = QString(buff);

        if(uartRead.contains("yprtb", Qt::CaseSensitive))
        {
            uartFloatYaw = uartRead.section("\t", 1, 1).toFloat();
            uartFloatHeight = uartRead.section("\t", 4, 4).toFloat();
            uartFloatBattery = uartRead.section("\t", 5,5).toFloat();

            if(oldAvailable)
            {
                if(std::abs(uartFloatHeight - uartFloatHeightOld) > 1)
                    uartFloatHeight = uartFloatHeightOld;
            }

            uartFloatHeightOld = uartFloatHeight;
            oldAvailable = true;

            Q_EMIT GUIstoreActualHeight(uartFloatHeight);
            Q_EMIT PIDstoreActualYaw(uartFloatYaw);

            uartFloatBatteryAve += uartFloatBattery;
            uartFloatBatteryAve = uartFloatBattery/BattCtr;
            BattCtr++;
            if(BattCtr==10000)
            {
//                qDebug() << "Battery Level : " << uartFloatBattery << "%";
                Q_EMIT GUIstoreActualBattery(uartFloatBattery);
                uartFloatBatteryAve = 0;
                BattCtr = 1;
            }

            uartFloatYaw -= offsetYaw;
            uartFloatHeight -= offsetHeight;

            Q_EMIT GUIstoreActualYaw(uartFloatYaw);
            Q_EMIT PIDstoreActualHeight(uartFloatHeight); 
        }
        else
        {
            //if(uartRead.contains("Not connected!", Qt::CaseSensitive)){}
            //else
            //    qDebug() << uartRead;
        }
    }
}

/****************************************************************
 *PRIVATE FUNCTION
 ***************************************************************/

void OdroidSerial::writeSerialBytes()
{
    tempYaw = getEncodedYaw();
    tempPitch = getEncodedPitch();
    tempRoll = getEncodedRoll();
    tempThrottle = getEncodedThrottle();

    buffer[0] = tempYaw >> 8;
    buffer[1] = tempYaw;
    buffer[2] = tempPitch >> 8;
    buffer[3] = tempPitch;
    buffer[4] = tempRoll >> 8;
    buffer[5] = tempRoll;
    buffer[6] = tempThrottle >> 8;
    buffer[7] = tempThrottle;

//    qDebug() << "pidy" << PIDyaw << uartFloatYaw << offsetYaw;
//    qDebug() << "pidp" << PIDpitch;
//    qDebug() << "pidr" << PIDroll;
//    qDebug() << "pidt" << PIDthrottle << uartFloatHeight << offsetHeight;
  std::ofstream ss;
  ss.open("Log.txt", std::fstream::app);
  ss << "pidy" << PIDyaw << "\tpidp"<< PIDpitch << "\tpidr" << PIDroll << "\tpidt" << PIDthrottle << std::endl;
  ss.close();
    
  qDebug() << "pidy" << PIDyaw << "\tpidp"<< PIDpitch << "\tpidr" << PIDroll << "\tpidt" << PIDthrottle;
    if(isStarted)
    {
        buffer[0] = 0xA0; // 2 | 0x8
        buffer[1] = 0x00;
        int writeCheck = write(serialPort, buffer, 2);
        isStarted = FALSE;
    }
    else if(isArmed)
    {
        buffer[0] = 0xB0; // 3 | 0x8
        buffer[1] = 0x00;
        int writeCheck = write(serialPort, buffer, 2);
        isArmed = FALSE;
    }
    else
    {
        //printf(" : %d, %d, %d, %d, %d, %d, %d, %d\n", buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], buffer[8], buffer[9]);
        int writeCheck = write(serialPort, buffer, 8);
    }
}

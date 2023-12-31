#include "odroidlistener.h"
#include "odroidserial.h"
#include "tcprosstatman.h"
#include "yawcontroller.h"
#include "heightcontroller.h"
#include "prcontroller.h"
#include <QThread>
#include <QApplication>
#include <signal.h>

void signal_callback_handler(int signum)
{
   printf("Caught signal %d\n",signum);
   // Cleanup and close up stuff here

   // Terminate program
    qApp->exit();
}

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    ros::init(argc, argv, "Merry_Odroid");
    printf("Started Merry_Odroid node.\n");

    // Initializations
    ros::NodeHandle n_odroid;
    OdroidListener *odroidListener = new OdroidListener(&n_odroid, "quad2");
    OdroidSerial *odroidSerial = new OdroidSerial("/dev/ttyACM99", 10000, 50); // Port, reader fq, writer fq
    TcpRosStatMan *tcprosstatman = new TcpRosStatMan("quad2","BASESERVER",10,3,&n_odroid);

    //Yaw Controller
    YawController *yawController = new YawController(100, -5, 0, 0, 1540, 1460);
    //Pitch Controller
    PRController *pitchController = new PRController(1, -200, -0.0, -0.0, 1515, 1485);
    //Roll Controller
    PRController *rollController = new PRController(1, 200, 0.0, 0, 1500, 1500);
    //Height Controller
    HeightController *throttleController = new HeightController(100, -500, -25, 0, 2000, 1200);

    QThread * thread1 = new QThread();
    QThread * thread2 = new QThread();
    QThread * thread3 = new QThread();

    // threads for YPRT Controllers
    QThread * thread4 = new QThread();
    QThread * thread5 = new QThread();
    QThread * thread6 = new QThread();
    QThread * thread7 = new QThread();

    odroidListener->moveToThread(thread1);
    odroidSerial->moveToThread(thread2);
    tcprosstatman->moveToThread(thread3);

    //move YPRT Controllers to Thread
    yawController->moveToThread(thread4);
    pitchController->moveToThread(thread5);
    rollController->moveToThread(thread6);
    throttleController->moveToThread(thread7);

    // Start threads
    QObject::connect(thread1, SIGNAL(started()), odroidListener, SLOT(start()));
    QObject::connect(thread2, SIGNAL(started()), odroidSerial, SLOT(start()));
    QObject::connect(thread2, SIGNAL(started()), odroidSerial, SLOT(uartListener()));
    QObject::connect(thread2, SIGNAL(started()), odroidSerial, SLOT(uartWriter()));
    QObject::connect(thread3, SIGNAL(started()), tcprosstatman, SLOT(start()));
    QObject::connect(thread4, SIGNAL(started()), yawController, SLOT(start()));
    QObject::connect(thread5, SIGNAL(started()), pitchController, SLOT(start()));
    QObject::connect(thread6, SIGNAL(started()), rollController, SLOT(start()));
    QObject::connect(thread7, SIGNAL(started()), throttleController, SLOT(start()));

    // Connections from odroidlistener
    QObject::connect(odroidListener, SIGNAL(quad_start()), odroidSerial, SLOT(startQuad()));
    QObject::connect(odroidListener, SIGNAL(quad_start()), yawController, SLOT(setOffset()));
    QObject::connect(odroidListener, SIGNAL(quad_start()), pitchController, SLOT(setOffset()));
    QObject::connect(odroidListener, SIGNAL(quad_start()), rollController, SLOT(setOffset()));
    QObject::connect(odroidListener, SIGNAL(quad_start()), throttleController, SLOT(setOffset()));
    QObject::connect(odroidListener, SIGNAL(quad_start()), odroidSerial, SLOT(setOffset()));
    QObject::connect(odroidListener, SIGNAL(quad_arm()), odroidSerial, SLOT(armQuad()));
    QObject::connect(odroidListener, SIGNAL(quad_land()), odroidSerial, SLOT(commandLand()));
    // Connection from tcprosstatman
    QObject::connect(tcprosstatman, SIGNAL(disconnected()), odroidSerial,SLOT(commandLand()));

    // Connection from odroidserial
    QObject::connect(odroidSerial, SIGNAL(land(float)), throttleController, SLOT(storeGoal(float)));

    // CONTROLLER CONNECTIONS
    // Connection for Yaw Controller
    QObject::connect(odroidSerial, SIGNAL(PIDstoreActualYaw(float)), yawController, SLOT(storeActual(float))); // Get Data
    QObject::connect(odroidListener, SIGNAL(yawGoal(float)), yawController, SLOT(storeGoal(float))); // Get Goal
    QObject::connect(yawController, SIGNAL(getPWM(unsigned int)), odroidSerial, SLOT(storePIDYaw(unsigned int))); // PID Output

    // Connection for Pitch Controller
    QObject::connect(odroidListener, SIGNAL(pitchError(float)), pitchController, SLOT(storeError(float))); // Get Error
    QObject::connect(pitchController, SIGNAL(getPWM(unsigned int)), odroidSerial, SLOT(storePIDPitch(unsigned int))); // PID Output

    // Connection for Roll Controller
    QObject::connect(odroidListener, SIGNAL(rollError(float)), rollController, SLOT(storeError(float))); // Get Error
    QObject::connect(rollController, SIGNAL(getPWM(unsigned int)), odroidSerial, SLOT(storePIDRoll(unsigned int))); // PID Output

    // Connection for Height Controller
    QObject::connect(odroidSerial, SIGNAL(PIDstoreActualHeight(float)), throttleController, SLOT(storeActual(float))); // Get Data
    QObject::connect(odroidListener, SIGNAL(throttleGoal(float)), throttleController, SLOT(storeGoal(float))); // Get Goal
    QObject::connect(throttleController, SIGNAL(getPWM(unsigned int)), odroidSerial, SLOT(storePIDThrottle(unsigned int))); // PID Output

    // Connections for telemetry
    QObject::connect(odroidSerial, SIGNAL(GUIstoreActualYaw(float)), odroidListener, SLOT(send_yaw(float)));
    QObject::connect(odroidSerial, SIGNAL(GUIstoreActualHeight(float)), odroidListener, SLOT(send_throttle(float)));
    QObject::connect(odroidSerial, SIGNAL(GUIstoreActualBattery(float)), odroidListener, SLOT(send_batt(float)));
    QObject::connect(pitchController, SIGNAL(pTerm(float)), odroidListener, SLOT(send_pitch_pterm(float)));
    QObject::connect(pitchController, SIGNAL(iTerm(float)), odroidListener, SLOT(send_pitch_iterm(float)));
    QObject::connect(pitchController, SIGNAL(dTerm(float)), odroidListener, SLOT(send_pitch_dterm(float)));
    QObject::connect(pitchController, SIGNAL(sOutput(float)), odroidListener, SLOT(send_pitch_output(float)));
    QObject::connect(rollController, SIGNAL(pTerm(float)), odroidListener, SLOT(send_roll_pterm(float)));
    QObject::connect(rollController, SIGNAL(iTerm(float)), odroidListener, SLOT(send_roll_iterm(float)));
    QObject::connect(rollController, SIGNAL(dTerm(float)), odroidListener, SLOT(send_roll_dterm(float)));
    QObject::connect(rollController, SIGNAL(sOutput(float)), odroidListener, SLOT(send_roll_output(float)));

    thread1->start();
    thread2->start();
    thread3->start();

    // YPRT Controllers
    thread4->start();
    thread5->start();
    thread6->start();
    thread7->start();

    signal(SIGINT, signal_callback_handler);
    app.exec();
//    odroidSerial->stopThread_ = true;
}

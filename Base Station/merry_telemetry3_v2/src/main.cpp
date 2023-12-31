#include "telemetry.h"
#include "tcprosstatman.h"
#include <QApplication>
#include <QThread>

void gui_connections(Telemetry* w, TcpRosStatMan* tcprosstatman1, TcpRosStatMan* tcprosstatman2)
{
    QObject::connect(tcprosstatman1, SIGNAL(connected()), w, SLOT(quad1_connected()));
    QObject::connect(tcprosstatman2, SIGNAL(connected()), w, SLOT(quad2_connected()));
    QObject::connect(tcprosstatman1, SIGNAL(disconnected()), w, SLOT(quad1_disconnected()));
    QObject::connect(tcprosstatman2, SIGNAL(disconnected()), w, SLOT(quad2_disconnected()));
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    ros::init(argc, argv, "Merry_GUI");

    ros::NodeHandle n_stat;

    TcpRosStatMan tcprosstatman1("BASESERVER", "quad1", 10, 1, &n_stat);
    TcpRosStatMan tcprosstatman2("BASESERVER", "quad2", 10, 1, &n_stat);

    Telemetry *w = new Telemetry();

    w->show();

    gui_connections(w, &tcprosstatman1, &tcprosstatman2);

    return a.exec();
}

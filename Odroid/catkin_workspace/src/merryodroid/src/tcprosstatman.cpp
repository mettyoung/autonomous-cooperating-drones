#include "tcprosstatman.h"
#include <QDebug>

TcpRosStatMan::TcpRosStatMan(QString this_ns, QString that_ns, double rate_send, double timeout, ros::NodeHandle *n){
    this->n = n;
    this->dt_send = 1/rate_send;
    this->dt_receive = timeout;
    this->isConnected = false;
    this->status = false;
    this->msg.data = true;
    this->isDisconnectedEmitted = false;
    this->isConnectedEmitted = false;
    ss = this_ns + "/status";
    ss2 = that_ns + "/status";
    this->subscriber = n->subscribe(ss2.toStdString(), 1000, &TcpRosStatMan::subscriberCallback, this);
    this->publisher = n->advertise<std_msgs::Bool>(ss.toStdString(), 1000);
}
void TcpRosStatMan::start(){
    timer_send = new QTimer();
    timer_send -> start((int)(dt_send*1000));
    connect(timer_send, SIGNAL(timeout()), this, SLOT(send()));

    timer_receive = new QTimer();
    timer_receive -> start((int)(dt_receive*1000));
    connect(timer_receive, SIGNAL(timeout()), this, SLOT(receive()));

    qDebug() << ss << "TcpRosStatMan Started";
}
void TcpRosStatMan::send(){
    publisher.publish(msg);
    ros::spinOnce();
}
void TcpRosStatMan::receive(){
    if(!status){
	if(!isDisconnectedEmitted)
	{
            Q_EMIT disconnected();
	    isDisconnectedEmitted = true;
	    isConnectedEmitted = false;
	}
        isConnected = false;
        //qDebug() << ss2 << "Disconnected!";
    }
    status = false;
}
void TcpRosStatMan::subscriberCallback(const std_msgs::Bool::ConstPtr &msg){
    if(!isConnected){
	if(!isConnectedEmitted)
	{
            Q_EMIT connected();
	    isConnectedEmitted = true;
	    isDisconnectedEmitted = false;
	}
        //qDebug() << ss2 << "Connected!";
    }
    status = true;
    isConnected = true;
}


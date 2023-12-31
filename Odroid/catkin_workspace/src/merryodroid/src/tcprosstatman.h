#include<ros/ros.h>
#include<std_msgs/Bool.h>
#include<QObject>
#include<stdio.h>
#include <QTimer>
#ifndef TCPROSSTATMAN_H
#define TCPROSSTATMAN_H
class TcpRosStatMan : public QObject{
    Q_OBJECT
    public:
        TcpRosStatMan(QString this_ns, QString that_ns, double rate_send, double timeoutMS, ros::NodeHandle *n);
    public Q_SLOTS:
        void start();
    private:
        ros::NodeHandle *n;
        ros::Publisher publisher;
        ros::Subscriber subscriber;
        std_msgs::Bool msg;

        void subscriberCallback(const std_msgs::Bool::ConstPtr& msg);

        // Timer Related
        float dt_send, dt_receive;
        QTimer *timer_send, *timer_receive;

        bool status;
        bool isConnected;

	bool isConnectedEmitted;
	bool isDisconnectedEmitted;

        QString ss, ss2;

    private Q_SLOTS:
        void send();
        void receive();

    Q_SIGNALS:
        void connected();
        void disconnected();

};
#endif

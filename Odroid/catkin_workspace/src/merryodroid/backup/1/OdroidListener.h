#include <QObject>
#include <ros/ros.h>
#include <string>
#include <boost/timer.hpp>
#include <QTimer>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include "std_msgs/String.h"

class OdroidListener : public QObject
{
    Q_OBJECT

private:
    // NodeHandle is needed for the node communication
    ros::NodeHandle *n_odroid;
    // Subscribers from the controllers
    ros::Subscriber sub_startQuad;
    ros::Subscriber sub_armQuad;
    ros::Subscriber sub_throttle;

    // Namespaces for the topics of the subscribers.
    std::string ns_startQuad;
    std::string ns_armQuad;
    std::string ns_throttle;

    // Calls ros::SpinOnce every 100ms; to give way to Qt Event Loop.
    QTimer *timer_;

private Q_SLOTS:
    // Callback function for the QTimer.
    void run();

public Q_SLOTS:
    // QTimer. Start() starts the object to work!
    void start();

Q_SIGNALS:
    void throttleVal(uint thro);
    void quad_arm();
    void quad_start();

public:
    // Takes the pointer to the NodeHandle for the constructor.
    OdroidListener(ros::NodeHandle* nodeHandle, const char* nsThrottle, const char* nsStartQuad, const char *nsArmQuad);

    // Wrapper functions to emit sinals
    void emitThrottle(uint throttle);
    void emitStartSignal(bool startSig);
    void emitArmSignal(bool armSig);
};

//// Wrapper function that invokes a member function to solve the problem of
//// pointer to member functions.
void cb_throttle(const std_msgs::Int16::ConstPtr& throttle);
void cb_start(const std_msgs::Bool::ConstPtr& start);
void cb_arm(const std_msgs::Bool::ConstPtr& arm);

extern OdroidListener* objectHandleSignal;




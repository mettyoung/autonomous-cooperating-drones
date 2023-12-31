#include "odroidlistener.h"

OdroidListener* objectHandleSignal;

// Constructor
OdroidListener::OdroidListener(ros::NodeHandle* nodeHandle, const char* nsThrottle, const char* nsStartQuad,
                               const char* nsArmQuad): n_odroid(nodeHandle)
{
    objectHandleSignal = this;
    ns_armQuad = nsArmQuad;
    ns_startQuad = nsStartQuad;
    ns_throttle = nsThrottle;
}

// Private Slot
void OdroidListener::run()
{
    ros::spinOnce();
}

// Public Slot
void OdroidListener::start()
{
    sub_throttle = n_odroid->subscribe(ns_throttle, 1, cb_throttle);
    sub_startQuad = n_odroid->subscribe(ns_startQuad,1, cb_start);
    sub_armQuad = n_odroid->subscribe(ns_armQuad,1, cb_arm);
    timer_ = new QTimer();
    timer_->start(100);
    connect(timer_, SIGNAL(timeout()), this, SLOT(run()));
}

// Wrapper Functions
void OdroidListener::emitThrottle(uint throttle)
{
    printf("\nOdroidListener throttle value: %d\n", throttle);
    Q_EMIT throttleVal(throttle);
}

void OdroidListener::emitStartSignal(bool startSig)
{
    if(startSig = TRUE)
        Q_EMIT quad_start();
}

void OdroidListener::emitArmSignal(bool armSig)
{
    if(armSig = TRUE)
        Q_EMIT quad_arm();
}


// Callback Functions
void cb_throttle(const std_msgs::Int16::ConstPtr& throttle)
{
    //printf("%d received\n", throttle.get()->data);
    objectHandleSignal->emitThrottle(throttle.get()->data);
}
void cb_start(const std_msgs::Bool::ConstPtr& start)
{
    printf("Starting quad.");
    objectHandleSignal->emitStartSignal(start);
}
void cb_arm(const std_msgs::Bool::ConstPtr& arm)
{
    printf("Arming quad.");
    objectHandleSignal->emitArmSignal(arm.get()->data);
}


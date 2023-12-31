#include <QObject>
#include <ros/ros.h>
#include <string>
#include <boost/timer.hpp>
#include <merryMsg/Msg.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <QTimer>
#include <QString>
#include <iostream>

class OdroidListener : public QObject
{
    Q_OBJECT

private:
    // NodeHandle is needed for the node communication
    ros::NodeHandle *n_odroid;

    // Subscribers from the controllers
    ros::Subscriber sub_startQuad;
    ros::Subscriber sub_armQuad;
    ros::Subscriber sub_landQuad;
    // Goals
    ros::Subscriber sub_height_goal;
    ros::Subscriber sub_yaw_goal;
    ros::Subscriber sub_pitch_goal;
    ros::Subscriber sub_roll_goal;

    ros::Publisher pub_height_actual;
    ros::Publisher pub_yaw_actual;
    ros::Publisher pub_pitch_actual;
    ros::Publisher pub_roll_actual;
    ros::Publisher pub_batt;

    ros::Publisher pub_pitch_pterm;
    ros::Publisher pub_pitch_iterm;
    ros::Publisher pub_pitch_dterm;
    ros::Publisher pub_pitch_output;
    ros::Publisher pub_roll_pterm;
    ros::Publisher pub_roll_iterm;
    ros::Publisher pub_roll_dterm;
    ros::Publisher pub_roll_output;

    // Namespaces for the topics of the subscribers.
    QString ns_startQuad_sub;
    QString ns_armQuad_sub;
    QString ns_landQuad_sub;
    QString ns_heightGoal_sub;
    QString ns_yawGoal_sub;
    QString ns_pitchGoal_sub;
    QString ns_rollGoal_sub;

    QString ns_heightActual_pub;
    QString ns_yawActual_pub;
    QString ns_pitchActual_pub;
    QString ns_rollActual_pub;
    QString ns_batt_pub;

    QString ns_pitch_pterm_pub, ns_pitch_iterm_pub, ns_pitch_dterm_pub, ns_pitch_output_pub;
    QString ns_roll_pterm_pub, ns_roll_iterm_pub, ns_roll_dterm_pub, ns_roll_output_pub;

    // Actual
    merryMsg::Msg throttle_, yaw_, pitch_, roll_;
    merryMsg::Msg batt_, pterm_, iterm_, dterm_, output_;
    // Calls ros::SpinOnce every 100ms; to give way to Qt Event Loop.
    QTimer *timer_;

private Q_SLOTS:
    // Callback function for the QTimer.
    void run();

public Q_SLOTS:
    // QTimer. Start() starts the object to work!
    void start();
    void send_throttle(float throttle);
    void send_yaw(float yaw);
    void send_pitch(float pitch);
    void send_roll(float roll);
    void send_batt(float batt);

    void send_pitch_pterm(float pterm);
    void send_pitch_iterm(float iterm);
    void send_pitch_dterm(float dterm);
    void send_pitch_output(float output);
    void send_roll_pterm(float pterm);
    void send_roll_iterm(float iterm);
    void send_roll_dterm(float dterm);
    void send_roll_output(float output);
Q_SIGNALS:
    void throttleGoal(float throttle);
    void yawGoal(float yaw);
    void pitchError(float pitch);
    void rollError(float roll);
    void quad_arm();
    void quad_start();
    void quad_land();

public:
    // Takes the pointer to the NodeHandle for the constructor.
    OdroidListener(ros::NodeHandle* nodeHandle, QString quad);

    // Wrapper functions to emit sinals
    void emitThrottleGoal(const merryMsg::Msg::ConstPtr& throttle);
    void emitYawGoal(const merryMsg::Msg::ConstPtr& yaw);
    void emitPitchGoal(const merryMsg::Msg::ConstPtr& pitch);
    void emitRollGoal(const merryMsg::Msg::ConstPtr& roll);
    void emitStartSignal(const std_msgs::Bool::ConstPtr& start);
    void emitArmSignal(const std_msgs::Bool::ConstPtr& arm);
    void emitLandSignal(const std_msgs::Bool::ConstPtr& land);
};





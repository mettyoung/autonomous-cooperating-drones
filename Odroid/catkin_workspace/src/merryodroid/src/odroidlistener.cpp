#include "odroidlistener.h"
#include <QDebug>
// Constructor
OdroidListener::OdroidListener(ros::NodeHandle* nodeHandle, QString quad): n_odroid(nodeHandle)
{
    ns_armQuad_sub = quad + "/arm";
    ns_startQuad_sub = quad + "/start";
    ns_landQuad_sub = quad + "/land";

    ns_heightGoal_sub = quad + "/goal/height";
    ns_yawGoal_sub = quad + "/goal/yaw";
    ns_pitchGoal_sub = quad + "/goal/pitch";
    ns_rollGoal_sub = quad + "/goal/roll";

    ns_heightActual_pub = quad + "/actual/height";
    ns_yawActual_pub = quad + "/actual/yaw";
    ns_pitchActual_pub = quad + "/actual/pitch";
    ns_rollActual_pub = quad + "/actual/roll";
    ns_batt_pub = quad + "/batt";

    ns_pitch_pterm_pub = quad + "/pitch/pterm";
    ns_pitch_iterm_pub = quad + "/pitch/iterm";
    ns_pitch_dterm_pub = quad + "/pitch/dterm";
    ns_pitch_output_pub = quad + "/pitch/output";

    ns_roll_pterm_pub = quad + "/roll/pterm";
    ns_roll_iterm_pub = quad + "/roll/iterm";
    ns_roll_dterm_pub = quad + "/roll/dterm";
    ns_roll_output_pub = quad + "/roll/output";
}

// Private Slot
void OdroidListener::run()
{
    ros::spinOnce();
}

// Public Slot
void OdroidListener::start()
{
    sub_height_goal = n_odroid->subscribe(ns_heightGoal_sub.toStdString(), 1, &OdroidListener::emitThrottleGoal, this);
    sub_yaw_goal = n_odroid->subscribe(ns_yawGoal_sub.toStdString(), 1, &OdroidListener::emitYawGoal, this);
    sub_pitch_goal = n_odroid->subscribe(ns_pitchGoal_sub.toStdString(), 1, &OdroidListener::emitPitchGoal, this);
    sub_roll_goal = n_odroid->subscribe(ns_rollGoal_sub.toStdString(), 1, &OdroidListener::emitRollGoal, this);

    sub_startQuad = n_odroid->subscribe(ns_startQuad_sub.toStdString(), 1, &OdroidListener::emitStartSignal, this);
    sub_armQuad = n_odroid->subscribe(ns_armQuad_sub.toStdString(), 1, &OdroidListener::emitArmSignal, this);
    sub_landQuad = n_odroid->subscribe(ns_landQuad_sub.toStdString(), 1, &OdroidListener::emitLandSignal, this);

    pub_height_actual = n_odroid->advertise<merryMsg::Msg>(ns_heightActual_pub.toStdString(), 1000);
    pub_yaw_actual = n_odroid->advertise<merryMsg::Msg>(ns_yawActual_pub.toStdString(), 1000);
    pub_pitch_actual = n_odroid->advertise<merryMsg::Msg>(ns_pitchActual_pub.toStdString(), 1000);
    pub_roll_actual = n_odroid->advertise<merryMsg::Msg>(ns_rollActual_pub.toStdString(), 1000);
    pub_batt = n_odroid->advertise<merryMsg::Msg>(ns_batt_pub.toStdString(), 1000);

    pub_pitch_pterm = n_odroid->advertise<merryMsg::Msg>(ns_pitch_pterm_pub.toStdString(), 1000);
    pub_pitch_iterm = n_odroid->advertise<merryMsg::Msg>(ns_pitch_iterm_pub.toStdString(), 1000);
    pub_pitch_dterm = n_odroid->advertise<merryMsg::Msg>(ns_pitch_dterm_pub.toStdString(), 1000);
    pub_pitch_output = n_odroid->advertise<merryMsg::Msg>(ns_pitch_output_pub.toStdString(), 1000);
    pub_roll_pterm = n_odroid->advertise<merryMsg::Msg>(ns_roll_pterm_pub.toStdString(), 1000);
    pub_roll_iterm = n_odroid->advertise<merryMsg::Msg>(ns_roll_iterm_pub.toStdString(), 1000);
    pub_roll_dterm = n_odroid->advertise<merryMsg::Msg>(ns_roll_dterm_pub.toStdString(), 1000);
    pub_roll_output = n_odroid->advertise<merryMsg::Msg>(ns_roll_output_pub.toStdString(), 1000);

    timer_ = new QTimer();
    timer_->start(100);
    connect(timer_, SIGNAL(timeout()), this, SLOT(run()));
}

// Send telemetry data

void OdroidListener::send_throttle(float throttle)
{
    throttle_.data = throttle;
    throttle_.header.stamp = ros::Time::now();
    pub_height_actual.publish(throttle_);
}

void OdroidListener::send_yaw(float yaw)
{
    yaw_.data = yaw;
    yaw_.header.stamp = ros::Time::now();
    pub_yaw_actual.publish(yaw_);
}

void OdroidListener::send_pitch(float pitch)
{
    pitch_.data = pitch;
    pitch_.header.stamp = ros::Time::now();
    pub_pitch_actual.publish(pitch_);
}

void OdroidListener::send_roll(float roll)
{
    roll_.data = roll;
    roll_.header.stamp = ros::Time::now();
    pub_roll_actual.publish(roll_);
}

void OdroidListener::send_batt(float batt)
{
    //std::cout << batt << std::endl;
    batt_.data = batt;
    pub_batt.publish(batt_);
}

void OdroidListener::send_pitch_pterm(float pterm)
{
    pterm_.data = pterm;
    pterm_.header.stamp = ros::Time::now();
    pub_pitch_pterm.publish(pterm_);
}

void OdroidListener::send_pitch_iterm(float iterm)
{
    iterm_.data = iterm;
    iterm_.header.stamp = ros::Time::now();
    pub_pitch_iterm.publish(iterm_);
}

void OdroidListener::send_pitch_dterm(float dterm)
{
    dterm_.data = dterm;
    dterm_.header.stamp = ros::Time::now();
    pub_pitch_dterm.publish(dterm_);
}

void OdroidListener::send_pitch_output(float output)
{
    output_.data = output;
    output_.header.stamp = ros::Time::now();
    pub_pitch_output.publish(output_);
}

void OdroidListener::send_roll_pterm(float pterm)
{
    pterm_.data = pterm;
    pterm_.header.stamp = ros::Time::now();
    pub_roll_pterm.publish(pterm_);
}

void OdroidListener::send_roll_iterm(float iterm)
{
    iterm_.data = iterm;
    iterm_.header.stamp = ros::Time::now();
    pub_roll_iterm.publish(iterm_);
}

void OdroidListener::send_roll_dterm(float dterm)
{
    dterm_.data = dterm;
    dterm_.header.stamp = ros::Time::now();
    pub_roll_dterm.publish(dterm_);
}

void OdroidListener::send_roll_output(float output)
{
    output_.data = output;
    output_.header.stamp = ros::Time::now();
    pub_roll_output.publish(output_);
}

// Wrapper Functions
void OdroidListener::emitThrottleGoal(const merryMsg::Msg::ConstPtr& throttle)
{
    //printf("\nOdroidListener throttle value: %d\n", throttle);
    Q_EMIT throttleGoal(throttle.get()->data);
}

void OdroidListener::emitYawGoal(const merryMsg::Msg::ConstPtr& yaw)
{
    //printf("\nOdroidListener throttle value: %d\n", throttle);
    Q_EMIT yawGoal(yaw.get()->data);
}

void OdroidListener::emitPitchGoal(const merryMsg::Msg::ConstPtr& pitch)
{
    //printf("\nOdroidListener throttle value: %d\n", throttle);
    //std::cout << " hi : " << pitch.get()->data << std::endl;
    Q_EMIT pitchError(pitch.get()->data);
}

void OdroidListener::emitRollGoal(const merryMsg::Msg::ConstPtr& roll)
{
    //printf("\nOdroidListener throttle value: %d\n", throttle);
    Q_EMIT rollError(roll.get()->data);
}

void OdroidListener::emitStartSignal(const std_msgs::Bool::ConstPtr& start)
{
    if(start)
        Q_EMIT quad_start();
}

void OdroidListener::emitArmSignal(const std_msgs::Bool::ConstPtr& arm)
{
    if(arm)
        Q_EMIT quad_arm();
}

void OdroidListener::emitLandSignal(const std_msgs::Bool::ConstPtr& land)
{
    Q_EMIT quad_land();
}

#ifndef TELEMETRY_H
#define TELEMETRY_H


#include <merryMsg/Msg.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tcprosstatman.h"

#include <QWidget>
#include <QObject>
#include <QTimer>
#include <QDateTime>
#include <QDebug>
#include "qcustomplot.h"

namespace Ui {
class Telemetry;
}

class Telemetry : public QWidget
{
    Q_OBJECT

public:
    //explicit Telemetry(QWidget *parent = 0);
    Telemetry();
    ~Telemetry();

private:
    Ui::Telemetry *ui;
    QTimer *timer;
    double zero_reference;
    ros::NodeHandle n_gui;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    void init_subscribers();
    void init_publishers();
    void init_plot_graphs();
    void set_axis_labels();
    void enable_graph_legends(bool enable);
    void set_interactions_settings(bool drag, bool zoom);
    void init_yValues();

    double yaw_tarTemp1;
    double yaw_tarTemp2;

    // QVectors for actual sensor readings
    QVector<double> xThro1, xYaw1;
    QVector<double> yThro1, yYaw1;
    QVector<double> xThro2, xYaw2;
    QVector<double> yThro2, yYaw2;

    // QVectors for PID term from pitch and roll
    QVector<double> xPTimePitch[2], xPTimeRoll[2];
    QVector<double> xITimePitch[2], xITimeRoll[2];
    QVector<double> xDTimePitch[2], xDTimeRoll[2];
    QVector<double> xOutputPitch[2], xOutputRoll[2];

    QVector<double> yPTermPitch[2], yPTermRoll[2];
    QVector<double> yITermPitch[2], yITermRoll[2];
    QVector<double> yDTermPitch[2], yDTermRoll[2];
    QVector<double> yOutputPitch[2], yOutputRoll[2];

    // QVectors for desired sensor readings
    QVector<double> xThroTarget1, xYawTarget1;
    QVector<double> yThroTarget1, yYawTarget1;
    QVector<double> xThroTarget2, xYawTarget2;
    QVector<double> yThroTarget2, yYawTarget2;

    // QVectors for desired less actual
    QVector<double> xThroDiff1, xYawDiff1;
    QVector<double> yThroDiff1, yYawDiff1;
    QVector<double> xThroDiff2, xYawDiff2;
    QVector<double> yThroDiff2, yYawDiff2;

    // QVectors for goal waypoints
    QVector<double> xCoordinatesTar1, yCoordinatesTar1;
    QVector<double> xCoordinatesTar2, yCoordinatesTar2;
    QVector<double> xCoordinates1, yCoordinates1;
    QVector<double> xCoordinates2, yCoordinates2;
    QVector<double> xCoordinatesP1, yCoordinatesP1;
    QVector<double> xCoordinatesP2, yCoordinatesP2;
    QVector<double> xCoordinatesD1, yCoordinatesD1;
    QVector<double> xCoordinatesD2, yCoordinatesD2;
    QVector<double> time1, time2, time3;

    geometry_msgs::Point waypoint[2];

    // Subscribers for actual values
    ros::Subscriber sub_thro1, sub_thro2;
    ros::Subscriber sub_yaw1, sub_yaw2;
    ros::Subscriber sub_coordinates1, sub_coordinates2;

    // Subscribers for the desired values
    ros::Subscriber sub_thro_tar1, sub_thro_tar2;
    ros::Subscriber sub_yaw_tar1, sub_yaw_tar2;
    ros::Subscriber sub_coordinates_tar1, sub_coordinates_tar2;

    // Subscribers for the battery levels
    ros::Subscriber sub_batt1, sub_batt2;

    // Publisher for landing command
    ros::Publisher pub_land1, pub_land2;

    // Publishers for the control tester
    ros::Publisher pub_height, pub_yaw, pub_start, pub_arm;
    ros::Publisher pub_height2, pub_yaw2, pub_start2, pub_arm2;
    merryMsg::Msg cHeight, cYaw, cHeight2;

    // Subscribers for P, I, D and output term
    ros::Subscriber PIDO[2][2][4];

    void updatePTermPitch0(const merryMsg::Msg::ConstPtr& pTerm);
    void updateITermPitch0(const merryMsg::Msg::ConstPtr& iTerm);
    void updateDTermPitch0(const merryMsg::Msg::ConstPtr& dTerm);
    void updateOutputPitch0(const merryMsg::Msg::ConstPtr& output);

    void updatePTermRoll0(const merryMsg::Msg::ConstPtr& pTerm);
    void updateITermRoll0(const merryMsg::Msg::ConstPtr& iTerm);
    void updateDTermRoll0(const merryMsg::Msg::ConstPtr& dTerm);
    void updateOutputRoll0(const merryMsg::Msg::ConstPtr& output);


    void updatePTermPitch1(const merryMsg::Msg::ConstPtr& pTerm);
    void updateITermPitch1(const merryMsg::Msg::ConstPtr& iTerm);
    void updateDTermPitch1(const merryMsg::Msg::ConstPtr& dTerm);
    void updateOutputPitch1(const merryMsg::Msg::ConstPtr& output);

    void updatePTermRoll1(const merryMsg::Msg::ConstPtr& pTerm);
    void updateITermRoll1(const merryMsg::Msg::ConstPtr& iTerm);
    void updateDTermRoll1(const merryMsg::Msg::ConstPtr& dTerm);
    void updateOutputRoll1(const merryMsg::Msg::ConstPtr& output);

    void updateThrottleTableActual1(const merryMsg::Msg::ConstPtr& thro);
    void updateYawTableActual1(const merryMsg::Msg::ConstPtr& yaw);

    void updateThrottleTableActual2(const merryMsg::Msg::ConstPtr& thro);
    void updateYawTableActual2(const merryMsg::Msg::ConstPtr& yaw);

    void updateThrottleTableDesired1(const merryMsg::Msg::ConstPtr& thro);
    void updateYawTableDesired1(const merryMsg::Msg::ConstPtr& yaw);

    void updateThrottleTableDesired2(const merryMsg::Msg::ConstPtr& thro);
    void updateYawTableDesired2(const merryMsg::Msg::ConstPtr& yaw);

    void updateCoordinatesTableDesired1(const geometry_msgs::Point& p);
    void updateCoordinatesTableDesired2(const geometry_msgs::Point& p);

private Q_SLOTS:
    void run();
    void adjustBatteryLevel1(const merryMsg::Msg::ConstPtr &battery);
    void adjustBatteryLevel2(const merryMsg::Msg::ConstPtr &battery);
    void actualCoordinates();

    void on_btn_height_clicked();
    void on_btn_yaw_clicked();
    void on_btn_start_clicked();
    void on_btn_arm_clicked();
    void on_btn_reset_clicked();
    void on_btn_print_clicked();
    void on_btn_legends_clicked();

    void on_btn_start1_clicked();
    void on_btn_arm1_clicked();
    void on_btn_land1_clicked();
    void on_btn_start2_clicked();
    void on_btn_arm2_clicked();
    void on_btn_land2_clicked();

public Q_SLOTS:
    void quad1_connected();
    void quad2_connected();

    void quad1_disconnected();
    void quad2_disconnected();
};

#endif // TELEMETRY_H

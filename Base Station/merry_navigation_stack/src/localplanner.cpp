#include "localplanner.h"

LocalPlanner::LocalPlanner(QString quad, QString camera) : isControlledByRviz(false), isAutonomousEnabled_(false),
    isAutonomousClicked_(false)
{
    ns_quad = quad;
    camerax_link = camera + "_link";

    if(quad == "quad1")
        markerPath_.id = 0;
    else if(quad == "quad2")
        markerPath_.id = 1;

    chosenGoal_.header.frame_id = "/map";
    chosenGoal_.ns = "goal";
    chosenGoal_.action = visualization_msgs::Marker::ADD;
    chosenGoal_.type = visualization_msgs::Marker::CUBE;
    chosenGoal_.scale.x = 0.2;
    chosenGoal_.scale.y = 0.2;
    chosenGoal_.scale.z = 0.2;
    chosenGoal_.color.a = 1.0f;
    chosenGoal_.pose.orientation.w = 0;
    chosenGoal_.pose.orientation.x = 0;
    chosenGoal_.pose.orientation.y = 0;
    chosenGoal_.pose.orientation.z = 0;
    chosenGoal_.pose.position.z = 0;
    chosenGoal_.id = markerPath_.id;
    if (markerPath_.id  == 0)
        chosenGoal_.color.b = 1.0f;
    else
        chosenGoal_.color.r = 1.0f;
}

void LocalPlanner::start()
{
    sub_markerPath = n.subscribe("global_planner/path", 10, &LocalPlanner::wayPoints_cb, this);
    subRviz_ = n.subscribe("move_base_simple/goal", 1, &LocalPlanner::moveToWayPoint_cb, this);

    pub_yaw_goal = n.advertise<merryMsg::Msg>(ns_quad.toStdString() + "/goal/yaw", 1);
    pub_pitch_goal = n.advertise<merryMsg::Msg>(ns_quad.toStdString() + "/goal/pitch", 1);
    pub_roll_goal = n.advertise<merryMsg::Msg>(ns_quad.toStdString() + "/goal/roll", 1);
    pub_waypoint_goal_ = n.advertise<geometry_msgs::Point>(ns_quad.toStdString() + "/goal/waypoint", 1);
    pub_marker_goal_ = n.advertise<visualization_msgs::Marker>(ns_quad.toStdString() + "/localplanner/waypoint", 1);
    isPathReached = false;
    isWayPointsAvailable = false;
    wayPointIterator = 0;
    cYaw = 0;
    cDistance = 0;

    timer_ = new QTimer();
    timer_->start(100);

    QObject::connect(timer_, SIGNAL(timeout()), this, SLOT(run()));
    QObject::connect(this, SIGNAL(goalYaw(double)), this, SLOT(send_yaw_goal(double)));
    QObject::connect(this, SIGNAL(goalPitch(double)), this, SLOT(send_pitch_goal(double)));
    QObject::connect(this, SIGNAL(goalRoll(double)), this, SLOT(send_roll_goal(double)));

    /// Autonomous QTimer
//    ros::NodeHandle n;
//    double hz = 1.;
//    n.param(ros::this_node::getNamespace() + ros::this_node::getName() + "/freq_autonomous", hz, hz);
//    timerAutonomous_ = new QTimer();
//    connect(timerAutonomous_, SIGNAL(timeout()), this, SLOT(runAutonomous()));
//    timerAutonomous_->start((int)((1./hz)*1000));
}

void LocalPlanner::hoverQuad()
{
    log_ << "Yaw = 0    Roll = 0    Pitch = 0\n";
    Q_EMIT setCommandQuad(0,0);
    Q_EMIT goalYaw(0);
    Q_EMIT goalRoll(0);
    Q_EMIT goalPitch(0);
}

void LocalPlanner::run()
{
    bool isLoop = false;
    do
    {
        isLoop = false;
        ros::spinOnce();

        // Remember, the fixed frame is set to "map".
        // Hence, the retrieved pose can be directly related with camera0_link and camera1_link
        // as long as both of their fixed frames are set to "map" which it may be!

        // To get the camera0_link and camera1_link, just lookup for the transform of "map" to camera0_link
        // and camera1_link.
        tf::StampedTransform sTransform;
        double timeDelayed;
        if(!queryPose.getPose(markerPath_.id, sTransform, timeDelayed))
        {
            // This may be used when SLAM stops processing and localization becomes unavailable. (I think)
            log_ << "Pose is <font color = \"red\"> INVALID</font>. Outdated by <i>" << timeDelayed << "</i> seconds\n";
            hoverQuad();
            if(isAutonomousClicked_)
               isAutonomousEnabled_ = true;
            eraseGoal();
            return;
        }
        else
        {
            log_ << "Pose is <font color = \"green\"> VALID</font>. Outdated by <i>" << timeDelayed << "</i> seconds\n";
        }

        // However, the goal is to find the euclidean distance and the bearing of the waypoint from the POV of the camerax_pose
        // with the X-axis as its reference axis. This can be done by computing by projecting the waypoint to the coordinate frame
        // of the camerax_pose.
        // T(camerax_link -> pose) = T(map->camerax_link).inverse * T(map->pose)
        waypoint = sTransform.inverseTimes(waypointTransform);

        if(isWayPointsAvailable || isControlledByRviz)
        {
            if (!isControlledByRviz)
                this->nextWayPoint();

            // This code retrieves the orientation of the waypoint with respect to the coordinate frame
            // of the camerax_link.
            //        yaw = tf::getYaw(waypoint.getRotation()) * 180 / M_PI;

            // Retrieve yaw in degrees [-180 -> 180]
            // This does not take into account the backview of the quadrotor.
            cYaw = atan2(waypoint.getOrigin().y(), waypoint.getOrigin().x()) * 180 / M_PI;
            // Retrieve the euclidean distance
            cDistance = std::sqrt(waypoint.getOrigin().x()*waypoint.getOrigin().x() +
                                  waypoint.getOrigin().y()*waypoint.getOrigin().y());

            log_ << "[Distance,yaw] = " << cDistance << " m," << -cYaw << " deg\n";
            // Now, do your own thing.
            // Case 1: If the euclidean distance to the goalpoint is greater than 0.5 meters,
            // the yaw reading is acceptable.
            if (cDistance >= .5)
            {
                // If the angle is within [-10,10] degrees, consider the yaw-ing process finished.
                if(std::abs(cYaw) <= 10)
                {
                    log_ << "Send Pitch Command = " << cDistance << "m\n";
                    Q_EMIT goalPitch(cDistance);
                    Q_EMIT goalYaw(0);
                    Q_EMIT setCommandQuad(cDistance, 0);
                    isAutonomousEnabled_ = true;
                    sendGoalToRviz(waypointTransform.getOrigin().x(), waypointTransform.getOrigin().y());
                }
                else
                {
                    isAutonomousEnabled_ = false;
                    log_ << "Send Yaw Command = " << -cYaw << " degrees\n";
                    Q_EMIT goalYaw(-cYaw);
                    Q_EMIT goalPitch(0);
                    Q_EMIT setCommandQuad(0,-cYaw);
                    sendGoalToRviz(waypointTransform.getOrigin().x(), waypointTransform.getOrigin().y());
                }
            }
            // Case 2: If distance is less than 0.5, yaw readings cannot be trusted.
            // We will utilize the x-vector and y-vector for position stabilization.
            else
            {
                //            log_ << "Drift control: [roll,pitch]" << (double)waypoint.getOrigin().y() << ", "
                //                 << (double)waypoint.getOrigin().x() << "\n";
                //            Q_EMIT goalRoll(waypoint.getOrigin().y());
                //            Q_EMIT goalPitch(waypoint.getOrigin().x());
                isPathReached = true;
                isLoop = true;
            }

            // If the waypoint origin is within 20cm radius, then the waypoint
            // has been reached!
            //if(std::abs(waypoint.getOrigin().y()) < 0.5 &&
            //           std::abs(waypoint.getOrigin().x()) < 0.5 && !isControlledByRviz)
            //        else
            //        {
            //            isPathReached = true;
            //        }
        }
        //    else
        //    {
        //        log_ << "Drift control: [roll,pitch]" << (double)waypoint.getOrigin().y() << ", "
        //             << (double)waypoint.getOrigin().x() << "\n";
        //        Q_EMIT goalRoll(waypoint.getOrigin().y());
        //        Q_EMIT goalPitch(waypoint.getOrigin().x());
        //    }
    } while(isLoop);
}

void LocalPlanner::wayPoints_cb(const visualization_msgs::Marker::ConstPtr& markerPath)
{
    if (markerPath->id == markerPath_.id && markerPath->type == visualization_msgs::Marker::POINTS)
    {
        isControlledByRviz = false;
        markerPath_.points.clear();
        markerPath_ = *markerPath;

        wayPointIterator = 0;
        isPathReached = true;
        isWayPointsAvailable = true;
        log_ << "Waypoints received from Global Planner = " << markerPath->points.size() << "\n";
        log_ << markerPath->pose.position.x << "," << markerPath->pose.position.y  << "\n";
    }
}

void LocalPlanner::sendGoalToRviz(double x, double y)
{
    chosenGoal_.action = visualization_msgs::Marker::ADD;
    chosenGoal_.header.stamp = ros::Time::now();
    chosenGoal_.pose.position.x = x;
    chosenGoal_.pose.position.y = y;
    pub_marker_goal_.publish(chosenGoal_);
}

void LocalPlanner::eraseGoal()
{
    chosenGoal_.action = visualization_msgs::Marker::DELETE;
    chosenGoal_.header.stamp = ros::Time::now();
    pub_marker_goal_.publish(chosenGoal_);
}

void LocalPlanner::nextWayPoint()
{
    if(isPathReached)
    {
        if(wayPointIterator < markerPath_.points.size())
        {
            log_ << "Next Waypoint " << wayPointIterator << "/" << markerPath_.points.size() << " is loaded!\n";
            p = markerPath_.points.at(wayPointIterator);
            waypointTransform.setOrigin(tf::Vector3(p.x, p.y, 0));
            waypointTransform.setRotation(tf::Quaternion(tf::Vector3(0,0,1), 0));
            wayPointIterator++;
            isPathReached = false;
            pub_waypoint_goal_.publish(p);
        }
        else
            isWayPointsAvailable = false;
    }
}

void LocalPlanner::send_yaw_goal(double yaw)
{
    yaw_goal.header.stamp = ros::Time::now();
    yaw_goal.data = yaw;

    pub_yaw_goal.publish(yaw_goal);
}

void LocalPlanner::send_pitch_goal(double pitch)
{
    pitch_goal.header.stamp = ros::Time::now();
    pitch_goal.data = pitch;

    pub_pitch_goal.publish(pitch_goal);
}

void LocalPlanner::send_roll_goal(double roll)
{
    roll_goal.header.stamp = ros::Time::now();
    roll_goal.data = roll;

    pub_roll_goal.publish(roll_goal);
}

void LocalPlanner::moveToWayPoint_cb(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    isControlledByRviz = true;
    // Only update the waypointTransform which will be periodically used in the main function.
    waypointTransform.setOrigin(tf::Vector3(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z));
    waypointTransform.setRotation(tf::Quaternion(pose->pose.orientation.x, pose->pose.orientation.y,
                                                 pose->pose.orientation.z, pose->pose.orientation.w));
    geometry_msgs::Point p;
    p.x = pose->pose.position.x;
    p.y = pose->pose.position.y;
    pub_waypoint_goal_.publish(p);
}

void LocalPlanner::runAutonomous()
{
    // It is time to spamming!
    if (isAutonomousEnabled_ || (isAutonomousClicked_ && !isWayPointsAvailable))
    {
        Q_EMIT requestNewPath();
        log_ << "Requested new path\nRequested new path\nRequested new path\n";
    }
}

void LocalPlanner::toggleAutonomous()
{
    if(isAutonomousEnabled_)
    {
        log_ << "Autonomous Planning: Terminates for robot\n";
        isAutonomousEnabled_ = false;
        isAutonomousClicked_ = false;
    }
    else
    {
        log_ << "Autonomous Planning: Commences for robot\n";
        isAutonomousEnabled_ = true;
        isAutonomousClicked_ = true;
    }
}

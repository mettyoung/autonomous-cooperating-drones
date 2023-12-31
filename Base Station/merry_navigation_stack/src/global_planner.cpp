#include "global_planner.h"
// Initialize static variable
bool GlobalPlanner::isInstantiated_ = false;
ros::Publisher GlobalPlanner::markerPub_;
ros::Publisher GlobalPlanner::goalPub_;

GlobalPlanner::GlobalPlanner(int id): isAutonomousEnabled_(false),
    isLand_(false), isFirstLoop_(false)
{
    markerPath_.header.frame_id = "/map";
    markerPath_.ns = "path";
    markerPath_.action = visualization_msgs::Marker::ADD;
    markerPath_.type = visualization_msgs::Marker::POINTS;
    markerPath_.scale.x = 0.3;
    markerPath_.scale.y = 0.3;
    markerPath_.scale.z = 0.3;
    markerPath_.color.a = 1.0f;
    markerPath_.pose.orientation.w = 0;
    markerPath_.pose.orientation.x = 0;
    markerPath_.pose.orientation.y = 0;
    markerPath_.pose.orientation.z = 0;
    markerPath_.pose.position.z = 0;
    markerPath_.id = id;

    if (id == 0)
        markerPath_.color.b = 1.0f;
    else
        markerPath_.color.r = 1.0f;

    chosenFrontier_.header.frame_id = "/map";
    chosenFrontier_.ns = "frontiers";
    chosenFrontier_.action = visualization_msgs::Marker::ADD;
    chosenFrontier_.type = visualization_msgs::Marker::SPHERE;
    chosenFrontier_.scale.x = 0.2;
    chosenFrontier_.scale.y = 0.2;
    chosenFrontier_.scale.z = 0.2;
    chosenFrontier_.color.a = 1.0f;
    chosenFrontier_.pose.orientation.w = 0;
    chosenFrontier_.pose.orientation.x = 0;
    chosenFrontier_.pose.orientation.y = 0;
    chosenFrontier_.pose.orientation.z = 0;
    chosenFrontier_.pose.position.z = 0;
    chosenFrontier_.id = id;
    if (id == 0)
        chosenFrontier_.color.b = 1.0f;
    else
        chosenFrontier_.color.r = 1.0f;


    ros::NodeHandle n;
    if (!isInstantiated_)
    {
        markerPub_ = n.advertise<visualization_msgs::Marker>("global_planner/path", 10);
        goalPub_ = n.advertise<visualization_msgs::Marker>("global_planner/goal", 10);
        isInstantiated_ = true;
    }
    frontierServiceClient_ = n.serviceClient<merry_navigation_stack::QueryForFrontiers>("mission_planner/request_for_frontier");
    QString ns = QString("quad") + QString::number(id+1) + QString("/land");
    pubLand_ = n.advertise<std_msgs::Bool>(ns.toStdString(), 1);
}

void GlobalPlanner::start()
{
    // QTimer loop
    timer_ = new QTimer();
    connect(timer_, SIGNAL(timeout()), this, SLOT(run()));
    // 10Hz
    timer_->start(100);
    ros::NodeHandle n;
    double hz = 1.;
    n.param(ros::this_node::getNamespace() + ros::this_node::getName() + "/freq_autonomous", hz, hz);
    /// Autonomous QTimer
    timerAutonomous_ = new QTimer();
    connect(timerAutonomous_, SIGNAL(timeout()), this, SLOT(runAutonomous()));
    timerAutonomous_->start((int)((1./hz)*1000));
}

void GlobalPlanner::on_reqBtn_clicked()
{
    // Manually call service here; Just for testing the functionality first
    merry_navigation_stack::QueryForFrontiers srv;
    // What robot? (Only works for 0 and 1)
    srv.request.id = markerPath_.id;

    cv::Mat display2;
    display2 = mapListener.inflatedMap().gridMap().clone();
    cv::cvtColor(display2, display2, CV_GRAY2BGRA);
//    log_ << DisplayImage(mapListener.inflatedMap().gridMap(), 2+markerPath_.id, "Inflated Map for Dijkstra " + QString::number(markerPath_.id));
    log_ << "GlobalPlanner call\n";
    uint x[2], y[2];
    tf::StampedTransform sTransform;
    if (frontierServiceClient_.call(srv) &&
            mapListener.inflatedMap().worldToMap(srv.response.currentPose.pose.position.x, srv.response.currentPose.pose.position.y,
                                                 y[0], x[0]) &&
            mapListener.inflatedMap().worldToMap(srv.response.frontier.pose.position.x, srv.response.frontier.pose.position.y,
                                                 y[1], x[1]))
    {
        log_ << "CurrentPose= " << x[0] << "," << y[0] << "\n";
        log_ << "Goal= " << x[1] << "," << y[1] << "\n";
        cv::circle(display2, cv::Point2i(x[0],y[0]), 5, cv::Scalar(255,0,0,255), -1);
        cv::circle(display2, cv::Point2i(x[1],y[1]), 5, cv::Scalar(0,0,255,255), -1);
        log_ << DisplayImage(display2, 2+markerPath_.id, "Dijkstra " + QString::number(markerPath_.id) + "R:begin B:end");
        currentFrontier_.x = srv.response.frontier.pose.position.x;
        currentFrontier_.y = srv.response.frontier.pose.position.y;

//        gridmap_2d::GridMap2D inflatedMap = mapListener.inflatedMap();
//        inflatedMap.inflateMap(0.10);

        if (Dijkstra::findPathViaDijkstra(mapListener.inflatedMap().gridMap(), cv::Point2i(x[0],y[0]), cv::Point2i(x[1],y[1]), path_))
        {
            log_ << "GlobalPlanner: Path found for robot!\n";
            log_ << "GlobalPlanner: Path Size = " << path_.size() << "\n";
            // Overwrite the previous path.
            markerPath_.points.clear();

            /// Display PATH
            cv::Mat display0 = mapListener.inflatedMap().gridMap().clone();
            cv::Mat display1 = mapListener.inflatedMap().gridMap().clone();
            cv::cvtColor(display0, display0, CV_GRAY2RGBA);
            cv::cvtColor(display1, display1, CV_GRAY2RGBA);
            int cn = display0.channels();
            for (int i = 0; i < path_.size(); i++)
            {
                display0.data[path_[i].y*cn*display0.cols+path_[i].x*cn+0] = 0;
                display0.data[path_[i].y*cn*display0.cols+path_[i].x*cn+1] = 255;
                display0.data[path_[i].y*cn*display0.cols+path_[i].x*cn+2] = 0;
            }
            cv::circle(display0, cv::Point2i(x[0],y[0]), 5, cv::Scalar(255,0,0,255), -1);
            cv::circle(display0, cv::Point2i(x[1],y[1]), 5, cv::Scalar(0,0,255,255), -1);
            cv::circle(display1, cv::Point2i(x[0],y[0]), 5, cv::Scalar(255,0,0,255), -1);
            cv::circle(display1, cv::Point2i(x[1],y[1]), 5, cv::Scalar(0,0,255,255), -1);
            log_ << DisplayImage(display0, 4+markerPath_.id, "Path " + QString::number(markerPath_.id));

            /// DISCRETIZE THE PATH
            std::vector<std::pair<cv::Point2i, cv::Point2i> > discretizedPath;
            Dijkstra::discretizePathInLineSegments(mapListener.inflatedMap().gridMap(), path_, discretizedPath);
            log_ << "GlobalPlanner: Discretized Path Size = " << discretizedPath.size() << "\n";
            /// DRAW DISCRETIZED PATH
            for (int i = 0; i < discretizedPath.size(); i++)
                cv::line(display1, discretizedPath[i].first, discretizedPath[i].second, cv::Scalar(0,255,0,255));
            log_ << DisplayImage(display1, 6+markerPath_.id, "Discretized Path " + QString::number(markerPath_.id));
            /// Convert path to RVIZ
            for (int i = 0; i < path_.size(); i++)
            {
                geometry_msgs::Point p;
                mapListener.inflatedMap().mapToWorld(path_[i].y, path_[i].x,
                                                     p.x, p.y);
                markerPath_.points.push_back(p);
            }
            /// Convert discretizedPath to RVIZ
            //            for (int i = 0; i < discretizedPath.size(); i++)
            //            {
            //                geometry_msgs::Point p;
            //                mapListener.inflatedMap().mapToWorld(discretizedPath[i].second.y, discretizedPath[i].second.x,
            //                                        p.x, p.y);
            //                markerPath_.points.push_back(p);
            //            }
            /// Visualize frontier goals
            chosenFrontier_.action = visualization_msgs::Marker::ADD;
            chosenFrontier_.pose.position.x = srv.response.frontier.pose.position.x;
            chosenFrontier_.pose.position.y = srv.response.frontier.pose.position.y;
            chosenFrontier_.header.stamp = ros::Time::now();
            goalPub_.publish(chosenFrontier_);
            /// Send the successive commands to local planner
            log_ << "GlobalPlanner: Send " << markerPath_.points.size() << "  waypoints to localplanner\n";
            markerPath_.action = visualization_msgs::Marker::ADD;
            markerPath_.header.stamp = ros::Time::now();
            markerPub_.publish(markerPath_);
        }
        else
        {
            Q_EMIT hoverQuad();
            geometry_msgs::Point p;
            double timeDelayed;
            if(queryPose.getPose(markerPath_.id, sTransform, timeDelayed))
            {
                p.x = sTransform.getOrigin().x();
                p.y = sTransform.getOrigin().y();
                // Send 1 point in place
                markerPath_.points.clear();
                markerPath_.points.push_back(p);
                log_ << "GlobalPlanner: Send " << markerPath_.points.size() << "  waypoints to localplanner\n";
                markerPath_.action = visualization_msgs::Marker::DELETE;
                markerPath_.header.stamp = ros::Time::now();
                markerPub_.publish(markerPath_);
            }
            else
            {
                markerPath_.action = visualization_msgs::Marker::DELETE;
                markerPath_.header.stamp = ros::Time::now();
                markerPub_.publish(markerPath_);
            }

            /// Visualize frontier goals
            chosenFrontier_.action = visualization_msgs::Marker::DELETE;
            chosenFrontier_.header.stamp = ros::Time::now();
//            goalPub_.publish(chosenFrontier_);
            log_ << "GlobalPlanner: Fails to find a path for robot\n";
            isLand_ = true;
        }
        log_ << "GlobalPlanner: Call service for GlobalPlanner successful!" << "\n";
    }
    else
    {
        Q_EMIT hoverQuad();
        geometry_msgs::Point p;
        double timeDelayed;
        if(queryPose.getPose(markerPath_.id, sTransform, timeDelayed))
        {
            p.x = sTransform.getOrigin().x();
            p.y = sTransform.getOrigin().y();
            // Send 1 point in place
            markerPath_.points.clear();
            markerPath_.points.push_back(p);
            log_ << "GlobalPlanner: Send " << markerPath_.points.size() << "  waypoints to localplanner\n";
            markerPath_.action = visualization_msgs::Marker::DELETE;
            markerPath_.header.stamp = ros::Time::now();
            markerPub_.publish(markerPath_);
        }
        else
        {
            markerPath_.action = visualization_msgs::Marker::DELETE;
            markerPath_.header.stamp = ros::Time::now();
            markerPub_.publish(markerPath_);
        }
        /// Visualize frontier goals
        chosenFrontier_.action = visualization_msgs::Marker::DELETE;
        chosenFrontier_.header.stamp = ros::Time::now();
//        goalPub_.publish(chosenFrontier_);
        isLand_ = true;
        log_ << "GlobalPlanner: Call service for GlobalPlanner failed!\n";
    }
}

void GlobalPlanner::toggleAutonomous()
{
    if(isAutonomousEnabled_)
    {
        log_ << "Autonomous Planning: Terminates for robot\n";
        isAutonomousEnabled_ = false;
        isFirstLoop_ = false;
        isLand_ = true;
    }
    else
    {
        log_ << "Autonomous Planning: Commences for robot\n";
        isAutonomousEnabled_ = true;
        isFirstLoop_ = true;
        isLand_ = false;
        isGateTrue_ = true;
    }
}

void GlobalPlanner::run()
{
    ros::spinOnce();
}

void GlobalPlanner::runAutonomous()
{
    // It is time to spamming!
    if (isAutonomousEnabled_)
        on_reqBtn_clicked();
    //    // Only execute when autonomous is enabled.
    //    if (isAutonomousEnabled_ && isGateTrue_)
    //    {
    //        if (isFirstLoop_)
    //        {
    //            log_ << "Autonomous Planner: Requesting for frontier target for robot " << markerPath_.id <<
    //                         "..." << "\n";
    //            on_reqBtn_clicked();
    //            isFirstLoop_ = false;
    //            return;
    //        }
    //        tf::TransformListener tf;
    //        tf::StampedTransform transform;
    //        geometry_msgs::Point currentPose;
    //        // If the quadrotor is to be commanded to land due to three possible reasons:
    //        //  1.) No more frontiers that are accessible for the robot.
    //        //  2.) Disconnection
    //        //  3.) Failure to localize
    //        if (!isLand_)
    //        {
    //            // Check if the quadrotor had already reach the certain radius of the frontier
    //            try
    //            {
    //                ros::Time now = ros::Time::now();
    //                tf.waitForTransform("map", cameraxLink.toStdString(), now, ros::Duration(3));
    //                tf.lookupTransform("map", cameraxLink.toStdString(), now, transform);
    //                currentPose.x = transform.getOrigin().x();
    //                currentPose.y = transform.getOrigin().y();

    //                double norm = (currentPose.x-currentFrontier_.x)*(currentPose.x-currentFrontier_.x) +
    //                        (currentPose.y-currentFrontier_.y)*(currentPose.y-currentFrontier_.y);

    //                if (sqrt(norm) <= 0.5)
    //                {
    //                    log_ << "Autonomous Planner: Robothas successfully reached" <<
    //                                 " within the AOE of 0.5 towards the frontier point; Distance remaining = " <<
    //                                 sqrt(norm) << " meters" << "\n";
    //                    log_ << "Autonomous Planner: Requesting for another frontier for robot " << markerPath_.id <<
    //                                 "..." << "\n";
    //                    on_reqBtn_clicked();
    //                }
    //            }
    //            catch(tf::TransformException ex)
    //            {
    //                log_ << "Autonomous Planner: Unable to determine if robot " << markerPath_.id <<
    //                             "has reached its destination. Pose is unavailable." << "\n";
    //                isLand_ = true;
    //            }

    //        }
    //        else
    //        {
    //            log_ << "Autonomous Planner: There is no more reachable destinations for robot " <<
    //                         markerPath_.id << ". The autonomous operation ends here. Thank you." << "\n";
    //            std_msgs::Bool a;
    //            a.data = true;
    //            pubLand_.publish(a);
    //            isGateTrue_ = false;
    //        }
    //    }
}

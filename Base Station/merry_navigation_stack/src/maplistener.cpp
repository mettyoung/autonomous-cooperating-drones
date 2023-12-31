#include "maplistener.h"

MapListener::MapListener(double inflationRadius, QObject *parent) :
    QObject(parent)
{
    inflationRadius_ = inflationRadius;
//    ros::NodeHandle n;
//    double r = inflationRadius;
//    n.param(ros::this_node::getNamespace() + ros::this_node::getName() + "/inflation_radius", inflationRadius_, r);
}

gridmap_2d::GridMap2D MapListener::sharedMap()
{
    QMutexLocker locker(&mapListenerMutex_);
    return sharedMap_;
}

gridmap_2d::GridMap2D MapListener::frontierMap()
{
    QMutexLocker locker(&mapListenerMutex_);
    return frontierMap_;
}

gridmap_2d::GridMap2D MapListener::inflatedMap()
{
    QMutexLocker locker(&mapListenerMutex_);
    return inflatedMap_;
}

void MapListener::sharedMap_cb(const nav_msgs::OccupancyGrid::ConstPtr &new_map)
{
    QMutexLocker locker(&mapListenerMutex_);
    // Buffer new map
    sharedMap_ = gridmap_2d::GridMap2D(new_map);
    frontierMap_ = gridmap_2d::GridMap2D(new_map);
    inflatedMap_ = gridmap_2d::GridMap2D(new_map);
    // Inflate the maps
    frontierMap_.inflateMap(inflationRadius_);
    inflatedMap_.inflateMap(inflationRadius_);
    pub_inflatedMap_.publish(inflatedMap_.toOccupancyGridMsg());
    // Insert the poses in the frontier map
    uint mx, my;
    tf::StampedTransform transform;
    double timeDelayed;
    cv::Mat temp = frontierMap_.gridMap().clone();
    for (int i = 0; i < 2; i++)
    {
        if(queryPose.getPose(i, transform, timeDelayed))
        {
            sharedMap_.worldToMap(transform.getOrigin().x(),
                                  transform.getOrigin().y(),
                                  my, mx);
            cv::circle(temp, cv::Point(mx, my), 15, cv::Scalar(0), -1);
        }
    }
    frontierMap_.updateGridMap(temp, false);
    /// Just for adding details for thesis presentation
//    log_ << DisplayImage(sharedMap_.gridMap(), 4, "Original Grid Map");

//    log_ << DisplayImage(sharedMap_.binaryMap(), 0, "Original Binary Map");
//    static cv::Mat a,b;
//    a = sharedMap_.distanceMap().clone();
//    float min = 10000000000;
//    float max = 0;
//    a/=0.05;
//    float *data = (float*)a.data;
//    for (int i = 0; i < a.rows; i++)
//        for (int j = 0; j < a.cols; j++)
//        {
//            if (data[i*a.cols+j] < min)
//                min = data[i*a.cols+j];
//            else if (data[i*a.cols+j] > max)
//                max = data[i*a.cols+j];
//        }
//    a.convertTo(b, CV_8UC1);
//    log_ << DisplayImage(b, 1, "Distance Map Equivalent");
//    log_ << DisplayImage(frontierMap_.binaryMap(), 2, "Binary image with inflation");
//    log_ << DisplayImage(frontierMap_.gridMap(), 3, "FInal frontier");
}

void MapListener::start()
{
    ros::NodeHandle n;
    mapSub_ = n.subscribe("/projected_map", 1, &MapListener::sharedMap_cb, this);
    pub_inflatedMap_ = n.advertise<nav_msgs::OccupancyGrid>("MapListener/inflatedMap", 1);
    timer_ = new QTimer();
    timer_->start(1);
    connect(timer_, SIGNAL(timeout()), this, SLOT(run()));
}

void MapListener::run()
{
    ros::spinOnce();
}

#include "tfcompleter.h"

// Constructor
TfCompleter::TfCompleter(uint freq): freq_(freq), isRunning_(false)
{
    map1_.x = 0; map1_.y = 0; map1_.z = 0; map1_.roll = 0; map1_.pitch = 0; map1_.yaw = 0;
    map2_.x = 0; map2_.y = 0; map2_.z = 1; map2_.roll = 0; map2_.pitch = 0; map2_.yaw = 0;
    camera1Link_.x = 0; camera1Link_.y = 0; camera1Link_.z = 0;
    camera1Link_.roll = 0; camera1Link_.pitch = 0; camera1Link_.yaw = 0;
    camera2Link_.x = 0; camera2Link_.y = 0; camera2Link_.z = 0;
    camera2Link_.roll = 0; camera2Link_.pitch = 0; camera2Link_.yaw = 0;

}


// Public Slot: The igniter of the tf completer.
void TfCompleter::start()
{
    timer_ = new QTimer();
    // Convert frequency to time period expressed in ms.
    timer_->start((int)(1./freq_*1000));
    connect(timer_, SIGNAL(timeout()), this, SLOT(run()));
}


// Private Function
void TfCompleter::keepAlive(const char* parent_frame, const char* child_frame, Transformation &m)
{

    // Get the latest data in the buffer. Then send the tf again in order to keeping it alive.
    try
    {
        // Get the transformation
        tf_.lookupTransform(parent_frame, child_frame,
                            ros::Time(0), transform_);

        // Store the transformation
        m.x = transform_.getOrigin().x();
        m.y = transform_.getOrigin().y();
        m.z = transform_.getOrigin().z();
        transform_.getBasis().getRPY(m.roll, m.pitch, m.yaw);

        // Broadcast/Echo the transformation
        transform_.setOrigin( tf::Vector3 (m.x, m.y, m.z));
        tf::Quaternion q;
        q.setRPY(m.roll, m.pitch, m.yaw);
        transform_.setRotation(q);
        br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), parent_frame, child_frame));
    }
    // This only means the buffer has no data yet. Hence, initialize the
    // transform
    catch (tf::TransformException ex)
    {
        transform_.setOrigin( tf::Vector3 (m.x, m.y, m.z));
        tf::Quaternion q;
        q.setRPY(m.roll, m.pitch, m.yaw);
        transform_.setRotation(q);
        br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), parent_frame, child_frame));
    }

}

// Private Slot: Keep Alive and Be Well. :)
void TfCompleter::run()
{

    // Phase 1: Save all transformations.
    // 1. map ---> map1 (always zero -- constant)
    // 2. map ---> map2 (changing)
    // 3. map1 --> camera1_link
    // 4. map2 --> camera2_link
    // 5. map1 --> map1Origin
    // 6. map2 --> map2Origin
    keepAlive("/map", "/map1", map1_);
    keepAlive("/map", "/map2", map2_);

    // If the process is not running, let the rgbdslam dictate
    // the transformation for cameraLink and camera2Link.
//    if (!isRunning_)
//    {
//        keepAlive("/map1", "/camera1_link", camera1Link_);
//        keepAlive("/map2", "/camera2_link", camera2Link_);
//    }
}

// Public Slot:
void TfCompleter::togglePause()
{
    isRunning_ = !isRunning_;
}

// Public Slots: Changing the course of the tfs.
void TfCompleter::setMapToMap2(Transformation m)
{
    transform_.setOrigin( tf::Vector3 (m.x, m.y, m.z));
    tf::Quaternion q;
    q.setRPY(m.roll, m.pitch, m.yaw);
    transform_.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "/map", "/map2"));
}

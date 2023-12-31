/* File: tfcompleter.h/.cpp
 * Description: The tfcompleter basically keeps the entire tf tree alive, whether the
 * process is running or not. It publishes the last buffered transformation in the following
 * frames:
 *      = /map -> /map1 (/map = /map1 will always be the case because /map1 is world frame itself)
 *      = /map -> /map2 (/map2 will be transformed to be overlapped with /map1)
 *      = /map1 -> /map1Origin (Top-left most pixel of map 1)
 *      = /map1 -> /camera1_link (Camera 1's pose)
 *      = /map2 -> /map2Origin (Top-left most pixel of map 2)
 *      = /map2 -> /camera2_link (Camera 2's pose)
 * Note: This class may share in a thread.
 */

#ifndef TFCOMPLETER_H
#define TFCOMPLETER_H

#include <QObject>
#include <QTimer>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>

// Transformation Structure
struct Transformation {
    float x, y, z;
    double roll, pitch, yaw;
};

class TfCompleter : public QObject
{
    Q_OBJECT

public:
    // Constructor 1:
    // @param freq: Specifies the rate for broadcasting the other frames.
    explicit TfCompleter(uint freq = 10);

public Q_SLOTS:
    // Execute this main function for the TfCompleter. Then all is well.
    // Inside it, it initializes the timer and fires up the timer. The
    // private method run() consists the tf broadcasting or keeping the
    // tf tree alive.
    void start();

    // If the process is running, then do not keep alive cameraLink_ and
    // camera2Link_ and keep alive the rest.
    // If the process is not running, then keep alive everything.
    void togglePause();

    // Slots where the emitted transformation will be broadcasted.
    void setMapToMap2(Transformation m);

private:
    // Saves the most recent transformation for the following frames.
    // All transformations are referenced to their parent frame. The
    // name of the variable is the child frame.
    Transformation map1_, map2_;
    Transformation camera1Link_, camera2Link_;

    // Tf Broadcaster and Listener object.
    tf::TransformBroadcaster br_;
    tf::TransformListener tf_;
    tf::StampedTransform transform_;

    // The timer object for specifying the rate for the transformation
    QTimer *timer_;
    uint freq_;

    // To monitor if the process is running or not.
    bool isRunning_;

    // The most recent transformation in the buffer is re-transmitted as /tf
    // with a new time stamp; in order to keep it alive.
    // @param parent_frame: The frame where we want the projection to be.
    // @param child_frame: The frame which we want to project.
    // @param m: Where the transformation is to be stored or used.
    void keepAlive(const char* parent_frame, const char* child_frame, Transformation& m);

private Q_SLOTS:
    // Tf Broadcasting function.
    void run();
};

#endif


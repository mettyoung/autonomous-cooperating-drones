#ifndef OCTOMAP_2D_H_
#define OCTOMAP_2D_H_

#include <octomap_server/OctomapServer.h>
#include "node.h"
#include <QImage>
class Octomap2D: public QObject, public octomap_server::OctomapServer {
Q_OBJECT
public:
    Octomap2D();
    void insertCloudCallback(Node* node, tf::StampedTransform sensorToWorldTf, bool isLastNode = false);
    void insertScan(const tf::Point& sensorOriginTf, const pcl::PointCloud<pcl::PointXYZRGB>& ground,
                    const pcl::PointCloud<pcl::PointXYZRGB>& nonground);
    void handlePostNodeTraversal(const ros::Time& rostime);
    void reset();

    void setFilters(double pointcloudMinZ, double pointcloudMaxZ);

Q_SIGNALS:
    void set2DMap(QImage);
};
#endif

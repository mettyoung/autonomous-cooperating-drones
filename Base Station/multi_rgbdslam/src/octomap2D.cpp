#include "octomap2D.h"
#include "Timer.h"

Octomap2D::Octomap2D(): octomap_server::OctomapServer(
        ros::NodeHandle(ros::this_node::getNamespace()+ ros::this_node::getName() + "/config")){}


void Octomap2D::insertCloudCallback(Node* node, tf::StampedTransform sensorToWorldTf, bool isLastNode){

    /// First Section = 2us
    ros::WallTime startTime = ros::WallTime::now();

    pcl::PointCloud<pcl::PointXYZRGB> pc;

    // The transform is used here.
    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

    pcl::PointCloud<pcl::PointXYZRGB> pc_ground; // segmented ground plane
    pcl::PointCloud<pcl::PointXYZRGB> pc_nonground; // everything else

    Timer timer;
    timer.start();
    /// Second Section = 250us
    // directly transform to map frame:
    pcl::transformPointCloud(*node->pc_col, pc, sensorToWorld);
    // just filter height range:
    pass.setInputCloud(pc.makeShared());
    pass.filter(pc);

    pc_nonground = pc;
    // pc_nonground is empty without ground segmentation
    pc_ground.header = pc.header;
    pc_nonground.header = pc.header;

    /// Third Section = 100 ms
    insertScan(sensorToWorldTf.getOrigin(), pc_ground, pc_nonground);

    double total_elapsed = (ros::WallTime::now() - startTime).toSec();
    ROS_DEBUG("Pointcloud insertion in OctomapServer done (%zu+%zu pts (ground/nonground), %f sec)", pc_ground.size(), pc_nonground.size(), total_elapsed);

    if (isLastNode)
        publishAll(sensorToWorldTf.stamp_);
}

void Octomap2D::insertScan(const tf::Point& sensorOriginTf, const pcl::PointCloud<pcl::PointXYZRGB>& ground,
                           const pcl::PointCloud<pcl::PointXYZRGB>& nonground){

    octomap::point3d sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);

    if (!m_octree->genKey(sensorOrigin, m_updateBBXMin)
            || !m_octree->genKey(sensorOrigin, m_updateBBXMax))
    {
        ROS_ERROR_STREAM("Could not generate Key for origin "<<sensorOrigin);
    }



    // instead of direct scan insertion, compute update to filter ground:
    octomap::KeySet free_cells, occupied_cells;
    // insert ground points only as free:
    for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = ground.begin(); it != ground.end(); ++it){
        octomap::point3d point(it->x, it->y, it->z);
        // maxrange check
        if ((m_maxRange > 0.0) && ((point - sensorOrigin).norm() > m_maxRange) ) {
            point = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
        }

        // only clear space (ground points)
        if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
            free_cells.insert(m_keyRay.begin(), m_keyRay.end());
        }

        octomap::OcTreeKey endKey;
        if (m_octree->genKey(point, endKey)){
            updateMinKey(endKey, m_updateBBXMin);
            updateMaxKey(endKey, m_updateBBXMax);
        } else{
            ROS_ERROR_STREAM("Could not generate Key for endpoint "<<point);
        }
    }

    // all other points: free on ray, occupied on endpoint:
    for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = nonground.begin(); it != nonground.end(); ++it){
        octomap::point3d point(it->x, it->y, it->z);
        // maxrange check
        if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) {

            // free cells
            if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
            }
            // occupied endpoint
            octomap::OcTreeKey key;
            if (m_octree->genKey(point, key)){
                occupied_cells.insert(key);

                updateMinKey(key, m_updateBBXMin);
                updateMaxKey(key, m_updateBBXMax);
            }
        } else {// ray longer than maxrange:;
            octomap::point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
            if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)){
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());

                octomap::OcTreeKey endKey;
                if (m_octree->genKey(new_end, endKey)){
                    updateMinKey(endKey, m_updateBBXMin);
                    updateMaxKey(endKey, m_updateBBXMax);
                } else{
                    ROS_ERROR_STREAM("Could not generate Key for endpoint "<<new_end);
                }


            }
        }
    }

    // mark free cells only if not seen occupied in this cloud
    for(octomap::KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
        if (occupied_cells.find(*it) == occupied_cells.end()){
            m_octree->updateNode(*it, false);
        }
    }

    // now mark all occupied cells:
    for (octomap::KeySet::iterator it = occupied_cells.begin(), end=free_cells.end(); it!= end; it++) {
        m_octree->updateNode(*it, true);
    }

    // TODO: eval lazy+updateInner vs. proper insertion
    // non-lazy by default (updateInnerOccupancy() too slow for large maps)
    //m_octree->updateInnerOccupancy();
    octomap::point3d minPt, maxPt;
    ROS_DEBUG_STREAM("Bounding box keys (before): " << m_updateBBXMin[0] << " " <<m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / " <<m_updateBBXMax[0] << " "<<m_updateBBXMax[1] << " "<< m_updateBBXMax[2]);

    // TODO: snap max / min keys to larger voxels by m_maxTreeDepth
    //   if (m_maxTreeDepth < 16)
    //   {
    //      OcTreeKey tmpMin = getIndexKey(m_updateBBXMin, m_maxTreeDepth); // this should give us the first key at depth m_maxTreeDepth that is smaller or equal to m_updateBBXMin (i.e. lower left in 2D grid coordinates)
    //      OcTreeKey tmpMax = getIndexKey(m_updateBBXMax, m_maxTreeDepth); // see above, now add something to find upper right
    //      tmpMax[0]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
    //      tmpMax[1]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
    //      tmpMax[2]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
    //      m_updateBBXMin = tmpMin;
    //      m_updateBBXMax = tmpMax;
    //   }

    // TODO: we could also limit the bbx to be within the map bounds here (see publishing check)
    m_octree->genCoords(m_updateBBXMin, m_octree->getTreeDepth(), minPt);
    m_octree->genCoords(m_updateBBXMax, m_octree->getTreeDepth(), maxPt);
    ROS_DEBUG_STREAM("Updated area bounding box: "<< minPt << " - "<<maxPt);
    ROS_DEBUG_STREAM("Bounding box keys (after): " << m_updateBBXMin[0] << " " <<m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / " <<m_updateBBXMax[0] << " "<<m_updateBBXMax[1] << " "<< m_updateBBXMax[2]);

    if (m_compressMap)
        m_octree->prune();
}

void Octomap2D::reset()
{
    visualization_msgs::MarkerArray occupiedNodesVis;
    occupiedNodesVis.markers.resize(m_treeDepth +1);
    ros::Time rostime = ros::Time::now();
    m_octree->clear();
    // clear 2D map:
    m_gridmap.data.clear();
    m_gridmap.info.height = 0.0;
    m_gridmap.info.width = 0.0;
    m_gridmap.info.resolution = 0.0;
    m_gridmap.info.origin.position.x = 0.0;
    m_gridmap.info.origin.position.y = 0.0;

    ROS_INFO("Cleared octomap");

    for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){

        occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
        occupiedNodesVis.markers[i].header.stamp = rostime;
        occupiedNodesVis.markers[i].ns = "map";
        occupiedNodesVis.markers[i].id = i;
        occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    visualization_msgs::MarkerArray freeNodesVis;
    freeNodesVis.markers.resize(m_treeDepth +1);

    for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i){

        freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
        freeNodesVis.markers[i].header.stamp = rostime;
        freeNodesVis.markers[i].ns = "map";
        freeNodesVis.markers[i].id = i;
        freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
        freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }
}

void Octomap2D::setFilters(double pointcloudMinZ, double pointcloudMaxZ)
{
    m_pointcloudMaxZ = pointcloudMaxZ;
    m_pointcloudMinZ = pointcloudMinZ;
}

void Octomap2D::handlePostNodeTraversal(const ros::Time& rostime){

    if (m_publish2DMap)
        m_mapPub.publish(m_gridmap);


    cv::Mat map(m_gridmap.info.width, m_gridmap.info.height, CV_8UC1);

    for(unsigned int j = 0; j < m_gridmap.info.height; ++j){
        for(unsigned int i = 0; i < m_gridmap.info.width; ++i){
            if (m_gridmap.data[j*m_gridmap.info.width+i] == 100)
                map.data[i*m_gridmap.info.height+j] = 0;
            else if (m_gridmap.data[j*m_gridmap.info.width+i] == 0)
                map.data[i*m_gridmap.info.height+j] = 255;
            else
                map.data[i*m_gridmap.info.height+j] = 205;
        }
    }

    Q_EMIT set2DMap(QImage((uint8_t*)map.data, map.cols, map.rows, QImage::Format_ARGB32));
    // Emit here

}


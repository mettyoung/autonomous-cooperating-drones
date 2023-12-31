/* This file is part of RGBDSLAM.
 *
 * RGBDSLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RGBDSLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RGBDSLAM.  If not, see <http://www.gnu.org/licenses/>.
 */


//Documentation see header file
#include "pcl/ros/conversions.h"
#include <pcl/io/io.h>
//#include "pcl/common/transform.h"
#include "pcl_ros/transforms.h"
#include "openni_listener.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <cv.h>
//#include <ctime>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include "node.h"
#include "misc.h"
//#include <image_geometry/pinhole_camera_model.h>
//#include "pcl/ros/for_each_type.h"

//For rosbag reading
#include <rosbag/view.h>
#include <boost/foreach.hpp>


#include "parameter_server.h"
#include "scoped_timer.h"
//for comparison with ground truth from mocap and movable cameras on robots
#include <tf/transform_listener.h>

typedef message_filters::Subscriber<sensor_msgs::Image> image_sub_type;
typedef message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub_type;
typedef message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_type;
typedef message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_type;

OpenNIListener::OpenNIListener(GraphManager* graph_mgr)
: graph_mgr_(graph_mgr),
  no_cloud_sync_(NULL),
  save_bag_file(false),
  visua_sub_(NULL), depth_sub_(NULL),
  pause_(ParameterServer::instance()->get<bool>("start_paused")),
  getOneFrame_(false),
  first_frame_(true)
{
  ParameterServer* ps = ParameterServer::instance();
  int i, q;
  std::string iString;
  int camera_count = ps->get<int>("camera_count");
  q = ps->get<int>("subscriber_queue_size");

  processing_node_ = new bool[camera_count];

  bag = new rosbag::Bag[camera_count];
  bagfile_mutex = new QMutex[camera_count];

  data_id_ = new int[camera_count];
  rgba_buffers_ = new std::vector<cv::Mat>[camera_count];
  future_ = new QFuture<void>[camera_count];

  depth_mono8_img_ = new cv::Mat[camera_count];
  visualization_img_ = new cv::Mat[camera_count];
  visualization_depth_mono8_img_ = new cv::Mat[camera_count];

  std::string *visua_tpc = new std::string[camera_count];
  std::string *depth_tpc = new std::string[camera_count];
  std::string *cinfo_tpc = new std::string[camera_count];

  image_encoding_ = new std::string[camera_count];
  visua_sub_ = new image_sub_type*[camera_count];
  depth_sub_ = new image_sub_type*[camera_count];
  cinfo_sub_ = new cinfo_sub_type*[camera_count];
  no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>*[camera_count];

  for(i=0; i<camera_count; i++){
      processing_node_[i] = false;
      iString = boost::lexical_cast<std::string>(i);
      data_id_[i] = 0;
      depth_mono8_img_[i] = (cv::Mat());
      visualization_img_[i] = (cv::Mat());
      visualization_depth_mono8_img_[i] = (cv::Mat());
      image_encoding_[i] = ("rgb8");
      visua_tpc[i] = ps->get<std::string>("topic_image_mono"+iString);
      depth_tpc[i] = ps->get<std::string>("topic_image_depth"+iString);
      cinfo_tpc[i] = ps->get<std::string>("camera_info_topic"+iString);
  }

  ros::NodeHandle nh;
  tflistener_ = new tf::TransformListener(nh);

  for(i=0; i<camera_count; i++){
    //No cloud, but visual image and depth
    if(!visua_tpc[i].empty() && !depth_tpc[i].empty() && !cinfo_tpc[i].empty())
    {
        visua_sub_[i] = new image_sub_type(nh, visua_tpc[i], q);
        depth_sub_[i] = new image_sub_type(nh, depth_tpc[i], q);
        cinfo_sub_[i] = new cinfo_sub_type(nh, cinfo_tpc[i], q);
        no_cloud_sync_[i] = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(q),  *visua_sub_[i], *depth_sub_[i], *cinfo_sub_[i]);
        no_cloud_sync_[i]->registerCallback(boost::bind(&OpenNIListener::noCloudCallback, this, _1, _2, _3, i));
        ROS_INFO_STREAM("Listening to " << visua_tpc[i] << " and " << depth_tpc[i]);
    }

    detector_ = createDetector(ps->get<std::string>("feature_detector_type"));
    extractor_ = createDescriptorExtractor(ps->get<std::string>("feature_extractor_type"));

    if(ps->get<bool>("concurrent_node_construction")){
      ROS_DEBUG("Threads used by QThreadPool on this Computer %i. Will increase this by one, b/c the QtRos Thread is very lightweight", QThread::idealThreadCount());
      //QThreadPool::globalInstance()->setMaxThreadCount(QThread::idealThreadCount()*2+2);
    }
  }
}

OpenNIListener::~OpenNIListener(){
  delete tflistener_;
}

void OpenNIListener::getOneFrame(){
    getOneFrame_=true;
}

void OpenNIListener::togglePause(){
  pause_ = !pause_;
  ROS_INFO("Pause toggled to: %s", pause_? "true":"false");
  if(pause_) Q_EMIT setGUIStatus("Processing Thread Stopped");
  else Q_EMIT setGUIStatus("Processing Thread Running");
}
void OpenNIListener::noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                      const sensor_msgs::ImageConstPtr& depth_img_msg,
                                      const sensor_msgs::CameraInfoConstPtr& cam_info_msg,
                                      int camera_id){
  ScopedTimer s(__FUNCTION__);
  ROS_WARN_ONCE_NAMED("eval", "First RGBD-Data Received");
  ROS_DEBUG("Received data from kinect");
  //if(processing_node_[camera_id])
  //    return;
  processing_node_[camera_id] = true;
  ParameterServer* ps = ParameterServer::instance();

  if(++data_id_[camera_id] < ps->get<int>("skip_first_n_frames") || data_id_[camera_id] % ps->get<int>("data_skip_step") != 0){
  // If only a subset of frames are used, skip computations but visualize if gui is running
      ROS_INFO_THROTTLE(1, "Skipping Frame %i because of data_skip_step setting (this msg is only shown once a sec)", data_id_[camera_id]);
    if(ps->get<bool>("use_gui")){//Show the image, even if not using it
      cv::Mat depth_float_img = cv_bridge::toCvCopy(depth_img_msg)->image;
      cv::Mat visual_img =  cv_bridge::toCvCopy(visual_img_msg)->image;

      if(visual_img.rows != depth_float_img.rows || visual_img.cols != depth_float_img.cols){
        ROS_ERROR("depth and visual image differ in size! Ignoring Data");
        processing_node_[camera_id] = false;
        return;
      }
      depthToCV8UC1(depth_float_img, depth_mono8_img_[camera_id]); //float can't be visualized or used as mask in float format TODO: reprogram keypoint detector to use float values with nan to mask
      image_encoding_[camera_id] = visual_img_msg->encoding;
      Q_EMIT newRGBDImage(cvMat2QImage(visual_img, 0, camera_id), cvMat2QImage(depth_mono8_img_[camera_id],1, camera_id), camera_id); //visual_idx=0
    }
    processing_node_[camera_id] = false;
    return;
  }

  //Convert images to OpenCV format
  cv::Mat depth_float_img = cv_bridge::toCvCopy(depth_img_msg)->image;
  cv::Mat visual_img;
  if(image_encoding_[camera_id] == "bayer_grbg8"){
    cv_bridge::toCvShare(visual_img_msg);
    ROS_INFO("Converting from Bayer to RGB");
    cv::cvtColor(cv_bridge::toCvCopy(visual_img_msg)->image, visual_img, CV_BayerGR2RGB, 3);
  } else{
    ROS_DEBUG_STREAM("Encoding: " << visual_img_msg->encoding);
    visual_img =  cv_bridge::toCvCopy(visual_img_msg)->image;
  }
  if(visual_img.rows != depth_float_img.rows || visual_img.cols != depth_float_img.cols){
    ROS_ERROR("depth and visual image differ in size! Ignoring Data");
    processing_node_[camera_id] = false;
    return;
  }
  image_encoding_[camera_id] = visual_img_msg->encoding;
  depthToCV8UC1(depth_float_img, depth_mono8_img_[camera_id]); //float can't be visualized or used as mask in float format TODO: reprogram keypoint detector to use float values with nan to mask
  if(asyncFrameDrop(depth_img_msg->header.stamp, visual_img_msg->header.stamp)){
    processing_node_[camera_id] = false;
    return; 
  }

  if (bagfile_mutex[camera_id].tryLock() && save_bag_file){
     // todo: make the names dynamic
     bag[camera_id].write("/camera/rgb/image_mono", ros::Time::now(), visual_img_msg);
     bag[camera_id].write("/camera/depth/image", ros::Time::now(), depth_img_msg);
     ROS_INFO_STREAM("Wrote to bagfile " << bag[camera_id].getFileName());
     bagfile_mutex[camera_id].unlock();
  }

  if(ps->get<bool>("use_gui"))
      Q_EMIT newRGBDImage(cvMat2QImage(visual_img, 0, camera_id), cvMat2QImage(depth_mono8_img_[camera_id],1, camera_id), camera_id); //visual_idx=0

  if(pause_ && !getOneFrame_){
      processing_node_[camera_id] = false;
      return;
  }

  noCloudCameraCallback(visual_img, depth_float_img, depth_mono8_img_[camera_id], depth_img_msg->header, cam_info_msg, camera_id);
  processing_node_[camera_id] = false;
}

void OpenNIListener::noCloudCameraCallback(cv::Mat visual_img,
                                           cv::Mat depth,
                                           cv::Mat depth_mono8_img,
                                           std_msgs::Header depth_header,
                                           const sensor_msgs::CameraInfoConstPtr& cam_info,
                                           int camera_id){
  if(getOneFrame_) { //if getOneFrame_ is set, unset it and skip check for  pause
      getOneFrame_ = false;
  } else if(pause_) { //Visualization and nothing else
    return;
  }
  ScopedTimer s(__FUNCTION__);
  //######### Main Work: create new node ##############################################################
  Q_EMIT setGUIStatus("Computing Keypoints and Features");

  Node* node_ptr = new Node(camera_id, visual_img, depth, depth_mono8_img, cam_info, depth_header, detector_, extractor_);
  retrieveTransformations(depth_header, node_ptr, camera_id);//Retrieve the transform between the lens and the base-link at capturing time;
  callProcessing(visual_img, node_ptr, camera_id);

}

//Call function either regularly or as background thread
void OpenNIListener::callProcessing(cv::Mat visual_img, Node* node_ptr, int camera_id)
{
  std::clock_t parallel_wait_time=std::clock();
  if(!future_[camera_id].isFinished()){
    future_[camera_id].waitForFinished(); //Wait if GraphManager ist still computing.
    ROS_INFO_STREAM_NAMED("timings", "waiting time: "<< ( std::clock() - parallel_wait_time ) / (double)CLOCKS_PER_SEC  <<"sec");
  }

  //update for visualization of the feature flow
  visualization_img_[camera_id] = visual_img; //No copy
  visualization_depth_mono8_img_[camera_id] = depth_mono8_img_[camera_id];//No copy
  //With this define, processNode runs in the background and after finishing visualizes the results
  //Thus another Callback can be started in the meantime, to create a new node in parallel
  if(ParameterServer::instance()->get<bool>("concurrent_node_construction")) {
    ROS_DEBUG("Processing Node in parallel to the construction of the next node");
    if(ParameterServer::instance()->get<bool>("use_gui")){
      //visual_img points to the data received in the callback - which might be deallocated after the callback returns.
      //This will happen before visualization if processNode is running in background, therefore the data needs to be cloned
      visualization_img_[camera_id] = visual_img.clone();
      visualization_depth_mono8_img_[camera_id] = depth_mono8_img_[camera_id].clone();
    }
    future_[camera_id] = QtConcurrent::run(this, &OpenNIListener::processNode, node_ptr, camera_id);
  }
  else { //Non-concurrent
    processNode(node_ptr, camera_id);//regular function call
  }
}

void OpenNIListener::processNode(Node* new_node, int camera_id)
{
  ScopedTimer s(__FUNCTION__);
  ROS_INFO("START: processNode, node: %i, cam: %i", new_node->seq_id_, camera_id);
  Q_EMIT setGUIStatus("Adding Node to Graph");
  bool has_been_added = graph_mgr_->addNode(new_node, camera_id);

  //######### Visualization code  #############################################
  if(ParameterServer::instance()->get<bool>("use_gui")){
    if(has_been_added){
      if(ParameterServer::instance()->get<bool>("visualize_mono_depth_overlay")){
        cv::Mat feature_img = cv::Mat::zeros( visualization_img_[camera_id].rows, visualization_img_[camera_id].cols, CV_8UC1);
        graph_mgr_->drawFeatureFlow(camera_id, feature_img);
        Q_EMIT newFeatureFlowImage(cvMat2QImage(visualization_img_[camera_id],visualization_depth_mono8_img_[camera_id], feature_img, 2, camera_id), camera_id); //show registration
      } else {
        graph_mgr_->drawFeatureFlow(camera_id, visualization_img_[camera_id], cv::Scalar(0,0,255), cv::Scalar(0,128,0));
        Q_EMIT newFeatureFlowImage(cvMat2QImage(visualization_img_[camera_id], 2, camera_id), camera_id); //show registration
      }
    } else {
      if(ParameterServer::instance()->get<bool>("visualize_mono_depth_overlay")){
        cv::Mat feature_img = cv::Mat( visualization_img_[camera_id].rows, visualization_img_[camera_id].cols, CV_8UC1);
        cv::drawKeypoints(feature_img, new_node->feature_locations_2d_, feature_img, cv::Scalar(155), 5);
        Q_EMIT newFeatureFlowImage(cvMat2QImage(visualization_img_[camera_id],visualization_depth_mono8_img_[camera_id], feature_img, 2, camera_id), camera_id); //show registration
      } else {
        cv::drawKeypoints(visualization_img_[camera_id], new_node->feature_locations_2d_, visualization_img_[camera_id], cv::Scalar(0, 100,0), 5);
        Q_EMIT newFeatureFlowImage(cvMat2QImage(visualization_img_[camera_id], 2, camera_id), camera_id); //show registration
      }
    }
  }
  if(!has_been_added) delete new_node;
  ROS_INFO("END: processNode, node: %i, cam: %i", new_node->seq_id_, camera_id);

}

/// Create a QImage from image. The QImage stores its data in the rgba_buffers_ indexed by idx (reused/overwritten each call)
QImage OpenNIListener::cvMat2QImage(const cv::Mat& channel1, const cv::Mat& channel2, const cv::Mat& channel3, unsigned int idx, int camera_id){
  if(rgba_buffers_[camera_id].size() <= idx){
      rgba_buffers_[camera_id].resize(idx+1);
  }
  if(channel2.rows != channel1.rows || channel2.cols != channel1.cols ||
     channel3.rows != channel1.rows || channel3.cols != channel1.cols){
     ROS_ERROR("Image channels to be combined differ in size");
  }
  if(channel1.rows != rgba_buffers_[camera_id][idx].rows || channel1.cols != rgba_buffers_[camera_id][idx].cols){
    ROS_DEBUG("Created new rgba_buffer with index %i", idx);
    rgba_buffers_[camera_id][idx] = cv::Mat( channel1.rows, channel1.cols, CV_8UC4);
    //printMatrixInfo(rgba_buffers_[idx]);
  }
  cv::Mat mono_channel1 = channel1;
  if(channel1.type() != CV_8UC1) {
    mono_channel1 = cv::Mat( channel1.rows, channel1.cols, CV_8UC1);
    cv::cvtColor(channel1, mono_channel1, CV_RGB2GRAY);
  }
  std::vector<cv::Mat> input;
  cv::Mat alpha( channel1.rows, channel1.cols, CV_8UC1, cv::Scalar(255)); //TODO this could be buffered for performance
  input.push_back(mono_channel1);
  input.push_back(channel2);
  input.push_back(channel3);
  input.push_back(alpha);
  cv::merge(input, rgba_buffers_[camera_id][idx]);
  //printMatrixInfo(rgba_buffers_[idx]);
  return QImage((unsigned char *)(rgba_buffers_[camera_id][idx].data),
                rgba_buffers_[camera_id][idx].cols, rgba_buffers_[camera_id][idx].rows,
                rgba_buffers_[camera_id][idx].step, QImage::Format_RGB32 );
}

QImage OpenNIListener::cvMat2QImage(const cv::Mat& image, unsigned int idx, int camera_id){
  ROS_DEBUG_STREAM("Converting Matrix of type " << openCVCode2String(image.type()) << " to RGBA");
  if(rgba_buffers_[camera_id].size() <= idx){
      rgba_buffers_[camera_id].resize(idx+1);
  }
  ROS_DEBUG_STREAM("Target Matrix is of type " << openCVCode2String(rgba_buffers_[camera_id][idx].type()));
  if(image.rows != rgba_buffers_[camera_id][idx].rows || image.cols != rgba_buffers_[camera_id][idx].cols){
    ROS_DEBUG("Created new rgba_buffer with index %i", idx);
    rgba_buffers_[camera_id][idx] = cv::Mat( image.rows, image.cols, CV_8UC4);
    //printMatrixInfo(rgba_buffers_[idx], "for QImage Buffering");
  }
  char red_idx = 0, green_idx = 1, blue_idx = 2;
  if(image_encoding_[camera_id].compare("rgb8") == 0) { red_idx = 2; blue_idx = 0; }
  cv::Mat alpha( image.rows, image.cols, CV_8UC1, cv::Scalar(255)); //TODO this could be buffered for performance
  cv::Mat in[] = { image, alpha };
  // rgba[0] -> bgr[2], rgba[1] -> bgr[1],
  // rgba[2] -> bgr[0], rgba[3] -> alpha[0]

  if(image.type() == CV_8UC1){
    int from_to[] = { 0,0,  0,1,  0,2,  1,3 };
    mixChannels( in , 2, &rgba_buffers_[camera_id][idx], 1, from_to, 4 );
  } else {
    int from_to[] = { red_idx,0,  green_idx,1,  blue_idx,2,  3,3 }; //BGR+A -> RGBA
    mixChannels( in , 2, &rgba_buffers_[camera_id][idx], 1, from_to, 4 );
  }
  //printMatrixInfo(rgba_buffers_[idx], "for QImage Buffering");
  return QImage((unsigned char *)(rgba_buffers_[camera_id][idx].data),
                rgba_buffers_[camera_id][idx].cols, rgba_buffers_[camera_id][idx].rows,
                rgba_buffers_[camera_id][idx].step, QImage::Format_RGB32 );
}

//Retrieve the transform between the lens and the base-link at capturing time
void OpenNIListener::retrieveTransformations(std_msgs::Header depth_header, Node* node_ptr, int camera_id)
{
  ScopedTimer s(__FUNCTION__);
  ParameterServer* ps = ParameterServer::instance();
  std::string idString = boost::lexical_cast<std::string>(camera_id);
  std::string base_frame  = ps->get<std::string>("base_frame_name"+idString);
  std::string odom_frame  = ps->get<std::string>("odom_frame_name");
  std::string gt_frame    = ps->get<std::string>("ground_truth_frame_name");
  std::string depth_frame_id = depth_header.frame_id;
  ros::Time depth_time = depth_header.stamp;
  tf::StampedTransform base2points;

  try{
    tflistener_->waitForTransform(base_frame, depth_frame_id, depth_time, ros::Duration(0.005));
    tflistener_->lookupTransform(base_frame, depth_frame_id, depth_time, base2points);
    base2points.stamp_ = depth_time;
  }
  catch (tf::TransformException ex){
    ROS_WARN("%s",ex.what());
    ROS_WARN("Using Standard kinect /openni_camera -> /openni_rgb_optical_frame as transformation (This message is throttled to 1 per 5 seconds)");
    //emulate the transformation from kinect openni_camera frame to openni_rgb_optical_frame
    base2points.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
    base2points.setOrigin(tf::Point(0,-0.04,0));
    base2points.stamp_ = depth_time;
  }
  node_ptr->setBase2PointsTransform(base2points);

  if(!gt_frame.empty()){
    //Retrieve the ground truth data. For the first frame it will be
    //set as origin. the rest will be used to compare
    ROS_ERROR("GT_FRAME NOT EMPTY()");
    /*
    tf::StampedTransform ground_truth_transform;
    try{
      tflistener_->waitForTransform(gt_frame, "/openni_camera", depth_time, ros::Duration(0.005));
      tflistener_->lookupTransform(gt_frame, "/openni_camera", depth_time, ground_truth_transform);
      ground_truth_transform.stamp_ = depth_time;
      tf::StampedTransform b2p;
      //HACK to comply with Jürgen Sturm's Ground truth, though I can't manage here to get the full transform from tf
      b2p.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
      b2p.setOrigin(tf::Point(0,-0.04,0));
      ground_truth_transform *= b2p;
    }
    catch (tf::TransformException ex){
      ROS_WARN_THROTTLE(5, "%s - Using Identity for Ground Truth (This message is throttled to 1 per 5 seconds)",ex.what());
      ground_truth_transform = tf::StampedTransform(tf::Transform::getIdentity(), depth_time, "missing_ground_truth", "/openni_camera");
    }
    printTransform("Ground Truth", ground_truth_transform);
    node_ptr->setGroundTruthTransform(ground_truth_transform);
    */
  }
  if(!odom_frame.empty()){
    //Retrieve the ground truth data. For the first frame it will be
    //set as origin. the rest will be used to compare

    ROS_ERROR("ODOM_FRAME NOT EMPTY()");
    /*
    tf::StampedTransform current_odom_transform;
    try{
      tflistener_->waitForTransform(depth_frame_id, odom_frame, depth_time, ros::Duration(0.005));
      tflistener_->lookupTransform( depth_frame_id, odom_frame, depth_time, current_odom_transform);
    }
    catch (tf::TransformException ex){
      ROS_WARN_THROTTLE(5, "%s - No odometry available (This message is throttled to 1 per 5 seconds)",ex.what());
      current_odom_transform = tf::StampedTransform(tf::Transform::getIdentity(), depth_time, "missing_odometry", depth_frame_id);
      current_odom_transform.stamp_ = depth_time;
    }
    printTransform("Odometry", current_odom_transform);
    node_ptr->setOdomTransform(current_odom_transform);
    */
  }
  // End: Fill in Transformation -----------------------------------------------------------------------
}
void OpenNIListener::toggleBagRecording(){
  int camera_count = ParameterServer::instance()->get<int>("camera_count");
  for(int i=0; i<camera_count; i++){
      bagfile_mutex[i].lock();
      save_bag_file = !save_bag_file;
      // save bag
      if (save_bag_file)
      {
        time_t rawtime;
        struct tm * timeinfo;
        char buffer [80];

        time ( &rawtime );
        timeinfo = localtime ( &rawtime );
        // create a nice name
        std::string string1 = "kinect_cam" + boost::lexical_cast<std::string>(i);
        strftime (buffer,80,"_%Y-%m-%d-%H-%M-%S.bag",timeinfo);

        bag[i].open(string1 + buffer, rosbag::bagmode::Write);
        ROS_INFO_STREAM("Opened bagfile " << bag[i].getFileName());
      } else {
        ROS_INFO_STREAM("Closing bagfile " << bag[i].getFileName());
        bag[i].close();
      }
      bagfile_mutex[i].unlock();
  }
}


/*
 *  Node for publishing the transformations between the cameras
 *  Wim Lemkens <wim.lemkens@gmail.com>
 *  http://rosmultirgbd.wordpress.com
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

std::string child_link = "/camera_link";
std::string parent_link = "/world";




int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");

  static tf::TransformBroadcaster br;

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    tf::Transform transform;
//    ros::Time stamp = ros::Time:: now() + ros::Duration(10.0);
    ros::Time stamp = ros::Time:: now() + ros::Duration(0.5);

    transform.setOrigin( tf::Vector3(0.0398719220461, -0.0172424013639, 0.00846286767662) );
    transform.setRotation( tf::Quaternion(0.000153764251668, 0.00224424578764, -0.00518879412212, 0.999984007939) );
    br.sendTransform(tf::StampedTransform(transform, stamp, "/camera0_link", "/camera0_rgb_optical_frame"));
    transform.setOrigin( tf::Vector3(4.29575787941e-09, -7.66904416545e-10, -7.31808997471e-09) );
    transform.setRotation( tf::Quaternion(-1.60696735089e-09, -2.51402443955e-09, -2.14742006389e-08, 1) );
    br.sendTransform(tf::StampedTransform(transform, stamp, "/camera0_link", "/camera0_depth_optical_frame"));
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  ros::NodeHandle node;

  ros::spin();
  return 0;
};

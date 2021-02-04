/// \file
/// \brief This node published on the initpose topic in order to properly locate the different maps to merge
///
/// PUBLISHES:
///     /initpose (geometry_msgs/PoseWithCovariance): Publishes a pose to only one robot. 

#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <string>
#include <vector>
#include <math.h>

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "init_pose_node");
  ros::NodeHandle nh;

  // Create the initpose publisher and subscriber
  const auto pose0_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose_tb3_0", 100);
  const auto pose1_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose_tb3_1", 100);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
      geometry_msgs::PoseWithCovarianceStamped pose0_msg;
      geometry_msgs::PoseWithCovarianceStamped pose1_msg;

      pose0_msg.header.frame_id = "map";
      pose0_msg.pose.pose.position.x = -7.0;
      pose0_msg.pose.pose.position.y = -1.0;
      pose0_msg.pose.pose.position.z = 0.0;
      pose0_msg.pose.pose.orientation.w = 0.0;

      pose1_msg.header.frame_id = "map";
      pose1_msg.pose.pose.position.x = 7.0;
      pose1_msg.pose.pose.position.y = -1.0;
      pose1_msg.pose.pose.position.z = 0.0;
      pose1_msg.pose.pose.orientation.w = 0.0;

      pose0_pub.publish(pose0_msg);
      pose1_pub.publish(pose1_msg);

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
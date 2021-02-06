/// \file
/// \brief This node published on <robot_ns>/map topic in order to manipulate the map sizes to be the same
///
/// PUBLISHES:
///     /tb3_0/map (nav_msgs/OccupancyGrid): Publishes a new width, height and origin
///     /tb3_1/map (nav_msgs/OccupancyGrid): Publishes a new width, height and origin

#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
// #include <string>
#include <vector>
// #include <math.h>
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"

// define global message 
nav_msgs::OccupancyGrid map0_msg;
// nav_msgs::OccupancyGrid map0_msg = *msg.get();

// / \brief Grabs the position of the robot from the pose subscriber and stores it
// / \param msg - pose message
// / \param turt_pose - stored pose message
// / \returns nothing
// void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
// {
//   map0_msg.header = msg->header;
//   map0_msg.info = msg->info;
//   map0_msg.data = msg->data;

//   // std::cout << map0_msg << std::endl;

//   // map_data = map0_msg.data;

//   // ROS_INFO("data size is... [%d]", *map_data);
//   // std::cout << map_data << std::endl;

// //   map0_msg.info.width = 384;
// //   map0_msg.info.height = 384;
// //   map_meta0_pub.publish(map0_msg);
// }


int main(int argc, char * argv[])
{
  ros::init(argc, argv, "map_metadata_node");
  ros::NodeHandle nh;

  // Create the initpose publisher and subscriber
  // const auto map_meta0_sub = nh.subscribe<nav_msgs::OccupancyGrid>("map", 100, mapCallback);
  const auto map_meta0_pub = nh.advertise<nav_msgs::OccupancyGrid>("test_map", 100);
  // const auto map_meta0_sub = nh.subscribe<nav_msgs::OccupancyGrid>("tb3_0/map", 100, mapCallback);
  // const auto map_meta0_pub = nh.advertise<nav_msgs::OccupancyGrid>("tb3_0/map", 100);
  // const auto map_meta1_pub = nh.advertise<nav_msgs::OccupancyGrid>("tb3_1/map", 100);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
      // map0_msg.header.frame_id = "tb3_0/map";
      // map0_msg.info.width = 384;
      // map0_msg.info.height = 384;
      // map0_msg.data.resize(map0_msg.info.width * map0_msg.info.height);
      // std::fill(map0_msg.data, map0_msg.data + (map0_msg.info.height*map0_msg.info.width), value);
      // map0_msg.data = new_map;
      // map0_msg.info.origin.position.x = -7.0;
      // map0_msg.info.origin.position.y = -1.0;
      // map0_msg.info.origin.position.z = 0.0;
      // map0_msg.info.origin.orientation.w = 0.0;

      nav_msgs::OccupancyGrid new_map;
      new_map.header.frame_id = "new_map";
      new_map.info.resolution = 0.05;
      new_map.info.origin.position.x = -10.0;
      new_map.info.origin.position.y = -10.0;
      new_map.info.origin.position.z = 0.0;
      new_map.info.origin.orientation.w = 0.0;

      const size_t width_ = 384;
      const size_t height_ = 384;
      new_map.info.width = width_;
      new_map.info.height = height_;

      // new_map.data.resize(map0_msg.info.width * map0_msg.info.height);

      for (int i=0;  i < new_map.info.height * new_map.info.width; i++)
      {
        new_map.data.push_back(-1);
      }

      map_meta0_pub.publish(new_map);

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
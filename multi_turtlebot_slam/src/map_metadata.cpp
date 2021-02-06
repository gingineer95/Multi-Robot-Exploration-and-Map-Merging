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
nav_msgs::OccupancyGrid ugh;

/// / \brief Grabs the position of the robot from the pose subscriber and stores it
/// / \param msg - pose message
/// \returns nothing
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
{
  map0_msg.header = msg->header;
  map0_msg.info = msg->info;
  map0_msg.data = msg->data;
  ugh = map0_msg;

  // std::cout << "The width is " << map0_msg.info.width << std::endl;
}


int main(int argc, char * argv[])
{
  ros::init(argc, argv, "map_metadata_node");
  ros::NodeHandle nh;

  // Create the initpose publisher and subscriber
  // const auto map_meta0_sub = nh.subscribe<nav_msgs::OccupancyGrid>("map", 100, mapCallback);
  const auto new_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("new_map", 100);
  const auto map_meta0_sub = nh.subscribe<nav_msgs::OccupancyGrid>("tb3_0/map", 100, mapCallback);
  // const auto map_meta0_pub = nh.advertise<nav_msgs::OccupancyGrid>("tb3_0/map", 100);
  // const auto map_meta1_pub = nh.advertise<nav_msgs::OccupancyGrid>("tb3_1/map", 100);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
      nav_msgs::OccupancyGrid new_map;
      new_map.header.frame_id = "new_map";
      new_map.info.resolution = 0.05;
      new_map.info.origin.position.x =  -10.0;
      new_map.info.origin.position.y = -10.0;
      new_map.info.origin.position.z = 0.0;
      new_map.info.origin.orientation.w = 0.0;

      // const size_t width_ = 384;
      const size_t height_ = 384;
      const size_t bottom_width_ = 50; // Space between the bottom of the big map and the local slam map
      new_map.info.width = bottom_width_ + ugh.info.width;
      new_map.info.height = height_;

      // Map starts ing in from origin in bottom right
      // From the origin, the row components corresponds with width (x-dir which is actually up)
      // Create empty space on rhs of map
      for (int i=0;  i < new_map.info.width * 122; i++)
      {
        new_map.data.push_back(-1);
      }

      // // std::cout << "Current map size should be " << bottom_width_ * height_ << " and is actually " << new_map.data.size() << std::endl;

      int curr_counter = 0;
      int c1 = 0;
      int c2 = 0;
      auto curr_map_height = ugh.info.height - 1;

      // std::cout << "start all the logics " << std::endl;

      for (int item_counter=0; item_counter < ugh.info.height; item_counter++)
      {
        // std::cout << "Top of the inital for loop" << std::endl;
        for (int q=0; q < bottom_width_; q++)
        {
          new_map.data.push_back(-1);
          c1++;
          // std::cout << "c1 should get to 277ish " << c1 << std::endl;
        }

        for (int a = 0; a < ugh.info.width; a++)
        {
          new_map.data.push_back(ugh.data[curr_counter]);
          curr_counter++;
          // std::cout << "Current counter is " << curr_counter << std::endl;
        }
      } 

      for (int z=0;  z < ((height_ - ugh.info.height - 122) * new_map.info.width); z++)
      {
        new_map.data.push_back(-1);
      }


      new_map_pub.publish(new_map);

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
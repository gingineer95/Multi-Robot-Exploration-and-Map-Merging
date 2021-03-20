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
// nav_msgs::OccupancyGrid map0_msg, map1_msg, slam0_map, slam1_map;
nav_msgs::OccupancyGrid slam0_map, slam1_map;


/// / \brief Grabs the position of the robot from the pose subscriber and stores it
/// / \param msg - pose message
/// \returns nothing
void map0Callback(const nav_msgs::OccupancyGrid & msg)
{
  // map0_msg.header = msg->header;
  // map0_msg.info = msg->info;
  // map0_msg.data = msg->data;
  slam0_map = msg;
  slam0_map.header.frame_id = "new_tb3_0_map";
}

/// / \brief Grabs the position of the robot from the pose subscriber and stores it
/// / \param msg - pose message
/// \returns nothing
void map1Callback(const nav_msgs::OccupancyGrid & msg)
{
  // map1_msg.header = msg->header;
  // map1_msg.info = msg->info;
  // map1_msg.data = msg->data;
  slam1_map = msg;
  slam1_map.header.frame_id = "new_tb3_1_map";
}


int main(int argc, char * argv[])
{
  ros::init(argc, argv, "map_metadata_node");
  ros::NodeHandle nh;

  // Create the initpose publisher and subscriber
  const auto slam0_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("new_tb3_0_map", 100);
  // const auto map_meta0_sub = nh.subscribe<nav_msgs::OccupancyGrid>("tb3_0/map", 100, map0Callback);
  const auto map_meta0_sub = nh.subscribe("tb3_0/map", 100, map0Callback);

  const auto slam1_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("new_tb3_1_map", 100);
  // const auto map_meta1_sub = nh.subscribe<nav_msgs::OccupancyGrid>("tb3_1/map", 100, map1Callback);
  const auto map_meta1_sub = nh.subscribe("tb3_1/map", 100, map1Callback);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
      // nav_msgs::OccupancyGrid new_tb3_0_map, new_tb3_1_map;
      // new_tb3_0_map.header.frame_id = "new_tb3_0_map";
      // new_tb3_0_map.info.resolution = 0.05;
      // new_tb3_0_map.info.origin.position.x =  -10.0;
      // new_tb3_0_map.info.origin.position.y = -10.0;
      // new_tb3_0_map.info.origin.position.z = 0.0;
      // new_tb3_0_map.info.origin.orientation.w = 0.0;

      // new_tb3_1_map.header.frame_id = "new_tb3_1_map";
      // new_tb3_1_map.info.resolution = 0.05;
      // new_tb3_1_map.info.origin.position.x =  new_tb3_0_map.info.origin.position.x;
      // new_tb3_1_map.info.origin.position.y = new_tb3_0_map.info.origin.position.y;
      // new_tb3_1_map.info.origin.position.z = new_tb3_0_map.info.origin.position.z;
      // new_tb3_1_map.info.origin.orientation.w = new_tb3_0_map.info.origin.orientation.w;

      const size_t new_width_ = 384;
      const size_t new_height_ = 384;
      const size_t bottom0_width_ = 51; // Space between the bottom of the big map and the local slam0 map
      const size_t bottom1_width_ = 299; // Space between the bottom of the big map and the local slam1 map
      slam0_map.info.width = new_width_;
      slam0_map.info.height = new_height_;
      slam1_map.info.width = slam0_map.info.width;
      slam1_map.info.height = slam0_map.info.height;

      /////////////////////////////////////////////////////////////////////////////
      // For the frist turtlebot (tb3_0)
      ////////////////////////////////////////////////////////////////////////////
      // Map starts loading in from origin in bottom right
      // From the origin, the row components corresponds with width (x-dir which is actually up)
      // Create empty space on rhs of map (total width and 122 pixels 'high' (to the left))

      int c0 = 0; // start a counter for map0

      for (int i=0;  i < slam0_map.info.width * 100; i++) // orginally 123
      {
        slam0_map.data.push_back(-1);
      }

      for (int item_counter=0; item_counter < slam0_map.info.height; item_counter++)
      {

        for (int q=0; q < bottom0_width_; q++)
        {
          slam0_map.data.push_back(-1);
        }

        for (int a = 0; a < slam0_map.info.width; a++)
        {
          slam0_map.data.push_back(slam0_map.data[c0]);
          c0++;
        }

        for (int u=0; u < (slam0_map.info.width - slam0_map.info.width - bottom0_width_); u++)
        {
          slam0_map.data.push_back(-1);
        }
      } 

      for (int z=0;  z < ((new_height_ - slam0_map.info.height - 100) * slam0_map.info.width); z++)
      {
        slam0_map.data.push_back(-1);
      }

      /////////////////////////////////////////////////////////////////////////////
      // For the second turtlebot (tb3_1)
      ////////////////////////////////////////////////////////////////////////////
      // Map starts loading in from origin in bottom right
      // From the origin, the row components corresponds with width (x-dir which is actually up)
      // Create empty space on rhs of map (total width and 122 pixels 'high' (to the left))

      int c1 = 0; // start a counter for map 2

      // Fill in the empty space on the right hand side of the map
      for (int i=0;  i < slam1_map.info.width * 113; i++)
      {
        slam1_map.data.push_back(-1);
      }

      // Fill in the area where the map1 is, also the area above and below that
      for (int item_counter=0; item_counter < slam1_map.info.height; item_counter++)
      {
        // Space below the maps width
        for (int q=0; q < bottom1_width_; q++)
        {
          slam1_map.data.push_back(-1);
        }

        // Fill in the map data for the current width row
        for (int a = 0; a < slam1_map.info.width; a++)
        {
          slam1_map.data.push_back(slam1_map.data[c1]);
          c1++;
        }

        // Fill in the space above the map
        for (int u=0; u < (slam1_map.info.width - slam1_map.info.width - bottom1_width_); u++)
        {
          slam1_map.data.push_back(-1);
        }
      } 

      // Fill in the area on the left hand side of the map
      for (int z=0;  z < ((new_height_ - slam1_map.info.height - 113) * slam1_map.info.width); z++)
      {
        slam1_map.data.push_back(-1);
      }

      slam0_map_pub.publish(slam0_map);
      slam1_map_pub.publish(slam1_map);

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
/// \file
/// \brief This node reads in the robot's local SLAM map and creates a larger map to be used for the map_merge node
///
/// PUBLISHES:
///     /new_tb3_0_map (nav_msgs/OccupancyGrid): Publishes an expanded map with new width, height and origin
///     /new_tb3_1_map (nav_msgs/OccupancyGrid): Publishes an expanded map new width, height and origin
/// SUBSCRIBES:
///     /tb3_0/map (nav_msgs/OccupancyGrid): Reads the map created by SLAM
///     /tb3_1/map (nav_msgs/OccupancyGrid): Reads the map created by SLAM

#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <std_msgs/Header.h>
#include <nav_msgs/MapMetaData.h>

// Define global message 
nav_msgs::OccupancyGrid slam0_map, slam1_map;

/// \brief Reads the map data published from slam_toolbox
/// \param msg - map message
/// \returns nothing
void map0Callback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
{
  slam0_map.header = msg->header;
  slam0_map.info = msg->info;
  slam0_map.data = msg->data;
  std::cout << "Got to map0 callback" << std::endl;
}

/// \brief Reads the map data published from slam_toolbox
/// \param msg - map message
/// \returns nothing
void map1Callback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
{
  slam1_map.header = msg->header;
  slam1_map.info = msg->info;
  slam1_map.data = msg->data;
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "map_expansion_node");
  ros::NodeHandle nh;

  // Create the publisher and subscriber
  const auto new_tb3_0_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("new_tb3_0_map", 100);
  const auto map_meta0_sub = nh.subscribe<nav_msgs::OccupancyGrid>("tb3_0/map", 100, map0Callback);

  const auto new_tb3_1_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("new_tb3_1_map", 100);
  const auto map_meta1_sub = nh.subscribe<nav_msgs::OccupancyGrid>("tb3_1/map", 100, map1Callback);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    if ( (slam0_map.data.size() != 0) || (slam1_map.data.size() != 0) || slam0_map.info.origin.position.x !=0 || slam1_map.info.origin.position.x !=0)
    {
      // Create the new larger maps for each robot
      nav_msgs::OccupancyGrid new_tb3_0_map, new_tb3_1_map;
      new_tb3_0_map.header.frame_id = "new_tb3_0_map";
      new_tb3_0_map.info.resolution = 0.05;
      new_tb3_0_map.info.origin.position.x =  -10.0;
      new_tb3_0_map.info.origin.position.y = -10.0;
      new_tb3_0_map.info.origin.position.z = 0.0;
      new_tb3_0_map.info.origin.orientation.w = 0.0;

      new_tb3_1_map.header.frame_id = "new_tb3_1_map";
      new_tb3_1_map.info.resolution = 0.05;
      new_tb3_1_map.info.origin.position.x =  new_tb3_0_map.info.origin.position.x;
      new_tb3_1_map.info.origin.position.y = new_tb3_0_map.info.origin.position.y;
      new_tb3_1_map.info.origin.position.z = new_tb3_0_map.info.origin.position.z;
      new_tb3_1_map.info.origin.orientation.w = new_tb3_0_map.info.origin.orientation.w;

      // Set the new map width and heights 
      const size_t width_ = 384;
      const size_t height_ = 384;
      new_tb3_0_map.info.width = width_;
      new_tb3_0_map.info.height = height_;
      new_tb3_1_map.info.width = width_;
      new_tb3_1_map.info.height = height_;

      // Determine how much space to fill in with unknown cells bewteen bottom of the new sized map and the original slam map
      const size_t bottom0_width_ = (slam0_map.info.origin.position.x - new_tb3_0_map.info.origin.position.x) / new_tb3_0_map.info.resolution;
      const size_t bottom1_width_ = (slam1_map.info.origin.position.x - new_tb3_1_map.info.origin.position.x) / new_tb3_1_map.info.resolution;

      const size_t bottom0_height_ = (slam0_map.info.origin.position.y - new_tb3_0_map.info.origin.position.y) / new_tb3_0_map.info.resolution;
      const size_t bottom1_height_ = (slam1_map.info.origin.position.y - new_tb3_1_map.info.origin.position.y) / new_tb3_1_map.info.resolution;

      /////////////////////////////////////////////////////////////////////////////
      // For the frist turtlebot (tb3_0)
      ////////////////////////////////////////////////////////////////////////////
      // Map starts loading in from origin, which is the bottom right corner (for the deault orientation in rviz)
      // From the origin, the row components corresponds with width (+ x-dir which is up and + y-dir is to the left)
      // Fill in the all new cells in the new map with unknowns (-1)

      int c0 = 0; // start a counter for map0

      // Fill in the space between start of the new map to the start of the local SLAM map with -1s
      // (for the default orientation in Rviz, this is the space to the right of the SLAM map)
      for (int i=0;  i < new_tb3_0_map.info.width * bottom0_height_; i++)
      {
        new_tb3_0_map.data.push_back(-1);
      }

      // Fill in the spaces on either side of the local SLAM map with -1s, but dont replace the current values from the local SLAM map
      for (int item_counter=0; item_counter < slam0_map.info.height; item_counter++)
      {
        // For all new cells between the new starting width and the original SLAM starting width, fill with -1s
        // (for the default orientation in Rviz, this is the space below the SLAM map)
        for (int q=0; q < bottom0_width_; q++)
        {
          new_tb3_0_map.data.push_back(-1);
        }

        // Fill in the current SLAM map information, in its initial location
        for (int a = 0; a < slam0_map.info.width; a++)
        {
          new_tb3_0_map.data.push_back(slam0_map.data[c0]);
          c0++;
        }

        // For all new cells between the new ending width and the original SLAM end width, fill with -1s
        // (for the default orientation in Rviz, this is the space above the SLAM map)
        for (int u=0; u < (new_tb3_0_map.info.width - slam0_map.info.width - bottom0_width_); u++)
        {
          new_tb3_0_map.data.push_back(-1);
        }
      } 

      // Fill in the space between the end of the original SLAM map to the end of the new map with -1s
      // (for the default orientation in Rviz, this is the space to the left of the SLAM map)
      for (int z=0;  z < ((height_ - slam0_map.info.height - bottom0_height_) * new_tb3_0_map.info.width); z++)
      {
        new_tb3_0_map.data.push_back(-1);
      }

      /////////////////////////////////////////////////////////////////////////////
      // For the second turtlebot (tb3_1)
      ////////////////////////////////////////////////////////////////////////////
      // Map starts loading in from origin, which is the bottom right corner (for the deault orientation in rviz)
      // From the origin, the row components corresponds with width (+ x-dir which is up and + y-dir is to the left)
      // Fill in the all new cells in the new map with unknowns (-1)

      int c1 = 0; // start a counter for map 2

      // Fill in the space between start of the new map to the start of the local SLAM map with -1s
      // (for the default orientation in Rviz, this is the space to the right of the SLAM map)
      for (int i=0;  i < new_tb3_1_map.info.width *  bottom1_height_; i++)
      {
        new_tb3_1_map.data.push_back(-1);
      }

      // Fill in the spaces on either side of the local SLAM map with -1s, but dont replace the current values from the local SLAM map
      for (int item_counter=0; item_counter < slam1_map.info.height; item_counter++)
      {
        // For all new cells between the new starting width and the original SLAM starting width, fill with -1s
        // (for the default orientation in Rviz, this is the space below the SLAM map)
        for (int q=0; q < bottom1_width_; q++)
        {
          new_tb3_1_map.data.push_back(-1);
        }

        // Fill in the current SLAM map information, in its initial location
        for (int a = 0; a < slam1_map.info.width; a++)
        {
          new_tb3_1_map.data.push_back(slam1_map.data[c1]);
          c1++;
        }

        // For all new cells between the new ending width and the original SLAM end width, fill with -1s
        // (for the default orientation in Rviz, this is the space above the SLAM map)
        for (int u=0; u < (new_tb3_1_map.info.width - slam1_map.info.width - bottom1_width_); u++)
        {
          new_tb3_1_map.data.push_back(-1);
        }
      } 

      // Fill in the space between the end of the original SLAM map to the end of the new map with -1s
      // (for the default orientation in Rviz, this is the space to the left of the SLAM map)
      for (int z=0;  z < ((height_ - slam1_map.info.height -  bottom1_height_) * new_tb3_1_map.info.width); z++)
      {
        new_tb3_1_map.data.push_back(-1);
      }

      // Publish the new maps
      new_tb3_0_map_pub.publish(new_tb3_0_map);
      new_tb3_1_map_pub.publish(new_tb3_1_map);
    }

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
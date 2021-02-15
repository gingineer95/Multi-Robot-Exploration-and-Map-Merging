/// \file
/// \brief This node causes the robot to move autonomously via Frontier Exploration

#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include "std_msgs/Header.h"
#include <iostream>
#include <cmath> 
// #include "nav_msgs/MapMetaData.h"

// define global message 
nav_msgs::OccupancyGrid map_0_msg;
nav_msgs::OccupancyGrid FE_tb3_0_map, FE_tb3_1_map;

/// / \brief Grabs the position of the robot from the pose subscriber and stores it
/// / \param msg - pose message
/// \returns nothing
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
{
  map_0_msg.header = msg->header;
  map_0_msg.info = msg->info;
  map_0_msg.data = msg->data;
}


int main(int argc, char * argv[])
{
  ros::init(argc, argv, "frontier_exploration_node");
  ros::NodeHandle nh;

  // Create the initpose publisher and subscriber
  const auto tb3_0_FE_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("ugh/map", 100);
//   const auto tb3_0_FE_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("ughhhh", 100);
  const auto map_0_sub = nh.subscribe<nav_msgs::OccupancyGrid>("tb3_0/map", 100, mapCallback);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
        FE_tb3_0_map.header.frame_id = "FE_tb3_0_map";
        FE_tb3_0_map.info.resolution = map_0_msg.info.resolution;
        FE_tb3_0_map.info.width = map_0_msg.info.width;
        FE_tb3_0_map.info.height = map_0_msg.info.height;
        FE_tb3_0_map.info.origin.position.x = map_0_msg.info.origin.position.x;
        FE_tb3_0_map.info.origin.position.y = map_0_msg.info.origin.position.y ;
        FE_tb3_0_map.info.origin.position.z = map_0_msg.info.origin.position.z;
        FE_tb3_0_map.info.origin.orientation.w = map_0_msg.info.origin.orientation.w;
        FE_tb3_0_map.data = map_0_msg.data;

        int below_before, below, below_after, before, after, top_before, top, top_after;

        // For all the points in the occupancy grid
        // Starting one row up and on space in so that we dont have an issues with the first cell
        for (double x = FE_tb3_0_map.info.width + 1; x < (FE_tb3_0_map.info.width * FE_tb3_0_map.info.height) - FE_tb3_0_map.info.width; x++)
        {
            if (FE_tb3_0_map.data[x] == -1)
            {
                // Store frontier edges
                below_before = FE_tb3_0_map.data[x - FE_tb3_0_map.info.width - 1];
                below = FE_tb3_0_map.data[x - FE_tb3_0_map.info.width];
                below_after = FE_tb3_0_map.data[x - FE_tb3_0_map.info.width + 1];
                before = FE_tb3_0_map.data[x-1];
                after = FE_tb3_0_map.data[x+1];
                top_before = FE_tb3_0_map.data[x + FE_tb3_0_map.info.width - 1];
                top = FE_tb3_0_map.data[x + FE_tb3_0_map.info.width];
                top_after = FE_tb3_0_map.data[x + FE_tb3_0_map.info.width + 1];

                int f_edge[] = {below_before, below, below_after, before, after, top_before, top, top_after};
                int len = sizeof(f_edge)/sizeof(f_edge[0]);

                for(int i = 0; i < len; i++)
                {
                    if (f_edge[i] == 0)
                    {
                        std::cout << "Frontier edge is at index " << i << " of the array" << std::endl;

                        //Okay so I can identify where my edges are from the f_edge array, thats goot
                        // NEXT STEPS: Translate that back to the occupancy grid
                        // Maybe wanna do an occupancy grid instead of an array, cause I need to know the 
                        // Occupancy grid space to write over
                    }
                }

            }
        }  

        tb3_0_FE_map_pub.publish(FE_tb3_0_map);
        // ROS_INFO("Total Number of gray cells is: %e", Ngray);
        // std::cout<< "Counter total is " << counter << std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }

  return 0;
}
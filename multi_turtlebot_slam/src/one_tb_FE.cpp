/// \file
/// \brief This node causes the robot to move autonomously via Frontier Exploration

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <iostream>
#include <cmath> 

// define global message 
nav_msgs::OccupancyGrid map_0_msg;
nav_msgs::OccupancyGrid FE_tb3_0_map, region_map;

/// / \brief Grabs the position of the robot from the pose subscriber and stores it
/// / \param msg - pose message
/// \returns nothing
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
{
  map_0_msg.header = msg->header;
  map_0_msg.info = msg->info;
  map_0_msg.data = msg->data;

  std::cout << "map origin is " << map_0_msg.info.origin.position.x << " , " << map_0_msg.info.origin.position.y << std::endl;
}


int main(int argc, char * argv[])
{
  ros::init(argc, argv, "one_tb_FE_node");
  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::Pose robot_pose_;
  std::string map_frame = "map";
  std::string body_frame = "base_footprint";

    // Create the initpose publisher and subscriber
  const auto tb3_0_FE_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("edge/map", 1);
  const auto region_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("region/map", 1);
  const auto map_0_sub = nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, mapCallback);

  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  MoveBaseClient ac("move_base", true);

  // Wait 60 seconds for the action server to become available
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  ROS_INFO("Connected to move base server");

  int goal_c = 0;

//   boost::shared_ptr<nav_msgs::OccupancyGrid const> got_msg;
//   got_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("tb3_0/map", ros::Duration(10));
//   if(got_msg != NULL)
//   {
//     ROS_INFO("No map messages recieved yet");
//   }

  ros::Rate loop_rate(40);
  
  while (ros::ok())
  {
        FE_tb3_0_map.header.frame_id = "FE_map";
        FE_tb3_0_map.info.resolution = map_0_msg.info.resolution;
        FE_tb3_0_map.info.width = map_0_msg.info.width;
        FE_tb3_0_map.info.height = map_0_msg.info.height;
        FE_tb3_0_map.info.origin.position.x = map_0_msg.info.origin.position.x;
        FE_tb3_0_map.info.origin.position.y = map_0_msg.info.origin.position.y ;
        FE_tb3_0_map.info.origin.position.z = map_0_msg.info.origin.position.z;
        FE_tb3_0_map.info.origin.orientation.w = map_0_msg.info.origin.orientation.w;
        FE_tb3_0_map.data = map_0_msg.data;

        region_map.header.frame_id = "region_map";
        region_map.info.resolution = map_0_msg.info.resolution;
        region_map.info.width = map_0_msg.info.width;
        region_map.info.height = map_0_msg.info.height;
        region_map.info.origin.position.x = map_0_msg.info.origin.position.x;
        region_map.info.origin.position.y = map_0_msg.info.origin.position.y ;
        region_map.info.origin.position.z = map_0_msg.info.origin.position.z;
        region_map.info.origin.orientation.w = map_0_msg.info.origin.orientation.w;
        region_map.data = map_0_msg.data;

        int below_before, below, below_after, before, after, top_before, top, top_after;
        int below_before_i, below_i, below_after_i, before_i, after_i, top_before_i, top_i, top_after_i;
        int c = 0;
        int all_edges[FE_tb3_0_map.info.width * FE_tb3_0_map.info.height] = {};

        // For all the points in the occupancy grid
        // Starting one row up and on space in so that we dont have an issues with the first cell
        for (double x = FE_tb3_0_map.info.width + 1; x < (FE_tb3_0_map.info.width * FE_tb3_0_map.info.height) - FE_tb3_0_map.info.width; x++)
        {
            if (FE_tb3_0_map.data[x] == -1)
            {
                // Store frontier edges
                below_before_i = x - FE_tb3_0_map.info.width - 1;
                below_i = x - FE_tb3_0_map.info.width;
                below_after_i = x - FE_tb3_0_map.info.width + 1;
                before_i = x-1;
                after_i = x+1;
                top_before_i = x + FE_tb3_0_map.info.width - 1;
                top_i = x + FE_tb3_0_map.info.width;
                top_after_i = x + FE_tb3_0_map.info.width + 1;

                below_before = FE_tb3_0_map.data[below_before_i];
                below = FE_tb3_0_map.data[below_i];
                below_after = FE_tb3_0_map.data[below_after_i];
                before = FE_tb3_0_map.data[before_i];
                after = FE_tb3_0_map.data[after_i];
                top_before = FE_tb3_0_map.data[top_before_i];
                top = FE_tb3_0_map.data[top_i];
                top_after = FE_tb3_0_map.data[top_after_i];

                int f_edge[] = {below_before, below, below_after, before, after, top_before, top, top_after};
                int f_edge_index[] = {below_before_i, below_i, below_after_i, before_i, after_i, top_before_i, top_i, top_after_i};
                int len = sizeof(f_edge)/sizeof(f_edge[0]);

                for(int i = 0; i < len; i++)
                {
                    if (f_edge[i] == 0)
                    {
                        // If were actually at an edge, mark it
                        int mark_edge = f_edge_index[i];
                        FE_tb3_0_map.data[mark_edge] = 10;
                        all_edges[c] = mark_edge;

                        // Should really do this with vector, but I'm having issues
                        // myvector.push_back(mark_edge);

                        c++;

                    }
                }

            }
        } 

        tb3_0_FE_map_pub.publish(FE_tb3_0_map);

        int region[c-1] = {};
        int temp_group[c-1] = {};
        // int total_regions[10] = {};
        int centroids[20] = {};
        int j = 0;
        int group_c = 0;
        int region_c = 0;

        for (int q=0; q < c-1; q++)
        {
            // std::cout << "Currently evaluating at location " << all_edges[q] << std::endl;
            // int prev_i = q-1;
            int next_i = q+1;

            // int prev_value = all_edges[q] - all_edges[prev_i];
            int next_value = all_edges[next_i] - all_edges[q];

            if ((next_value == 1) || (next_value == FE_tb3_0_map.info.width) || (next_value == FE_tb3_0_map.info.width -1) || (next_value == FE_tb3_0_map.info.width + 1))
            {
                // Add to a region array
                region[j] = all_edges[q];
                region_map.data[all_edges[q]] = 70;
                temp_group[group_c] = all_edges[q];
                j++;
                group_c++;
                // std::cout << "ELEMENTS IN THIS GROUP ARE AT " << group_c << std::endl;
            }

            else
            {
                if (group_c < 5) // frontier region too small
                {
                }

                else
                {
                    // std::cout << "Size of group is " << group_c << std::endl;
                    int centroid = group_c / 2;
                    // std::cout << "Centroid value is " << centroid << std::endl;
                    // std::cout << "centroid location is " << temp_group[centroid] << std::endl;
                    centroids[region_c] = temp_group[centroid];
                    region_c++;
                    // std::cout << "Number of regions is now " << region_c << std::endl;
                }

                // Reset group array and counter
                group_c = 0;
                int temp_group[c-1] = {};
            }
        }

        // Reset 
        c = 0;
        all_edges[FE_tb3_0_map.info.width * FE_tb3_0_map.info.height] = {};

        region_map_pub.publish(region_map);

        // Find current location and move to the nearest centroid
        //Get robot pose
        geometry_msgs::TransformStamped transformS;
        transformS = tfBuffer.lookupTransform(body_frame, map_frame, ros::Time(0));
        transformS.header.stamp = ros::Time();
        transformS.header.frame_id = map_frame;
        transformS.child_frame_id = body_frame;

        robot_pose_.position.x = transformS.transform.translation.x;
        robot_pose_.position.y = transformS.transform.translation.y;
        // robot_pose_.position.x = -7.0;
        // robot_pose_.position.y = -1.0;
        robot_pose_.position.z = 0.0;
        robot_pose_.orientation.x = transformS.transform.rotation.x;
        robot_pose_.orientation.y = transformS.transform.rotation.y;
        robot_pose_.orientation.z = transformS.transform.rotation.z;
        robot_pose_.orientation.w = transformS.transform.rotation.w;

        std::cout << "Robot pose is " << robot_pose_.position.x << " , " << robot_pose_.position.y << std::endl;

        // Convert centroids to points
        int centroid_Xpts[region_c]={};
        int centroid_Ypts[region_c]={};
        int x_int, y_int;
        double dist_arr[region_c]={};
        for (int t = 0; t < region_c; t++)
        {
            geometry_msgs::Point point;
            double dist;
            point.x = (centroids[t] % FE_tb3_0_map.info.width)*FE_tb3_0_map.info.resolution + FE_tb3_0_map.info.origin.position.x;
            point.y = floor(centroids[t]/FE_tb3_0_map.info.width)*FE_tb3_0_map.info.resolution + FE_tb3_0_map.info.origin.position.y;
            // std::cout << "Centroid point is (double) " << point.x << " , " << point.y << std::endl;
            int x_int = (int) point.x;
            int y_int = (int) point.y;
            std::cout << "Centroid point is (int) " << x_int << " , " << y_int << std::endl;
            centroid_Xpts[t] = x_int;
            centroid_Ypts[t] = y_int;

            dist = pow(((pow(point.x - robot_pose_.position.x,2)) + (pow(point.y - robot_pose_.position.y,2))) , 0.5);
            // std::cout << "Distances are " << dist << std::endl;
            dist_arr[t] = dist;
        }

        //Find smallest distance
        double smallest = dist_arr[0];
        int move_to_pt;
        for(int u = 0; u < region_c-1 ; u++)
        {
            // std::cout << "Current distance value is " << dist_arr[u] << std::endl;
            if (dist_arr[u] < smallest)
            {
                // std::cout << "Smallest was " << smallest << std::endl;
                smallest = dist_arr[u];
                // std::cout << "Now smallest is " << smallest << std::endl;
                move_to_pt = u;
                // std::cout << "Index we need to move to is ... " << move_to_pt << std::endl;
            }
        }

        // Move to goal
        move_base_msgs::MoveBaseGoal goal_init;
        goal_init.target_pose.header.frame_id = "map"; // Needs to be AN ACTUAL FRAME
        goal_init.target_pose.header.stamp = ros::Time();

        goal_init.target_pose.pose.position.x = robot_pose_.position.x - 1;
        goal_init.target_pose.pose.position.y = robot_pose_.position.y - 1;
        // goal_init.target_pose.pose.position.x = -0.515;
        // goal_init.target_pose.pose.position.y = 0.370;
        goal_init.target_pose.pose.orientation.w = 1.0;

        // std::cout << "Goal is " << goal.target_pose.pose.position.x << " , " <<  goal.target_pose.pose.position.y << std::endl;

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map"; // Needs to be AN ACTUAL FRAME
        goal.target_pose.header.stamp = ros::Time();
        goal.target_pose.pose.position.x = centroid_Xpts[move_to_pt];
        goal.target_pose.pose.position.y = centroid_Ypts[move_to_pt];
        goal.target_pose.pose.orientation.w = 1.0;

        if(goal_c == 0)
        {
            std::cout << "Initial goal is " << goal_init.target_pose.pose.position.x << " , " <<  goal_init.target_pose.pose.position.y << std::endl;
            ROS_INFO("Sending inital goal");
            ac.sendGoal(goal);
            std::cout << "Sent Initial Goal" <<std::endl;
            // Wait for the action to return
            ac.waitForResult();
            std::cout << "Waiting for result" <<std::endl;
            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("You have reached the inital goal!");
            }
            else
            {
                ROS_INFO("The base failed for some reason");
            }
            goal_c++;
        }

        else
        {
            std::cout << "Goal is " << goal.target_pose.pose.position.x << " , " <<  goal.target_pose.pose.position.y << std::endl;
            ROS_INFO("Sending goal");
            ac.sendGoal(goal);
            std::cout << "Sent Goal" <<std::endl;
            // Wait for the action to return
            ac.waitForResult();
            std::cout << "Waiting for result" <<std::endl;
            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("You have reached the goal!");
            }
            else
            {
                ROS_INFO("The base failed for some reason");
            }
        }

        // if ((goal.target_pose.pose.position.x == 0) && (goal.target_pose.pose.position.y == 0))
        // {
        // }

        // else
        // {
        //     std::cout << "Goal is " << goal.target_pose.pose.position.x << " , " <<  goal.target_pose.pose.position.y << std::endl;
        //     ROS_INFO("Sending goal");
        //     ac.sendGoal(goal);
        //     std::cout << "Sent Goal" <<std::endl;
        //     // Wait for the action to return
        //     ac.waitForResult();
        //     std::cout << "Waiting for result" <<std::endl;
        //     if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        //     {
        //         ROS_INFO("You have reached the goal!");
        //     }
        //     else
        //     {
        //         ROS_INFO("The base failed for some reason");
        //     }
        // }

        ros::spinOnce();
        loop_rate.sleep();
    }

  return 0;
}
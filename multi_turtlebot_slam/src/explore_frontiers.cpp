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


class FrontExpl
{
    public:
        FrontExpl()
        {
            // Create the initpose publisher and subscriber
            tb3_0_FE_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("edges_map_0", 10);
            region_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("region_map_0", 10);
            map_0_sub = nh.subscribe<nav_msgs::OccupancyGrid>("map", 10, &FrontExpl::mapCallback, this);
            tf2_ros::TransformListener tfListener(tfBuffer);
            MoveBaseClient ac("move_base", true);

            std::cout << "Initialized all the things" << std::endl;
        }

        /// / \brief Grabs the position of the robot from the pose subscriber and stores it
        /// / \param msg - pose message
        /// \returns nothing
        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
        {
            map_0_msg.header = msg->header;
            map_0_msg.info = msg->info;
            map_0_msg.data = msg->data;

            std::cout << "Got to map callback" << std::endl;
        }

        void neighborhood(int cell)
        {
            // std::vector<int> neighbor_index;
            neighbor_index.resize(0);
            neighbor_value.resize(0);

            // Store neighboring values and indexes
            neighbor_index.push_back(cell - map_width - 1);
            neighbor_index.push_back(cell - map_width);
            neighbor_index.push_back(cell - map_width + 1);
            neighbor_index.push_back(cell-1);
            neighbor_index.push_back(cell+1);
            neighbor_index.push_back(cell + map_width - 1);
            neighbor_index.push_back(cell + map_width);
            neighbor_index.push_back(cell + map_width + 1);
        }

        // int point2cell(const geometry_msgs::Point & point)
        // {
        //     int x_cell = floor0((point.x - map_.info.origin.position.x)/map_.info.resolution);
        //     int y_cell = floor0((point.y - map_.info.origin.position.y)/map_.info.resolution);

        //     return(x_cell + (y_cell)*map_.info.width);
        // }

        // geometry_msgs::Point cell2point(const int & cell)
        // {
        //     geometry_msgs::Point point;
        //     point.x = (cell % map_.info.width)*map_.info.resolution + map_.info.origin.position.x;
        //     point.y = floor(cell/map_.info.width)*map_.info.resolution + map_.info.origin.position.y;
        //     return point;
        // }

        void find_all_edges()
        {
            // For all unknown points in the occupancy grid, find the value of the neighbors
            // Starting one row up and on space in so that we dont have an issues with the first point
                for (double x = map_width + 1; x < (map_width * map_height) - map_width; x++)
                {
                    if (map_data[x] == -1)
                    {
                        // If there is an unknown cell then check neighboring cells to find potential frontier edge (free cell)
                        neighborhood(x);

                        for(int i = 0; i < neighbor_index.size(); i++)
                        {
                            if (map_data[neighbor_index[i]] == 0)
                            {
                                // If were actually at an edge, mark it
                                int mark_edge = neighbor_index[i];
                                FE_tb3_0_map.data[mark_edge] = 10;
                                edge_vec.push_back(mark_edge);
                                num_edges++;

                            }
                        }

                    }
                } 

        }


        void main_loop()
        {
            typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
            MoveBaseClient ac("move_base", true);

            //wait for the action server to come up
            while(!ac.waitForServer(ros::Duration(5.0)))
            {
                ROS_INFO("Waiting for the move_base action server to come up");
            }
            ROS_INFO("Connected to move base server");

            tf2_ros::TransformListener tfListener(tfBuffer);
            ros::Rate loop_rate(1);

            while (ros::ok())
            {
                ros::spinOnce();

                // Map definitions for the edges and region maps
                FE_tb3_0_map.header.frame_id = "edges_map_0";
                FE_tb3_0_map.info.resolution = map_0_msg.info.resolution;
                FE_tb3_0_map.info.width = map_0_msg.info.width;
                FE_tb3_0_map.info.height = map_0_msg.info.height;
                FE_tb3_0_map.info.origin.position.x = map_0_msg.info.origin.position.x;
                FE_tb3_0_map.info.origin.position.y = map_0_msg.info.origin.position.y ;
                FE_tb3_0_map.info.origin.position.z = map_0_msg.info.origin.position.z;
                FE_tb3_0_map.info.origin.orientation.w = map_0_msg.info.origin.orientation.w;
                FE_tb3_0_map.data = map_0_msg.data;

                // Store the map data in a vector so we can read that shit
                int map_size = map_0_msg.data.size();
                map_data.resize(0);
                for (int q = 0; q < map_size; q++)
                {
                    map_data.push_back(map_0_msg.data[q]);
                }

                region_map.header.frame_id = "region_map_0";
                region_map.info.resolution = map_0_msg.info.resolution;
                region_map.info.width = map_0_msg.info.width;
                region_map.info.height = map_0_msg.info.height;
                region_map.info.origin.position.x = map_0_msg.info.origin.position.x;
                region_map.info.origin.position.y = map_0_msg.info.origin.position.y ;
                region_map.info.origin.position.z = map_0_msg.info.origin.position.z;
                region_map.info.origin.orientation.w = map_0_msg.info.origin.orientation.w;
                region_map.data = map_0_msg.data;

                // Start with inital number of edges is 1 and initalize vector to store edge index
                num_edges = 0;
                edge_vec.resize(0);
                map_width = FE_tb3_0_map.info.width;
                map_height = FE_tb3_0_map.info.height;

                find_all_edges();

                std::cout << "Number of edges is " << num_edges << std::endl;

                tb3_0_FE_map_pub.publish(FE_tb3_0_map);

                //////////////////////////////////////////////////////////////////////////////////////////////////////////
                // ALL GOOD AT GETTING EDGES
                // NEED TO FIX HOW WERE CREATING FRONTIERS
                //////////////////////////////////////////////////////////////////////////////////////////////////////////

                // // Evaluate the edges to find regions
                // int temp_group[num_edges-1] = {};
                // int centroids[30] = {};
                // int group_c = 0; // Counts the length of one group
                // int region_c = 0; // Counts the number of regions

                // for (int q=0; q < num_edges-1; q++)
                // {
                //     // neighborhood(all_edges[q]);
                //     // std::cout << "Currently evaluating at location " << all_edges[q] << std::endl;
                //     int next_i = q+1;

                //     int next_value = all_edges[next_i] - all_edges[q];

                //     if ((next_value == 1) || (next_value == FE_tb3_0_map.info.width) || (next_value == FE_tb3_0_map.info.width -1) || (next_value == FE_tb3_0_map.info.width + 1))
                //     {
                //         // Add to a region array
                //         region_map.data[all_edges[q]] = 110 + (20*region_c);
                //         region_map.data[all_edges[q+1]] = 110 + (20*region_c);
                //         temp_group[group_c] = all_edges[q];
                //         group_c++;
                //         // std::cout << "ELEMENTS IN THIS GROUP ARE AT " << group_c << std::endl;
                //     }

                //     else
                //     {
                //         if (group_c < 5) // frontier region too small
                //         {
                //         }
                        
                //         else
                //         {

                //             // std::cout << "Size of group is " << group_c << std::endl;
                //             int centroid = group_c / 2;
                //             // int centroid = group_c;
                //             ///////////////////////////////////////
                //             // Write something that says if we move to close to wall, skip
                //             //////////////////////////////////////
                //             region_map.data[all_edges[centroid]] = 125;
                //             centroids[region_c] = temp_group[centroid];
                //             region_c++;
                //             std::cout << "Number of regions is now " << region_c << std::endl;
                //         }

                //         // Reset group array and counter
                //         group_c = 0;
                //         int temp_group[num_edges-1] = {};
                //     }
                // }

                // region_map_pub.publish(region_map);

                // // Reset 
                // num_edges = 0;
                // all_edges[FE_tb3_0_map.info.width * FE_tb3_0_map.info.height] = {};

                // // Find current location and move to the nearest centroid
                // // Get robot pose
                // transformS = tfBuffer.lookupTransform(map_frame, body_frame, ros::Time(0), ros::Duration(3.0));

                // transformS.header.stamp = ros::Time();
                // transformS.header.frame_id = body_frame;
                // transformS.child_frame_id = map_frame;

                // robot_pose_.position.x = transformS.transform.translation.x;
                // robot_pose_.position.y = transformS.transform.translation.y;
                // robot_pose_.position.z = 0.0;
                // robot_pose_.orientation.x = transformS.transform.rotation.x;
                // robot_pose_.orientation.y = transformS.transform.rotation.y;
                // robot_pose_.orientation.z = transformS.transform.rotation.z;
                // robot_pose_.orientation.w = transformS.transform.rotation.w;

                // std::cout << "Robot pose is  " << robot_pose_.position.x << " , " << robot_pose_.position.y << std::endl;

                // // Convert centroids to points
                // double centroid_Xpts[30]={};
                // double centroid_Ypts[30]={};
                // double dist_arr[30]={};
                // point.x = -1.0;
                // point.y = -1.0;
                // int centroid_c = 0;

                // for (int t = 0; t < 30; t++)
                // {
                //     double dist;
                //     point.x = (centroids[t] % region_map.info.width)*region_map.info.resolution + region_map.info.origin.position.x;
                //     point.y = floor(centroids[t] / region_map.info.width)*region_map.info.resolution + region_map.info.origin.position.y;

                //     if((point.x == last_cent_x) || (point.y == last_cent_y) || (point.x == last_2cent_x) || (point.y == last_2cent_y))
                //     {
                //         std::cout << "Got the same centroid again, pass " << std::endl;
                //     }
                //     // But why are we getting the map origin as a centroid...?
                //     else if((point.x < map_0_msg.info.origin.position.x + 0.05) || (point.y < map_0_msg.info.origin.position.y + 0.05))
                //     {
                //         std::cout << "Why are we even near the map origin? " << std::endl;
                //     }
                //     else
                //     {
                //         // std::cout << "Centroid point is " << point.x << " , " << point.y << std::endl;
                //         centroid_Xpts[centroid_c] = point.x;
                //         centroid_Ypts[centroid_c] = point.y;
                //         dist = pow(((pow(point.x - robot_pose_.position.x,2)) + (pow(point.y - robot_pose_.position.y,2))) , 0.5);
                //         dist_arr[centroid_c] = dist;
                //         centroid_c++;
                //     }
                // }

                // //Find spot to move to that is close
                // double smallest = 9999999.0;
                // for(int u = 0; u < 30 ; u++)
                // {
                //     std::cout << "Current distance value is " << dist_arr[u] << u << std::endl;
                //     if (dist_arr[u] < 0.01)
                //     {
                //         // continue;
                //     }
                //     else if (dist_arr[u] < smallest)
                //     {
                //         smallest = dist_arr[u];
                //         std::cout << "Smallest DISTANCE value is " << smallest << std::endl;
                //         move_to_pt = u;
                //         // std::cout << "Index we need to move to is ... (should be less than 10) " << move_to_pt << std::endl;
                //     }
                //     else
                //     {
                //     }
                // }

                // std::cout << "Centroid we gonna go is " << centroid_Xpts[move_to_pt] << " , " <<  centroid_Ypts[move_to_pt] << std::endl;
                // std::cout << "At a distance of " << smallest << std::endl;
                // last_2cent_x = last_cent_x;
                // last_2cent_y = last_cent_y;
                // last_cent_x = centroid_Xpts[move_to_pt];
                // last_cent_y = centroid_Ypts[move_to_pt];

                // // STORE LAST CENTROID POINT AND SAY IF WE JUST WENT THERE, DONT GO AGAIN

                // // goal.target_pose.header.frame_id = "tb3_0/map"; // Needs to be AN ACTUAL FRAME
                // goal.target_pose.header.frame_id = "tb3_0/map"; // Needs to be AN ACTUAL FRAME
                // goal.target_pose.header.stamp = ros::Time();
                // goal.target_pose.pose.position.x = centroid_Xpts[move_to_pt];
                // goal.target_pose.pose.position.y = centroid_Ypts[move_to_pt];
                // goal.target_pose.pose.orientation.w = 1.0;

                // // std::cout << "Goal is " << goal.target_pose.pose.position.x << " , " <<  goal.target_pose.pose.position.y << std::endl;
                // ROS_INFO("Sending goal");
                // ac.sendGoal(goal);

                // // Wait for the action to return
                // ac.waitForResult(ros::Duration(60));

                // if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                // {
                //     ROS_INFO("You have reached the goal!");
                // }
                // else
                // {
                //     ROS_INFO("The base failed for some reason");
                // }


                edge_vec.resize(0);
                neighbor_index.resize(0);
                neighbor_value.resize(0);

                loop_rate.sleep();
            }
        }

    private:
        ros::NodeHandle nh;
        ros::Publisher tb3_0_FE_map_pub;
        ros::Publisher region_map_pub;
        ros::Subscriber map_0_sub;
        nav_msgs::OccupancyGrid map_0_msg;
        nav_msgs::OccupancyGrid FE_tb3_0_map, region_map;
        tf2_ros::Buffer tfBuffer;
        geometry_msgs::Pose robot_pose_;
        std::string map_frame = "map";
        std::string body_frame = "base_footprint";
        std::vector<signed int> edge_vec, neighbor_index, neighbor_value, map_data;
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
        int goal_c = 0;
        int num_edges = 0;
        move_base_msgs::MoveBaseGoal goal_init;
        move_base_msgs::MoveBaseGoal goal;
        geometry_msgs::Point point;
        int move_to_pt = 0;
        geometry_msgs::TransformStamped transformS;
        // int below_before, below, below_after, before, after, top_before, top, top_after;
        // int below_before_i, below_i, below_after_i, before_i, after_i, top_before_i, top_i, top_after_i;
        int map_width, map_height, edge_size;
        int edge_after, edge_top_before, edge_top, edge_top_after;
        int edge_after_i, edge_top_before_i, edge_top_i, edge_top_after_i;
        double last_cent_x = 0.0;
        double last_cent_y = 0.0;
        double last_2cent_x = 0.0;
        double last_2cent_y = 0.0;

};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "one_tb_FE_node");
    FrontExpl FE;
    FE.main_loop();
    ros::spin();

    return 0;
}
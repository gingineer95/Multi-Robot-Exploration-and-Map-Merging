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
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <iostream>
#include <cmath> 
#include <typeinfo>


class FrontExpl
{
    public:
        FrontExpl()
        {
            tb3_0_FE_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("edges_map_0", 10);
            region_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("region_map_0", 10);
            map_0_sub = nh.subscribe<nav_msgs::OccupancyGrid>("map", 10, &FrontExpl::mapCallback, this);
            marker_pub = nh.advertise<visualization_msgs::MarkerArray>("markers", 1, true);
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
            // neighbor_index.resize(0);
            // neighbor_value.resize(0);
            neighbor_index.clear();
            neighbor_value.clear();

            // Store neighboring values and indexes
            neighbor_index.push_back(cell - map_width - 1);
            neighbor_index.push_back(cell - map_width);
            neighbor_index.push_back(cell - map_width + 1);
            neighbor_index.push_back(cell-1);
            neighbor_index.push_back(cell+1);
            neighbor_index.push_back(cell + map_width - 1);
            neighbor_index.push_back(cell + map_width);
            neighbor_index.push_back(cell + map_width + 1);

            sort( neighbor_index.begin(), neighbor_index.end() );
            neighbor_index.erase( unique( neighbor_index.begin(), neighbor_index.end() ), neighbor_index.end() );
        }

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

                        sort( neighbor_index.begin(), neighbor_index.end() );
                        neighbor_index.erase( unique( neighbor_index.begin(), neighbor_index.end() ), neighbor_index.end() );

                        for(int i = 0; i < neighbor_index.size(); i++)
                        {
                            // Am I accidentally adding values multiple times here??
                            if (map_data[neighbor_index[i]] == 0)
                            {
                                // If were actually at an edge, mark it
                                mark_edge = neighbor_index[i];
                                FE_tb3_0_map.data[mark_edge] = 10;
                                edge_vec.push_back(mark_edge);

                                num_edges++;
                                mark_edge = 0;

                            }
                        }

                    }
                } 

        }

        // bool check_edges(int curr_cell, int next_cell)
        // {
        //     if (curr_cell == next_cell)
        //     {
        //         return false;
        //     }
        //     else
        //     {
        //         return true;
        //     }
        // }

        // void find_regions()
        // {
        //     std::vector<unsigned int> temp_group;

        //     for (int q = 0; q < num_edges; q++)
        //     {
        //         unique_flag = check_edges(edge_vec[q], edge_vec[q+1]);

        //         if (unique_flag == true)
        //         {
        //             // std::cout << "Finding regions around cell " << edge_vec[q] << std::endl;
        //             // std::cout << "Evaluating against " << edge_vec[q+1] << std::endl;

        //             neighborhood(edge_vec[q]);

        //             for(int i = 0; i < neighbor_index.size(); i++)
        //             {
        //                 // std::cout << "Neighbor index is " << neighbor_index[i] << std::endl;

        //                 if (neighbor_index[i] == edge_vec[q+1])
        //                 {
        //                     edge_index = edge_vec[q]; 
        //                     next_edge_index = edge_vec[q+1];
        //                     region_map.data[edge_index] = 110 + (20*region_c);
        //                     region_map.data[next_edge_index] = 110 + (20*region_c);
        //                     temp_group.push_back(edge_vec[q]);

        //                     std::cout << "Adding this value to temp group " << temp_group[group_c] << std::endl;

        //                     sort( temp_group.begin(), temp_group.end() );
        //                     temp_group .erase( unique( temp_group.begin(), temp_group.end() ), temp_group.end() );

        //                     group_c++;
        //                 }
        //             }

        //             if (group_c == prev_group_c) // then we didnt add any edges to our region
        //             {
        //                 if (group_c < 7) // frontier region too small
        //                 {
        //                 }
                        
        //                 else
        //                 {
        //                     std::cout << "Size of group is " << group_c << std::endl;

        //                     ///////////////////////////////////////
        //                     // Write something that says if we move to close to wall, skip
        //                     //////////////////////////////////////

        //                     centroid = group_c / 2;
        //                     // centroid_index = temp_group[centroid];
        //                     // centroid = 0;
        //                     centroid_index = temp_group[centroid];

        //                     // std::cout << "Centroid is number " << centroid << " out of " << group_c << std::endl;
        //                     // std::cout << "Centroid index is " << centroid_index << std::endl;

        //                     centroids.push_back(centroid_index);
        //                     region_c++;

        //                     // std::cout << "Number of regions is now " << region_c << std::endl;
        //                 }

        //                 // std::cout << "Number of regions total is " << region_c-1 << std::endl;
        //                 // std::cout << "And number of centorids is " << centroids.size() << std::endl;

        //                 // Reset group array and counter
        //                 group_c = 0;
        //                 // std::vector<unsigned int> temp_group;
        //                 // std::cout << "Value in current temp group are " << temp_group[0] << std::endl;

        //                 // for (int g = 0; g < temp_group.size(); g++)
        //                 // {
        //                 //     std::cout << temp_group[g] << std::endl;
        //                 // } 

        //                 temp_group.clear();
        //                 // std::cout << "Make sure we clear the temp_group " << temp_group[0] <<std::endl;
        //             }
        //             else
        //             {
        //                 prev_group_c = group_c;
        //             }
        //         }

        //         else
        //         {
        //             // std::cout << "Got a duplicate cell" << std::endl;
        //         }

        //     }

        // }

        void find_transform()
        {
            // Find current location and move to the nearest centroid
            // Get robot pose
            transformS = tfBuffer.lookupTransform(map_frame, body_frame, ros::Time(0), ros::Duration(3.0));

            transformS.header.stamp = ros::Time();
            transformS.header.frame_id = body_frame;
            transformS.child_frame_id = map_frame;

            robot_pose_.position.x = transformS.transform.translation.x;
            robot_pose_.position.y = transformS.transform.translation.y;
            robot_pose_.position.z = 0.0;
            robot_pose_.orientation.x = transformS.transform.rotation.x;
            robot_pose_.orientation.y = transformS.transform.rotation.y;
            robot_pose_.orientation.z = transformS.transform.rotation.z;
            robot_pose_.orientation.w = transformS.transform.rotation.w;

            std::cout << "Robot pose is  " << robot_pose_.position.x << " , " << robot_pose_.position.y << std::endl;
        }

        // void mark_centroids()
        // {
        //     visualization_msgs::MarkerArray centroid_arr;
        //     marker_pub.publish(centroid_arr);

        //     centroid_arr.markers.resize(centroid_Xpts.size());

        //     for (int i = 0; i < centroid_Xpts.size(); i++) 
        //     {
        //         centroid_arr.markers[i].header.frame_id = "map";
        //         centroid_arr.markers[i].header.stamp = ros::Time();
        //         centroid_arr.markers[i].ns = "centroid";
        //         centroid_arr.markers[i].id = i;

        //         centroid_arr.markers[i].type = visualization_msgs::Marker::CYLINDER;
        //         centroid_arr.markers[i].action = visualization_msgs::Marker::ADD;
        //         centroid_arr.markers[i].lifetime = ros::Duration(10);

        //         centroid_arr.markers[i].pose.position.x = centroid_Xpts[i];
        //         centroid_arr.markers[i].pose.position.y = centroid_Ypts[i];
        //         centroid_arr.markers[i].pose.position.z = 0;
        //         centroid_arr.markers[i].pose.orientation.x = 0.0;
        //         centroid_arr.markers[i].pose.orientation.y = 0.0;
        //         centroid_arr.markers[i].pose.orientation.z = 0.0;
        //         centroid_arr.markers[i].pose.orientation.w = 1.0;

        //         centroid_arr.markers[i].scale.x = 0.1;
        //         centroid_arr.markers[i].scale.y = 0.1;
        //         centroid_arr.markers[i].scale.z = 0.1;

        //         centroid_arr.markers[i].color.a = 1.0;
        //         centroid_arr.markers[i].color.r = 1.0;
        //         centroid_arr.markers[i].color.g = 0.0;
        //         centroid_arr.markers[i].color.b = 1.0;
        //     }

        //     marker_pub.publish(centroid_arr);
        // }

        void centroid_index_to_point()
        {
            for (int t = 0; t < region_c-1; t++)
            {
                point.x = (centroids[t] % region_map.info.width)*region_map.info.resolution + region_map.info.origin.position.x;
                point.y = floor(centroids[t] / region_map.info.width)*region_map.info.resolution + region_map.info.origin.position.y;

                if((fabs(point.x - last_cent_x) < 0.1) || (fabs(point.y - last_cent_y) < 0.1))
                {
                    // std::cout << "Got the same centroid again, pass " << std::endl;
                }
                else if((fabs(point.x - last_2cent_x) < 0.1) || (fabs(point.y - last_2cent_y) < 0.1))
                {
                    // std::cout << "Got the same centroid again, pass " << std::endl;
                }
                // But why are we getting the map origin as a centroid...?
                else if((point.x < map_0_msg.info.origin.position.x + 0.05) || (point.y < map_0_msg.info.origin.position.y + 0.05))
                {
                    // std::cout << "Why are we even near the map origin? " << std::endl;
                }
                else
                {
                    // std::cout << "Centroid point is " << point.x << " , " << point.y << std::endl;
                    centroid_Xpts.push_back(point.x);
                    centroid_Ypts.push_back(point.y);
                    // mark_centroids();
                    dist = pow(((pow(point.x - robot_pose_.position.x,2)) + (pow(point.y - robot_pose_.position.y,2))) , 0.5);
                    // std::cout << "Distance is " << dist << std::endl;
                    dist_arr.push_back(dist);
                    centroid_c++;
                }
            }
        }

        void find_closest_centroid()
        {
            // Find spot to move to that is close
            smallest = 9999999.0;
            for(int u = 0; u < region_c-1; u++)
            {
                // std::cout << "Current distance value is " << dist_arr[u] << u << std::endl;
                if (dist_arr[u] < 0.01)
                {
                }
                else if (dist_arr[u] < smallest)
                {
                    smallest = dist_arr[u];
                    // std::cout << "Smallest DISTANCE value is " << smallest << std::endl;
                    move_to_pt = u;
                    // std::cout << "Index we need to move to is ... (should be less than 10) " << move_to_pt << std::endl;
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

                if (map_0_msg.data.size()!=0)
                {

                    // Store the map data in a vector so we can read that shit
                    int map_size = map_0_msg.data.size();
                    // map_data.resize(0);
                    map_data.clear();
                    for (int q = 0; q < map_size; q++)
                    {
                        map_data.push_back(map_0_msg.data[q]);
                    }

                    // Store the map data in a vector so we can read that shit
                    int region_map_size = map_0_msg.data.size();
                    // region_map_data.resize(0);
                    region_map_data.clear();
                    for (int q = 0; q < region_map_size; q++)
                    {
                        region_map_data.push_back(map_0_msg.data[q]);
                    }

                    // Start with inital number of edges is 1 and initalize vector to store edge index
                    num_edges = 0;
                    // edge_vec.resize(0);
                    edge_vec.clear();
                    map_width = FE_tb3_0_map.info.width;
                    map_height = FE_tb3_0_map.info.height;
                    int region_map_width = region_map.info.width;
                    int region_map_height = region_map.info.height;

                    // std::cout << "Map width and height " << map_width << " , " << map_height << std::endl;
                    // std::cout << "Region map width and height " << region_map_width << " , " << region_map_height << std::endl;

                    find_all_edges();

                    sort( edge_vec.begin(), edge_vec.end() );
                    edge_vec.erase( unique( edge_vec.begin(), edge_vec.end() ), edge_vec.end() );

                    tb3_0_FE_map_pub.publish(FE_tb3_0_map);
                    // std::cout << "Number of edges is " << num_edges << " or " << edge_vec.size() << std::endl;

                    //////////////////////////////////////////////////////////////////////////////////////////////////////////
                    // ALL GOOD AT GETTING EDGES
                    // NEED TO FIX HOW WERE CREATING FRONTIERS
                    //////////////////////////////////////////////////////////////////////////////////////////////////////////

                    // find_regions();
                    // region_map_pub.publish(region_map);

                    find_transform();

                    centroid_index_to_point();
                    // std::cout << "Number of centroids should match number of regions " << centroid_c << " =? " << region_c << std::endl;

                    find_closest_centroid();

                    std::cout << "Centroid we gonna go is " << centroid_Xpts[move_to_pt] << " , " <<  centroid_Ypts[move_to_pt] << std::endl;
                    std::cout << "At a distance of " << smallest << std::endl;
                    last_2cent_x = last_cent_x;
                    last_2cent_y = last_cent_y;
                    last_cent_x = centroid_Xpts[move_to_pt];
                    last_cent_y = centroid_Ypts[move_to_pt];
                    // // STORE LAST CENTROID POINT AND SAY IF WE JUST WENT THERE, DONT GO AGAIN

                    // goal.target_pose.header.frame_id = "tb3_0/map"; // Needs to be AN ACTUAL FRAME
                    goal.target_pose.header.frame_id = "map"; // Needs to be AN ACTUAL FRAME
                    goal.target_pose.header.stamp = ros::Time();
                    goal.target_pose.pose.position.x = centroid_Xpts[move_to_pt];
                    goal.target_pose.pose.position.y = centroid_Ypts[move_to_pt];
                    goal.target_pose.pose.orientation.w = 1.0;

                    // std::cout << "Goal is " << goal.target_pose.pose.position.x << " , " <<  goal.target_pose.pose.position.y << std::endl;
                    ROS_INFO("Sending goal");
                    ac.sendGoal(goal);

                    // Wait for the action to return
                    ac.waitForResult(ros::Duration(60));

                    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    {
                        ROS_INFO("You have reached the goal!");
                    }
                    else
                    {
                        ROS_INFO("The base failed for some reason");
                    }

                    std::cout << "" << std::endl;
                }

                else
                {
                    ROS_INFO("Waiting for first map");
                }

                // Reset variables 
                num_edges = 0;
                // region_c = 0;
                region_c = 1;
                centroid_c = 0;
                centroid = 0;
                centroid_index = 0;
                // edge_vec.resize(0);
                // neighbor_index.resize(0);
                // neighbor_value.resize(0);
                // centroid_Xpts.resize(0);
                // centroid_Ypts.resize(0);
                edge_vec.clear();
                neighbor_index.clear();
                neighbor_value.clear();
                centroids.clear();
                centroid_Xpts.clear();
                centroid_Ypts.clear();

                loop_rate.sleep();
            }
        }

    private:
        ros::NodeHandle nh;
        ros::Publisher tb3_0_FE_map_pub, region_map_pub, marker_pub;
        ros::Subscriber map_0_sub;
        nav_msgs::OccupancyGrid map_0_msg;
        nav_msgs::OccupancyGrid FE_tb3_0_map, region_map;
        // visualization_msgs::MarkerArray centroid_arr;
        tf2_ros::Buffer tfBuffer;
        geometry_msgs::Pose robot_pose_;
        std::string map_frame = "map";
        std::string body_frame = "base_footprint";
        std::vector<signed int> edge_vec, neighbor_index, neighbor_value, map_data, region_map_data;
        std::vector<unsigned int> centroids;
        std::vector<double> centroid_Xpts, centroid_Ypts, dist_arr;
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
        int goal_c = 0;
        int num_edges = 0;
        int group_c = 0; // Counts the length of one group
        int prev_group_c = 0;
        int region_c = 1; // Counts the number of regions
        int centroid_c = 0;
        int centroid = 0;
        int centroid_index = 0;
        double smallest = 9999999.0;
        move_base_msgs::MoveBaseGoal goal_init;
        move_base_msgs::MoveBaseGoal goal;
        geometry_msgs::Point point;
        int move_to_pt = 0;
        geometry_msgs::TransformStamped transformS;
        // int below_before, below, below_after, before, after, top_before, top, top_after;
        // int below_before_i, below_i, below_after_i, before_i, after_i, top_before_i, top_i, top_after_i;
        int map_width, map_height, edge_size, mark_edge, edge_index, next_edge_index;
        int edge_after, edge_top_before, edge_top, edge_top_after;
        int edge_after_i, edge_top_before_i, edge_top_i, edge_top_after_i;
        double dist;
        double last_cent_x = 0.0;
        double last_cent_y = 0.0;
        double last_2cent_x = 0.0;
        double last_2cent_y = 0.0;
        bool unique_flag = true;


};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "one_tb_FE_node");
    FrontExpl FE;
    FE.main_loop();
    ros::spin();

    return 0;
}
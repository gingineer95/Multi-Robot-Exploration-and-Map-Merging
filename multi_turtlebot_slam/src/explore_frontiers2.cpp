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
            FE0_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("edges_map_0", 1);
            region0_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("region_map_0", 1);
            map_0_sub = nh.subscribe("map", 10, &FrontExpl::mapCallback, this);
            marker_pub = nh.advertise<visualization_msgs::MarkerArray>("markers", 1, true);
            tf2_ros::TransformListener tfListener(tfBuffer);
            MoveBaseClient ac("move_base", true);

            std::cout << "Initialized all the things" << std::endl;
        }

        /// / \brief Grabs the position of the robot from the pose subscriber and stores it
        /// / \param msg - pose message
        /// \returns nothing
        void mapCallback(const nav_msgs::OccupancyGrid & msg)
        {
            FE0_map = msg;
            region_map = msg;

            FE0_map.header.frame_id = "edges_map_0";
            region_map.header.frame_id = "region_map_0";

            std::cout << "Got to map callback" << std::endl;
            // std::cout << "Map with and height " << msg.info.width << " , " << msg.info.height << std::endl;
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
            std::cout << "Finding all the edges" << std::endl;
            // For all unknown points in the occupancy grid, find the value of the neighbors
            // Starting one row up and on space in so that we dont have an issues with the first point
                for (double x = map_width + 1; x < (map_width * map_height) - map_width - 1; x++)
                {
                    if (FE0_map.data.at(x) == -1)
                    {
                        // If there is an unknown cell then check neighboring cells find potential frontier edge (free cell)
                        neighborhood(x);

                        for(int i = 0; i < neighbor_index.size(); i++)
                        {
                            // int ni = neighbor_index.at(i);
                            // Am I accidentally adding values multiple times here??
                            if (FE0_map.data.at(neighbor_index.at(i)) == 0)
                            {
                                std::cout << "Marking an edge" << std::endl;
                                // If were actually at an edge, mark it
                                // mark_edge = neighbor_index.at(i);
                                FE0_map.data.at(neighbor_index.at(i)) = 10;
                                edge_vec.push_back(neighbor_index.at(i));

                                // num_edges++;
                                // mark_edge = 0;

                            }
                        }

                    }
                } 

        }

        bool check_edges(int curr_cell, int next_cell)
        {
            if (curr_cell == next_cell)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        void find_regions()
        {
            // std::vector<unsigned int> temp_group;
            std::cout << "Finding regions" << std::endl;

            // for (int q = 0; q < num_edges -1; q++) // This was working, but I'm trying to get rid of counters
            for (int q = 0; q < edge_vec.size() - 1; q++)
            {
                unique_flag = check_edges(edge_vec.at(q), edge_vec.at(q+1));

                if (unique_flag == true)
                {
                    // std::cout << "Finding regions around cell " << edge_vec[q] << std::endl;
                    // std::cout << "Evaluating against " << edge_vec[q+1] << std::endl;

                    neighborhood(edge_vec.at(q));

                    for(int i = 0; i < neighbor_index.size(); i++)
                    {
                        // std::cout << "Neighbor index is " << neighbor_index.at(i] << std::endl;

                        if (neighbor_index.at(i) == edge_vec.at(q+1))
                        {
                            edge_index = edge_vec.at(q); 

                            // Do we need to publish the region map?
                            // region_map.data.at(edge_index] = 110 + (20*region_c);
                            // region_map.data.at(next_edge_index] = 110 + (20*region_c);

                            temp_group.push_back(edge_vec.at(q));

                            // std::cout << "Adding this value to temp group " << temp_group.at(group_c] << std::endl;

                            sort( temp_group.begin(), temp_group.end() );
                            temp_group .erase( unique( temp_group.begin(), temp_group.end() ), temp_group.end() );

                            group_c++;
                        }
                    }

                    if (group_c == prev_group_c) // then we didnt add any edges to our region
                    {
                        if (group_c < 5) // frontier region too small
                        {
                        }
                        
                        else
                        {
                            // std::cout << "Size of group is " << group_c << std::endl;

                            ///////////////////////////////////////
                            // Write something that says if we move to close to wall, skip
                            //////////////////////////////////////

                            centroid = (temp_group.size()) / 2;
                            centroid_index = temp_group.at(centroid);

                            centroids.push_back(centroid_index);
                        }

                        // Reset group array and counter
                        group_c = 0;

                        temp_group.clear();
                    }
                    else
                    {
                        prev_group_c = group_c;
                    }
                }

                else
                {
                    // std::cout << "Got a duplicate cell" << std::endl;
                }

            }

        }

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

            // robot_cent_x = robot_pose_.position.x;
            // robot_cent_y = robot_pose_.position.y;
        }

        void mark_centroids()
        {
            visualization_msgs::MarkerArray centroid_arr;
            marker_pub.publish(centroid_arr);

            centroid_arr.markers.resize(centroid_Xpts.size());

            for (int i = 0; i < centroid_Xpts.size(); i++) 
            {
                centroid_arr.markers.at(i).header.frame_id = "map";
                centroid_arr.markers.at(i).header.stamp = ros::Time();
                centroid_arr.markers.at(i).ns = "centroid";
                centroid_arr.markers.at(i).id = i;

                centroid_arr.markers.at(i).type = visualization_msgs::Marker::CYLINDER;
                centroid_arr.markers.at(i).action = visualization_msgs::Marker::ADD;
                centroid_arr.markers.at(i).lifetime = ros::Duration(10);

                centroid_arr.markers.at(i).pose.position.x = centroid_Xpts.at(i);
                centroid_arr.markers.at(i).pose.position.y = centroid_Ypts.at(i);
                centroid_arr.markers.at(i).pose.position.z = 0;
                centroid_arr.markers.at(i).pose.orientation.x = 0.0;
                centroid_arr.markers.at(i).pose.orientation.y = 0.0;
                centroid_arr.markers.at(i).pose.orientation.z = 0.0;
                centroid_arr.markers.at(i).pose.orientation.w = 1.0;

                centroid_arr.markers.at(i).scale.x = 0.1;
                centroid_arr.markers.at(i).scale.y = 0.1;
                centroid_arr.markers.at(i).scale.z = 0.1;

                centroid_arr.markers.at(i).color.a = 1.0;
                centroid_arr.markers.at(i).color.r = 1.0;
                centroid_arr.markers.at(i).color.g = 0.0;
                centroid_arr.markers.at(i).color.b = 1.0;
            }

            marker_pub.publish(centroid_arr);
        }

        void centroid_index_to_point()
        {
            for (int t = 0; t < centroids.size(); t++)
            {
                point.x = (centroids.at(t) % region_map.info.width)*region_map.info.resolution + region_map.info.origin.position.x;
                point.y = floor(centroids.at(t) / region_map.info.width)*region_map.info.resolution + region_map.info.origin.position.y;

                for (int w = 0; w < prev_cent_x.size(); w++)
                {
                    if ( (fabs( prev_cent_x.at(w) - point.x) < 0.1) && (fabs( prev_cent_y.at(w) - point.y) < 0.1) )
                    {
                        std::cout << "We already went to this centroid " << prev_cent_x.at(w) << " , " << prev_cent_y.at(w) << std::endl;
                        std::cout << "compared to current point " << point.x << " , " << point.y << std::endl;
                        goto bad_centroid;
                    }
                }

                if((fabs(point.x) < 0.1) && (fabs(point.y) < 0.1))
                {
                    // std::cout << "Too close to the origin, why? " << std::endl;
                    goto bad_centroid;
                }
                else if((point.x < FE0_map.info.origin.position.x + 0.05) && (point.y < FE0_map.info.origin.position.y + 0.05))
                {
                    // std::cout << "Why are we even near the map origin? " << std::endl;
                    goto bad_centroid;
                }
                else
                {
                    // std::cout << "Centroid point is " << point.x << " , " << point.y << std::endl;
                    centroid_Xpts.push_back(point.x);
                    centroid_Ypts.push_back(point.y);

                    mark_centroids();

                    double delta_x = point.x - robot_pose_.position.x; 
                    double delta_y = point.y - robot_pose_.position.y; 
                    double sum = (pow(delta_x ,2)) + (pow(delta_y ,2));
                    dist = pow( sum , 0.5 );

                    // std::cout << "Distance is " << dist << std::endl;
                    dist_arr.push_back(dist);
                }

                bad_centroid:
                std::cout << "Got a bad centroid and I'm hoping this will fix the problem" << std::endl;
            }
        }

        void find_closest_centroid()
        {
            // Find spot to move to that is close
            smallest = 9999999.0;
            for(int u = 0; u < dist_arr.size(); u++)
            {
                // std::cout << "Current distance value is " << dist_arr.at(u] << " at index " << u << std::endl;

                if (dist_arr.at(u) < 0.1)
                {
                    std::cout << "Index that is too small " << dist_arr.at(u) << std::endl;
                }
                else if (dist_arr.at(u) < smallest)
                {
                    smallest = dist_arr.at(u);
                    std::cout << "Smallest DISTANCE value is " << smallest << std::endl;
                    move_to_pt = u;
                    // std::cout << "Index we need to move to is ... (should be less than 10) " << move_to_pt << std::endl;
                }
                else
                {
                    // std::cout << "This distance is bigger then 0.1 and the smallest value " << std::endl;
                }
            }
        }

        void main_loop()
        {
            std::cout << "Entered the main loop" << std::endl;

            typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
            MoveBaseClient ac("move_base", true);

            //wait for the action server to come up
            while(!ac.waitForServer(ros::Duration(5.0)))
            {
                ROS_INFO("Waiting for the move_base action server to come up");
            }
            ROS_INFO("Connected to move base server");

            tf2_ros::TransformListener tfListener(tfBuffer);
            ros::Rate loop_rate(40);

            while (ros::ok())
            {
                std::cout << "While ros okay" << std::endl;
                std::cout << "Can you call SpinOnce too many times?" << std::endl;

                ros::spinOnce();

                if (FE0_map.data.size()!=0)
                {
                    map_width = FE0_map.info.width;
                    map_height = FE0_map.info.height;

                    find_all_edges();

                    if (edge_vec.size() == 0)
                    {
                        goto skip;
                    }

                    sort( edge_vec.begin(), edge_vec.end() );
                    edge_vec.erase( unique( edge_vec.begin(), edge_vec.end() ), edge_vec.end() );

                    FE0_map_pub.publish(FE0_map);

                    find_regions();
                    // region0_map_pub.publish(region_map);

                    find_transform();

                    centroid_index_to_point();

                    find_closest_centroid();

                    std::cout << "Centroid we gonna go is " << centroid_Xpts.at(move_to_pt) << " , " <<  centroid_Ypts.at(move_to_pt) << std::endl;
                    std::cout << "At a distance of " << smallest << std::endl;

                    prev_cent_x.push_back(centroid_Xpts.at(move_to_pt));
                    prev_cent_y.push_back(centroid_Ypts.at(move_to_pt));

                    // goal.target_pose.header.frame_id = "tb3_0/map"; // Needs to be AN ACTUAL FRAME
                    goal.target_pose.header.frame_id = "map"; // Needs to be AN ACTUAL FRAME
                    goal.target_pose.header.stamp = ros::Time();
                    goal.target_pose.pose.position.x = centroid_Xpts.at(move_to_pt);
                    goal.target_pose.pose.position.y = centroid_Ypts.at(move_to_pt);
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

                skip:
                std::cout << "Skipped to the end since we didnt have any edge_vec" << std::endl;

                sleep(1.0);
                // Reset variables 
                std::cout << "Resetting vairbales" << std::endl;
                centroid = 0;
                centroid_index = 0;

                std::cout << "Clearing all arrays " << std::endl;
                dist_arr.clear();
                edge_vec.clear();
                neighbor_index.clear();
                neighbor_value.clear();
                centroids.clear();
                centroid_Xpts.clear();
                centroid_Ypts.clear();

                std::cout << "Loop sleeping and then doing all this again" << std::endl;

                ros::spinOnce();

                std::cout << "Spinning like a ballerina" << std::endl;

                loop_rate.sleep();

                std::cout << "Asleep and done with main_loop" << std::endl;
            }
        }

    private:
        ros::NodeHandle nh;
        ros::Publisher FE0_map_pub, region0_map_pub, marker_pub;
        ros::Subscriber map_0_sub;
        // nav_msgs::OccupancyGrid FE0_map;
        nav_msgs::OccupancyGrid FE0_map, region_map;
        // visualization_msgs::MarkerArray centroid_arr;
        tf2_ros::Buffer tfBuffer;
        geometry_msgs::Pose robot_pose_;
        std::string map_frame = "map";
        std::string body_frame = "base_footprint";
        std::vector<signed int> edge_vec, neighbor_index, neighbor_value, map_data, region_map_data;
        std::vector<unsigned int> centroids, temp_group;
        std::vector<double> centroid_Xpts, centroid_Ypts, dist_arr, prev_cent_x, prev_cent_y;
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
        int group_c = 0; // Counts the length of one group
        int prev_group_c = 0;
        int centroid = 0;
        int centroid_index = 0;
        double smallest = 9999999.0;
        move_base_msgs::MoveBaseGoal goal_init;
        move_base_msgs::MoveBaseGoal goal;
        geometry_msgs::Point point;
        int move_to_pt = 0;
        geometry_msgs::TransformStamped transformS;
        int map_width, map_height, edge_size, mark_edge, edge_index, next_edge_index;
        double dist = 0.0;
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
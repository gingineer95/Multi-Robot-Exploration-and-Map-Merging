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
#include <std_srvs/Empty.h>
#include <multi_robot_exploration/tb3_0_start.h>
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
            region0_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("region0_map_0", 1);
            // map_0_sub = nh.subscribe("tb3_0/map", 10, &FrontExpl::mapCallback, this);
            map_0_sub = nh.subscribe("map", 10, &FrontExpl::mapCallback, this);
            tf2_ros::TransformListener tfListener(tfBuffer);
            MoveBaseClient ac("tb3_0/move_base", true);

            // Create the start service 
            start0_srv = nh.advertiseService("tb3_0_start", &FrontExpl::startCallback, this);

            std::cout << "Initialized all the 0 things" << std::endl;
        }

        /// \brief Starts the frontier exploration
        /// \param req - service request
        /// \param res - service response
        /// \returns true when done
        bool startCallback(multi_robot_exploration::tb3_0_start::Request &req, multi_robot_exploration::tb3_0_start::Response &res)
        {
            std::cout << "Got to start tb3_0 service" << std::endl;
            start0_flag = true;
            return true;
        }

        /// / \brief Grabs the position of the robot from the pose subscriber and stores it
        /// / \param msg - pose message
        /// \returns nothing
        void mapCallback(const nav_msgs::OccupancyGrid & msg)
        {
            FE0_map = msg;
            region0_map = msg;

            FE0_map.header.frame_id = "edges_map_0";
            region0_map.header.frame_id = "region0_map_0";

            std::cout << "Got to map0 callback" << std::endl;
            // std::cout << "Map with and height " << msg.info.width << " , " << msg.info.height << std::endl;
        }

        void neighborhood(int cell)
        {
            // std::vector<int> neighbor0_index;
            // neighbor0_index.resize(0);
            // neighbor0_value.resize(0);
            neighbor0_index.clear();
            neighbor0_value.clear();

            // Store neighboring values and indexes
            neighbor0_index.push_back(cell - map_width - 1);
            neighbor0_index.push_back(cell - map_width);
            neighbor0_index.push_back(cell - map_width + 1);
            neighbor0_index.push_back(cell-1);
            neighbor0_index.push_back(cell+1);
            neighbor0_index.push_back(cell + map_width - 1);
            neighbor0_index.push_back(cell + map_width);
            neighbor0_index.push_back(cell + map_width + 1);

            sort( neighbor0_index.begin(), neighbor0_index.end() );
            neighbor0_index.erase( unique( neighbor0_index.begin(), neighbor0_index.end() ), neighbor0_index.end() );
        }

        void find_all_edges()
        {
            std::cout << "Finding all the tb3_0 edges" << std::endl;
            // For all unknown points in the occupancy grid, find the value of the neighbors
            // Starting one row up and on space in so that we dont have an issues with the first point
                for (double x = map_width + 1; x < (map_width * map_height) - map_width - 1; x++)
                {
                    if (FE0_map.data.at(x) == -1)
                    {
                        // If there is an unknown cell then check neighboring cells find potential frontier edge (free cell)
                        neighborhood(x);

                        for(int i = 0; i < neighbor0_index.size(); i++)
                        {
                            // int ni = neighbor0_index.at(i);
                            // Am I accidentally adding values multiple times here??
                            if (FE0_map.data.at(neighbor0_index.at(i)) == 0)
                            {
                                // std::cout << "Marking an edge" << std::endl;
                                // If were actually at an edge, mark it
                                // mark_edge = neighbor0_index.at(i);
                                FE0_map.data.at(neighbor0_index.at(i)) = 10;
                                edge0_vec.push_back(neighbor0_index.at(i));

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
            // std::vector<unsigned int> temp_group0;
            std::cout << "Finding regions for tb3_0" << std::endl;

            // for (int q = 0; q < num_edges -1; q++) // This was working, but I'm trying to get rid of counters
            for (int q = 0; q < edge0_vec.size() - 1; q++)
            {
                unique_flag = check_edges(edge0_vec.at(q), edge0_vec.at(q+1));

                if (unique_flag == true)
                {
                    // std::cout << "Finding regions around cell " << edge0_vec[q] << std::endl;
                    // std::cout << "Evaluating against " << edge0_vec[q+1] << std::endl;

                    neighborhood(edge0_vec.at(q));

                    for(int i = 0; i < neighbor0_index.size(); i++)
                    {
                        // std::cout << "Neighbor index is " << neighbor0_index.at(i] << std::endl;

                        if (neighbor0_index.at(i) == edge0_vec.at(q+1))
                        {
                            edge_index = edge0_vec.at(q); 

                            // Do we need to publish the region map?
                            // region0_map.data.at(edge_index] = 110 + (20*region_c);
                            // region0_map.data.at(next_edge_index] = 110 + (20*region_c);

                            temp_group0.push_back(edge0_vec.at(q));

                            // std::cout << "Adding this value to temp group " << temp_group0.at(group0_c] << std::endl;

                            sort( temp_group0.begin(), temp_group0.end() );
                            temp_group0 .erase( unique( temp_group0.begin(), temp_group0.end() ), temp_group0.end() );

                            group0_c++;
                        }
                    }

                    if (group0_c == prev_group0_c) // then we didnt add any edges to our region
                    {
                        if (group0_c < 5) // frontier region too small
                        {
                        }
                        
                        else
                        {
                            // std::cout << "Size of group is " << group0_c << std::endl;

                            ///////////////////////////////////////
                            // Write something that says if we move to close to wall, skip
                            //////////////////////////////////////

                            centroid0 = (temp_group0.size()) / 2;
                            centroid0_index = temp_group0.at(centroid0);

                            centroids0.push_back(centroid0_index);
                        }

                        // Reset group array and counter
                        group0_c = 0;

                        temp_group0.clear();
                    }
                    else
                    {
                        prev_group0_c = group0_c;
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
            transformS = tfBuffer.lookupTransform(map0_frame, body0_frame, ros::Time(0), ros::Duration(3.0));

            transformS.header.stamp = ros::Time();
            transformS.header.frame_id = body0_frame;
            transformS.child_frame_id = map0_frame;

            robot0_pose_.position.x = transformS.transform.translation.x;
            robot0_pose_.position.y = transformS.transform.translation.y;
            robot0_pose_.position.z = 0.0;
            robot0_pose_.orientation.x = transformS.transform.rotation.x;
            robot0_pose_.orientation.y = transformS.transform.rotation.y;
            robot0_pose_.orientation.z = transformS.transform.rotation.z;
            robot0_pose_.orientation.w = transformS.transform.rotation.w;

            std::cout << "Robot pose is  " << robot0_pose_.position.x << " , " << robot0_pose_.position.y << std::endl;

            // robot_cent_x = robot0_pose_.position.x;
            // robot_cent_y = robot0_pose_.position.y;
        }

        // void mark_centroids0()
        // {
        //     visualization_msgs::MarkerArray centroid_arr;
        //     marker_pub.publish(centroid_arr);

        //     centroid_arr.markers.resize(centroid0_Xpts.size());

        //     for (int i = 0; i < centroid0_Xpts.size(); i++) 
        //     {
        //         centroid_arr.markers.at(i).header.frame_id = "map";
        //         centroid_arr.markers.at(i).header.stamp = ros::Time();
        //         centroid_arr.markers.at(i).ns = "centroid";
        //         centroid_arr.markers.at(i).id = i;

        //         centroid_arr.markers.at(i).type = visualization_msgs::Marker::CYLINDER;
        //         centroid_arr.markers.at(i).action = visualization_msgs::Marker::ADD;
        //         centroid_arr.markers.at(i).lifetime = ros::Duration(10);

        //         centroid_arr.markers.at(i).pose.position.x = centroid0_Xpts.at(i);
        //         centroid_arr.markers.at(i).pose.position.y = centroid0_Ypts.at(i);
        //         centroid_arr.markers.at(i).pose.position.z = 0;
        //         centroid_arr.markers.at(i).pose.orientation.x = 0.0;
        //         centroid_arr.markers.at(i).pose.orientation.y = 0.0;
        //         centroid_arr.markers.at(i).pose.orientation.z = 0.0;
        //         centroid_arr.markers.at(i).pose.orientation.w = 1.0;

        //         centroid_arr.markers.at(i).scale.x = 0.1;
        //         centroid_arr.markers.at(i).scale.y = 0.1;
        //         centroid_arr.markers.at(i).scale.z = 0.1;

        //         centroid_arr.markers.at(i).color.a = 1.0;
        //         centroid_arr.markers.at(i).color.r = 1.0;
        //         centroid_arr.markers.at(i).color.g = 0.0;
        //         centroid_arr.markers.at(i).color.b = 1.0;
        //     }

        //     marker_pub.publish(centroid_arr);
        // }

        void centroid_index_to_point()
        {
            for (int t = 0; t < centroids0.size(); t++)
            {
                point.x = (centroids0.at(t) % region0_map.info.width)*region0_map.info.resolution + region0_map.info.origin.position.x;
                point.y = floor(centroids0.at(t) / region0_map.info.width)*region0_map.info.resolution + region0_map.info.origin.position.y;

                for (int w = 0; w < prev_cent_0x.size(); w++)
                {
                    if ( (fabs( prev_cent_0x.at(w) - point.x) < 0.01) && (fabs( prev_cent_0y.at(w) - point.y) < 0.01) )
                    {
                        std::cout << "Already went to this centroid " << prev_cent_0x.at(w) << " , " << prev_cent_0y.at(w) << std::endl;
                        // std::cout << "compared to current point " << point.x << " , " << point.y << std::endl;
                        goto bad_centroid;
                    }
                }

                // if((fabs(point.x) < 0.1) && (fabs(point.y) < 0.1))
                // {
                //     // std::cout << "Too close to the origin, why? " << std::endl;
                //     goto bad_centroid;
                // }
                if((point.x < FE0_map.info.origin.position.x + 0.05) && (point.y < FE0_map.info.origin.position.y + 0.05))
                {
                    // std::cout << "Why are we even near the map origin? " << std::endl;
                    goto bad_centroid;
                }
                else
                {
                    // std::cout << "Centroid point is " << point.x << " , " << point.y << std::endl;
                    centroid0_Xpts.push_back(point.x);
                    centroid0_Ypts.push_back(point.y);

                    // mark_centroids0();

                    double delta_x = point.x - robot0_pose_.position.x; 
                    double delta_y = point.y - robot0_pose_.position.y; 
                    double sum = (pow(delta_x ,2)) + (pow(delta_y ,2));
                    dist0 = pow( sum , 0.5 );

                    // std::cout << "Distance is " << dist0<< std::endl;
                    dist0_arr.push_back(dist0);
                }

                bad_centroid: 
                std::cout << "Ignore bad centroid" << std::endl;
            }
        }

        void find_closest_centroid()
        {
            // Find spot to move to that is close
            smallest = 9999999.0;
            for(int u = 0; u < dist0_arr.size(); u++)
            {
                // std::cout << "Current distance value is " << dist0_arr.at(u] << " at index " << u << std::endl;

                if (dist0_arr.at(u) < 0.1)
                {
                    // std::cout << "Index that is too small " << dist0_arr.at(u) << std::endl;
                }
                else if (dist0_arr.at(u) < smallest)
                {
                    smallest = dist0_arr.at(u);
                    // std::cout << "Smallest DISTANCE value is " << smallest << std::endl;
                    move_to_pt = u;
                    // std::cout << "Index we need to move to is ... (should be less than 10) " << move_to_pt << std::endl;
                }
                else
                {
                    // std::cout << "This distance is bigger then 0.1 and the smallest value " << std::endl;
                }
            }
        }

        void edge_index_to_point()
        {
            std::cout << "Marking edges " << edge0_vec.size() << std::endl;
            for (int t = 0; t < edge0_vec.size(); t++)
            {
                point.x = (edge0_vec.at(t) % FE0_map.info.width)*FE0_map.info.resolution + FE0_map.info.origin.position.x;
                point.y = floor(edge0_vec.at(t) / FE0_map.info.width)*FE0_map.info.resolution + FE0_map.info.origin.position.y;

                std::cout << "Point is " << point.x << ", " << point.y << std::endl;

                centroid0_Xpts.push_back(point.x);
                centroid0_Ypts.push_back(point.y);

                // mark_centroids0();

                double delta_x = point.x - robot0_pose_.position.x; 
                double delta_y = point.y - robot0_pose_.position.y; 
                double sum = (pow(delta_x ,2)) + (pow(delta_y ,2));
                dist0 = pow( sum , 0.5 );

                std::cout << "Distance is " << dist0 << std::endl;
                dist0_arr.push_back(dist0);
            }
        }

        void main_loop()
        {
            std::cout << "Entered the main loop for 0" << std::endl;

            typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
            MoveBaseClient ac("tb3_0/move_base", true);

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
                std::cout << "While ros okay for 0" << std::endl;
                // std::cout << "Can you call SpinOnce too many times?" << std::endl;

                ros::spinOnce();

                if (FE0_map.data.size()!=0 && start0_flag == true)
                {
                    map_width = FE0_map.info.width;
                    map_height = FE0_map.info.height;

                    find_all_edges();

                    if (edge0_vec.size() == 0)
                    {
                        goto skip;
                    }

                    sort( edge0_vec.begin(), edge0_vec.end() );
                    edge0_vec.erase( unique( edge0_vec.begin(), edge0_vec.end() ), edge0_vec.end() );

                    FE0_map_pub.publish(FE0_map);

                    find_regions();
                    // region0_map_pub.publish(region0_map);

                    find_transform();

                    centroid_index_to_point();

                    if ( ( centroid0_Xpts.size() == 0 ) || ( centroid0_Ypts.size() == 0) )
                    {
                        // Go to closest edge
                        centroid0_Xpts.clear();
                        centroid0_Ypts.clear();
                        dist0_arr.clear();
                        std::cout << "Couldnt find a centroid, move to closest edge instead" << std::endl;
                        edge_index_to_point();
                        
                    }

                    find_closest_centroid();

                    std::cout << "Moving to centroid " << centroid0_Xpts.at(move_to_pt) << " , " <<  centroid0_Ypts.at(move_to_pt) << std::endl;
                    std::cout << "At a distance of " << smallest << std::endl;

                    prev_cent_0x.push_back(centroid0_Xpts.at(move_to_pt));
                    prev_cent_0y.push_back(centroid0_Ypts.at(move_to_pt));

                    // goal.target_pose.header.frame_id = "tb3_0/map"; // Needs to be AN ACTUAL FRAME
                    goal.target_pose.header.frame_id = "map"; // Needs to be AN ACTUAL FRAME
                    goal.target_pose.header.stamp = ros::Time();
                    goal.target_pose.pose.position.x = centroid0_Xpts.at(move_to_pt);
                    goal.target_pose.pose.position.y = centroid0_Ypts.at(move_to_pt);
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
                std::cout << "Starting loop over" << std::endl;

                sleep(1.0);
                // Reset variables 
                std::cout << "Resetting vairbales" << std::endl;
                centroid0 = 0;
                centroid0_index = 0;

                std::cout << "Clearing all arrays " << std::endl;
                dist0_arr.clear();
                edge0_vec.clear();
                neighbor0_index.clear();
                neighbor0_value.clear();
                centroids0.clear();
                centroid0_Xpts.clear();
                centroid0_Ypts.clear();

                std::cout << "Loop sleeping and then doing all this again" << std::endl;

                ros::spinOnce();

                // std::cout << "Spinning like a ballerina" << std::endl;

                loop_rate.sleep();

                std::cout << "Asleep and done with main_loop" << std::endl;
            }
        }

    private:
        ros::NodeHandle nh;
        ros::Publisher FE0_map_pub, region0_map_pub, marker_pub;
        ros::Subscriber map_0_sub;
        ros::ServiceServer start0_srv;
        // nav_msgs::OccupancyGrid FE0_map;
        nav_msgs::OccupancyGrid FE0_map, region0_map;
        // visualization_msgs::MarkerArray centroid_arr;
        tf2_ros::Buffer tfBuffer;
        geometry_msgs::Pose robot0_pose_;
        std::string map0_frame = "tb3_0/map";
        std::string body0_frame = "tb3_0/base_footprint";
        std::vector<signed int> edge0_vec, neighbor0_index, neighbor0_value, map0_data, region0_map_data;
        std::vector<unsigned int> centroids0, temp_group0;
        std::vector<double> centroid0_Xpts, centroid0_Ypts, dist0_arr, prev_cent_0x, prev_cent_0y;
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
        int group0_c = 0; // Counts the length of one group
        int prev_group0_c = 0;
        int centroid0 = 0;
        int centroid0_index = 0;
        double smallest = 9999999.0;
        move_base_msgs::MoveBaseGoal goal_init;
        move_base_msgs::MoveBaseGoal goal;
        geometry_msgs::Point point;
        int move_to_pt = 0;
        geometry_msgs::TransformStamped transformS;
        int map_width, map_height, edge_size, mark_edge, edge_index, next_edge_index;
        double dist0= 0.0;
        bool unique_flag = true;
        bool start0_flag = false;


};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "tb3_0_FE_node");
    FrontExpl FE;
    FE.main_loop();
    ros::spin();

    return 0;
}
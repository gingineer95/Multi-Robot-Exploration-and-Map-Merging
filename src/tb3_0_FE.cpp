/// \file
/// \brief This node causes the robot to move autonomously via Frontier Exploration

/// PUBLISHES:
///     /edges_map_0 (nav_msgs/OccupancyGrid): Publishes the same map as SLAM, with visualization of the frontier edges
/// SUBSCRIBES:
///     /map (nav_msgs/OccupancyGrid): Reads the map created by SLAM to determine frontiers
/// SERVICES:
///     /tb3_0_start (bool): Starts the robot's frontier exploraiton algorithm

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
            map_0_sub = nh.subscribe("map", 10, &FrontExpl::mapCallback, this);
            start0_srv = nh.advertiseService("tb3_0_start", &FrontExpl::startCallback, this);
            std::cout << "Initialized tb3_0 publishers, subcribers and service" << std::endl;
        }

        /// \brief Starts frontier exploration
        /// \param req - service request
        /// \param res - service response
        /// \returns true when done
        bool startCallback(multi_robot_exploration::tb3_0_start::Request &req, multi_robot_exploration::tb3_0_start::Response &res)
        {
            std::cout << "Got to start tb3_0 service" << std::endl;
            start0_flag = true;
            return true;
        }

        /// \brief Reads the map data published from slam_toolbox
        /// \param msg - map message
        /// \returns nothing
        void mapCallback(const nav_msgs::OccupancyGrid & msg)
        {
            FE0_map = msg;
            FE0_map.header.frame_id = "edges_map_0";
            std::cout << "Got to map0 callback" << std::endl;
        }

        /// \brief Stores a vector of index values of the 8 cell neighborhood relative to the input cell
        /// \param cell - map cell
        /// \returns nothing
        void neighborhood(int cell)
        {
            // Clear any previous values in the vectors
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

        /// \brief Stores a vector of frontier edges
        /// \returns nothing
        void find_all_edges()
        {
            std::cout << "Finding all the tb3_0 edges" << std::endl;
            
            // Starting one row up and on space in on the map so there are no indexing issues
            for (double x = map_width + 1; x < (map_width * map_height) - map_width - 1; x++)
            {
                // For all cells in the map, check if a cell is unknown
                if (FE0_map.data.at(x) == -1)
                {
                    // If there is an unknown cell, then check neighboring cells to find a potential frontier edge (free cell)
                    neighborhood(x);

                    for(int i = 0; i < neighbor0_index.size(); i++) // For all neighboring cells
                    {
                        if (FE0_map.data.at(neighbor0_index.at(i)) == 0)
                        {
                            // If one of the neighboring cells is free, store it in the edges vector
                            FE0_map.data.at(neighbor0_index.at(i)) = 10; // Visualize the frontier edge cells
                            edge0_vec.push_back(neighbor0_index.at(i));

                        }
                    }

                }
            } 

        }

        /// \brief Compares two cells to see if the are identical or unique
        /// \param - curr_cell: The current cell being evaluated
        /// \param - next_cell: The next cell to be evaluated
        /// \returns true or false
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

        /// \brief Given the frontier edges, group the neighboring edges into regions
        /// \returns nothing
        void find_regions()
        {
            std::cout << "Finding regions for tb3_0" << std::endl;

            for (int q = 0; q < edge0_vec.size() - 1; q++)
            {
                // For each frontier edge, check that the next value is unique and not a repeat
                unique_flag = check_edges(edge0_vec.at(q), edge0_vec.at(q+1));

                if (unique_flag == true)
                {
                    // If we have an original frontier edge, check the neighboring cells
                    neighborhood(edge0_vec.at(q));

                    for(int i = 0; i < neighbor0_index.size(); i++)
                    {
                        // For all the cells nighrboring the frontier edge, check to see if there is another frontier edge
                        if (neighbor0_index.at(i) == edge0_vec.at(q+1))
                        {
                            // If a frontier edge is in the neightborhood of another frontier edge, add it to a region
                            edge_index = edge0_vec.at(q); 
                            temp_group0.push_back(edge0_vec.at(q));;

                            sort( temp_group0.begin(), temp_group0.end() );
                            temp_group0 .erase( unique( temp_group0.begin(), temp_group0.end() ), temp_group0.end() );

                            // Increase the counter to keep track of the region's size
                            group0_c++;
                        }
                    }

                    if (group0_c == prev_group0_c) // If we didnt any any edeges to our region, region is complete
                    {
                        if (group0_c < 5) // frontier region too small
                        {
                            // If the forntier region is smaller than 5 cells, dont use it
                        }
                        
                        else
                        {
                            // If the frontier region is larger than 5 cells, find the regions centroid
                            centroid0 = (temp_group0.size()) / 2;
                            centroid0_index = temp_group0.at(centroid0);
                            centroids0.push_back(centroid0_index);
                        }

                        // Reset the region vector and size counter
                        group0_c = 0;
                        temp_group0.clear();
                    }

                    else
                    {
                        // If we found a frontier edge, increase the group size
                        prev_group0_c = group0_c;
                    }
                }

                else
                {
                    // If we got a duplicate cell, do nothing
                }

            }

        }

        /// \brief Finds the transform between the map frame and the robot's base_footprint frame
        /// \returns nothing
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
            robot0_pose_.orientation = transformS.transform.rotation;
            // robot0_pose_.orientation.y = transformS.transform.rotation.y;
            // robot0_pose_.orientation.z = transformS.transform.rotation.z;
            // robot0_pose_.orientation.w = transformS.transform.rotation.w;

            std::cout << "Robot pose is  " << robot0_pose_.position.x << " , " << robot0_pose_.position.y << std::endl;
        }

        /// \brief Given a centroid cell, convert the centroid to x-y coordinates in the map frame and determine its distance from the robot
        /// \returns nothing
        void centroid_index_to_point()
        {
            for (int t = 0; t < centroids0.size(); t++)
            {
                // For all the centorid cells, find the x and y coordinates in the map frame
                point.x = (centroids0.at(t) % FE0_map.info.width)*FE0_map.info.resolution + FE0_map.info.origin.position.x;
                point.y = floor(centroids0.at(t) / FE0_map.info.width)*FE0_map.info.resolution + FE0_map.info.origin.position.y;

                for (int w = 0; w < prev_cent_0x.size(); w++)
                {
                    // Compare the previous centroids to the current calculated centroid
                    if ( (fabs( prev_cent_0x.at(w) - point.x) < 0.01) && (fabs( prev_cent_0y.at(w) - point.y) < 0.01) )
                    {
                        // If the current centroid is too close to a previous centroid, skip
                        std::cout << "Already went to this centroid " << prev_cent_0x.at(w) << " , " << prev_cent_0y.at(w) << std::endl;
                        goto bad_centroid;
                    }
                }

                if((point.x < FE0_map.info.origin.position.x + 0.05) && (point.y < FE0_map.info.origin.position.y + 0.05))
                {
                    // If the centroid is too close to the map orgin, skip
                    goto bad_centroid;
                }

                else
                {
                    // If the centroid is valid, add its x and y values to their respective vectors
                    centroid0_Xpts.push_back(point.x);
                    centroid0_Ypts.push_back(point.y);

                    // Determine the distance between the current centroid and the robot's position
                    double delta_x = point.x - robot0_pose_.position.x; 
                    double delta_y = point.y - robot0_pose_.position.y; 
                    double sum = (pow(delta_x ,2)) + (pow(delta_y ,2));
                    dist0 = pow( sum , 0.5 );

                    // Store the distance value in a vector
                    dist0_arr.push_back(dist0);
                }

                // Skip to the end of the for loop if there was an invalid centroid
                bad_centroid: 
                std::cout << "Ignore bad centroid" << std::endl;
            }
        }

        /// \brief Given all the centroid distance values, find the closest centroid to move to
        /// \returns nothing
        void find_closest_centroid()
        {
            // Set the first smallest distance to be large
            smallest = 9999999.0;
            for(int u = 0; u < dist0_arr.size(); u++)
            {
                // For each centroid distance, determine if the current centroid is closer than the previous closest centroid

                if (dist0_arr.at(u) < 0.1)
                {
                    // If the centroid distance is less than 0.1m, its too close. Ignore it
                }
                else if (dist0_arr.at(u) < smallest)
                {
                    // If the current distance element is smaller than the previous closest centroid
                    // replace the smallest distance variable with the current distance 
                    smallest = dist0_arr.at(u);
                    
                    // Update the centroid that the robot will move to
                    move_to_pt = u;
                }

                else
                {
                    // If the current distance value is larger than 0.1 and the smallest_dist value, then ignore it
                }
            }
        }

        /// \brief Given the frontier edges, convert the cell to x-y coordinates in the map frame and determine its distance from the robot
        /// \returns nothing
        void edge_index_to_point()
        {
            for (int t = 0; t < edge0_vec.size(); t++)
            {
                // For all the frontier edge cells, find the x and y coordinates in the map frame
                point.x = (edge0_vec.at(t) % FE0_map.info.width)*FE0_map.info.resolution + FE0_map.info.origin.position.x;
                point.y = floor(edge0_vec.at(t) / FE0_map.info.width)*FE0_map.info.resolution + FE0_map.info.origin.position.y;

                // Add the cells x and y values to their respective vectors
                centroid0_Xpts.push_back(point.x);
                centroid0_Ypts.push_back(point.y);

                // Determine the distance between the current frontier edge and the robot's position
                double delta_x = point.x - robot0_pose_.position.x; 
                double delta_y = point.y - robot0_pose_.position.y; 
                double sum = (pow(delta_x ,2)) + (pow(delta_y ,2));
                dist0 = pow( sum , 0.5 );

                // Store the distance value in a vector
                dist0_arr.push_back(dist0);
            }
        }

        /// \brief Calls all other functions to find frontier edges, regions and a goal to move to. Then uses the action server to move to that goal
        /// \returns nothing
        void main_loop()
        {
            std::cout << "Entered the main loop for 0" << std::endl;

            typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
            MoveBaseClient ac("tb3_0/move_base", true);

            // Wait for the action server to come up
            while(!ac.waitForServer(ros::Duration(5.0)))
            {
                ROS_INFO("Waiting for the move_base action server to come up");
            }
            ROS_INFO("Connected to move base server");

            tf2_ros::TransformListener tfListener(tfBuffer);
            ros::Rate loop_rate(40);

            while (ros::ok())
            {
                std::cout << "While ros okay for tb3_0" << std::endl;
                ros::spinOnce();

                // If there is no map data and / or start service isnt called, do nothing and instead start the loop over again
                if (FE0_map.data.size()!=0 && start0_flag == true)
                {
                    map_width = FE0_map.info.width;
                    map_height = FE0_map.info.height;

                    // Find the frontier edges
                    find_all_edges();

                    // If there are no frontier edges, skip to the end of the main_loop and try again
                    if (edge0_vec.size() == 0)
                    {
                        goto skip;
                    }

                    sort( edge0_vec.begin(), edge0_vec.end() );
                    edge0_vec.erase( unique( edge0_vec.begin(), edge0_vec.end() ), edge0_vec.end() );

                    FE0_map_pub.publish(FE0_map);
                    // Given the forntier edges, find the frontier regions and their centroids
                    find_regions();

                    // Find the transfrom between the map frame the base_footprint frame
                    // in order to determine the robot's position
                    find_transform();

                    // Given the centroid vector, convert the controids from map cells to x-y coordinates in the map frame
                    // so a goal can be sent to move_base
                    centroid_index_to_point();

                    // If there are no values in the centroid x or y vectors, find the closest frontier edge to move to
                    if ( ( centroid0_Xpts.size() == 0 ) || ( centroid0_Ypts.size() == 0) )
                    {
                        centroid0_Xpts.clear();
                        centroid0_Ypts.clear();
                        dist0_arr.clear();
                        std::cout << "Couldnt find a centroid, move to closest edge instead" << std::endl;

                        // Given the edge vector, convert the edges from map cells to x-y coordinates in the map frame
                        // so a goal can be sent to move_base                       
                        edge_index_to_point(); 
                    }

                    // Of all the centroids, determine which centroid is the closest
                    // Choose the closest centroid to move to
                    find_closest_centroid();

                    std::cout << "Moving to centroid " << centroid0_Xpts.at(move_to_pt) << " , " <<  centroid0_Ypts.at(move_to_pt) << std::endl;
                    std::cout << "At a distance of " << smallest << std::endl;

                    // Add the current centroid to the vector of previously visited centroids
                    prev_cent_0x.push_back(centroid0_Xpts.at(move_to_pt));
                    prev_cent_0y.push_back(centroid0_Ypts.at(move_to_pt));

                    // Determine the move_base goal 
                    goal.target_pose.header.frame_id = "tb3_0/map"; // Needs to be AN ACTUAL FRAME
                    // goal.target_pose.header.frame_id = "map"; // Needs to be AN ACTUAL FRAME
                    goal.target_pose.header.stamp = ros::Time();
                    goal.target_pose.pose.position.x = centroid0_Xpts.at(move_to_pt);
                    goal.target_pose.pose.position.y = centroid0_Ypts.at(move_to_pt);
                    goal.target_pose.pose.orientation.w = 1.0;

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

                // If the size of the map data is 0, then run through the loop agai
                else
                {
                    ROS_INFO("Waiting for first map");
                }

                // Skip to the end of the loop if there are no fontier regions
                skip:
                std::cout << "Starting loop over" << std::endl;
                sleep(1.0);

                std::cout << "Resetting vairbales and clearing all vectors" << std::endl;
                centroid0 = 0;
                centroid0_index = 0;
                dist0_arr.clear();
                edge0_vec.clear();
                neighbor0_index.clear();
                neighbor0_value.clear();
                centroids0.clear();
                centroid0_Xpts.clear();
                centroid0_Ypts.clear();

                ros::spinOnce();

                std::cout << "Spinning like a ballerina" << std::endl;

                loop_rate.sleep();

                std::cout << "Asleep and done with main_loop" << std::endl;
            }
        }

    private:
        ros::NodeHandle nh;
        ros::Publisher FE0_map_pub;
        ros::Subscriber map_0_sub;
        ros::ServiceServer start0_srv;
        tf2_ros::Buffer tfBuffer;
        nav_msgs::OccupancyGrid FE0_map;
        geometry_msgs::Pose robot0_pose_;
        geometry_msgs::Point point;
        geometry_msgs::TransformStamped transformS;
        move_base_msgs::MoveBaseGoal goal;
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
        std::string map0_frame = "tb3_0/map";
        std::string body0_frame = "tb3_0/base_footprint";
        std::vector<signed int> edge0_vec, neighbor0_index, neighbor0_value;
        std::vector<unsigned int> centroids0, temp_group0;
        std::vector<double> centroid0_Xpts, centroid0_Ypts, dist0_arr, prev_cent_0x, prev_cent_0y;
        int group0_c=0, prev_group0_c=0, centroid0=0, centroid0_index=0, move_to_pt=0, map_width=0, map_height=0, mark_edge=0, edge_index=0;
        double smallest = 9999999.0, dist0= 0.0;
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
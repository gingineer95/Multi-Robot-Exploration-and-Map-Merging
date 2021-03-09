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
            // Create the initpose publisher and subscriber
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

            // Remove the last cell since its the last cell of the map
            // edge_vec.pop_back();

            // // Remove duplicates
            // sort( edge_vec.begin(), edge_vec.end() );
            // edge_vec.erase( unique( edge_vec.begin(), edge_vec.end() ), edge_vec.end() );
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
        }

        void centroid_index_to_point()
        {
            for (int t = 0; t < region_c-1; t++)
            {
                point.x = (edge_vec[t] % FE_tb3_0_map.info.width)*FE_tb3_0_map.info.resolution + FE_tb3_0_map.info.origin.position.x;
                point.y = floor(edge_vec[t] / FE_tb3_0_map.info.width)*FE_tb3_0_map.info.resolution + FE_tb3_0_map.info.origin.position.y;

                if((point.x == last_cent_x) || (point.y == last_cent_y) || (point.x == last_2cent_x) || (point.y == last_2cent_y))
                {
                    // std::cout << "Got the same centroid again, pass " << std::endl;
                }
                // But why are we getting the map origin as a centroid...?
                else if((point.x < map_0_msg.info.origin.position.x + 0.05) || (point.y < map_0_msg.info.origin.position.y + 0.05))
                {
                    std::cout << "Why are we even near the map origin? " << std::endl;
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
            for(int u = 0; u < edge_vec.size(); u++)
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
            ros::Rate loop_rate(10);

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
                    map_data.resize(0);
                    for (int q = 0; q < map_size; q++)
                    {
                        map_data.push_back(map_0_msg.data[q]);
                    }

                    // Start with inital number of edges is 1 and initalize vector to store edge index
                    num_edges = 0;
                    edge_vec.resize(0);
                    map_width = FE_tb3_0_map.info.width;
                    map_height = FE_tb3_0_map.info.height;

                    // std::cout << "Map width and height " << map_width << " , " << map_height << std::endl;
                    // std::cout << "Region map width and height " << region_map_width << " , " << region_map_height << std::endl;

                    std::cout << "So I'm okay here" << std::endl;

                    find_all_edges();

                    std::cout << "But can I publish?" << std::endl;

                    tb3_0_FE_map_pub.publish(FE_tb3_0_map);

                    for (int i = 0; i < edge_vec.size(); i++)
                    {
                        // std::cout << "This is an index, right? " << edge_vec[i] << std::endl;
                        neighborhood(edge_vec[i]);

                        for(int j = 0; j < neighbor_index.size(); j++)
                        {
                            // Am I accidentally adding values multiple times here??
                            if (map_data[neighbor_index[j]] == 100)
                            {
                                // Get rid of edges near walls
                                edge_vec.erase(edge_vec.begin()+i);

                            }
                        }


                    }

                    //////////////////////////////////////////////////////////////////////////////////////////////////////////
                    // ALL GOOD AT GETTING EDGES
                    // NEED TO FIX HOW WERE CREATING FRONTIERS
                    //////////////////////////////////////////////////////////////////////////////////////////////////////////

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
                    std::cout << "Waiting for map data" << std::endl;
                }
                // Reset variables 
                num_edges = 0;
                region_c = 0;
                centroid_c = 0;
                centroid = 0;
                centroid_index = 0;
                edge_vec.clear();
                neighbor_index.clear();
                neighbor_value.clear();
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
        visualization_msgs::MarkerArray centroid_arr;
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
        int region_c = 0; // Counts the number of regions
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

};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "one_tb_FE_node");
    FrontExpl FE;
    FE.main_loop();
    ros::spin();

    return 0;
}
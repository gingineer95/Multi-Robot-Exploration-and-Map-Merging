Kailey Smith
# Multi-Robot Exploration and Map Merging

![real_FE_noisy](https://user-images.githubusercontent.com/70979347/111921896-0b3fe980-8a65-11eb-9349-c4512cfde041.gif)

## Project Description
The goal is to use multiple robots to autonomously explore an environment and create one global merged map comprised of each's robot individual map. This can be achived with or without knwoledge of the robot's initial positions. 

Frontier Exploration was implemented as the autonomous navigation algorithm. Map merging was achieved by modifying the [multirobot_map_merge](http://wiki.ros.org/multirobot_map_merge) node. 

Currently this repository contains the software necessary to run frontier exploration and map merging of multiple robots in simulation using Gazebo. It also contains the software necessary to run frontier exploration on one actual Turtlebot3. Actively developing multi-robot exploration and map merging on actual Turtlebot3s. Also developing modularity for functionality with any number of robots. 

Note: The original multirobot_map_merge node was written to run gmapping, but for this package the gmapping functionality has been removed in favor of using `slam_toolbox`. To achieve map merging however, each robot's occupancy grid must be the same size and have the same origin. To compensate for this, the `map_expansion` node was written which creates a new, larger map and inserts `slam_toolbox`'s data into the new map. 

To see this package in action, please view my portfolio and post and video deomonstration here: https://gingineer95.github.io/2021/03/21/multirobot-map-merge/

## 3rd Party Packages
- [Multirobot_map_merge](http://wiki.ros.org/multirobot_map_merge): Node that merges the individual maps into one gloval map
- [Aws-robomaker-bookstore-world](https://github.com/aws-robotics/aws-robomaker-bookstore-world): Gazebo world of a bookstore
- [Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) (navigation, gazebo, teleop and bringup)
- [Slam_toolbox](https://github.com/SteveMacenski/slam_toolbox): Used for simultaneous localization and mapping
- [Teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard): Generic keyboard teleop for twist robots
- [Gazebo_ros](http://wiki.ros.org/gazebo_ros) 

## Installation Instructions
- This package is called multi_robot_exploration
- I dont have a rosinstall at the moment (my apologies, coming very soon) but for now please use the packages above for referance

## Getting Started
### Multi-Robot Exploration and Map Merging

![2tb_FE](https://user-images.githubusercontent.com/70979347/111922117-22cba200-8a66-11eb-8fbc-e9a0257c7e63.gif)


#### Known Initial Robot Positions
- This package is initially set up to run in simulation given that you know where the robot's spawn position. 
- To run multi-robot exploration and map merging in a simulated bookstore, `source` your workspace and run `roslaunch multi_robot_exploration two_tb_exploration.launch`
- You should see two Turtlebots spawn in Gazebo and Rviz. In Rviz you should see three maps being published: `tb3_0/map`, `tb3_1/map` and `map`
    - Please give the simulation a minuite to load
    - You'll know everything is up and running when the merged map appears (published on the `map` topic)
    - `tb3_0/map` and `tb3_1/map` are the individual robot maps which are published from `slam_toolbox`
- The robots require a start service be called before they start executing frontier exploration
- To start frontier exploration for both robots run `rosservice call /tb3_0_start` 
and `rosservice call /tb3_1_start`
    - You'll know this has been executed sucessfully when red and purple goal arrows appear in Rviz, as well as the robot's planned path
    - Also when the robots start moving (of course)
- Now sit back and watch the robots explore the bookstore!

#### Unknown Initial Robot Positions

![rviz_unknown_pos](https://user-images.githubusercontent.com/70979347/111922165-57d7f480-8a66-11eb-8364-e0724359a6b5.png)


- For unknown initial positions, the robots must spawn relatively close to eachother. This is because the multirobot_map_merging node needs a sufficinet amount of maps to overlap in order to use a feature detection algorithm to stitch the individial maps together. 
    - See the [multirobot_map_merge](http://wiki.ros.org/multirobot_map_merge) for more documentation.
- To run `source` your workspace and run `roslaunch multi_robot_exploration two_tb_exploration.launch known_initial_pos:=false first_tb3_x_pos:=-1.0 first_tb3_y_pos:=-6.0 second_tb3_x_pos:=-1.5 second_tb3_y_pos:=-6.0`
    - You can edit the initial position values directly in the launch file instead of you would like
- As with the known initial positions, you should see two Turtlebots spawn in Gazebo and Rviz
- Again, the robots require a start service be called before they start executing frontier exploration
- To start frontier exploration for both robots run `rosservice call /tb3_0_start` 
and `rosservice call /tb3_1_start`

### Frontier Exploration on an actual Turtlebot3
- Please see the [Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) on how to set up your Turtlebot if you haven't already
    - Note: This package was written to work with the `burger` Turtlebot model. If you would like to use another model, you will have to add / edit your own urdfs and any referance to the `burger` Turtlebot in the current launch / config files. 
- Start a `roscore` in your PC
- SSH into your Turtlebot and run `roslaunch turtlebot3_bringup turtlebot3_robot.launch`
- To start frontier exploration run `roslaunch single_tb_FE.launch` on a terminal on your PC. 
- Enjoy the soothing sights and sounds of your Turtlebot exploring real live frontiers

**Note: For best results, use your Turtlebot on smooth surfaces as opposed to carpet**

## Future Development
- As mentioned above, I am currently working on making this software more modular so that adding more robots to this operation is as simple as adding another namespace
- I am also working on running frontier exploration and map mergering with the software I have for two robots right now
- Build on the existing multirobot_map_merge node so that robots can always start from unknown initial positions. The idea is to have the robots start exploring without any knowledge of each other while in the background I run a feature matching algorthim. Once enough of two robot's map features match, I will create a merged map and then have robots run frontier exploration on the merged map. This way, both robots can continue searching frontiers without re-exploring areas that have already been covered. 
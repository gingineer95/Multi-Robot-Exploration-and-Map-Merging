Kailey Smith
# Multi-Robot Exploration and Map Merging

## Project Description
The goal is to use multiple robots to autonomously explore an environment and create one global merged map comprised of each's robot individual map. This can be achived with or without knwoledge of the robot's initial positions. 

Frontier Exploration was implemented as the autonomous navigation algorithm. Map merging was achieved by modifying the [multirobot_map_merge](http://wiki.ros.org/multirobot_map_merge) node. 

Currently this repository contains the software necessary to run frontier exploration and map merging of multiple robots in simulation using Gazebo. It also contains the software necessary to run frontier exploration on one actual Turtlebot3. Actively developing multi-robot exploration and map merging on actual Turtlebot3s. Also developing modularity for functionality with any number of robots. 

Note: The original multirobot_map_merge node was written to run gmapping, but for this package the gmapping functionality has been removed in favor of using `slam_toolbox`. To achieve map merging however, each robot's occupancy grid must be the same size and have the same origin. To compensate for this, the `map_expansion` node was written which creates a new, larger map and inserts `slam_toolbox`'s data into the new map. 

## 3rd Party Packages
- [Multirobot_map_merge](http://wiki.ros.org/multirobot_map_merge): Node that merges the individual maps into one gloval map
- [Aws-robomaker-bookstore-world](https://github.com/aws-robotics/aws-robomaker-bookstore-world): Gazebo world of a bookstore
- [Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) (navigation, gazebo, teleop and bringup)
- [Slam_toolbox](https://github.com/SteveMacenski/slam_toolbox): Used for simultaneous localization and mapping
- [Teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard): Generic keyboard teleop for twist robots
- [Gazebo_ros](http://wiki.ros.org/gazebo_ros) 

## Installation Instructions
-This package is called multi_robot_exploration

## Getting Started
### Multi-Robot Exploration and Map Merging
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
- For unknown initial positions, the robots must spawn relatively close to eachother. This is because the multirobot_map_merging node needs a sufficinet amount of maps to overlap in order to use a feature detection algorithm to stitch the individial maps together. 
    - See the [multirobot_map_merge](http://wiki.ros.org/multirobot_map_merge) for more documentation.
- To run `source` your workspace and run `roslaunch multi_robot_exploration two_tb_exploration.launch known_initial_pos:=false first_tb3_x_pos:=-1.0 first_tb3_y_pos:=-6.0 second_tb3_x_pos:=-1.5 second_tb3_y_pos:=-6.0`
    - You can edit the initial position values directly in the launch file instead of you would like
- As with the known initial positions, you should see two Turtlebots spawn in Gazebo and Rviz
- Again, the robots require a start service be called before they start executing frontier exploration
- To start frontier exploration for both robots run `rosservice call /tb3_0_start` 
and `rosservice call /tb3_1_start`

### Frontier Exploration on an actual Turtlebot3


## Future Work
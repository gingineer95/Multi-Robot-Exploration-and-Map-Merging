Kailey Smith
# Multi-Robot Exploration and Map Merging

## Description
The goal is to use multiple robots to autonomously explore an environment and create one global merged map comprised of each's robot individual map. This can be achived with or without knwoledge of the robot's initial positions. 

Frontier Exploration was implemented as the autonomous navigation algorithm. Map merging was achieved by modifying the [multirobot_map_merge](http://wiki.ros.org/multirobot_map_merge) node. 

Currently this repository contains the software necessary to run frontier exploration and map merging of multiple robots in simulation using Gazebo. It also contains the software necessary to run frontier exploration on one actual Turtlebot3. Actively developing multi-robot exploration and map merging on actual Turtlebot3s. 

## 3rd Party Packages
- [Multirobot_map_merge](http://wiki.ros.org/multirobot_map_merge): Node that merges the individual maps into one gloval map
- [Aws-robomaker-bookstore-world](https://github.com/aws-robotics/aws-robomaker-bookstore-world): Gazebo world of a bookstore
- [Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)(navigation, gazebo, teleop and bringup)
- [Slam_toolbox](https://github.com/SteveMacenski/slam_toolbox): Used for simultaneous localization and mapping
- [Gazebo_ros](http://wiki.ros.org/gazebo_ros)
- [Teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard): Generic keyboard teleop for twist robots. 


## Getting Started

## Future Work
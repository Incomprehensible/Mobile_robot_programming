# Mobile_robot_programming
Code for the course **700.240 (24S) Mobile Robot Programming** at the University of Klagenfurt.

## Assignment 1: ROS 2 Navigation and Control
Objectives:
• The robot shall navigate along a square with a side length of `5 m`.
• During the navigation, the robot shall continuously report its `6D pose` (position + orientation).
• The user shall be able to modify the robot’s `speed` with a bash command.
• Additional information on the `RPY angles` and total traveled `distance` shall be computed and extracted.

Used software:
• ROS 2 Humble
• Robot: Turtlebot 3
• Gazebo classic

For the detailed information regarding the code please see the [report](report.md).

## Instructions to run
### Dependencies
Install dependencies:
```zsh
home~$ sudo apt install ros-humble-navigation2
home~$ sudo apt install ros-humble-nav2-bringup
home~$ sudo apt install ros-humble-turtlebot3-gazebo
home~$ sudo apt install ros-humble-turtlebot3-teleop
```

Overwrite environment variables:
```zsh
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/model
```

### How to build
First go to the `src` directory:
```zsh
$ cd ~/mobile_robots_programming/src
$ colcon build
$ source install/setup.bash
```

### How to launch
To run the project with simulation and both the subscriber node and controller node running type the commands:
```zsh
$ ros2 run assignment_1 subscriber_node
$ ros2 launch assignment_1 controller.launch.py
```

# ROS Navigation Packages for Scout Mobile Base

## Packages

This repository contains navigation and simulation packages for scout robot. You will need packages from "scout_base" to use this repository. 

* scout_description: urdf definitions for scout-based mobile platforms
* scout_webots_sim: webots-based simulator for scout
* scout_navigation: a demonstration navigation setup for scout

## Basic usage of the ROS package

Please setup "scout_base" properly before proceeding to the following steps.

1. Install dependent libraries

    ```
    $ sudo apt-get install -y ros-$ROS_DISTRO-ros-controllers
    $ sudo apt-get install -y ros-$ROS_DISTRO-joint-state-publisher-gui
    $ sudo apt-get install -y ros-$ROS_DISTRO-navigation
    $ sudo apt-get install -y ros-$ROS_DISTRO-teb-local-planner
    $ sudo apt-get install -y ros-$ROS_DISTRO-webots-ros 
    ```

2. Clone the packages into your catkin workspace and compile

    (the following instructions assume your catkin workspace is at: ~/catkin_ws/src)

    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/westonrobot/scout_nav.git
    $ cd ..
    $ catkin_make
    ```

4. Launch ROS nodes
 
* Start the Webots-based simulation (Scout V1)

    ```
    $ roslaunch scout_bringup scout_base_webots_sim.launch
    ```

* Start the Gazebo-based simulation (Scout V2)

    ```
    $ roslaunch scout_bringup scout_base_gazebo_sim.launch
    ```

* Start the keyboard tele-op node

    ```
    $ roslaunch scout_bringup scout_teleop_keyboard.launch
    ```

# ROS Packages for Scout Mobile Base

## Packages

* scout_bringup: launch and configuration files to start ROS nodes 
* scout_base: a ROS wrapper around Scout SDK to monitor and control the robot
* scout_msgs: scout related message definitions
* (scout_robot: meta package for the Scout robot ROS packages)

## Basic Usage

1. Install dependent ROS packages

    ```
    $ sudo apt install ros-melodic-controller-manager
    $ sudo apt install ros-melodic-teleop-twist-keyboard
    ```

    Change ros-melodic-* in the command to ros-kinetic-* if you're using ROS Kinetic.

2. Clone the packages into your catkin workspace and compile

    (the following instructions assume your catkin workspace is at: ~/catkin_ws/src)

    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/westonrobot/scout_ros.git
    $ cd ..
    $ catkin_make
    ```

3. Launch ROS nodes
 
* Start the base node 

    ```
    $ roslaunch scout_bringup scout_minimal.launch
    ```
* Start the keyboard tele-op node

    ```
    $ roslaunch scout_bringup scout_teleop_keyboard.launch
    ```
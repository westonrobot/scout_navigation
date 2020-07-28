# ROS Packages for Scout Mobile Base

## Packages

* scout_bringup: launch and configuration files to start ROS nodes 
* scout_base: a ROS wrapper around Scout SDK to monitor and control the robot
* scout_msgs: scout related message definitions
* (scout_ros: meta package for the Scout robot ROS packages)

The following diagram may help you to understand how the components are inter-connected with each other:

<img src="./docs/diagram.png" height="135" >

The purple blocks represent ROS packages included within this repository.

## Communication interface setup

Please refer to the [README](https://github.com/westonrobot/wrp_sdk#hardware-interface) of "wrp_sdk" package for setup of communication interfaces.

## Basic usage of the ROS package

1. Install dependent libraries

    ```
    $ sudo apt install libasio-dev
    $ sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard
    ```

2. Clone the packages into your catkin workspace and compile

    (the following instructions assume your catkin workspace is at: ~/catkin_ws/src)

    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/westonrobot/wrp_sdk.git
    $ git clone https://github.com/westonrobot/scout_ros.git
    $ cd ..
    $ catkin_make
    ```

4. Launch ROS nodes
 
* Start the base node 

    ```
    $ roslaunch scout_bringup scout_minimal.launch
    ```

    or (if you're using a serial port)
        
    ```
    $ roslaunch scout_bringup scout_minimal_uart.launch
    ```

* Start the keyboard tele-op node

    ```
    $ roslaunch scout_bringup scout_teleop_keyboard.launch
    ```

    **SAFETY PRECAUSION**: 

    The default command values of the keyboard teleop node are high, make sure you decrease the speed commands before starting to control the robot with your keyboard! Have your remote controller ready to take over the control whenever necessary. 

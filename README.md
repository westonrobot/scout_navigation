# ROS Navigation Packages for Scout Mobile Base

## Packages

This repository contains navigation and simulation packages for scout robot. 


* scout_gazebo_sim: gazebo-based simulator (experimental)
* scout_webots_sim: webots-based simulator for scout
* scout_webots_nav_sample: a demonstration navigation setup for scout

## Basic usage of the ROS package

Please setup "[scout_base](https://github.com/westonrobot/scout_base.git)" properly before proceeding to the following steps.

1. Install dependent libraries

    ```
    $ sudo apt-get install -y ros-$ROS_DISTRO-ros-controllers \
                              ros-$ROS_DISTRO-joint-state-publisher-gui \
                              ros-$ROS_DISTRO-navigation \
                              ros-$ROS_DISTRO-gmapping \
                              ros-$ROS_DISTRO-pointcloud-to-laserscan \
                              ros-$ROS_DISTRO-teb-local-planner \
                              ros-$ROS_DISTRO-pcl-ros \
                              ros-$ROS_DISTRO-webots-ros
    ```

2. Clone the packages into your catkin workspace and compile

    (the following instructions assume your catkin workspace is at: ~/catkin_ws/src)

    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/westonrobot/scout_base.git
    $ git clone https://github.com/westonrobot/scout_navigation.git
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


## Sample Development

This sections outlines how you would get started on creating your own package for a scout platform with custom hardware extensions, such as a LIDAR and IMU. This package consist of the files required for both the simulation and the interface with the physical platform and sensors.

This package is based on:

1. A RPLIDARextension on the Platforms
2. Communication between computer and platform via CAN bus
3. Usage of webots for simulation (instead of gazebo)

### 1. Seting up package and directory
```
$ catkin_create_package ${YOUR_PACKAGE_NAME} scout_webots_sim geometry_msgs message_generation pcl_ros roscpp roslaunch rospy scout_base sensor_msgs std_msgs tf webots_ros pointcloud_to_laserscan scout_webots_sim
$ cd ${YOUR_PACKAGE_DIRECTORY}
$ mkdir simulation description/urdf rviz maps param

```

### 2. Creating webots Simulation
* Launch webots
* Create a new project directory in ${YOUR_PACKAGE_DIRECTORY}/simulation : Wizards -- New Project Directory -- Add a rectangle arena
* Copy the scout_v2.proto into the ${YOUR_PACKAGE_DIRECTORY}/simulation/protos directory
* Copy the scout_base_controller folder into the ${YOUR_PACKAGE_DIRECTORY}/simulation/
* Import the scout robot by adding a proto node: add -- PROTO nodes (Current Projects) -- ScoutV2(robot) -- add

### 3. Setting up Custom Extensions
Adding your own extension consists of 3 main steps:
1. Adding extension in webots model
2. Adding the transforms for the extension in the .xacro file
3. Writing the C++ code to interface between the webots extension and ROS

The example of adding a RPLIDAR module is explained in this example. 
#### webots model
1. Under ScoutV2, select extensionSlot and click <add> to add your desired extension
2. Create folder "${YOUR_PACKAGE_DIRECTORY}/simulation/exported_models"
3. Export model with extensions into the exported models folder
4. The exported model can now be imported into any world within the project.

#### .xacro
1. Create a new .xacro file
2. Import the base .xacro file of the scout model using: ```<xacro:include filename="$(find scout_description)/urdf/scout_v2.xacro" />```
3. Add the links and joints of the new extension

#### ROS/webots interface
1. Create a class "ExtensionExample" that inherits the WebotsExtension class
2. Define the setup protocol as shown in [lidar_extension.cpp](scout_webots_nav_sample/src/lidar_extension.cpp)


### 3. Exporting your new model to a different world
Note: this is to export the model to a new world but within the same project. If a new project directory is required, the new directory must be set up again

After the robot and its additional sensors has been modeled in the webots world, it is like that you would like to place this robots in different simulated environments, such as a workshop space or an indoor apartment. This sections brief outlines the main steps involved

1. Export the new robot model (with extensions) using the webots GUI:
   * Right-click the model node
   * Click Export
   * Save the model in a the folder ${YOUR_PACKAGE_DIRECTORY}/simulation/exported models
2. Load the world in webots you want to import the model into.
3. Add the export robot model
   * Add node
   * Import...
   * Select the model file that you exported in step 1

With this, you should be able to see the scout platform with its extension inside your chosen environment. 

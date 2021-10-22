# ROS Navigation Packages for Scout Mobile Base

## Packages

This repository contains navigation and simulation packages for scout robot. 

* scout_nav_platform: a demostrational navigation platform with lidar and IMU
* scout_ros_nav: a demonstrational ROS navigation setup for scout
* scout_webots_sim: webots-based simulation support for scout

## Basic usage

Please setup "[scout_base](https://github.com/westonrobot/scout_base.git)" repository properly before proceeding to the following steps.

### 1. Install dependent libraries

```
$ sudo apt-get install -y ros-$ROS_DISTRO-ros-controllers \
                          ros-$ROS_DISTRO-joint-state-publisher-gui \
                          ros-$ROS_DISTRO-navigation \
                          ros-$ROS_DISTRO-teb-local-planner \
                          ros-$ROS_DISTRO-gmapping \
                          ros-$ROS_DISTRO-pointcloud-to-laserscan \
                          ros-$ROS_DISTRO-pcl-ros \
                          ros-$ROS_DISTRO-webots-ros
                          ros-$ROS_DISTRO-teleop-twist-keyboard
```

### 2. Clone the packages into your catkin workspace and compile

(the following instructions assume your catkin workspace is at: ~/catkin_ws/src)

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/westonrobot/scout_ros.git
$ git clone https://github.com/westonrobot/scout_navigation.git
$ cd ..
$ catkin_make
```

### 3. Launch ROS nodes
 
* Start the Webots-based simulation

    ```
    $ roslaunch scout_webots_sim scout_base.launch
    ```

* Start the keyboard tele-op node

    ```
    $ roslaunch scout_bringup scout_teleop_keyboard.launch
    ```

* Start the mapping or navigation nodes

    ```
    $ roslaunch scout_nav_platform scout_sim_mapping.launch
    ```
    ```
    $ roslaunch scout_nav_platform scout_sim_navigation.launch
    ```
### 4. Saving and Loading of maps

* To save the map generated from mapping:
    ```
    rosrun map_server map_saver -f {YOUR_CATKIN_WORKSPACE}/src/scout_navigation/scout_ros_nav/maps/{YOUR_MAP_NAME}
    ```
    The map_server should be run after the map obtained is satisfactory and can be be used for navigation. The map will be saved as YOUR_MAP_NAME.yaml and YOUR_MAP_NAME.pgm


* To select the .yaml map file, edit the map_file value in the launch file (scout_sim_navigation.launch or scout_robot_navigation.launch)

    ```
    <arg name="map_file" default="$(find scout_ros_nav)/maps/webots_indoor.yaml"/>  
    ```

    should be changed to

    ```
    <arg name="map_file" default="${YOUR_CATKIN_WORKSPACE}/src/scout_navigation/scout_ros_nav/maps/{YOUR_MAP_NAME}.yaml"/>
    ```
    

## Custom Development

This sections outlines how you would get started on creating your own package for a scout platform with custom hardware extensions, such as a LIDAR and IMU. This package consist of the files required for both the simulation and the interface with the physical platform and sensors.

The demonstrational package scout_nav_platform has the following setup:

* A RPLIDAR extension on the platform
* Communication between computer and platform via CAN bus
* Usage of webots for simulation (instead of gazebo)

### 1. Seting up package and directory

```
$ catkin_create_package ${YOUR_PACKAGE_NAME} scout_webots_sim geometry_msgs message_generation pcl_ros roscpp roslaunch rospy scout_base sensor_msgs std_msgs tf webots_ros pointcloud_to_laserscan
$ cd ${YOUR_PACKAGE_DIRECTORY}
$ mkdir urdf rviz webots_setup
```

### 2. Creating webots simulation

* Launch webots
* Create a new project directory in ${YOUR_PACKAGE_DIRECTORY}/webots_setup : Wizards -- New Project Directory -- Add a rectangle arena
* Copy the scout_v2.proto into the ${YOUR_PACKAGE_DIRECTORY}/webots_setup/protos directory
* Copy the scout_base_controller folder into the ${YOUR_PACKAGE_DIRECTORY}/webots_setup/
* Import the scout robot by adding a proto node: add -- PROTO nodes (Current Projects) -- ScoutV2(robot) -- add

### 3. Setting up custom extensions

Adding your own extension consists of 3 main steps:

* Adding extension in webots model
* Adding the transforms for the extension in the .xacro file
* Writing the C++ code to interface between the webots extension and ROS

The example of adding a RPLIDAR module is explained in this example.
 
#### webots model

* Under ScoutV2, select extensionSlot and click <add> to add your desired extension
* Create folder "${YOUR_PACKAGE_DIRECTORY}/webots_setup/models"
* Export model with extensions into the exported models folder
* The exported model can now be imported into any world within the project.

#### urdf/xacro model

* Create a new .xacro file
* Import the base .xacro file of the scout model using: ```<xacro:include filename="$(find scout_description)/urdf/scout_v2.xacro" />```
* Add the links and joints of the new extension

#### ROS/webots interface

* Create a class "ExtensionExample" that inherits the WebotsExtension class
* Define the setup protocol as shown in [lidar_extension.cpp](scout_ros_nav/src/lidar_extension.cpp)

### 4. Exporting your new model to a different world

Note: this is to export the model to a new world but within the same project. If a new project directory is required, the new directory must be set up again

After the robot and its additional sensors has been modeled in the webots world, it is like that you would like to place this robots in different simulated environments, such as a workshop space or an indoor apartment. This sections brief outlines the main steps involved

* Export the new robot model (with extensions) using the webots GUI:
   * Right-click the model node
   * Click Export
   * Save the model in a the folder ${YOUR_PACKAGE_DIRECTORY}/simulation/exported models
* Load the world in webots you want to import the model into.
* Add the export robot model
   * Add node
   * Import...
   * Select the model file that you exported in step 1

Having finished the above steps, you should be able to see the scout platform with its extensions inside your chosen environment. 

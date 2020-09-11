# scout_simulation

Weston Robot ROS - Scout Simulation



# Sample Development

## 1. Seting up package and directory
```
$ catkin_create_package ${YOUR_PACKAGE_NAME} roslaunch geometry_msgs roscpp rospy scout_base sensor_msgs std_msgs message_generation tf webots_ros pcl_ros
$ cd ${YOUR_PACKAGE_DIRECTORY}
$ mkdir simulation description/urdf

```

## 2. Creating webots Simulation
* Launch webots
* Create a new project directory in ${YOUR_PACKAGE_DIRECTORY}/simulation : Wizards -- New Project Directory -- Add a rectangle arena
* Copy the scout_v2.proto into the ${YOUR_PACKAGE_DIRECTORY}/simulation/protos directory
* Copy the scout_base_controller folder into the ${YOUR_PACKAGE_DIRECTORY}/simulation/
* Import the scout robot by adding a proto node: add -- PROTO nodes (Current Projects) -- ScoutV2(robot) -- add

## 3. Setting up Custom Extensions
Adding your own extension consists of 3 main steps:
1. Adding extension in webots model
2. Adding the transforms for the extension in the .xacro file
3. Writing the C++ code to interface between the webots extension and ROS

The example of adding a RPLIDAR module is explained in this example. 
### webots model
1. Under ScoutV2, select extensionSlot and click <add> to add your desired extension
2. #TODO

### .xacro
1. Create a new .xacro file
2. Import the base .xacro file of the scout model using: ```<xacro:include filename="$(find scout_description)/urdf/scout_v2.xacro" />```
3. Add the links and joints of the new extension

### ROS/webots interface
1. Create a class "ExtensionExample" that inherits the WebotsExtension class
2. #TODO


## Exporting your new model to a different world
Note: this is to export the model to a new world but within the same project. If a new project directory is required, the new directory must be set up again

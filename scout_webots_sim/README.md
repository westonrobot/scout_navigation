# Scout Webots Sim

This ROS package is aimed at interfacing ROS controller on webots as a ROS node.

Within webots, there is a in-built controller called "ROS controller", Through this controller, ROS services can be called. 

However, for most applications, using ROS nodes (instead of calling individual services) acts as an easier interface for users. 

This package essentially creates a node that calls the ROS services to interact with the webots controller.


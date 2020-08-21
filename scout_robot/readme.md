The scout robot simulated in gazebo.

System environment:

Ubuntu 16.04
ros kinetic
gazebo7

Required configuration environment:

sudo apt-get install ros-kinetic-navigation
sudo apt-get install ros-kinetic-slam-gmapping
sudo apt-get install ros-kinetic-openslam-gmapping
sudo apt-get install ros-kinetic-teleop-twist-keyboard
sudo apt-get install ros-kinetic-teb-local-planner

Start the robot world:

roslaunch scout_robot scout_world.launch

If you can't open the gazebo,you can see this blog: 
https://blog.csdn.net/qq_37427972/article/details/82853655

And don't forget to install the graphics driver.

We can teleop the robot by using this command:

rosrun teleop_twist_keyboard teleop_twist_keyboard.py 

Start the gmapping:

roslaunch scout_robot gmapping.launch

After finishing to build the map,we can save the map data by using this:

rosrun map_server map_saver -f map

Start the navigation:

roslaunch scout_robot navigation.launch

Then,we can let the robot to the navigation goal by using the 2D Pose Estimate in RVIZ.
The moredetails in teb_local_planner,you can see this website:
http://wiki.ros.org/teb_local_planner/Tutorials

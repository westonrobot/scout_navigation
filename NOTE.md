```
$ sudo apt-get install ros-kinetic-joint-state-controller
$ sudo apt-get install ros-kinetic-effort-controllers
$ sudo apt-get install ros-kinetic-position-controllers
```

```
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=scout_robot/cmd_vel
```

xacro to URDF
```
$ rosrun xacro xacro scout.urdf.xacro > agilex_scout.urdf
```

URDF to PROTO
```
$ python urdf2webots.py --input=someRobot.urdf [--output=outputFile] [--box-collision] [--normal]
```

```
$ rostopic pub -1 /scout_motor_fl_controller/command std_msgs/Float64 "data: 0.5"
```

Convert xacro to urdf

```
$ rosrun xacro xacro -o model.urdf model.urdf.xacro
```

Convert urdf to sdf

```
$ gz sdf -p scout_v2.urdf > scout_v2.sdf
```
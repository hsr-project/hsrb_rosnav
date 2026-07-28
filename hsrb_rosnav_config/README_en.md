# hsrb_rosnav_conv

## Quick start

Launch gazebo simulator:

```
$ roslaunch hsrb_gazebo_launch hsrb_mock_home_world.launch use_manipulation:=false use_navigation:=false use_perception:=false use_task:=false use_teleop:=false use_web:=false use_laser_odom:=false
```

NOTE: The original hsrb_gazebo_launch/hsrb_mock_home_world.launch runs
may nodes unnecessary for the navigation. If you feel painful to check
navigaiotn with this world, please use the following launch file.

```
$ roslaunch hsrb_rosnav_config simple_gazebo_world.launch
```

Launch navigation system:

```
$ roslaunch hsrb_rosnav_config hsrb_nav.launch
```

## Run rostest

You can use rostest to check the package works correctly in gazebo.

```
$ rostest hsrb_rosnav_config hsrb_navigation.test gui:=true rviz:=true
```

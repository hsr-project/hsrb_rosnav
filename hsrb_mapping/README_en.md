# hsrb_mapping

## Quick start

Launch gazebo simulator:

```
$ roslaunch hsrb_gazebo_launch hsrb_mock_home_world.launch use_manipulation:=false use_navigation:=false use_perception:=false use_task:=false use_teleop:=false use_web:=false use_laser_odom:=false
```

Launch teleop_joy:

```
$ roslaunch hsrb_mapping teleop_joy.launch
```

Launch gmapping:

```
$ roslaunch hsrb_mapping gmapping.launch
```




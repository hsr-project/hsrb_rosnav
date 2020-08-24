
# Troubleshooting

### ROS environment confirmation

In case you do not know if the ROS environment is configured correctly or which workspace is in use now, first, please try to use the `roscd` command.

Although `roscd` is a command to move to a package workspace directory, if you execute it without any arguments it will move you to the top of the workspace.

```
$ roscd
$ pwd
/home/<username>/catkin_ws/devel
```

If it is displayed like the above, then `~/catkin_ws` is set as the workspace.

```
$ roscd
$ pwd
/opt/ros/\RosDistro
```

If it is displayed like the above, then the ROS system is set, but the workspace is not set.

```
$ roscd
roscd: command not found
```

If it is displayed like the above, then the ROS system itself is not set.  In either case, please carry out the ROS system configuration and environment configuration by executing the workspace configuration file `~/catkin_ws/devel/setup.bash`.

In addition, you can even check the ROS environment variables by executing `env`.

```
$ env | grep ROS
```

The following execution example is an environment configuration when using a simulator on a development PC.  In the case of an environment configuration that uses a real robot, `ROS_MASTER_URI` and such differs.

```
$ env | grep ROS
ROS_ROOT=/opt/ros/kinetic/share/ros
ROS_PACKAGE_PATH=/home/robotuser/catkin_ws/src:/opt/ros/kinetic/share
ROS_MASTER_URI=http://localhost:11311
ROSLISP_PACKAGE_DIRECTORIES=/home/robotuser/catkin_ws/devel/share/common-lisp
ROS_DISTRO=kinetic
ROS_IP=192.168.64.6
ROS_HOME=/home/robotuser/.ros
ROS_ETC_DIR=/opt/ros/kinetic/etc/ros
```

- Main confirmation items
  - `ROS_DISTRO` : Is the ROS version correct?
  - `ROS_PACKAGE_PATH` : Is the ROS package path appropriate?
  - `ROS_MASTER_URI` : Does the ROS Master URI configuration indicate the correct master?

If we assume `env | grep GAZEBO`, then we can check the Gazebo model file path.

```
$ env | grep GAZEBO
GAZEBO_MODEL_PATH=/opt/ros/kinetic/share/tmc_gazebo_worlds/models:/opt/ros/kinetic/share/tmc_gazebo_worlds/models:/opt/ros/kinetic/share/tmc_gazebo_worlds/models:/opt/ros/kinetic/share/tmc_gazebo_worlds/models
```

## Gazebo related troubleshooting

### 3D model is not displayed at Gazebo startup

Immediately after installing the software or immediately after booting Ubuntu, if you start up Gazebo, then sometimes, without displaying the 3D model, the 3D model display area remains black.


#### Solution #1 : Wait a few minutes

In general, although Gazebo reads a model at the time it first starts up, sometimes it may take time.  Sometimes the solution is to wait a few minutes.


#### Solution #2 : Restart Gazebo

Gazebo may sometimes fail to display a 3D model.  It may come to be displayed by stopping all of the ROS nodes, then restarting Gazebo again.


#### Solution #3 : Try executing rostest

The HSR robot and the environment model may come to be displayed by Gazebo without starting the simulation by executing the following rostest commands.

```
$ rostest hsrb_mapping hsrb_mapping.test gui:=true rviz:=true
```

Or

```
$ rostest hsrb_rosnav_config hsrb_navigation.test gui:=true rviz:=true
```

#### Solution #4 : Try only executing Gazebo

Confirm whether you can execute Gazebo using the minimum configuration.

```
$ roslaunch gazebo_ros empty_world.launch
```

![Gazebo - Empty World](images/hsrb-gazebo_empty-world.png)

If this kind of grid is not displayed, then it is thought that the computer's specifications are too low.  In the case of a virtual computer, please examine its configuration and the execution on the real computer.


### TF is not published by Gazebo

#### Solution #1 : Click on the Gazebo "Play" button

If it is the case that `hsrb_mock_home_world.launch` of `hsrb_gazebo_launch` is started up using the HSR Gazebo simulator, then the Gazebo simulator starts in a default state and does not run.

```
roslaunch hsrb_gazebo_launch hsrb_mock_home_world.launch
```

Click on the "Play" button in the lower left of Gazebo, then the simulation starts running.


![HSR Gazebo - Play Button](images/hsrb-gazebo_play-button.png)


## RViz related troubleshooting

### The whole body of the robot is displayed in white in RViz

As for the state where the whole body of the robot is displayed in white in RViz, although "Fixed Frame" is `/map`, it is thought to be the case that `/map` is not published.

![HSR RViz - White Robot without /map frame](images/hsrb-rviz_white-robot_without-map.png)

#### Solution #1 : Start a node that publishes `/map`

Please execute a program to publish `/map`.

If making a map, then execute the following command.

```
roslaunch hsrb_mapping gmapping.launch
```

If performing autonomous movement, then execute the following command.

```
roslaunch hsrb_rosnav_config hsrb_nav.launch
```

#### Solution #2 : In the case that `/map` is not used, change "Fixed Frame"

Although in this tutorial, we basically use `/map`, at other times, if it is the case that we do not use `/map` and not publish it, please change "Fixed Frame" to `/base_link` and such.


## Joystick controller related troubleshooting

### Cannot move the robot using the joysticks

#### Solution #1 : While pressing the "Enable" button, move the joysticks

It is necessary to move a joystick while pushing the "Enable" button to send speed commands to the robot from the joystick controller.

In the case of an Xbox 360 compatible (XInput) controller, the "L1" button (the 8th button) is the "Enable" button.


#### Solution #2 : Confirm the configuration and operating environment of the joystick controller

Confirm whether the joystick is recognized as a device.  If it is recognized, then usually the result of `ls /dev/input/` will include `js0`.

```
$ ls /dev/input/
by-id  by-path  event0  event1  event2  event3  event4  js0  mice  mouse0
```

If the joystick is recognized as a device, then check its operation.


**Terminal #1**

```
$ roscore
```

**Terminal #2**

```
$ roslaunch hsrb_mapping teleop_joy.launch
```

**Terminal #3**

```
$ rostopic list
/diagnostics
/hsrb/command_velocity
/joy
/rosout
/rosout_agg
```

Because there should be a `/joy` topic, confirm the movement using `echo`.

When you move the joysticks, the numerical value `axes:` changes.

When you press the buttons, the numerical value `buttons:` changes.  In particular, please check that button#10 (the buttons are in an array starting with the 0th element on the left to the 11th numerical value) becomes 1.

```
$ rostopic echo /joy
header: 
  seq: 2136
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: ''
axes: [-0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, 0.09228315204381943, 0.0]
buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

---
:
:

:
:
header: 
  seq: 2137
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: ''
axes: [-0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, 0.08955255150794983, 0.0]
buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
---
:
:

:
:
header: 
  seq: 2138
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: ''
axes: [-0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, 0.08955255150794983, 0.0]
buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0]
---
:
:

```





<!-- EOF -->

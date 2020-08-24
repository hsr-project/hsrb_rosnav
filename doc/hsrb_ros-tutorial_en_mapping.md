
# Map creation

Create an environment space map using the HSR ROS map creation package hsrb_mapping.


## Software startup

### In the case of a simulator

Start up the HSR's Gazebo simulator and RViz.

**Terminal #1**

```
$ roslaunch hsrb_rosnav_config simple_gazebo_world.launch gui:=true rviz:=true
```

![HSR Gazebo - Start simple_gazebo_world](images/hsrb-gazebo_start_simple_gazebo_world.png)

Sometimes, the first time Gazebo is started an error is output and the model is not displayed.  In that case, stop Gazebo, then please start the software again.

- Reference: [Gazebo related troubleshooting](hsrb_ros-tutorial_en_trouble-shooting.md#gazebo-trouble-shooting)
    - [hsrb_ros-tutorial_en_trouble-shooting.md#gazebo-trouble-shooting](hsrb_ros-tutorial_en_trouble-shooting.md#gazebo-trouble-shooting)


Start up the map generation program from another terminal.

**Terminal #2**

```
$ roslaunch hsrb_mapping hector.launch
```

RViz, which started the map generation program, is in the following state.

![HSR RViz - Begin Mapping](images/hsrb-rviz_mapping-hector_start.png)


- Reference: [RViz related troubleshooting](hsrb_ros-tutorial_en_trouble-shooting.md#rviz-trouble-shooting)
    - [hsrb_ros-tutorial_en_trouble-shooting.md#rviz-trouble-shooting](hsrb_ros-tutorial_en_trouble-shooting.md#rviz-trouble-shooting)


### In the case of a real robot

Carry out each operation using the ROS environment that started the ROS master on the real robot.  For that reason, please first input into each terminal `hsrb_mode` changing the ROS environment.

- **Memo #1** : Change the ROS master to the real robot using `hsrb_mode`.
- **Memo #2** : To return to the simulation environment, please execute `exit` or `sim_mode`.

Also, because the ROS node `/pose_integrator` that is executed inside the real robot conflicts with a node inside of the map creation process
that is processed after this, it is necessary to stop `/pose_integrator` using the following commands.

**Terminal #1**

```
$ hsrb_mode
$ rosnode kill /pose_integrator
```

Because RViz still has not started up, it is OK to start it using the following command in the same terminal as the above.

```
$ rviz -d $(rospack find hsrb_rosnav_config)/launch/hsrb.rviz
```

In the above configuration, we exchanged "In the case of simulation" section's "Gazebo simulator" with "The real robot and the real environment".  Then we are in a state where we are monitoring that environment through RViz.

Afterwards, start the map generator program from another terminal in the same way as was done with the simulation.  However, please do not forget to make the real robot the ROS master using `hsrb_mode`.

**Terminal #2**

```
$ hsrb_mode
$ roslaunch hsrb_mapping hector.launch
```



## Map generation

The map is created while moving the robot in the environment.  Using the laser sensor on the HSR's base it detects obstacles in the space, then constructs and generates a map from that information.

There are a few ways to move the robot.

- Movement via RQT Robot Steering.
- Movement via a joystick game pad.
- Movement via keyboard.

Please move the robot in the environment space, and complete the map.

- **Warning #1** : In the case of a real robot, please execute `hsrb_mode` when you start each terminal.
- **Warning #2** : In the case of a real robot, please execute it while carefully verifying the movement enviroment and situation.

### Movement operations by means of RQT Robot Steering

Operate the robot using RQT Robot Steering.
First, start rqt.

**Terminal #3**

```
$ rqt
```

Next, choose **Robot Tools** > **Robot Steering** from Plugins found in the rqt menubar.

![HSR RQT - Plugin - Robot Steering](images/hsrb-rqt_plugin_robot-steering.png)

Set the movement velocity topic `/hsrb/command_velocity` into the text box.

![HSR RQT - Robot Steering /hsrb/command_velocity](images/hsrb-rqt_robot-steering_set-topic.png)

Move the robot.

- The vertical slider : Forward and backward velocity commands.
- The horizontal slider : Rotational angular velocity commands.

When the robot moves, the range where the laser sensor reaches changes, and the map gradually widens.  Move the robot appropriately in the environment space and complete the map.

![HSR RViz - End Mapping](images/hsrb-rviz_mapping-hector_end.png)

### Movement operations by means of a joystick game controller

Connect a game controller with joysticks to the Ubuntu PC.

Use teleop_joy.launch prepared in the hsrb_mapping package.  It supports joystick controllers compatible with the Xbox 360 (XInput compliant).

**Terminal #3**
```
$ roslaunch hsrb_mapping teleop_joy.launch
```

The operation method for the joystick is as follows.

- While pushing the left shoulder button (L1)
	- Left-stick / up-down : Forward and backward velocity commands.
	- Left-stick / left-right : Left and right velocity commands.
	- Right-stick / left-right : Rotational angular velocity commands.

Please move the robot using the joysticks in the environment space.

- Reference : [Joystick related troubleshooting](hsrb_ros-tutorial_en_trouble-shooting.md#joystick-trouble-shooting)
    - [hsrb_ros-tutorial_en_trouble-shooting.md#joystick-trouble-shooting](hsrb_ros-tutorial_en_trouble-shooting.md#joystick-trouble-shooting)


### Movement operations by means of a keyboard

You can set movement velocity command values with a keyboard.  Use teleop_keyboard.launch prepared in the hsrb_mapping package.

If it is the case that the necessary package for keyboard control is not installed, then install it using the following command.

```
$ sudo apt-get install ros-kinetic-teleop-twist-keyboard
```

Start teleop_keyboard.launch in a terminal.

**Terminal #3**

```
$ roslaunch hsrb_mapping teleop_keyboard.launch
```

The following kind of screen is displayed in the terminal.

```
:
:
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit

currently:	speed 0.5	turn 1
```

The main movement controls for the keyboard are as follows.  It is necessary to continuously press a key to continue sending a speed command, and the speed command value returns to 0 when you release its key.

- `i` : Move forwards
- `,` : Move backwards
- `j` : Turn left
- `l` : Turn right
- `J` : Strafe left（Capital `J` / while pressing the shift key）
- `L` : Strafe right (Capital `L` / while pressing the shift key）
- `q` : Increase the speed command value by 10%
- `z` : Decrease the speed command value by 10%

Please move the robot with the keyboard while looking at the RViz screen.  It is necessary that the terminal window has focus during keyboard input.


## Saving a map

To save the map that was completed by moving the robot, execute the following command.

**Terminal #4**
```
$ rosrun map_server map_saver
```

- **Warning** : The names `map_server` and `map_saver` are different, please be careful because they are easy to mix up.

Execution results:

```
$ rosrun map_server map_saver
[ INFO] [1514984244.267861644]: Waiting for the map
[ INFO] [1514984244.487766203]: Received a 192 X 192 map @ 0.050 m/pix
[ INFO] [1514984244.492864163]: Writing map occupancy data to map.pgm
[ INFO] [1514984244.503525258]: Writing map occupancy data to map.yaml
[ INFO] [1514984244.509132104]: Done
```

The following two files are saved to the current directory.

- map.pgm
- map.yaml

Because map.pgm is a regular image file, it can be edited using ordinary image editor software like GIMP and such.

Because these files will be used in "Navigation - autonomous movement" in the next section, we will place them in a location that is easy to specify from ROS.

```
$ mv map.pgm map.yaml ~/.ros/
```


## Stopping the software

When you have confirmed that the map has been written out, stop the programs that were used for map generation.  Stop them by entering Ctrl-C into each terminal that started a program.

- Program to be terminated
  - Map generation program : hector.launch
  - The program that is used for robot movement
    - RQT Robot Steering : rqt
    - The joystick program : teleop_joy.launch
    - The keyboard control program : teleop_keyboard.launch

Because Gazebo and RViz will be used even in the next section ("Navigation"), it is fine to just leave them running.  If you want to stop them, then in their terminal, in the same way as with the other programs, enter Ctrl-C and Gazebo and all of the nodes for RViz will stop.


<!-- EOF -->

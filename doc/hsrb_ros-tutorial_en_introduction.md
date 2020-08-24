
# Introduction

## About this tutorial

This tutorial explains the usage of the Toyota Partner Robot HSR ROS packages inside of hsrb_rosnav:

- Map creation package : hsrb_mapping
- Autonomous movement package : hsrb_rosnav_config

The structure of this tutorial is shown below:

- [Preface (This document)](hsrb_ros-tutorial_en_introduction.md)
- [Map creation](hsrb_ros-tutorial_en_mapping.md)
- [Autonomous movement](hsrb_ros-tutorial_en_navigation.md)
- [Troubleshooting](hsrb_ros-tutorial_en_trouble-shooting.md)


## Software install

### System requirements

The system requirements to use this tutorial are listed below:

- Ubuntu 16.04 64bit
- ROS Kinetic


### ROS and HSR software install

The ROS and HSR software installation is at the URL below.
Please follow "**Setting up the development PC**" from the "**HSR Manual**".

If you have already installed this software, then please proceed to the next step, "**hsrb_ros package install**".

- ROS Kinetic installation to Ubuntu
 - [http://wiki.ros.org/kinetic/Installation/Ubuntu][ab085263]

  [ab085263]: http://wiki.ros.org/kinetic/Installation/Ubuntu "ROS Kinetic Installation on Ubuntu"

- HSR Manual
  - Docs » 6. How to use the HSR » 6.2. Setup for using the HSR » 6.2.1. Setting up the development PC
  - [https://docs.hsr.io/manual_en/howto/pc_install.html][29d6e905]

  [29d6e905]: https://docs.hsr.io/manual_en/howto/pc_install.html "HSR - PC Install"


### hsrb_ros package install

Install the ROS map creation and autonomous movement packages for the Toyota HSR2015 (hsrb).

First, install rosinstall and wstools.

```
$ sudo apt-get update
$ sudo apt-get install python-wstool
```

Please make a new workspace for this tutorial as follows.
Here we will set the name to `catkin_ws`.

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ wstool init src
$ wstool set -t src --git hsrb_ros https://github.com/tork-a/hsrb_ros.git
$ wstool update -t src
$ rosdep update && rosdep install -y -r --from-paths src --ignore-src
$ catkin_make
$ source devel/setup.bash
```

Because some of the commands can be long, a newline may be inserted by the display environment, but please input everything following the `$` as a continuous command.

- wstool : The tool which automatically gets and unpacks from GitHub and other source code into a workspace.
  - [http://wiki.ros.org/wstool](http://wiki.ros.org/wstool)
- rosdep : Command to automatically install the system packages that the source code
depends on.
  - [http://wiki.ros.org/ROS/Tutorials/rosdep](http://wiki.ros.org/ROS/Tutorials/rosdep)

With the above the basic installation the software is complete.

If you want to check if ROS and the workspace are correctly installed and configured, please refer to "**Checking the ROS environment**" in "**Troubleshooting**".

- Reference: [Checking the ROS environment](hsrb_ros-tutorial_en_trouble-shooting.md#checking-ros-environment)
  - [hsrb_ros-tutorial_en_trouble-shooting.md#checking-ros-environment](hsrb_ros-tutorial_en_trouble-shooting.md#checking-ros-environment)


#### Configuration to execute setup.bash automatically at terminal startup

Whenever you open a terminal, it is necessary to execute `source devel/setup.bash`.  Because it is troublesome to execute this command every time,
we will set it so that the `source` command is automatically executed when the terminal is started.

Execute the following commands, then the `source devel/setup.bash` command will be added to the end of the `~/.bashrc` file.

- **Warning**: If you make the `>>` into `>` in the following commands, you will overwrite the `~/.bashrc` file deleting your configuration, so please be careful.

```
$ echo "### For HSR ROS Navigation" >> ~/.bashrc
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
It is a general practice to change the ROS workspace depending on the inidividual project.  At the end of this tutorial, we recommend that you revert the configuration that uses `catkin_ws` mentioned above.  Please delete the two lines that were added to the end of the `~/.bashrc` file.

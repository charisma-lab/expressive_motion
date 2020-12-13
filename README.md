# Expressive Motion in ROS
The goal of this repository is to implement expressive motion in robots using laban settings. 

This ROS1 package has been tested on ROS Version **melodic** installed as `ros-desktop-full-install` on Ubuntu 18.04.4(bionic) on the laptop + Raspberry Pi 4 Model B. This package was tested on a Neato Robot.

## Setup

### Install packages:

#### Standard ROS Packages
* move-base
* map-server

#### CHARISMA packages 

**On raspberry pi:**
* https://github.com/charisma-lab/neato_controller (7dwarves branch)

**On laptop:**
* https://github.com/charisma-lab/expressive_motion (devel branch)

#### How to install?
* `git clone` them in `catkin_ws/src`
* From `catkin_ws` directory , run `catkin_make` followed by `catkin_make install`

### Hardware Setup

For localization purposes we use Aruco markers and an overhead camera. 2 Aruco markers are placed on the edges of a desired test area to landmark the corners. Aruco markers are placed on top of the robot as well as the target (human) to detect the positions of both of them w.r.t the boundaries.

The robot is connected to a Raspberry Pi to control its functionalities. We use the Neato Botvac D robot and a logitech overhead camera.

#### Print and place aruco markers:

Using [https://chev.me/arucogen/](https://chev.me/arucogen/) print Aruco Markers, 1, 5 and `obstacle` of size 4x4 and size at least 200. 
Place the aruco markers - `1` on the robot, `5` on the hat (the object which will go on person) and `obstacle` to mark one corner of the area which the robot will be operating in.

### Setting up the environment
On the laptop, as well as the raspberry pi make sure you have the following variables set-up in `~/.bashrc` file:

* NEATO_NAME
* ROS_MASTER_URI
* NEATO_PORT

### ROS network setup on multiple machines
You will need to connect the robot and the laptop to the same network and then run a ros master-slave configuration between them. [ROS network setup](http://wiki.ros.org/ROS/NetworkSetup)


## Running the system:

You will need multiple terminals. Make sure not to confuse the laptop's terminals with the ones where you are running an SSH session with the raspberry pi

#### Localizing the robot
0. Edit `catkin_ws/src/neato_localization/scripts/tracking_aruco_markers.py` to change camera number, on the laptop.
1. In first terminal, run `roslaunch neato_localization neato_localization.launch`. This is to start localization package. Using the overhead camera it will start the localizer and publish poses and orientation of all the markers it sees, on the laptop.
2. To visualize if every marker and robot is positioned correctly, in a new terminal on the laptop, run `rviz` and the select the appropriate pose topics.

#### Activating the controllers on the robot
1. In first terminal, on the raspberry pi, run `roslaunch neato_controller bringup.launch`. This is to start neato robot's low level driver. 

#### Robot motion pattern generation
1. In second terminal, on the laptop, run `roslaunch neato_localization generate_waypoints_for_motion.launch`. This is to generate path for a particular emotion. This expects user's input on the `/TSI_values` topic in order to generate a motion pattern for the robot to follow.
2. In third terminal, on the laptop, run `rosrun neato_localization test_gui.py`. This rosnode is for the user to set inputs to the system.
The user can choose to either provide the inputs via terminal commands or via the GUI. For the inputs the user can either choose time-space-interaction values or check a robot state of his choice.

* Input via terminal
Run `rosrun neato_localization test_gui.py -t 0.1 -s 0.2 -i 0.3` for setting time-space-interaction values via terminal commands
Run `rosrun neato_localization test_gui.py -state happy` for setting robot-state via terminal commands. You can choose happy/grumpy/sleepy for robot states.

* Input via GUI
Run `rosrun neato_localization test_gui.py`. This will bring up the GUI. You can choose your inputs and then hit deploy.

All these user inputs will be published on the `/TSI_values` topic.

#### Moving the robot
1. In second terminal, on the raspberry pi, run `roslaunch neato_planner expressive_motion.launch`. The path generated from generate_waypoints_for_motion, will now be fetched by this node and then implemented by the robot.


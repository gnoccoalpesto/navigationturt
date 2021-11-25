IRM PROJECT - TURTLEBOT3 NAVIGATION in closed environment

TASK1) simulation of the environment

TASK 2) autonomous exploration and mapping w/ slam

TASK 3) autonomous navigation using map and Navigation Stack

TASK4) autonomous path creation and cleaning of a room with simulated UV light


--INSTALLATION--

import this package in a workspace containing turtlebot3 package

Install XTERM terminal emulator to visualize nodes' dialogs windows

run this command to create an ENV variable with turtle bot model:

export TURLTEBOT3_MODEL=burger


tested on (dockerized) ubuntu 18.04, ROS melodic, python 2.7 


--USAGE--

roslaunch navigationturt taskK.launch

substitute K with the correspective number {1,2,3,4} of the chosen task


--ROSLAUNCH ARGUMENTS--

Here follows a list of arguments that can be modified in tasks' launchers.

TASK 1, 2, 3, 4

model: use doc to properly choose it, by default selected from ENV variable

x_pos, y_pos: spawning position's coordinates


TASK 2, 3, 4

gui: if true, opens Gazebo window; default false

open_rviz: if true, opens Rviz window; default true

move_forward_only: if true, robot can't go backward; default false


TASK 2

slam_methods: slam algorithm used; default gmapping


TASK 2, 4

teleoperation_mode: if true, manual control is activated; default false


TASK 3, 4

map_file: location on map.yaml file; default automatically found by name

TASK 3

objectves_file_location: location of .txt file containing navigation objectives; default automatically found by name



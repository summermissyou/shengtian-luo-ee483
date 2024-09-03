#!/bin/bash


source /environment.sh

source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash --extend
source /code/submission_ws/devel/setup.bash --extend

dt-exec-BG roslaunch --wait agent agent_node.launch
dt-exec-FG rosrun dt-joystick-demo dt-joystick-demo-node.py

copy-ros-logs
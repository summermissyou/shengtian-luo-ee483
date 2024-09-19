#!/bin/bash

source /environment.sh

source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash --extend
source /code/submission_ws/devel/setup.bash --extend

dt-exec-FG roslaunch --wait agent agent_node.launch
copy-ros-logs
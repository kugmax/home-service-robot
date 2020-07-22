#!/bin/sh

xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch " & sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch " & sleep 5
xterm -e " roslaunch rvizConfig rvizConfig.launch " & sleep 5
xterm -e " rosrun pick_objects pick_objects " & sleep 5
xterm -e " rosrun add_markers add_markers "

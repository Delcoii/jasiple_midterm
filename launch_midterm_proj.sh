#! /bin/bash

gnome-terminal -e "roslaunch turtlecar turtlecar.launch" & sleep 2
gnome-terminal -e "rosrun turtlesim turtlesim_node"
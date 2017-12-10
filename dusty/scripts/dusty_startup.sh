#!/bin/sh

source /opt/ros/kinetic/setup.zsh
source /home/pi/ros/devel/setup.zsh
export ROS_MASTER_URI=http://192.168.0.180:11311
export ROS_IP=192.168.0.180
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/pi/ros/src:/opt/ros/kinetic/share

roslaunch dusty dusty.launch

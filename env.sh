#!/bin/bash
# This is an example file if you want to have a separate "control computer" to run the realtime loop
# of the gravity compensation separate from the computer you're calling roslaunch from.
# If you do have such a computer, that computer will need to have a file like this on it
# so that that computer knows how to source relevant ROS definitions and where to find roscore.

# set the ROS_MASTER_URI to my laptop machine (no reason to run rosmaster on the realtime computer)
export ROS_MASTER_URI="http://helvellyn:11311"
# source the kortex_ros_control_ws workspace (the downloaded and built catkin workspace on the control computer)
source ~/gravity_comp_ws/devel/setup.bash
exec "$@"

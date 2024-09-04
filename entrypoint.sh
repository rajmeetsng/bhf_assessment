#!/bin/bash

source install/setup.bash 
exec ros2 run bhf_vo_new bhf_vo_new src/


#set -e

# Source the ROS 2 environment (if necessary)
#source /opt/ros/humble/setup.bash

# Source the workspace (if necessary)
#source /bhf_assessment/install/setup.bash

# Execute the passed command arguments (like running the ROS 2 node)
#exec "ros2 run bhf_vo_new bhf_vo_new src/"


#!/bin/bash

# Copy this file into /usr/local/bin/
# edit permissions with sudo chmod +x /usr/local/bin/boxes-brain-launch.sh

set -e

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source your workspace overlay
source /home/brendan/boxes-brain/install/setup.bash

# Launch your top-level launch file
exec ros2 launch entry_point boxes.launch.py
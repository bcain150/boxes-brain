#!/bin/bash

# Copy this file into /usr/local/bin/
# ensure permissions with sudo chmod +x /usr/local/bin/boxes-brain-launch.sh
set -e

# remove ros logs from last 7 days
find /home/bcain/.ros/log/ -mindepth 1 -maxdepth 1 -mtime +7 -exec rm -rf {} + 2>/dev/null || true

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source your workspace overlay
source /home/bcain/boxes-brain/install/setup.bash

# Launch your top-level launch file
exec ros2 launch entry_point boxes.launch.py
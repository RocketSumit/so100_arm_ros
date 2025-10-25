#!/bin/bash
set -e

# Detect shell type and source appropriate setup file
if [ -n "$ZSH_VERSION" ]; then
    # Running in zsh
    source /opt/ros/humble/setup.zsh
    # Source workspace if it exists
    if [ -f "/home/$USERNAME/ros2_ws/install/setup.zsh" ]; then
        source "/home/$USERNAME/ros2_ws/install/setup.zsh"
    fi
else
    # Running in bash
    source /opt/ros/humble/setup.bash
    # Source workspace if it exists
    if [ -f "/home/$USERNAME/ros2_ws/install/setup.bash" ]; then
        source "/home/$USERNAME/ros2_ws/install/setup.bash"
    fi
fi

# Initialize rosdep if not done already
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    sudo rosdep init
fi
rosdep update

# Setup ROS domain ID if provided
if [ -n "$ROS_DOMAIN_ID" ]; then
    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
fi

# Print ROS environment info
printenv | grep -i "ros"

# Execute the command passed to the container
exec "$@"
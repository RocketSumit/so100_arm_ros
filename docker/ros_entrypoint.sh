#!/bin/bash
set -e

# Bash or other shells
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
fi
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

# Initialize rosdep if needed
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    echo "Initializing rosdep..."
    sudo rosdep init
fi
rosdep update || echo "rosdep update failed or already up-to-date."

# Set ROS_DOMAIN_ID if provided
if [ -n "$ROS_DOMAIN_ID" ]; then
    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
fi

# Print ROS environment summary
echo "ROS Environment:"
printenv | grep -i "ros" || true

# Execute the provided command (e.g. zsh, bash, etc.)
exec "$@"

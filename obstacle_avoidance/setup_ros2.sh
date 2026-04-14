#!/bin/bash
# ROS2 Humble setup script - Sources all necessary ROS2 configurations

# Source ROS2 Humble installation first
source /opt/ros/humble/setup.bash

# Determine workspace directory
WORKSPACE_DIR="/home/avengers/ros2_ws"

# Source the workspace setup
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
    echo "✓ Sourced ROS2 workspace: $WORKSPACE_DIR"
else
    echo "⚠ WARNING: Workspace setup not found at $WORKSPACE_DIR/install/setup.bash"
    echo "  Please build the workspace first with:"
    echo "    cd $WORKSPACE_DIR"
    echo "    colcon build"
fi

echo "✓ ROS2 Humble sourced from /opt/ros/humble/setup.bash"
echo "✓ ROS_DISTRO: $ROS_DISTRO"
echo "✓ ROS_VERSION: $ROS_VERSION"

# Optional: Set useful ROS2 environment variables
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

echo "✓ ROS setup complete!"

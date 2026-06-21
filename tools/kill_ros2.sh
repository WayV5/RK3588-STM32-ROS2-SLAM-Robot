#!/bin/bash
# Kill all ROS2 nodes and launch processes on this machine
# Usage: ./kill_ros2.sh

echo "Killing all ROS2 processes..."

# SIGINT first (graceful shutdown)
pkill -INT -f "ros2 " 2>/dev/null
sleep 1

# SIGTERM for stragglers
pkill -TERM -f "ros2 " 2>/dev/null
sleep 0.5

# SIGKILL for anything left
pkill -KILL -f "ros2 " 2>/dev/null

# Also kill component containers
pkill -KILL -f "component_container" 2>/dev/null

echo "Done."

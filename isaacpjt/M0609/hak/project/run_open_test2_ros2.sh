#!/usr/bin/env bash
# test2.usd + ROS2 브리지로 Isaac Sim 을 띄운다.
set -e

source /opt/ros/jazzy/setup.bash
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$HOME/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib"

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-미설정}  RMW=${RMW_IMPLEMENTATION:-기본}"

cd "$HOME/isaacsim"
exec ./isaac-sim.sh --exec "$HOME/cobot3_ws/isaacpjt/M0609/hak/project/open_test2_ros2.py" "$@"

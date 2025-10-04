#!/usr/bin/env bash
set -euo pipefail

WS_DIR=${WS_DIR:-"$HOME/ros2_ws"}
mkdir -p "$WS_DIR/src"

if [ ! -d "$WS_DIR/src/livox_ros_driver2" ]; then
  git clone https://github.com/Livox-SDK/livox_ros_driver2.git "$WS_DIR/src/livox_ros_driver2"
else
  cd "$WS_DIR/src/livox_ros_driver2"
  git fetch --all
  git pull --rebase
fi

cd "$WS_DIR/src/livox_ros_driver2"
if [ ! -f package.xml ] && [ -f package_ROS2.xml ]; then
  ln -sf package_ROS2.xml package.xml
fi

echo "Driver located at: $WS_DIR/src/livox_ros_driver2"



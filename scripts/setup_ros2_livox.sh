#!/usr/bin/env bash
set -euo pipefail

# Install ROS 2 Humble (Jammy) and build tools
sudo apt-get update -y
sudo apt-get install -y locales curl gnupg2 lsb-release software-properties-common build-essential cmake git python3-pip python3-colcon-common-extensions

sudo locale-gen en_US en_US.UTF-8
export LANG=en_US.UTF-8

if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get update -y
sudo apt-get install -y ros-humble-ros-base

grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

# Create workspace
WS_DIR=${WS_DIR:-$HOME/ros2_ws}
mkdir -p "$WS_DIR/src"

# Clone Livox ROS Driver2
if [ ! -d "$WS_DIR/src/livox_ros_driver2" ]; then
  git clone https://github.com/Livox-SDK/livox_ros_driver2.git "$WS_DIR/src/livox_ros_driver2"
fi

# Dependencies
sudo apt-get install -y libpcl-dev
sudo apt-get install -y ros-humble-pcl-conversions ros-humble-pcl-ros ros-humble-rclcpp ros-humble-sensor-msgs ros-humble-std-msgs ros-humble-diagnostic-updater

# Build
source /opt/ros/humble/setup.bash
cd "$WS_DIR"
colcon build --symlink-install

echo "source $WS_DIR/install/setup.bash" >> ~/.bashrc
echo "Setup completed. Open a new shell or 'source ~/.bashrc' then run launch scripts."



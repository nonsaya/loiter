#!/usr/bin/env bash
set -euo pipefail

sudo apt update -y
sudo apt install -y curl gpg

# Setup PPA for Ubuntu 22.04
curl -s --compressed "https://koide3.github.io/ppa/ubuntu2204/KEY.gpg" | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/koide3_ppa.gpg >/dev/null
echo "deb [signed-by=/etc/apt/trusted.gpg.d/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2204 ./" | sudo tee /etc/apt/sources.list.d/koide3_ppa.list

sudo apt update -y
sudo apt install -y libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev
sudo apt install -y libgtsam-points-dev

# Install GLIM for ROS2 Humble (CPU)
sudo apt install -y ros-humble-glim-ros

sudo ldconfig

echo "GLIM CPU (PPA) installation finished."

#!/usr/bin/env bash
set -euo pipefail

# Dependencies (CPU)
sudo apt update -y
sudo apt install -y git build-essential cmake ninja-build pkg-config \
  libomp-dev libboost-all-dev libmetis-dev \
  libfmt-dev libspdlog-dev \
  libglm-dev libglfw3-dev libpng-dev libjpeg-dev

# GTSAM
if [ ! -d "$HOME/src/gtsam" ]; then
  mkdir -p "$HOME/src" && cd "$HOME/src"
  git clone https://github.com/borglab/gtsam
fi
cd "$HOME/src/gtsam"
# Use a known working tag for GLIM
git fetch --all
(git checkout 4.3a0 || git checkout 4.2a9)
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_WITH_TBB=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
make -j"$(nproc)"
sudo make install

# Iridescence (optional but recommended for visualization)
if [ ! -d "$HOME/src/iridescence" ]; then
  cd "$HOME/src"
  git clone https://github.com/koide3/iridescence --recursive
fi
cd "$HOME/src/iridescence"
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j"$(nproc)"
sudo make install

# gtsam_points (CPU build)
if [ ! -d "$HOME/src/gtsam_points" ]; then
  cd "$HOME/src"
  git clone https://github.com/koide3/gtsam_points
fi
cd "$HOME/src/gtsam_points"
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA=OFF
make -j"$(nproc)"
sudo make install

# Update shared library cache
sudo ldconfig

# GLIM + ROS2 interface into ros2_ws
WS_DIR=${WS_DIR:-"/home/nonsaya-n/repo/loiter/ros2_ws"}
mkdir -p "$WS_DIR/src"
cd "$WS_DIR/src"
[ -d glim ] || git clone https://github.com/koide3/glim
[ -d glim_ros2 ] || git clone https://github.com/koide3/glim_ros2

# Build GLIM (CPU)
source /opt/ros/humble/setup.bash
cd "$WS_DIR"
colcon build --symlink-install --cmake-args \
  -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_VIEWER=ON -DBUILD_WITH_MARCH_NATIVE=OFF

echo "GLIM (CPU, source) build completed. Source $WS_DIR/install/setup.bash to use."

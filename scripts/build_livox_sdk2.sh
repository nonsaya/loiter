#!/usr/bin/env bash
set -euo pipefail

SDK_DIR=${SDK_DIR:-"$HOME/livox_lidar_sdk"}
if [ ! -d "$SDK_DIR" ]; then
  git clone https://github.com/Livox-SDK/Livox-SDK2.git "$SDK_DIR"
else
  cd "$SDK_DIR" && git fetch --all && git pull --rebase
fi

cd "$SDK_DIR"
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_PCAP=OFF -DBUILD_TESTING=OFF ..
make -j$(nproc)
sudo make install
sudo ldconfig
echo "Livox-SDK2 installed to /usr/local"



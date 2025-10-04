# Livox MID360 on ROS 2

手順概要（Jetson, Ubuntu 22.04 / ROS 2 Humble）

1. ROS2 とビルド環境の導入
   - `scripts/setup_ros2_livox.sh` を実行
2. ドライバ取得
   - `scripts/fetch_livox_driver.sh` を実行（`$HOME/ros2_ws` または `WS_DIR` で指定）
3. 設定
   - `livox_mid360_bringup/config/livox_mid360.yaml` の `host_ip` を `192.168.1.50`, センサ `ip` を `192.168.1.3` で確認
4. ビルド
   - `cd $HOME/ros2_ws && colcon build --symlink-install`
5. 起動
   - `source $HOME/ros2_ws/install/setup.bash`
   - `ros2 launch livox_mid360_bringup livox_mid360.launch.py`
6. 確認
   - `ros2 topic list` に `/livox/points`, `/livox/imu` が出ること



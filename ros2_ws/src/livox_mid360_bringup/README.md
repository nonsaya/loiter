起動手順 (ROS 2 Humble)

1. ROS 2/依存をインストール
   - `../../../../scripts/setup_ros2_livox.sh`
2. ドライバ取得
   - `../../../../scripts/fetch_livox_driver.sh` (必要なら `WS_DIR` を `/home/nonsaya-n/repo/loiter/ros2_ws` に設定)
3. ビルド
   - `cd $HOME/ros2_ws && colcon build --symlink-install`
4. 起動
   - `source $HOME/ros2_ws/install/setup.bash`
   - `ros2 launch livox_mid360_bringup livox_mid360.launch.py`

期待トピック:
 - `/livox/points`
 - `/livox/imu`



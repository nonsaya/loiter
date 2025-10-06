MID360ドライバの起動
source /opt/ros/humble/setup.bash
source /home/nonsaya-n/repo/loiter/ros2_ws/install/setup.bash
ros2 launch livox_mid360_bringup livox_mid360.launch.py

GLIMの起動
source /opt/ros/humble/setup.bash
source ~/glim_src_ws/install/setup.bash
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/home/nonsaya-n/glim_config


ビューアを無効にする方法
libstandard_viewer.soを削除しますconfig_ros.json。すると、GLIM は GUI なしで起動します。

MAVROSの起動
source /opt/ros/humble/setup.bash
ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyTHS1:921600
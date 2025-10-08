source /opt/ros/humble/setup.bash
source /home/nonsaya-n/repo/loiter/ros2_ws/install/setup.bash
ros2 launch glim_bringup glim_cpu.launch.py




進めます。ローカル環境（ディスプレイあり）での MID360 ドライバ起動は次のコマンドです。

- 起動（ターミナル1）
```bash
# ROS 2 とワークスペースのオーバーレイを読み込み
source /opt/ros/humble/setup.bash
source /home/nonsaya-n/repo/loiter/ros2_ws/install/setup.bash

# MID360 ドライバ起動（PointCloud2 は /livox/lidar で出力）
ros2 launch livox_mid360_bringup livox_mid360.launch.py
```

- 動作確認（ターミナル2）
```bash
source /opt/ros/humble/setup.bash
source /home/nonsaya-n/repo/loiter/ros2_ws/install/setup.bash

# トピック一覧
ros2 topic list | grep livox

# IMU 周波数
ros2 topic hz /livox/imu

# PointCloud2 を1メッセージ確認（センサQoS）
ros2 topic echo /livox/lidar --qos-profile sensor_data -n 1
```

補足:
- `Jetson=192.168.1.50`, `MID360=192.168.1.3` は `livox_mid360.yaml` → `mid360_net.json` に反映済みです（`/home/nonsaya-n/repo/loiter/ros2_ws/src/livox_mid360_bringup/config/`）。
- ドキュメントは `Libox2Glim.md` にまとまっています。[nonsaya/loiter (GitHub)](https://github.com/nonsaya/loiter)


# 設定
mkdir -p /home/nonsaya-n/glim_config
cat >/home/nonsaya-n/glim_config/config_ros.json <<'EOF'
{
  "imu_topics": ["/livox/imu"],
  "points_topics": ["/livox/lidar"],
  "extension_modules": ["librviz_viewer.so"]
}
EOF

# 起動（Livoxドライバが配信中であること）
source /opt/ros/humble/setup.bash
/opt/ros/humble/lib/glim_ros/glim_rosnode --ros-args -p config_path:=/home/nonsaya-n/glim_config



source /opt/ros/humble/setup.bash
/opt/ros/humble/lib/glim_ros/glim_rosnode --ros-args -p config_path:=/home/nonsaya-n/repo/loiter/ros2_ws/src/glim_bringup/config/glim_config





nonsaya-n@nonsaya-n:~$ source /opt/ros/humble/setup.bash
/opt/ros/humble/lib/glim_ros/glim_rosnode --ros-args -p config_path:=/home/nonsaya-n/repo/loiter/ros2_ws/src/glim_bringup/config/glim_config
[2025-10-08 16:55:55.307] [glim] [info] config_path: /home/nonsaya-n/repo/loiter/ros2_ws/src/glim_bringup/config/glim_config
[2025-10-08 16:55:55.308] [glim] [warning] param preprocess/crop_bbox_frame not found
[2025-10-08 16:55:55.308] [glim] [warning] use default_value=lidar
[2025-10-08 16:55:55.308] [glim] [warning] param preprocess/crop_bbox_min not found
[2025-10-08 16:55:55.308] [glim] [warning] use default_value=vec(0.000000,0.000000,2520226428574574081373339486912186593969752373246036236682374806822710926063617448083537679729199040213515213129919167400192520837145588698535136327494577178001204422884002055061504.000000)
[2025-10-08 16:55:55.308] [glim] [warning] param preprocess/crop_bbox_max not found
[2025-10-08 16:55:55.308] [glim] [warning] use default_value=vec(0.000000,0.000000,2520226428574574081373339486912186593969752373246036236682374806822710926063617448083537679729199040213515213129919167400192520837145588698535136327494577178001204422884002055061504.000000)
[2025-10-08 16:55:55.308] [glim] [info] load libodometry_estimation_cpu.so
[2025-10-08 16:55:55.308] [glim] [warning] param sensors/imu_bias_noise not found
[2025-10-08 16:55:55.309] [glim] [warning] use default_value=0.001
[2025-10-08 16:55:55.309] [glim] [warning] param odometry_estimation/initialization_window_size not found
[2025-10-08 16:55:55.309] [glim] [warning] use default_value=1
[2025-10-08 16:55:55.309] [glim] [info] load libsub_mapping.so
[2025-10-08 16:55:55.310] [glim] [warning] param sub_mapping/keyframe_randomsampling_rate not found
[2025-10-08 16:55:55.310] [glim] [warning] use default_value=0.1
[2025-10-08 16:55:55.310] [glim] [warning] param sub_mapping/keyframe_voxel_resolution not found
[2025-10-08 16:55:55.310] [glim] [warning] use default_value=0.5
[2025-10-08 16:55:55.310] [glim] [warning] param sub_mapping/keyframe_voxelmap_levels not found
[2025-10-08 16:55:55.310] [glim] [warning] use default_value=3
[2025-10-08 16:55:55.310] [glim] [warning] param sub_mapping/keyframe_voxelmap_scaling_factor not found
[2025-10-08 16:55:55.310] [glim] [warning] use default_value=2
[2025-10-08 16:55:55.310] [glim] [info] load libmemory_monitor.so
[2025-10-08 16:55:55.311] [glim] [info] load libstandard_viewer.so
[2025-10-08 16:55:55.657] [glim] [info] load librviz_viewer.so
[2025-10-08 16:55:57.703] [odom] [info] estimate initial IMU state
Initial error: 98210, values: 66
iter      cost      cost_change    lambda  success iter_time
   0  3.919098e+01    9.82e+04    1.00e-05     1    2.64e-03
   1  4.930962e+00    3.43e+01    1.00e-06     1    2.52e-03
   2  4.930961e+00    1.25e-06    1.00e-07     1    2.47e-03
[2025-10-08 16:55:57.716] [odom] [info] initial IMU state estimation result
[2025-10-08 16:55:57.716] [odom] [info] T_world_imu=se3(-0.000817,-0.000627,-0.003150,0.014806,-0.012036,-0.002843,0.999814)
[2025-10-08 16:55:57.716] [odom] [info] v_world_imu=vec(-0.003638,-0.012086,-0.029252)
[2025-10-08 16:55:57.716] [odom] [info] imu_bias=vec(0.000020,0.000083,0.000267,0.003681,-0.000867,-0.000992)



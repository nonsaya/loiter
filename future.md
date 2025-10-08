# Future Roadmap (Loiter → Autonomous Flight)

## Stage 0: Current Baseline (Completed)
- Livox MID360 driver + GLIM (CPU) + MAVROS + Bridge
- External odometry → EKF3 intake (LOITER stable)
- One-shot tmux launcher (`scripts/launch_loiter_tmux.sh`)

## Stage 1: Wireless one-command mission (arm → takeoff → hover → move → land)
- Goal: Over wireless trigger, perform:
  1) Arm
  2) Takeoff to target altitude (e.g., 2.0 m)
  3) Hover N seconds
  4) Move +X (ROS) by 2.0 m, hover N seconds
  5) Land
- Plan:
  - Use MAVROS services and topics:
    - `/mavros/cmd/arming` (srv: CommandBool)
    - `/mavros/set_mode` (srv: SetMode) → GUIDED / LOITER as needed
    - `/mavros/setpoint_raw/local` or `/mavros/setpoint_position/local` for position goals
    - `/mavros/cmd/takeoff` or local position setpoint for altitude step
  - Build a simple mission node (Python) with a state machine:
    - states: IDLE → ARM → TAKEOFF → HOVER → MOVE_X → HOVER → LAND → DONE
    - parameters: takeoff_altitude, dx, hover_seconds
  - Safety interlocks:
    - Pre-arm checks (IMU health, RC failsafe, EKF healthy)
    - Timeout & abort to LOITER, disarm on command

## Stage 2: Extend mission complexity
- Multi-waypoint local missions (square/figure-8, altitude changes)
- Time-based vs. position-converged acceptance with tolerances
- Velocity-capped profiles (jerk-limited via setpoint_raw/local)
- Add a mission YAML and loader (rosparam / yaml-cpp)

## Stage 3: Remote mapping on another PC
- Run GLIM on a separate PC in the same network:
  - Subscribe to `/livox/lidar` and `/livox/imu` via DDS
  - Publish `/glim_ros/{odom,map,pose}` back onto the network
- Visualization:
  - RViz2 on remote PC: map + trajectory overlay
  - Record rosbag selectively (odom, map keyframes, tf)
- Sync & clocks:
  - Ensure chrony/PTP where possible
  - Monitor `/mavros/timesync_status`

## Stage 4: Obstacle detection & flight-stop
- Near-term simple approach:
  - Use point cloud proxemics: stop if min range < threshold in a forward FOV
  - Node subscribes to `/livox/lidar`, outputs a `std_msgs/Bool stop_request`
  - Bridge/mission node suppresses setpoints while `stop_request=true`
- Mid-term:
  - Voxel grid or ring buffer occupancy ahead of drone
  - Slowdown then stop based on clearance
- Long-term:
  - Local planner (e.g., sampling-based) for go/no-go corridor

## Stage 5: Robustness & Safety
- EKF3 tuning envelopes documented; auto-fallback logic:
  - If `/mavros/estimator_status` unhealthy → command LOITER
  - If `/mavros/state` not ARMED but should be → abort mission
- Loss-of-link policy (wireless):
  - RC failsafe → RTL/LOITER
  - Mission node heartbeat → if lost N sec, stop setpoints
- Battery/thermal monitoring via MAVROS topics

## Stage 6: Ops & Tooling
- Scripts:
  - `scripts/launch_loiter_tmux.sh` for bring-up
  - `scripts/mission_demo.sh` (to be added): runs Stage 1 demo
- Docs:
  - `Libox2Glim.md` kept as source-of-truth for rebuild
  - This file (`future.md`) for roadmap/status
- CI (optional): lint build for bridge/mission nodes

## Stage 7: Mission Node (to be implemented)
- Package: `loiter_mission`
- Features:
  - Parameterized mission (altitude, dx, hover times)
  - Services to start/stop mission from remote PC
  - Feedback via diagnostics
- Interfaces:
  - Input: `/mavros/local_position/pose`, `/mavros/state`, `/mavros/estimator_status`
  - Output: `/mavros/cmd/arming`, `/mavros/set_mode`, `/mavros/setpoint_position/local`

## Notes & References
- MAVROS interfaces: use ODOMETRY-IN for EKF3 external nav; VISION routes optional
- Frame choice: prefer `frame_id=odom`, `child_frame_id=base_link`
- Covariance tuning guide is in `Libox2Glim.md` (Section 13)

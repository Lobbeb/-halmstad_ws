# lrs_halmstad

Current package-level runbook for the Halmstad 1-to-1 UAV-UGV baseline.

Current tested setup:
- ROS 2 Jazzy
- Gazebo Harmonic
- `warehouse` world
- 1 UGV (`a201_0000`) + 1 UAV (`dji0`)

## Main entry points

- Main follow launch: `run_follow.launch.py`
- Main workspace helper flow: run the wrappers from the workspace root with `./run.sh ...`
- Recommended full-stack helper: `./run.sh tmux_1to1 warehouse`

## Core topics

UGV:
- `/a201_0000/cmd_vel`
- `/a201_0000/amcl_pose_odom`
- `/a201_0000/initialpose`
- `/a201_0000/navigate_to_pose`

UAV:
- `/dji0/camera0/image_raw`
- `/dji0/camera0/camera_info`
- `/dji0/pose`
- `/dji0/psdk_ros2/flight_control_setpoint_ENUposition_yaw`

Perception / coordination:
- `/coord/leader_detection`
- `/coord/leader_detection_status`
- `/coord/leader_estimate`
- `/coord/leader_estimate_status`
- `/coord/events`

OMNeT topics when enabled:
- `/omnet/sim_time`
- `/omnet/link_distance`
- `/omnet/rssi_dbm`
- `/omnet/snir_db`
- `/omnet/packet_error_rate`
- `/omnet/radio_distance`

## Recommended 1-to-1 bring-up

From the workspace root:

```bash
./run.sh gazebo_sim warehouse
./run.sh spawn_uav warehouse uav_name:=dji0
./run.sh localization warehouse
./run.sh nav2
./run.sh 1to1_follow warehouse
```

If you prefer the tmux wrapper:

```bash
./run.sh tmux_1to1 warehouse
```

## Direct launch usage

Environment setup:

```bash
cd <workspace_root>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
```

Start the follow stack directly:

```bash
ros2 launch lrs_halmstad run_follow.launch.py
```

Useful overrides:

```bash
ros2 launch lrs_halmstad run_follow.launch.py leader_mode:=odom
ros2 launch lrs_halmstad run_follow.launch.py leader_mode:=estimate start_leader_estimator:=true
ros2 launch lrs_halmstad run_follow.launch.py ugv_mode:=nav2
ros2 launch lrs_halmstad run_follow.launch.py ugv_mode:=external
```

## Nav2 notes

- For `ugv_mode:=nav2`, start localization and Nav2 before the follow launch.
- The UGV driver can publish `/a201_0000/initialpose` automatically before sending goals.
- File-based routes can be supplied through `goal_sequence_file`.
- Use `ugv_mode:=external` if another node will send Nav2 goals.

Standalone Nav2 test:

```bash
ros2 run lrs_halmstad ugv_nav2_goal_tester --ros-args \
  -r __ns:=/a201_0000 \
  -p pattern:=square \
  -p pattern_size_m:=2.0
```

## Visual follow / YOLO notes

- `leader_mode:=odom` is the simpler odometry-driven path.
- `leader_mode:=estimate` uses the visual estimation stack.
- The normal WSL2 setup runs YOLO on CPU.
- `leader_detector` and `leader_tracker` both publish to `/coord/leader_detection`.
- `leader_estimator` publishes the world-frame estimate on `/coord/leader_estimate`.

## OMNeT notes

- `gazebo_pose_tcp_bridge` serves Gazebo/ROS pose snapshots to the OMNeT side.
- `omnet_metrics_bridge` republishes live network metrics into ROS topics.
- `run_follow.launch.py` can start the ROS-side OMNeT metrics bridge when enabled.

Start the pose bridge manually:

```bash
ros2 run lrs_halmstad gazebo_pose_tcp_bridge
```

## Dataset tools

Generate OBB labels:

```bash
ros2 run lrs_halmstad make_obb warehouse_v3
ros2 run lrs_halmstad make_obb warehouse_v3 --overlay
```

Prune frames where the target is not actually visible:

```bash
ros2 run lrs_halmstad prune_negatives warehouse_v3 --dry-run
ros2 run lrs_halmstad prune_negatives warehouse_v3
```

## Contract checks

Base contract:

```bash
ros2 run lrs_halmstad contract_check orchard dji0
```

With follow stack:

```bash
REQUIRE_UAV_ADAPTER=1 REQUIRE_FOLLOW_STACK=1 ros2 run lrs_halmstad contract_check orchard dji0
```

With detection + estimator:

```bash
REQUIRE_UAV_ADAPTER=1 REQUIRE_FOLLOW_STACK=1 REQUIRE_DETECTION=1 REQUIRE_ESTIMATOR=1 ros2 run lrs_halmstad contract_check orchard dji0
```

## Notes

- The current baseline should be treated as the reference point for further work.
- The helper wrappers under `scripts/` are the preferred way to run the supported workflow.
- If docs here and the root README disagree, check the current helper scripts and `run_follow.launch.py`.

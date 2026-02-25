# Halmstad ROS 2 + Gazebo Testbed — Workspace Snapshot

Status
------
This repository is a source-only snapshot of the Halmstad Stage-1 robotics testbed (ROS 2 Jazzy + Gazebo). It intentionally excludes build artifacts, runtime outputs, and virtual environments so it stays lightweight and clone-ready.

Table of contents
-----------------
- Overview
- Repository layout
- Prerequisites
- Quickstart (one-command bootstrap)
- Manual setup steps (detailed)
- Running experiments
- Important internal tooling
- Recommended development workflow
- VS Code / Pylance tips
- Branching / pushing notes
- Troubleshooting
- License & contact

Overview
--------
This workspace contains:
- Simulation and experiment orchestration (scripts/run_round.sh)
- A ROS 2 package `lrs_halmstad` with launch files, utilities and small nodes (contract checks, event relay)
- Optional OMNeT++ bridge (if present)
- Helpers and documentation to reproduce experiments on another machine

Goal: clone this repo on a new machine, run a single bootstrap script and be able to build and run experiments with minimal manual setup.

Prerequisites (host machine)
----------------------------
- Ubuntu or WSL Ubuntu (recommended; instructions assume WSL/Ubuntu)
- Python 3.10+ (system Python)
- ROS 2 Jazzy installed via apt (rclpy and ROS libs come from apt packages)
- colcon build tools (colcon-cli, colcon-common-extensions)
- git, rsync (for export workflows), rosdep
- Optional: Gazebo version compatible with installed ROS 2 distro if running simulations

Quickstart — one command (after cloning)
----------------------------------------
1. Clone:
```bash
git clone https://github.com/Lobbeb/halmstad_ws.git
cd ~/halmstad_ws
```


2. Source overlays (if not already sourced by the bootstrap script):
```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-selected lrs_halmstad --symlink-install
source install/setup.bash
```

3. Run experiments or utilities (examples below).

requirements.txt (pip-only)
---------------------------
This file contains only pip-installable Python libraries used by scripts (not rclpy).
Add extra pip packages used in analysis scripts (numpy, pandas, matplotlib) if required.

Manual setup steps (detailed)
----------------------------
If prefer to run steps yourself:

1. Install ROS 2 Jazzy (follow ROS docs).  
2. On WSL Ubuntu, ensure Distro and OpenGL setup for Gazebo (if using GPU passthrough).  
3. From workspace root:
```bash
# source ROS system
source /opt/ros/jazzy/setup.bash

# optional: create & activate venv
python3 -m venv .venv
source .venv/bin/activate

# pip deps
python -m pip install -U pip
python -m pip install -r requirements.txt

# ros dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# build
colcon build --symlink-install
source install/setup.bash
```

Running experiments (current runbook)
-------------------------------------
This section documents the commands currently used to run the Halmstad Gazebo + ROS 2 simulation stack, spawn UAVs, start movement/follow logic, and start the OMNeT TCP bridge.

Terminal setup (run in every terminal or put in your .bashrc)
--------------------------------------
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
```

Build (after code changes)
--------------------------
```bash
colcon build --symlink-install
source install/setup.bash
```

Recommended launch order (full simulation)
------------------------------------------
1. Start Gazebo + Clearpath UGV (GUI):
```bash
ros2 launch clearpath_gz simulation.launch.py world:=orchard use_sim_time:=true gui:=true
```

2. Spawn UAV(s):
- Single UAV (`dji0`) with integrated camera/gimbal:
```bash
ros2 launch lrs_halmstad spawn1m100gimbal.launch.py
```
- Multiple UAVs (`dji0`, `dji1`, `dji2`) with integrated cameras:
```bash
ros2 launch lrs_halmstad spawn_uavs.launch.py world:=orchard uav_mode:=teleport
```

3. Start motion / follow logic (choose one):
- Follow stack (UGV motion + UAV follow + optional YOLO leader estimate):
```bash
ros2 launch lrs_halmstad run_round_follow_motion.launch.py
```
- Motion-only sequence (UAV sweep then UGV motion):
```bash
ros2 launch lrs_halmstad run_round_motion.launch.py
```

4. Start OMNeT pose bridge (TCP pose snapshots from ROS odom topics):
```bash
ros2 run lrs_halmstad gazebo_pose_tcp_bridge
```

5. Optional checks:
```bash
ros2 topic list -t --no-daemon
ros2 topic echo /coord/leader_estimate_status
ros2 topic hz /dji0/camera0/image_raw
```

Gazebo + UGV simulation (GUI)
-----------------------------
Direct launch command:
```bash
ros2 launch clearpath_gz simulation.launch.py world:=orchard use_sim_time:=true gui:=true
```

Common arguments (when calling `clearpath_gz` directly):
- `world:=orchard` (world name)
- `use_sim_time:=true|false`
- `gui:=true|false`

Spawn UAVs
----------
Direct multi-UAV launch:
```bash
ros2 launch lrs_halmstad spawn_uavs.launch.py world:=orchard uav_mode:=teleport
```

`spawn_uavs.launch.py` arguments:
- `world:=orchard|construction|office|pipeline|solar_farm|warehouse`
- `uav_mode:=teleport|physics`

Single UAV + camera/gimbal launch (integrated model)
----------------------------------------------------
```bash
ros2 launch lrs_halmstad spawn1m100gimbal.launch.py world:=orchard name:=dji0
```

`spawn1m100gimbal.launch.py` arguments:
- `world:=<gazebo_world>` (default `orchard`)
- `name:=<uav_name>` (default `dji0`)

Low-level UAV spawn (`spawn_robot.launch.py`)
---------------------------------------------
Use this when you want one UAV with full control over camera attachment and spawn pose.

Example (teleport/static UAV with attached camera and camera bridge):
```bash
ros2 launch lrs_halmstad spawn_robot.launch.py \
  world:=orchard name:=dji0 type:=m100 \
  uav_mode:=teleport with_camera:=true bridge_camera:=true camera_name:=camera0 \
  x:=0.0 y:=0.0 z:=9.0 R:=0.0 P:=0.0 Y:=0.0
```

`spawn_robot.launch.py` arguments:
- `world:=<world>` (default `empty`)
- `name:=<model_name>` (default `m100`)
- `type:=m100|...` (currently `m100` is the tested UAV type)
- `uav_mode:=teleport|physics`
- `with_camera:=true|false`
- `bridge_camera:=true|false` (bridge `/<name>/<camera_name>/(image_raw,camera_info)`)
- `camera_name:=camera0` (default `camera0`)
- `x:=<m>` `y:=<m>` `z:=<m>`
- `R:=<rad>` `P:=<rad>` `Y:=<rad>`
- `model:=...` (legacy arg, currently not used by the generated-SDF spawn path)

Movement and follow orchestration
---------------------------------
Motion-only launch:
```bash
ros2 launch lrs_halmstad run_round_motion.launch.py \
  world:=orchard uav_name:=dji0 ugv_cmd_topic:=/a201_0000/cmd_vel
```

`run_round_motion.launch.py` arguments:
- `params_file:=<yaml>` (default `config/run_round_motion_defaults.yaml`)
- `world:=<world>` (default `orchard`)
- `uav_name:=<uav_name>` (default `dji0`)
- `ugv_cmd_topic:=<topic>` (default `/a201_0000/cmd_vel`)
- `uav_log_csv:=<path>` (default empty)

Follow launch (UGV motion + `follow_uav` + optional `leader_estimator`):
```bash
ros2 launch lrs_halmstad run_round_follow_motion.launch.py \
  world:=orchard uav_name:=dji0 leader_mode:=odom
```

YOLO estimate mode (UAV follows UGV estimate from camera detections):
```bash
ros2 launch lrs_halmstad run_round_follow_motion.launch.py \
  world:=orchard uav_name:=dji0 leader_mode:=estimate \
  yolo_weights:=/home/ruben/halmstad_ws/models/yolov5n.pt yolo_device:=cpu
```

`run_round_follow_motion.launch.py` arguments:
- `params_file:=<yaml>` (default `config/run_round_follow_defaults.yaml`)
- `world:=<world>` (default `orchard`)
- `uav_name:=<uav_name>` (default `dji0`)
- `leader_mode:=odom|pose|estimate` (default `odom`)
- `leader_perception_enable:=true|false` (default `false`)
- `start_leader_estimator:=auto|true|false` (default `auto`)
- `ugv_cmd_topic:=<topic>` (default `/a201_0000/cmd_vel`)
- `ugv_odom_topic:=<topic>` (default `/a201_0000/platform/odom`)
- `leader_image_topic:=<topic>` (default `/<uav_name>/camera0/image_raw`)
- `leader_camera_info_topic:=<topic>` (default `/<uav_name>/camera0/camera_info`)
- `leader_depth_topic:=<topic>` (default empty / disabled)
- `leader_uav_pose_topic:=<topic>` (default `/<uav_name>/pose_cmd`)
- `yolo_weights:=<weights.pt>` (default `/home/ruben/halmstad_ws/models/yolov5n.pt`)
- `yolo_device:=cpu|cuda` (default `cpu`)
- `event_topic:=<topic>` (default `/coord/events`)
- `ugv_start_delay_s:=<seconds>` (default `0.0`; readiness gate handles startup)

YOLO estimate mode notes
------------------------
- `leader_estimator` publishes:
  - `/coord/leader_estimate` (`geometry_msgs/msg/PoseStamped`)
  - `/coord/leader_estimate_status` (`std_msgs/msg/String`)
- If status shows `yolo=disabled yolo_reason=ultralytics_not_installed`, install the Python package:
```bash
python3 -m pip install --user ultralytics
```
- If YOLO is enabled but no detections, status will typically show `NO_DET`, `HOLD`, or `REACQUIRE`.

OMNeT bridge (TCP pose snapshots)
---------------------------------
This workspace currently uses `gazebo_pose_tcp_bridge` (ROS odom -> TCP snapshot server) for the OMNeT side.

Default start:
```bash
ros2 run lrs_halmstad gazebo_pose_tcp_bridge
```

Default behavior:
- Binds TCP server on `127.0.0.1:5555`
- Streams snapshot data from:
  - `/a201_0000/platform/odom` as model `robot`
  - `/dji0/pose_cmd/odom` as model `dji0`

Override bridge parameters:
```bash
ros2 run lrs_halmstad gazebo_pose_tcp_bridge --ros-args \
  -p bind_host:=127.0.0.1 \
  -p port:=5555 \
  -p odom_topics:="['/a201_0000/platform/odom','/dji0/pose_cmd/odom']" \
  -p model_names:="['robot','dji0']"
```

`gazebo_pose_tcp_bridge` ROS parameters:
- `odom_topic:=<topic>` (legacy single-topic fallback)
- `model_name:=<name>` (legacy single-model fallback)
- `odom_topics:=['<topic1>','<topic2>', ...]`
- `model_names:=['<name1>','<name2>', ...]` (same length as `odom_topics`)
- `bind_host:=<ip>` (default `127.0.0.1`)
- `port:=<tcp_port>` (default `5555`)
- `stale_timeout_sec:=<seconds>` (default `2.0`)

Optional helper: pose_cmd -> odom converter
-------------------------------------------
`follow_uav` now publishes `/<uav>/pose_cmd/odom` directly, so this is usually not needed for the follow stack.

Use this helper only if another node publishes `PoseStamped` and you need an `Odometry` topic:
```bash
ros2 run lrs_halmstad pose_cmd_to_odom
```

`pose_cmd_to_odom` parameters:
- `pose_topic:=<topic>` (default `/dji0/pose_cmd`)
- `odom_topic:=<topic>` (default `/dji0/pose_cmd/odom`)
- `frame_id:=<frame>` (default empty = copy from pose header)
- `child_frame_id:=<frame>` (default `base_link`)
- `copy_header_stamp:=true|false` (default `true`)

Legacy script-based round runner
--------------------------------
The old orchestration script still exists (`scripts/run_round.sh`), but this README intentionally documents the ROS2-native `ros2 launch` / `ros2 run` commands above for day-to-day use.

Key nodes and scripts
---------------------
- scripts/run_round.sh — orchestrates rosbag recording, publishes event markers, runs sweeps.
- src/lrs_halmstad/lrs_halmstad/coord_event_relay.py — relay node: publish to /coord/events_cmd, node republishes reliably to /coord/events (prevents missed markers in bag).
- src/lrs_halmstad/lrs_halmstad/contract_check.py — pre-run contract check used by scripts/run_round.sh to assert required topics/services are present.
- scripts/uav_setpose_sweep.py — drives UAV set_pose sequences (verify this script is persistent rclpy process on your machine).

Design recommendations preserved here
------------------------------------
- Keep build artifacts out of Git (build/, install/, log/).
- Keep runtime data out of Git (runs/, bags, large binary outputs).
- Keep the repo source-only and reproducible by rosdep + colcon.

Git & repo notes (how this snapshot was made)
---------------------------------------------
- The repository was created by exporting a clean copy of an existing workspace (no nested .git folders were copied).
- The workspace root here is intended to be the Git root (`~/halmstad_ws` after clone).

If starting from an existing workspace with nested package-level repositories:
- Option A (used here): export a clean copy without nested `.git` and commit as a monorepo snapshot.
- Option B (preserve history): convert inner repos to submodules or use `git subtree` to preserve history (more advanced).

VS Code / Pylance tips
---------------------
If VS Code shows "Import 'rclpy' could not be resolved":

- Start VS Code from an environment with ROS 2 sourced:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
code .
```
- Or add extraPaths in `.vscode/settings.json`:
```json
{
  "python.analysis.extraPaths": [
    "/opt/ros/jazzy/lib/python3.10/site-packages",
    "${workspaceFolder}/install/**/lib/python3.10/site-packages"
  ]
}
```
Adjust Python path version (3.10) to your system Python.

Git push guidance
-----------------
To push to your GitHub repo and overwrite a remote branch safely:
```bash
git remote add origin https://github.com/Lobbeb/Master_Thesis.git
git branch -M main
git push --force-with-lease -u origin main
```
Use `--force-with-lease` rather than `--force` to reduce accidental overwrites.

If the repo should keep additional packages (e.g. `lrs_omnet_bridge`) make sure they exist under `src/` before committing.

Troubleshooting
---------------
- Missing rclpy in editor: see VS Code tips above.
- rosdep failures: ensure `sudo apt update` and ROS 2 apt repos configured.
- colcon build errors: inspect `log/` for failing package build logs; run `colcon build --event-handlers console_cohesion+` for clearer output.

Security & large files
----------------------
- Do not commit `install/`, `build/`, `log/` or binary bags — they bloat the repo.
- Use Git LFS only if very large binary artifacts must be versioned.

Recommended workflow for laptop (recover exact state quickly)
------------------------------------------------------------
1. Clone:
```bash
git clone https://github.com/Lobbeb/Master_Thesis.git ~/halmstad_ws
cd ~/halmstad_ws
```
2. Bootstrap:
```bash
bash scripts/bootstrap_ws.sh
```
3. Source and run experiments:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash scripts/run_round.sh run001 baseline a201_0000 orchard_world
```

License
-------
Add a license file appropriate for the project (MIT suggested if permissive sharing is desired).

Contact / Handoff
-----------------
Use issues or repository PRs for any follow-ups. For urgent local help, run the bootstrap script and attach logs from `colcon` (command + failing package logs).

Change log (snapshot notes)
---------------------------
- This repository is a snapshot intended for portability. It contains source and utilities required to reproduce Stage-1 experiments; build products and runtime outputs are intentionally excluded.

End of file.

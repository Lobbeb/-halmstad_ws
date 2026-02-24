# Runtime Recovery Checkpoint (2026-02-24)

## Purpose
This document captures the recovery work that restored the project to a working core state after prolonged spawn / Gazebo / ROS runtime failures.

Current confirmed working state:
- Terminal A: `ros2 launch clearpath_gz simulation.launch.py world:=orchard`
- Terminal B: `ros2 launch lrs_halmstad spawn_uavs.launch.py world:=orchard`
- Terminal D: `bash scripts/run_round.sh run_regress_test follow dji0 orchard`
- UGV spawns and moves, UAVs spawn and are visible again, baseline follow run works.

## Root Cause (What Actually Broke)
Two issues were overlapping:

1. **Spawn SDF XML corruption in UAV spawn path**
- `spawn_robot.launch.py` / `spawn_gimbal.launch.py` generated SDF using `Command(["ros2 run lrs_halmstad generate_sdf ..."])`.
- On this machine, FastDDS emitted `RTPS_TRANSPORT_SHM` warnings during startup.
- Those warnings could leak into the generated SDF string before `<?xml ...?>`.
- Gazebo then failed with XML parse errors (`XML_ERROR_PARSING_DECLARATION`, `Unable to read SDF string: ... RTPS ... <?xml ...>`).

2. **Runtime contamination from stale Gazebo/bridge/image processes**
- `scripts/kill_everything.sh` was not killing several `ros_gz_image/image_bridge` and Clearpath-related processes.
- This left stale services/topics and caused inconsistent behavior between relaunches.
- FastDDS SHM lock failures (`Failed init_port fastrtps_port...`) became pervasive and made the system look random / partially broken.

## What Was Fixed (Minimal Runtime/Spawn Fix Set)
These changes are focused on restoring reliable bring-up and spawn behavior, without changing the YOLO estimator/follower architecture.

### 1) Direct `generate_sdf` executable in spawn launch files (prevents SDF XML corruption)
Changed files:
- `src/lrs_halmstad/launch/spawn_robot.launch.py`
- `src/lrs_halmstad/launch/spawn_gimbal.launch.py`

What changed:
- Replaced `ros2 run lrs_halmstad generate_sdf` inside `Command(...)`
- With direct executable path resolved from `get_package_prefix('lrs_halmstad')`

Why:
- Avoids `ros2 run` wrapper stderr noise contaminating the XML string passed to Gazebo `create -string`.

### 2) `scripts/env.sh` hardened for WSL + FastDDS + Gazebo model visibility
Changed file:
- `scripts/env.sh`

What changed:
- `RMW_FASTRTPS_USE_SHM=0`
- `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`
- Adds Gazebo resource/model paths for:
  - `~/halmstad_ws/src/lrs_halmstad/models`
  - `~/halmstad_ws/install/lrs_halmstad/share`

Why:
- Reduces SHM lockfile/port failures in WSL.
- Helps Gazebo resolve `model://matrice_100` and `model://zenmuse_z3` mesh URIs so UAVs render in the GUI.

### 3) `scripts/kill_everything.sh` made aggressive enough to actually reset runtime
Changed file:
- `scripts/kill_everything.sh`

What changed:
- Kills additional leftover processes, including:
  - `ros_gz_image/image_bridge`
  - `ros_gz_bridge/parameter_bridge`
  - `interactive_marker_twist_server/marker_server`
  - `a201_0000`-namespaced leftover processes
  - `ros2` daemon
- Adds `kill -9` fallback for stubborn leftovers.

Why:
- The old cleanup script left many active bridge/image processes running, causing inconsistent spawn/bring-up behavior.

## Files Intentionally NOT Part of the Runtime Recovery Fix
These are core project/YOLO files and were not part of the spawn/runtime recovery changes:
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py`
- `src/lrs_halmstad/lrs_halmstad/follow_uav.py`
- `scripts/run_round.sh`

## Known Good Bring-Up Procedure (A / B / D)
Run this exactly to avoid stale state.

### 1) Rebuild + hard cleanup
```bash
cd ~/halmstad_ws
source scripts/env.sh
colcon build --packages-select lrs_halmstad
bash scripts/kill_everything.sh
```

### 2) In every terminal you use (A, B, D, E)
```bash
cd ~/halmstad_ws
source scripts/env.sh
source install/setup.bash
export ROS_DOMAIN_ID=0
```

### 3) Terminal A (simulation)
```bash
ros2 launch clearpath_gz simulation.launch.py world:=orchard
```

Quick checks (other terminal):
```bash
ros2 topic list | grep '^/clock$'
ros2 service list | grep /world/orchard/set_pose
```
Expected:
- `/clock` exists
- `/world/orchard/set_pose` exists

### 4) Terminal B (spawn UAVs)
```bash
ros2 launch lrs_halmstad spawn_uavs.launch.py world:=orchard
```
Expected:
- multiple `Entity creation successful.` lines
- `/dji0`, `/dji1`, `/dji2` camera topics available

Quick checks:
```bash
ros2 topic list | grep -E '^/dji[012]/camera0/(image_raw|camera_info)$'
```

### 5) Terminal D (baseline regression run)
```bash
bash scripts/run_round.sh run_regress_test follow dji0 orchard
```
Expected:
- contract check passes
- bag records
- UGV moves and follow path works again

## Fast Diagnostics (If It Breaks Again)

### A) Check cleanup really worked
```bash
bash scripts/kill_everything.sh
```
Inspect the `Remaining relevant processes` section. If many `ros_gz_image/image_bridge` or `a201_0000` processes remain, cleanup is incomplete.

### B) Check environment is applied in the current terminal
```bash
echo "$ROS_DOMAIN_ID $RMW_IMPLEMENTATION $RMW_FASTRTPS_USE_SHM $FASTDDS_BUILTIN_TRANSPORTS"
echo "$GZ_SIM_RESOURCE_PATH" | tr ':' '\n' | grep halmstad_ws
```
Expected:
- `0 rmw_fastrtps_cpp 0 UDPv4`
- paths including `src/lrs_halmstad/models` and `install/lrs_halmstad/share`

### C) If UAV spawn fails with XML parse errors
Look for:
- `XML_ERROR_PARSING_DECLARATION`
- `Unable to read SDF string: ... RTPS ... <?xml ...>`

That indicates SDF string contamination (spawn path issue). The direct `generate_sdf` executable patch must be present in `spawn_robot.launch.py` and `spawn_gimbal.launch.py`.

## Notes / Constraints for Continuing Work
- **AMD GPU**: `YOLO_DEVICE=cuda` will not work. Use `YOLO_DEVICE=cpu` unless a supported backend is configured.
- **ROS domain**: keep all terminals on the same `ROS_DOMAIN_ID` (currently `0` for local bring-up).
- **Stale processes are a real source of failure**: always run `bash scripts/kill_everything.sh` before relaunching A/B when behavior looks inconsistent.

## Recommended Next Step (Before More Changes)
Create a backup copy of this now-working workspace state (zip/folder copy) before resuming estimator/YOLO tuning.

This checkpoint is the recovery baseline to return to if later experimental changes break runtime behavior again.
# Control/Perception Split (Current Project Scope)

The current simulation is used to validate:
- `LEADER_MODE=odom` as the authoritative follow+leash control path
- YOLO (`leader_estimator`) as a perception subsystem (detections/status/logging)
- runtime/harness stability

The current simulation is **not** treated as authoritative for camera-on-UAV perception-driven following, because the simulated camera setup is not representative of the later PX4/real-attached-camera stage.

Recommended run modes:

1. Control validation (authoritative) + YOLO perception in parallel:
```bash
WITH_CAMERAS=true \
LEADER_MODE=odom \
LEADER_PERCEPTION_ENABLE=true \
YOLO_WEIGHTS=/home/william/halmstad_ws/models/yolov5n.pt \
YOLO_DEVICE=cpu \
LEADER_IMAGE_TOPIC=/dji0/camera0/image_raw \
bash scripts/run_round.sh run_odom_follow_yolo_perception_0001 follow dji0 orchard
```

2. Estimate-mode follow (experimental / later-stage tool only):
```bash
WITH_CAMERAS=true \
LEADER_MODE=estimate \
YOLO_WEIGHTS=/home/william/halmstad_ws/models/yolov5n.pt \
YOLO_DEVICE=cpu \
LEADER_IMAGE_TOPIC=/dji0/camera0/image_raw \
bash scripts/run_round.sh run_estimate_experimental_0001 follow dji0 orchard
```

`scripts/run_round.sh` now uses `FOLLOW_PROFILE=auto` by default:
- resolves to `odom_core` in `LEADER_MODE=odom` (closer to Stage 2.0 semantics)
- resolves to `estimate_robust` in `LEADER_MODE=estimate|pose` (Phase 1/2 shaping enabled)

You can override explicitly with:
- `FOLLOW_PROFILE=odom_core`
- `FOLLOW_PROFILE=estimate_robust`

# IMPLEMENTATION LOG

## 2026-03-03 - Robust bag shutdown + controller backend smoothing guard

### What changed
Hardened `run_follow_with_bag.sh` shutdown so rosbag recorder stop is now deterministic: `SIGINT` with timeout, then `SIGTERM`, then `SIGKILL` as final fallback. This prevents Terminal C from hanging when rosbag ignores the first signal.  
Added conservative controller-backend tuning defaults in `follow_uav` that only apply when `uav_backend=controller` and the values still look like baseline defaults; explicit user overrides are preserved.

### Affected mode(s)
- Mode 1 (set_pose): affected only by safer run teardown behavior
- Mode 2 (controller backend): affected by safer teardown + smoother command profile at default settings

### How to verify
1. Static checks (already run):
```bash
cd ~/halmstad_ws
bash -n run_follow_with_bag.sh
python3 -m py_compile src/lrs_halmstad/lrs_halmstad/follow_uav.py
python3 -m py_compile src/lrs_halmstad/launch/run_round_follow_motion.launch.py
```
Expected output:
- `OK:run_follow_with_bag.sh`
- `OK:follow_uav.py`
- `OK:run_round_follow_motion.launch.py`

2. Runtime check for non-hanging shutdown:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash

bash run_follow_with_bag.sh run_mode2_shutdown_check orchard dji0 odom \
  uav_backend:=controller \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu
```
Expected output:
- Run exits back to shell without hanging after launch shutdown.
- If recorder is slow to stop, wrapper prints escalation messages:
  - `rosbag still running after SIGINT; escalating to SIGTERM`
  - `rosbag still running after SIGTERM; escalating to SIGKILL`
- `runs/run_mode2_shutdown_check/bag_info.txt` is written.

3. Optional process check after run:
```bash
pgrep -af "run_follow_with_bag.sh|ros2 bag record" || true
```
Expected output:
- No leftover process from the completed run.

### Topics/services changed (required/optional updates)
- No topic/service contract change.
- New optional env controls in `run_follow_with_bag.sh`:
  - `BAG_STOP_INT_TIMEOUT_S` (default `4`)
  - `BAG_STOP_TERM_TIMEOUT_S` (default `3`)
  - `BAG_STOP_KILL_TIMEOUT_S` (default `2`)
- New optional `follow_uav` params (controller profile):
  - `controller_profile_enable`
  - `controller_profile_smooth_alpha`
  - `controller_profile_max_step_m_per_tick`
  - `controller_profile_cmd_xy_deadband_m`
  - `controller_profile_yaw_deadband_rad`
  - `controller_profile_min_cmd_period_s`

### Files changed in this implementation block
- `run_follow_with_bag.sh`
- `src/lrs_halmstad/lrs_halmstad/follow_uav.py`
- `src/lrs_halmstad/config/run_round_follow_defaults.yaml`
- `Markdowns/Implementation_Documentation/IMPLEMENTATION_LOG.md`

## 2026-03-03 - Two-mode UAV backend integration (`setpose|controller`)

### What changed
Added a neutral backend switch (`uav_backend:=setpose|controller`) to the follow launch path and follow node so one follow pipeline can drive two actuation backends without renaming the rest of the stack.  
Mode 1 (`setpose`) keeps the previous behavior as default. Mode 2 (`controller`) now starts the `simulator` backend node from `run_round_follow_motion.launch.py`, and `follow_uav` publishes controller-interface commands (`psdk_ros2/...`, `update_pan`, `update_tilt`) instead of calling `SetEntityPose` directly.  
Controller seed state in `follow_uav` is synchronized from launch backend initial pose arguments (`backend_initial_x/y/yaw_deg`) to avoid startup mismatch.

Also updated run wrappers and preflight so backend mode is explicit, validated, and recorded (`meta.yaml`), and so mode-specific conflicts are detected earlier.

### Affected mode(s)
- Mode 1 (set_pose): affected (default preserved, now explicit via `uav_backend=setpose`)
- Mode 2 (controller backend): affected (new integrated path via same follow entrypoint)

### How to verify
1. Static syntax/compile checks (already run):
```bash
cd ~/halmstad_ws
bash -n run_follow_with_bag.sh
bash -n run_follow_preflight.sh
python3 -m py_compile src/lrs_halmstad/lrs_halmstad/follow_uav.py src/lrs_halmstad/launch/run_round_follow_motion.launch.py
```
Expected output:
- No syntax errors.
- `OK:run_follow_with_bag.sh`
- `OK:run_follow_preflight.sh`
- `OK:python_compile`

2. Runtime smoke test, Mode 1 (baseline set_pose):
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash

bash run_follow_with_bag.sh run_mode1_test orchard dji0 odom \
  uav_backend:=setpose \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu
```
Expected output:
- Preflight prints `backend=setpose` and ends with `[run_follow_preflight] ok`
- Follow node startup log includes `backend=setpose`
- UGV and UAV move, run ends cleanly if `shutdown_when_ugv_done:=true`
- Run artifacts exist in `runs/run_mode1_test/` with `meta.yaml`, `launch.log`, `rosbag.log`, `bag_info.txt`, bag directory

3. Runtime smoke test, Mode 2 (controller backend):
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash

bash run_follow_with_bag.sh run_mode2_test orchard dji0 odom \
  uav_backend:=controller \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu
```
Expected output:
- Preflight prints `backend=controller` and ends with `[run_follow_preflight] ok`
- Launch output includes simulator process start (controller backend path)
- Follow node startup log includes `backend=controller`
- UAV command topics become active:
  - `/<uav>/psdk_ros2/flight_control_setpoint_ENUposition_yaw`
  - `/<uav>/update_pan`
  - `/<uav>/update_tilt`
- Run artifacts exist in `runs/run_mode2_test/` with same contract as Mode 1

### Topics/services changed (required/optional updates)
- Required launch arg (new):
  - `uav_backend:=setpose|controller` (default: `setpose`)
- Required service dependency (unchanged, still required):
  - `/world/<world>/set_pose` (used by setpose backend and by simulator backend)
- Optional/conditional topics (controller mode):
  - `/<uav>/psdk_ros2/flight_control_setpoint_ENUposition_yaw`
  - `/<uav>/update_pan`
  - `/<uav>/update_tilt`
  - `/<uav>/pose` (simulator-published pose stream)
- Metadata update:
  - `run_follow_with_bag.sh` now writes `uav_backend` into `meta.yaml`

### Files changed in this implementation block
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py`
- `src/lrs_halmstad/lrs_halmstad/follow_uav.py`
- `src/lrs_halmstad/config/run_round_follow_defaults.yaml`
- `run_follow_with_bag.sh`
- `run_follow_preflight.sh`

## 2026-03-03 - Mode 2 runtime fix (simulator import crash)

### What changed
Mode 2 UAV non-movement was traced to `simulator` crashing at startup with `ModuleNotFoundError: lrs_halmstad.world_names`.  
Added missing module `src/lrs_halmstad/lrs_halmstad/world_names.py` with `gazebo_world_name()` so both `simulator.py` and `command.py` imports resolve at runtime.

Also added controller-mode fail-fast in `run_round_follow_motion.launch.py`: if simulator backend exits, launch now shuts down immediately instead of silently continuing with a non-moving UAV.

### Affected mode(s)
- Mode 1 (set_pose): not functionally changed
- Mode 2 (controller backend): fixed (simulator now starts instead of crashing)

### How to verify
1. Build after fix (required for runtime import path):
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select lrs_halmstad
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
```
Expected output:
- Build succeeds with no Python module install errors.

2. Static checks (already run):
```bash
cd ~/halmstad_ws
python3 -m py_compile src/lrs_halmstad/lrs_halmstad/world_names.py \
  src/lrs_halmstad/lrs_halmstad/simulator.py \
  src/lrs_halmstad/lrs_halmstad/command.py \
  src/lrs_halmstad/launch/run_round_follow_motion.launch.py
```
Expected output:
- No syntax/compile errors.

3. Runtime check, Mode 2:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash

bash run_follow_with_bag.sh run_mode2_fixcheck orchard dji0 odom \
  uav_backend:=controller \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu
```
Expected output:
- `simulator` starts without `ModuleNotFoundError`.
- Follow node reports `backend=controller`.
- UAV moves instead of staying at spawn.
- If simulator crashes for any reason, launch terminates quickly with message:
  `[run_round_follow_motion] simulator backend exited in controller mode; shutting down launch`

### Topics/services changed (required/optional updates)
- No topic/service contract changes in this fix.
- Added internal module dependency:
  - `lrs_halmstad.world_names` (now present in package source)

### Files changed in this implementation block
- `src/lrs_halmstad/lrs_halmstad/world_names.py` (new)
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py` (controller-mode fail-fast on simulator exit)

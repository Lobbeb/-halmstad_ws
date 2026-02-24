# Stage 3 YOLO + Follow/Leash Implementation Context (2026-02-24)

## Purpose of This Document
This file is a consolidated implementation log and context handoff for the work done in this session.

It is intended to serve two roles:
- Thesis writing support (what was implemented, how, why, and how it was validated)
- High-quality context seed for a future chat/session so work can continue without re-discovery

This document focuses on **actual implementation and validation work**, not only ad-hoc troubleshooting notes.

---

## Final Project Interpretation (Important Architectural Clarification)
A key clarification was established during this session and is now reflected in the codebase and workflow:

- **Follow + leash** is the authoritative UAV steering/controller behavior.
- **YOLO** is a **perception subsystem** (object/hazard detection, status, detections, logging), not the steering source in the current simulation stage.
- In the current sim, the camera is not fully representative of a real UAV-attached perception setup (and PX4 is not yet active), so using YOLO-derived `/coord/leader_estimate` to steer the UAV is considered **experimental only**.

### Current accepted mode split
- `LEADER_MODE=odom`: authoritative follow+leash control validation path (use this to judge UAV behavior)
- `LEADER_PERCEPTION_ENABLE=true`: run YOLO/leader_estimator in parallel for perception/status/logging only
- `LEADER_MODE=estimate`: experimental branch/tool for later work (especially when PX4/real attachment is in place)

This interpretation reduced confusion and aligned the implementation with the intended project direction.

---

## What Was Implemented (Summary)

### 1) YOLO-based perception node (`leader_estimator.py`) [Stage 3 / Stage 2.2–2.3 integration]
Implemented and extended a YOLO-based leader estimation node that:
- Subscribes to UAV camera image (`camera_topic`, known-good: `/dji0/camera0/image_raw`)
- Subscribes to camera info and optional depth image
- Subscribes to UAV pose with bootstrap fallback (to avoid startup deadlock)
- Publishes:
  - `/coord/leader_estimate` (`geometry_msgs/PoseStamped`)
  - `/coord/leader_estimate_status` (`std_msgs/String`)

Key features implemented:
- **Status-first behavior** (publishes status periodically even before images/detections)
- YOLO disabled mode (status-only, no estimate)
- YOLO diagnostics and explicit `yolo_reason`
- Range modes and source reporting (`const`, `depth`, `ground`, `auto`)
- Depth gating with fallback to constant range
- Ground projection range path (experimental estimate mode support)
- Ground range sanity bounds (`ground_min_range_m`, `ground_max_range_m`)
- Smoothing and hold-last-valid logic
- Tracking/debounce/reject diagnostics in status (`reject_reason`, etc.)
- Bootstrap UAV pose fallback (broke circular dependency with `follow_uav` on `/dji0/pose_cmd`)
- NumPy 2 compatibility (removed `tf_transformations` dependency, replaced with local yaw/quat helpers)

### 2) Follow controller (`follow_uav.py`) robustness/stability work (Phase 1 + Phase 2)
The existing follow+leash concept was preserved, but robustness and motion quality features were added on top.

#### Phase 1 (robustness)
- `max_step_m_per_tick` clamp on commanded XY target (prevents snapping/teleport-like jumps)
- `FOLLOW_STEP_CLAMPED` event emission
- Quality-aware command shaping based on estimator status (confidence, latency, freshness/state)
- Optional quality-based hold behavior
- Yaw clamp, yaw deadband, XY deadband, and yaw update gating
- Int/float parameter override compatibility restored for key timing params (`tick_hz`, `est_hz` in respective nodes)

#### Phase 2 (higher-quality shaping, still same concept)
- Explicit state machine (`TRACK`, `HOLD`, `REACQUIRE`, `DEGRADED`)
- Trajectory generator (position/velocity/acceleration-limited shaping)
- Relative-frame shaping/refinement (reduce coupled world-frame jitter/orbiting)
- Estimate-heading-from-motion logic (in estimate mode) to avoid relying on noisy estimate yaw

Important: these Phase 1/2 improvements were useful and validated, but later re-scoped as primarily relevant to **experimental estimate-mode following**, not the main current control path.

### 3) Experiment/run harness (`scripts/run_round.sh`) improvements
The run harness was significantly improved and hardened.

Implemented features include:
- `LEADER_MODE=estimate` support (experimental estimate-follow mode)
- `LEADER_IMAGE_TOPIC`, YOLO params, depth params, estimator/follower tunables
- Empty ROS parameter override avoidance (`-p x:=` crash prevention)
- Odom flow fallback behavior (retry `odom/filtered` when `odom` has no flow)
- Metadata (`meta.yaml`) append-only logging of estimator/follower/YOLO/run params
- Stronger UGV test motion support (profiles, variation, turn patterns, pauses, rates, etc.)

#### Major UGV motion system fix
A deep root cause was found and fixed:
- The old UGV motion script started a new `ros2 topic pub` process for every short segment.
- DDS startup/discovery overhead often consumed most of the segment duration.
- Result: UGV appeared deadstill or barely moved.

This was fixed by replacing segment-wise CLI publishing with a **persistent Python `rclpy` publisher** that publishes `TwistStamped` continuously for the whole UGV phase.

Result:
- UGV movement became reliable, tunable, and visibly testable
- Aggressive/longer runs became possible and meaningful again

### 4) Cleanup and environment hardening
#### `scripts/kill_everything.sh`
Strengthened cleanup of leftover ROS/Gazebo/bridge/image bridge processes to prevent stale state between runs.

#### `scripts/env.sh`
Hardened environment setup for this workstation/WSL runtime:
- ROS domain consistency
- FastDDS shared-memory mitigation (reducing problematic SHM behavior in this environment)
- Gazebo resource/model path setup

### 5) Spawn/runtime recovery (critical bring-up reliability work)
A major runtime/spawn issue was diagnosed and fixed:

#### Root cause (spawn XML corruption)
- `spawn_uavs.launch.py` (via `spawn_robot.launch.py` / `spawn_gimbal.launch.py`) generated SDF using a `Command(...)` path that invoked `ros2 run lrs_halmstad generate_sdf ...`
- FastDDS SHM warnings were emitted and leaked into the generated SDF string
- Gazebo received `RTPS warning text + XML` instead of pure XML
- Result: `XML_ERROR_PARSING_DECLARATION` and UAV spawn failures

#### Fix
- Updated launch spawn paths to call the installed `generate_sdf` executable directly (instead of `ros2 run ...`) to avoid stdout contamination
- Restored reliable UAV spawning and removed the XML parsing corruption failure mode

This was a key recovery milestone that restored a usable development environment.

---

## Validation and Testing Performed

### A. Baseline / regression path (Stage 2.0 style)
Validated baseline follow run with:
- `bash scripts/run_round.sh run_regress_test follow dji0 orchard`
- Contract checks and bagging restored/working after runtime fixes

### B. YOLO integration / estimate-mode experiments (earlier in session)
Multiple runs validated that the YOLO pipeline itself works end-to-end (experimental estimate-follow mode), including:
- YOLO disabled status-only behavior
- YOLO enabled estimate publishing
- Bags with nonzero `/coord/leader_estimate` and `/coord/leader_estimate_status`

### C. A/B experiment support (YOLOv5n vs YOLOv5s)
A/B workflow implemented and exercised with:
- `~/halmstad_ws/models/yolov5n.pt`
- `~/halmstad_ws/models/yolov5s.pt`
Metadata includes `yolo_variant` and related run settings.

### D. Phase 1 (robustness) quantitative validation
A dedicated `pose_cmd` smoothness comparison showed a strong improvement over a bad/jittery estimate-follow run.

#### Example measured improvements (Phase 1 vs bad estimate-follow baseline)
- **Reversal rate** dropped dramatically (from ~1.0 to ~0.02)
- `step_xy` p90/p95/max reduced sharply (large snap-jumps removed)
- `yaw_step` p90/p95/max reduced sharply (yaw snapping bounded)

Interpretation:
- Phase 1 successfully removed the "snap/teleport" failure mode in command stream behavior.

### E. Phase 2 (trajectory/state-machine/relative-frame shaping) validation
Compared Phase 2 to Phase 1 using `/dji0/pose_cmd` smoothness metrics.

Observed result:
- Phase 2 accepted as a pass
- Further improved or maintained bounded command shaping (especially `step_xy` max and p95)
- Slight increase in some yaw metrics but still well-clamped and acceptable
- Event logs confirmed state machine and clamping features were active

### F. Final authoritative architecture validation (ODOM steering + YOLO perception-only)
This became the **main accepted validation path** for the current sim.

Command used (authoritative current mode):
```bash
WITH_CAMERAS=true \
LEADER_MODE=odom \
LEADER_PERCEPTION_ENABLE=true \
YOLO_WEIGHTS=/home/william/halmstad_ws/models/yolov5n.pt \
YOLO_DEVICE=cpu \
LEADER_IMAGE_TOPIC=/dji0/camera0/image_raw \
LEADER_EST_RANGE_MODE=auto \
LEADER_CAM_PITCH_OFFSET_DEG=-90.0 \
LEADER_GROUND_MIN_RANGE_M=3.0 \
LEADER_GROUND_MAX_RANGE_M=40.0 \
bash scripts/run_round.sh run_odom_follow_yolo_perception_0002 follow dji0 orchard
```

#### Runtime logs confirmed the intended split
- `Starting leader_estimator in perception-only mode (follow uses odom)`
- `Starting follow_uav (leader_mode=odom, follow_profile=odom_core)`

#### GUI result (user observation)
- UGV and UAV behavior looked good in the GUI
- This was the first clean end-to-end run clearly aligned with actual project intent

#### Bag summary (authoritative current-state evidence)
Run: `runs/estimator/run_odom_follow_yolo_perception_0002/bag`
- Duration: ~247.5s
- `/dji0/pose_cmd`: 380
- `/coord/follow_dist_cmd`: 380
- `/coord/follow_tracking_error_cmd`: 380
- `/coord/leader_estimate`: 553
- `/coord/leader_estimate_status`: 623
- UGV odom/tf/cmd_vel topics all active and nonzero

Interpretation:
- Follow+leash steering path is active and healthy (`odom` authoritative)
- YOLO perception pipeline is active in parallel (detections/status/logging)
- The re-scope/cleanup is functioning as intended

---

## Important Lessons / Decisions (for Thesis Narrative)

### 1) Perception pipeline "working" is not the same as control behavior being valid
It is possible to have bags with valid estimates/status while the UAV path quality is poor or misleading in simulation. This occurred when estimate-mode following was interpreted too strongly in a setup where the camera was not fully representative of a UAV-attached perception loop.

### 2) Simulation test setup matters (camera attachment and PX4 realism)
The current sim setup is sufficient to validate:
- follow+leash controller (with odom as leader source)
- YOLO perception module behavior (detections/status/logging)
- runtime/harness robustness

It is **not** a strong validity test for final camera-on-UAV perception-driven following until later PX4/real attachment stages.

### 3) Harness/runtime engineering matters as much as algorithm work
Multiple major blockers were due to tooling/runtime/harness issues (spawn XML corruption, stale processes, UGV motion publisher design), not algorithm bugs. Fixing these was necessary to obtain trustworthy test results.

---

## Current Codebase Alignment (After Cleanup)
The codebase now reflects the intended project direction:

- `LEADER_MODE=odom` + `LEADER_PERCEPTION_ENABLE=true` is the recommended current simulation mode
- `FOLLOW_PROFILE=auto` resolves to `odom_core` in odom mode (keeps behavior close to Stage 2.0 semantics)
- Phase 1/2 shaping remains available for estimate-mode experiments (`estimate_robust`) and future work
- YOLO/estimate path remains implemented and available, but is treated as experimental in current sim

This preserves work already done while avoiding over-committing to a non-representative estimate-follow test path.

---

## Suggested Thesis Framing (Based on Today’s Outcomes)
A strong thesis framing for this stage would be:

- The YOLO perception subsystem was successfully integrated and validated (detections/status/logging/bagging)
- The follow+leash control pipeline remained the authoritative steering method in the current simulation stage
- Robustness and motion-quality improvements were developed and quantitatively evaluated for estimate-driven experiments
- Final validation for this stage uses odom-based following plus parallel YOLO perception, deferring camera-attached/PX4-coupled behavior validation to later phases

This is technically accurate and aligned with the project’s intended architecture.

---

## Practical Runbook (Current Recommended Workflow)

### Bring-up (all terminals use same env)
In every terminal (A/B/C/E):
```bash
cd ~/halmstad_ws
source scripts/env.sh
source install/setup.bash
export ROS_DOMAIN_ID=0
```

### Terminal A (simulation)
```bash
ros2 launch clearpath_gz simulation.launch.py world:=orchard
```

### Terminal B (UAV spawn)
```bash
ros2 launch lrs_halmstad spawn_uavs.launch.py world:=orchard
```

### Terminal C/D (authoritative current validation mode)
```bash
WITH_CAMERAS=true \
LEADER_MODE=odom \
LEADER_PERCEPTION_ENABLE=true \
YOLO_WEIGHTS=/home/william/halmstad_ws/models/yolov5n.pt \
YOLO_DEVICE=cpu \
LEADER_IMAGE_TOPIC=/dji0/camera0/image_raw \
LEADER_EST_RANGE_MODE=auto \
LEADER_CAM_PITCH_OFFSET_DEG=-90.0 \
LEADER_GROUND_MIN_RANGE_M=3.0 \
LEADER_GROUND_MAX_RANGE_M=40.0 \
bash scripts/run_round.sh <run_id> follow dji0 orchard
```

### Cleanup between runs
```bash
bash scripts/kill_everything.sh
```

---

## What Is Ready to Continue Next
Based on the final successful odom+YOLO-perception run and the architectural cleanup:

- **Current stage (follow+leash + YOLO perception integration)** is in a stable, usable state.
- The project is ready to proceed to the next phase (PX4-oriented work / later integration steps), with the current simulation mode serving as a reliable baseline.

---

## Notes for a Future Chat (Context Handoff)
If this document is used to initialize a new chat, the next assistant should assume:
1. The project has already implemented YOLO perception (`leader_estimator.py`) and follow+leash control (`follow_uav.py`).
2. The accepted current sim mode is **odom-follow + YOLO perception-only**.
3. Estimate-mode follow exists but is experimental and should not be over-interpreted in current sim.
4. Spawn/runtime/harness issues were fixed (including SDF XML corruption and UGV motion publisher reliability).
5. The user has already validated a clean run with `run_odom_follow_yolo_perception_0002` and good GUI behavior.


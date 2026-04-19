# Multi-UAV Phase Freeze / Handoff (2026-04-16)

Scope freeze for this phase: stop here.
No `dji0 -> UGV` forwarding yet. No detector reopen. No scope widening.

## 1) Current accepted architecture state

- Dual-support motion baseline around `dji0` is active.
- Support cameras are optional (`support_with_camera:=true|false`).
- Support observation overlay runs instance-safe detection for `dji1` and `dji2`.
- First lightweight aggregation now exists at `dji0` (local merge point only).

## 2) What was actually implemented

Core files:

- `src/lrs_halmstad/launch/support_follow_odom.launch.py`
- `src/lrs_halmstad/launch/support_observation.launch.py`
- `src/lrs_halmstad/lrs_halmstad/perception/support_detection_mux.py`
- `src/lrs_halmstad/setup.py` (console script registration)
- `scripts/run_support_follow_odom.sh`
- `scripts/run_support_observation.sh`

Core runtime nodes for this phase:

- `support_follow_dji0_pose_to_odom`
- `support_follow_dji1_simulator`, `support_follow_dji1_odom_controller`
- `support_follow_dji2_simulator`, `support_follow_dji2_odom_controller`
- `support_dji1_leader_detector`, `support_dji2_leader_detector`
- `support_detection_mux`

Phase-defining topics:

- Support inputs:
  - `/coord/support/dji1/leader_detection`
  - `/coord/support/dji1/leader_detection_status`
  - `/coord/support/dji1/leader_detection_events`
  - `/coord/support/dji2/leader_detection`
  - `/coord/support/dji2/leader_detection_status`
  - `/coord/support/dji2/leader_detection_events`
- New dji0-local merged outputs:
  - `/coord/dji0/leader_detection`
  - `/coord/dji0/leader_detection_status`

## 3) What was validated

Strongly proven:

- Support-follow overlay spawned and ran both support UAV tracks around `dji0`.
- Support observation started two independent detector instances with separated topics.
- Both support streams produced real detection/status content in live runs (including `valid:true` and `state=OK` captures).
- Mux node started in launch, subscribed to both support streams, and published `dji0` merged outputs.
- Merged detection payloads were observed with mux metadata (`mux_source`, `mux_strategy`).

Proven with practical caveats:

- Runtime had intermittent `NO_DET` windows (view/scene/load dependent), so per-sample confidence should be treated as operational, not perfect.
- Some CLI probes were flaky unless explicitly scoped and formatted (domain fixed, sometimes `--field data` needed for clean status capture).

Harness/domain/debugging noise (not treated as architecture regressions):

- ROS domain/daemon drift causing temporary empty graph/topic views.
- `scripts/run_support_observation.sh` preflight checks occasionally hanging in this environment.
- Non-functional log noise from Gazebo ODE contact overflow and Nav2 control-loop warnings/goal failures.

## 4) Current clean boundary

Current system boundary is now:

`dji1/dji2 -> dji0`

with the dji0-local merged outputs:

- `/coord/dji0/leader_detection`
- `/coord/dji0/leader_detection_status`

No forwarding from `dji0` into UGV-facing topics is part of this frozen phase.

## 5) Exact next step later

Next implementation phase (and only next phase):

`dji0 -> UGV` forwarding

using the existing dji0-local merged outputs as the input boundary.

Detector experimentation remains paused.

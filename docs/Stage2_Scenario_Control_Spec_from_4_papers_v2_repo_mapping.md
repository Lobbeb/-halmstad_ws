Stage 2 Scenario & Control Spec (Copilot-Ready)
Purpose: Extract implementation-ready specifications from the selected reference papers (AuNa, Collaborative Inspection, Communication-based Leashing, Heterogeneous Multi-Robot Collaboration) so the code can be implemented in VS Code using Copilot without needing to read the PDFs.
Scope: This document is for development. It converts paper concepts into concrete behavior definitions, topic/message interfaces, parameters, pseudocode, and acceptance tests for your ROS 2 + Gazebo testbed.
# How to use this document with Copilot

Copy the relevant section (Scenario definition + parameters + pseudocode + acceptance tests) into Copilot chat and ask it to implement exactly that, using your existing repository structure and topic names. Avoid asking Copilot to 'read the paper'—instead give it the spec below.
# Your current baseline (Stage 1) assumptions

Baseline control: UAV motion via Gazebo SetEntityPose (deterministic waypoint updates), UGV motion via /cmd_vel primitives.
Baseline logging: rosbag2 (MCAP) records /clock, UGV odom(/filtered), /cmd_vel, /tf(/static), /dji0/pose (PoseStamped), and /coord/events.
Stage 2 should reuse the same run harness + logging + analysis pipeline, and only change the scenario behavior and (later) communication conditions.
# Paper-derived design patterns (what you can safely take)

## 1) AuNa (Teper et al.) — staged co-simulation + distance-error formulation

Use AuNa to justify the staged methodology (baseline scenarios first, communication effects as controlled variables later) and to anchor the 'desired spacing' / 'distance error' concept for following behavior.
Implementation-ready takeaways:
Define a desired distance (or time-gap) between agents/vehicles and compute an error term e = d − d_r.
Use fixed update rates and deterministic parameterization for repeatable evaluation.
Treat network effects as experimental conditions layered on top of a stable robotics scenario.
## 2) Collaborative Inspection (Amorim et al., JIRS 2025) — mission workflow + repeatability

Use this paper as the scenario template for an inspection-style multi-robot workflow and for the idea of fixed initialization parameters for repeatable experiments.
Implementation-ready takeaways:
Separate roles: UGV performs ground navigation and local tasks; UAV performs aerial inspection phases.
Use fixed initial conditions and waypoint-like routes to ensure experiment repeatability.
Expose explicit phase boundaries for later offline segmentation (e.g., START/END events).
## 3) Communication-based Leashing (Hauert et al., ICRA 2010) — keep within comm range (leash)

Use this paper to justify a minimal following strategy based on communication constraints: keep the UAV within a maximum distance to maintain link quality, and use a target distance inside that bound.
Implementation-ready takeaways:
Leash constraint: enforce distance(UAV, UGV) ≤ d_max (communication range surrogate).
Goal: keep distance close to a desired value d_target for stability and coverage.
Fallback behavior when out of range: move directly toward the leader until reconnected/within bounds.
## 4) Heterogeneous Multi-Robot Collaboration (de Castro et al., Machines 2024) — coordination triggers + comm-loss routines

Use this paper to justify bidirectional coordination messages (request/ack), role switching (UGV triggers UAV actions), and explicit behaviors under communication degradation (stop/wait/return).
Implementation-ready takeaways:
Define a small coordination vocabulary: REQUEST, ACK, EXECUTE, DONE (and optional COMM_LOST/RESUME).
Keep coordination deterministic by triggering UAV actions based on UGV reaching specific waypoints/conditions.
Define safe fallback under comm loss (for simulation: hover/wait at last known safe pose).
# Stage 2 candidate scenarios (choose one as your next implementation)

## Scenario B1: UAV follows UGV with fixed standoff (leash + spacing)

Goal: Replace the placeholder grid survey with a more thesis-aligned behavior: the UAV maintains a fixed horizontal standoff distance behind the UGV while the UGV executes its (deterministic) path.
### B1 Parameters (copy to Copilot)


### B1 Inputs/Outputs (ROS 2 interfaces)

### B1 Behavior definition (deterministic, no randomness)

At each control tick:
• Read latest UGV pose (x_g, y_g) and heading yaw_g from /odom.
• Compute a target point behind UGV along its heading: (x_t, y_t) = (x_g, y_g) − d_target * [cos(yaw_g), sin(yaw_g)].
• Compute current horizontal distance d = ||(x_u, y_u) − (x_g, y_g)|| if UAV pose known; otherwise assume last commanded UAV pose.
• If d > d_max: override target point to move toward UGV more aggressively (e.g., use d_target = min(d_target, d_max/2) or just target (x_g, y_g) with safe offset).
• Set UAV target pose to (x_t, y_t, z_alt) and yaw = yaw_g (optional).
• Call /world/orchard/set_pose for entity dji0 with the target pose.
• Publish /dji0/pose (PoseStamped) as a burst (5 samples) to guarantee rosbag capture.
### B1 Pseudocode (copy to Copilot)

# tick loop at tick_hz
odom = latest_odom()
xg, yg = odom.pose.pose.position.x, odom.pose.pose.position.y
yaw_g = yaw_from_quaternion(odom.pose.pose.orientation)

# target behind UGV
xt = xg - d_target * cos(yaw_g)
yt = yg - d_target * sin(yaw_g)

# optional leash constraint
d = hypot(last_uav_x - xg, last_uav_y - yg)
if d > d_max:
    # pull back inside range
    xt = xg - (d_max/2.0) * cos(yaw_g)
    yt = yg - (d_max/2.0) * sin(yaw_g)

set_pose(entity='dji0', x=xt, y=yt, z=z_alt, yaw=yaw_g)

publish_pose_burst('/dji0/pose', xt, yt, z_alt, yaw_g, count=5)
publish_event('/coord/events', 'FOLLOW_TICK')
### B1 Acceptance tests (definition of done)

• Running one round produces a rosbag containing /dji0/pose with > 0 messages during the UAV phase.
• Mean standoff distance error |d − d_target| is finite and stable (computed offline).
• Distance constraint respected: max distance(UAV, UGV) ≤ d_max + small epsilon (offline check).
• Round remains deterministic: repeated runs with same parameters produce identical commanded UAV target sequence (within float tolerance).
## Scenario B2: Inspection workflow with triggers (UGV route triggers UAV actions)

Goal: Keep the UGV on a deterministic route; trigger UAV inspection actions at predefined UGV milestones. This aligns closely with the inspection workflow literature and is easy to make deterministic.
### B2 Minimal coordination vocabulary (copy to Copilot)

Use /coord/events (String) or define a dedicated topic. Minimal set:
• INSPECTION_REQUEST:<id>
• INSPECTION_ACK:<id>
• INSPECTION_DONE:<id>
• COMM_LOST
• RESUME
### B2 Definition of done

• At least one trigger point exists (e.g., when UGV reaches waypoint i) that causes an INSPECTION_REQUEST and UAV action phase.
• Events appear in rosbag and allow clean offline segmentation of phases.
• Behavior is repeatable across runs.
# Notes for your BibTeX

You will add BibTeX entries for the following two PDFs (new to your library):
• Communication-based Leashing of Real Flying Robots (ICRA 2010) — Hauert et al.
• Heterogeneous Multi-Robot Collaboration for Coverage Path Planning in Partially Known Dynamic Environments (Machines 2024) — de Castro et al.
# What not to claim (until implemented)

• Do not claim continuous UAV flight control or autopilot integration unless verified end-to-end.
• Do not claim OMNeT++ network impairment injection is active unless a run demonstrates it and metadata stores the configuration.
• Do not claim perception/YOLO is integrated unless you have a node producing detections and logging it.

# Repo Mapping (plug-and-play for your current codebase)

This section maps the Stage 2 scenario spec directly to your repository files and entrypoints. Use it as the 'where to implement' guide and as context you paste into Copilot so it edits the correct places.
## Entrypoints and environment setup

Entrypoint (single round): scripts/run_round.sh
Environment setup: scripts/env.sh (sourced by run scripts)
## UAV set-pose control (current implementation)

File: src/lrs_halmstad/lrs_halmstad/command.py
Function: main()
Gazebo service: /world/{WORLD_NAME}/set_pose
Service message type: gz.msgs.Pose
World name is parameterised; primary world is orchard; alternative is minimal.
## UAV pose feedback stream (logging guarantee)

File: src/lrs_halmstad/lrs_halmstad/command.py
Location: main() (lines ~73–80)
Topic: /dji0/pose (geometry_msgs/PoseStamped)
Mechanism: publish a 5-message burst after each SetEntityPose call so rosbag captures pose samples even when simulator pose topics do not update on teleport.
## Coordination events (phase segmentation)

Method: published by bash using ros2 topic pub
File: scripts/run_round.sh
Helper: publish_event()
Topic: /coord/events (std_msgs/String)
## UGV odometry input (recommended for follow logic)

Recommended: /a201_0000/platform/odom/filtered (nav_msgs/Odometry)
Alternative: /a201_0000/platform/odom (nav_msgs/Odometry)
## Run output layout (artifacts you should preserve)

Run root: ~/runs/<run_id>/
Rosbag folder: bag/ (MCAP)
Metadata: meta.yaml
# Where to implement Scenario B1 (Follow + Leash)

Keep scripts/run_round.sh as the orchestrator. Implement the follow behavior as a new command mode inside command.py OR as a small new node that run_round.sh launches before starting the round.
## Option 1 (lowest friction): add a new command mode in command.py

Add a command argument such as command:=follow that:
• subscribes to /a201_0000/platform/odom/filtered
• runs a fixed-rate tick loop (e.g., 5 Hz)
• computes target pose behind the UGV (d_target, z_alt) with leash constraint (d_max)
• calls SetEntityPose for dji0 each tick
• publishes /dji0/pose burst after each setpose (reuse your existing burst code)
• terminates cleanly on Ctrl+C
## Option 2 (cleaner separation): new node follow_uav.py

Create src/lrs_halmstad/lrs_halmstad/follow_uav.py and launch it from run_round.sh. This keeps command.py stable and isolates Stage 2 logic.
# Copilot prompts (tailored to your repo)

Use these prompts as-is. Replace only parameter values if needed.
## Prompt A: implement command:=follow in command.py

In this repo, the round entrypoint is scripts/run_round.sh and env setup is scripts/env.sh.
UAV SetEntityPose is implemented in src/lrs_halmstad/lrs_halmstad/command.py in main().
That file already publishes /dji0/pose (geometry_msgs/PoseStamped) as a 5-message burst after each setpose (lines ~73–80).
Implement a new command mode in command.py: command:=follow.

Spec:
- Subscribe to /a201_0000/platform/odom/filtered (nav_msgs/Odometry).
- Run a fixed tick loop at 5 Hz (no randomness).
- Compute UGV yaw from odom quaternion.
- Compute a target point behind the UGV: (xt, yt) = (xg, yg) - d_target*[cos(yaw), sin(yaw)].
- Use z_alt = 10.0 m constant altitude.
- Leash constraint: if current distance d(UAV,UGV) > d_max, pull UAV back inside range by targeting (xg,yg) - (d_max/2)*[cos(yaw), sin(yaw)].
- Call /world/{WORLD_NAME}/set_pose for entity 'dji0' every tick (WORLD_NAME is taken from existing args, default orchard).
- After every setpose, publish the existing /dji0/pose 5-message burst (reuse existing code).
- Make it terminate cleanly on Ctrl+C.

Also update scripts/run_round.sh to call this command mode during the UAV phase instead of the grid survey (keep /coord/events markers unchanged).
## Prompt B: update run_round.sh minimal changes

Update scripts/run_round.sh (entrypoint) so the UAV phase runs the new follow behavior.
Currently /coord/events are published via publish_event() and UAV is moved via command.py setpose grid survey.
Change only the UAV phase section:
- Publish UAV_PHASE_START
- Launch the follow command (use the same argument style your script already uses for command.py):
  ros2 run lrs_halmstad command --ros-args -p command:=follow -p world:=orchard
- Let it run for a fixed duration (e.g., 60s) while UGV executes its deterministic motion (or run it after UGV, but keep consistent)
- Stop the follow process cleanly
- Publish UAV_PHASE_END
Do not change logging start/stop, bag topics, or metadata writing.
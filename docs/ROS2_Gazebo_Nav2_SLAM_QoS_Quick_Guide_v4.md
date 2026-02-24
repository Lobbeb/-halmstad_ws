ROS 2 / Gazebo / Nav2 / SLAM / QoS Quick Guide (for Copilot prompts)
Purpose: Condense the most relevant configuration concepts and practical steps from the official documentation pages you listed. This is meant as a working reference for your thesis testbed setup (after Task 3 / OMNeT++ integration) and for feeding Copilot concise, correct context.
Generated: 2026-02-06

# Nav2 Configuration Guide

Source: https://docs.nav2.org/configuration/index.html

## What this page is

Reference index for Nav2 servers and their YAML parameters (planner, controller, behavior tree navigator, costmaps, lifecycle management, etc.). It also links to core concepts such as costmap layers, robot footprint, state estimation, and plugin selection.
## Why it matters for your testbed

If you want a stable, repeatable UGV navigation baseline (before ML planners), Nav2 is the canonical ROS 2 stack. Your experimental validity depends on correct TF/frames, costmap tuning, and consistent planner/controller parameters across runs.
## Key takeaways

- Nav2 is composed of multiple servers (planner, controller, behavior, smoother, etc.) configured via parameter YAML files.
- Environmental representation is typically a 2D costmap (global + local) with layers (obstacles, inflation, etc.).
- Robot footprint (or radius) must match the simulated robot for collision checking to be meaningful.
- Behavior Tree (BT) is used as the high-level navigation logic; changes here can change timing/behavior across runs.
## What to do in your project (practical checklist)

1. Keep a single versioned Nav2 params YAML for your experiment runs (do not hand-tune interactively without recording changes).
1. Lock down footprint/radius, costmap resolution, inflation radius, obstacle sources (e.g., LiDAR), and controller parameters.
1. Decide a baseline planner/controller plugin combo (e.g., NavFn/Smac + DWB/RPP) and document why.
1. Add Nav2 params file path + git commit hash into your run metadata (same place you store condition/batch info).
## Common pitfalls

- Frame mismatch (map/odom/base_link) causes navigation to fail or drift silently.
- Overfitting parameters to one world; keep scenario-fixed configs or explicitly version per scenario.
- Not recording parameter changes -> results become non-reproducible.
## Copilot-friendly prompt snippet

We use Nav2 as a baseline navigation stack in ROS 2. Please propose a minimal, reproducible Nav2 params YAML structure for a Husky-like UGV in Gazebo, including global/local costmap basics, footprint, planner+controller plugin selection, and lifecycle. Assume we log runs with metadata and need configs to be versioned and stable.
## Commands, examples, and templates

Baseline sanity checks (use in any terminal):
  ros2 topic list
  ros2 node list
  ros2 service list
  ros2 topic info <topic> -v
  ros2 topic hz <topic>
  ros2 topic echo <topic> --once

TF checks:
  ros2 run tf2_ros tf2_echo <from_frame> <to_frame>
  ros2 run tf2_tools view_frames

Rosbag checks:
  ros2 bag info <bag_dir>
  ros2 bag play <bag_dir>

Nav2 launch (typical pattern):
  ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=/path/to/nav2_params.yaml

Lifecycle / server status:
  ros2 lifecycle nodes
  ros2 lifecycle get /controller_server

Dump parameters for reproducibility:
  ros2 param dump /controller_server > controller_server.params.yaml
### Minimal Nav2 params YAML skeleton (template)

nav2_bringup:
  ros__parameters:
    use_sim_time: true

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"

planner_server:
  ros__parameters:
    use_sim_time: true
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 6
      height: 6
      plugins: ["obstacle_layer", "inflation_layer"]

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: map
      robot_base_frame: base_link
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

# Nav2 First-Time Robot Setup Guide

Source: https://docs.nav2.org/setup_guides/index.html

## What this page is

Step-by-step checklist for getting a robot ready for Nav2: TF setup, URDF/SDF in Gazebo, odometry, robot_localization, sensors, mapping/localization, footprint, and plugin setup.
## Why it matters for your testbed

This is the quickest way to verify you have the required “plumbing” correct. It prevents wasting time tuning planners when the real problem is missing TF, bad odom, or wrong sensor frames.
## Key takeaways

- Transforms are foundational: you must have a consistent TF tree (map/odom/base_link and sensor frames).
- Odometry and state estimation must be stable; robot_localization can smooth/fuse odom inputs.
- Sensors in Gazebo must publish in correct frames and rates; Nav2 depends on consistent observation sources.
- Footprint configuration affects collision checking and feasibility of paths.
## What to do in your project (practical checklist)

1. Confirm the TF chain for the Husky in sim: base_link, odom, and any sensor frames used by costmaps.
1. Verify odometry quality and rate (and if using robot_localization EKF, confirm it publishes filtered odom).
1. Ensure LiDAR topics (or simulated scan sources) are correctly framed and stable for obstacle layers.
1. Document the exact frame names you will treat as canonical in the thesis (and enforce them in launch files).
## Common pitfalls

- Multiple odom sources publishing competing TF (double-publishing transforms).
- Laser/camera frame names not matching what costmap params expect.
- Incorrect robot footprint/radius leading to unexpected collision behavior.
## Copilot-friendly prompt snippet

Use Nav2 First-Time Robot Setup as a checklist. Given a Gazebo-simulated Husky UGV, list concrete verification steps (commands/topics/TF checks) to confirm transforms, odometry, sensors, and footprint are correctly configured for Nav2.
## Commands, examples, and templates

Baseline sanity checks (use in any terminal):
  ros2 topic list
  ros2 node list
  ros2 service list
  ros2 topic info <topic> -v
  ros2 topic hz <topic>
  ros2 topic echo <topic> --once

TF checks:
  ros2 run tf2_ros tf2_echo <from_frame> <to_frame>
  ros2 run tf2_tools view_frames

Rosbag checks:
  ros2 bag info <bag_dir>
  ros2 bag play <bag_dir>

Concrete Nav2 readiness checks:
  # frame chain examples
  ros2 run tf2_ros tf2_echo odom base_link
  ros2 run tf2_ros tf2_echo base_link <sensor_frame>

  # odometry stream
  ros2 topic hz /a201_0000/platform/odom
  ros2 topic echo /a201_0000/platform/odom --once

  # cmd_vel acceptance (stop with Ctrl-C)
  ros2 topic pub /a201_0000/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10

# Nav2 Tutorial: Navigating while Mapping (SLAM)

Source: https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html

## What this page is

A practical integration pattern for running Nav2 while generating a map with SLAM (commonly slam_toolbox). It outlines the basic launch sequence and workflow for mapping while navigating.
## Why it matters for your testbed

Your requirements mention LiDAR mapping and path planning. This tutorial gives you a conservative baseline pipeline (Nav2 + SLAM) that can serve as a reference condition before introducing ML planners or comms-impaired conditions.
## Key takeaways

- A baseline mapping+navigation loop typically requires a robot interface bring-up, Nav2 launch, and SLAM launch.
- SLAM outputs the map and TF that Nav2 consumes for global planning.
- The workflow encourages standard interfaces and launch separation (robot bring-up vs nav vs SLAM).
## What to do in your project (practical checklist)

1. Decide whether Stage 1 uses a fixed map (static world) or online SLAM; write it down as an experimental choice.
1. If using SLAM, define the topics and frames: /scan, /odom, base_link, map.
1. Record SLAM-related outputs in rosbag when you start evaluating mapping (e.g., map updates, TF).
1. Keep mapping parameters versioned and in run metadata (similar to Nav2 params).
## Common pitfalls

- Trying SLAM + heavy logging + GUI in WSL2 may degrade performance; prefer headless logging.
- Unstable /scan timing or bad TF causes SLAM divergence and makes navigation unreliable.
## Copilot-friendly prompt snippet

Propose a clean launch architecture for 'Nav2 + slam_toolbox' in Gazebo: bring-up launch, nav2 launch, slam launch. List required topics/frames and what to record in rosbag for reproducible evaluation.
## Commands, examples, and templates

Baseline sanity checks (use in any terminal):
  ros2 topic list
  ros2 node list
  ros2 service list
  ros2 topic info <topic> -v
  ros2 topic hz <topic>
  ros2 topic echo <topic> --once

TF checks:
  ros2 run tf2_ros tf2_echo <from_frame> <to_frame>
  ros2 run tf2_tools view_frames

Rosbag checks:
  ros2 bag info <bag_dir>
  ros2 bag play <bag_dir>

Canonical launch ordering (Nav2 tutorial pattern):
  # 0) Robot interfaces
  ros2 launch <robot_bringup_pkg> <robot_launch>.py

  # 1) Nav2 (SLAM provides /map and map->odom)
  ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True

  # 2) SLAM Toolbox
  ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

Quick checks:
  ros2 topic echo /map --once
  ros2 run tf2_ros tf2_echo map odom

# slam_toolbox (Jazzy) Documentation

Source: https://docs.ros.org/en/ros2_packages/jazzy/api/slam_toolbox/

## What this page is

Package documentation for slam_toolbox, a 2D SLAM solution commonly used with Nav2. It includes nodes, capabilities, and how the package is structured in ROS 2.
## Why it matters for your testbed

If your UGV is expected to 'scan and map', slam_toolbox is a standard baseline. It provides map building outputs that you can later compare against ML-assisted approaches (and test under comms impairment).
## Key takeaways

- slam_toolbox is often used for 2D laser-based SLAM in ROS 2 systems.
- It is typically paired with Nav2 for navigation while mapping or for generating a map for later localization.
- Correct /scan, /odom, and TF are prerequisites; SLAM quality degrades quickly if those are inconsistent.
## What to do in your project (practical checklist)

1. Identify the laser scan topic in your Husky sim (or add a simulated LiDAR if missing).
1. Verify scan frame_id and TF from base_link to the laser frame.
1. Select a baseline slam_toolbox mode for your experiments (online mapping vs building a map then using localization).
1. Add map-related topics to recording only when you reach the mapping evaluation stage (avoid unnecessary data now).
## Common pitfalls

- Feeding SLAM with filtered odom that is not consistent with TF can cause mapping drift.
- Not matching scan frame ids and TF causes silent failures.
## Copilot-friendly prompt snippet

We want a minimal slam_toolbox config for Gazebo Husky LiDAR mapping. List required topics (/scan, /tf, /odom) and common parameters to set, and show how to integrate it with Nav2 for a baseline mapping pipeline.
## Commands, examples, and templates

Baseline sanity checks (use in any terminal):
  ros2 topic list
  ros2 node list
  ros2 service list
  ros2 topic info <topic> -v
  ros2 topic hz <topic>
  ros2 topic echo <topic> --once

TF checks:
  ros2 run tf2_ros tf2_echo <from_frame> <to_frame>
  ros2 run tf2_tools view_frames

Rosbag checks:
  ros2 bag info <bag_dir>
  ros2 bag play <bag_dir>

Typical SLAM Toolbox launch:
  ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

With custom params:
  ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True params_file:=/path/to/slam_params.yaml

Requires:
  - LaserScan topic (commonly /scan)
  - TF: odom->base_link and base_link->laser

# ros_gz_bridge (Jazzy) Documentation

Source: https://docs.ros.org/en/jazzy/p/ros_gz_bridge/

## What this page is

Official documentation for bridging message types between Gazebo Transport and ROS 2. It includes the parameter_bridge tool and configuration-file based bridging.
## Why it matters for your testbed

In a Gazebo-based testbed, many sensor streams and state streams originate in Gazebo. Bridging is the clean way to expose them to ROS 2 nodes (Nav2, SLAM, logging, custom coordination).
## Key takeaways

- parameter_bridge can be launched with a YAML config file to define multiple bridges at once.
- Bridging is typed: you must map compatible Gazebo and ROS message types.
- A config-driven bridge is easier to version-control than ad-hoc CLI bridges.
## What to do in your project (practical checklist)

1. Create a versioned bridge config YAML for the streams you actually need (LiDAR, camera, model poses if relevant).
1. Keep the bridge in a launch file so experiments always start with the same wiring.
1. Add a quick 'bridge health check' step in your runbook (topic list + message echo).
1. If you later use OMNeT++ to impair comms, keep the bridge stable and treat the impairment as a separate layer.
## Common pitfalls

- Assuming a Gazebo topic exists or publishes at the rate you expect; verify with gz tools and ROS topic echo.
- Mismatched message types or wrong topic names -> bridge runs but nothing flows.
## Copilot-friendly prompt snippet

Create a ros_gz_bridge parameter_bridge YAML config template for our project. We want to bridge a simulated LiDAR scan and optionally a camera image stream into ROS 2, and keep it versioned for reproducible experiments.
## Commands, examples, and templates

Bridge syntax: /TOPIC@ROS_TYPE@GZ_TYPE

Examples (from official docs):
  ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
  ros2 run ros_gz_bridge parameter_bridge /rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image

Config file mode:
  ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/path/to/bridge.yaml
### bridge.yaml (template)

bridges:
  - ros_topic_name: "/scan"
    gz_topic_name: "/scan"
    ros_type_name: "sensor_msgs/msg/LaserScan"
    gz_type_name: "gz.msgs.LaserScan"
    direction: GZ_TO_ROS

# ros_gz Meta-package (Jazzy)

Source: https://docs.ros.org/en/jazzy/p/ros_gz/

## What this page is

Top-level ROS 2 package grouping for Gazebo integration components (including ros_gz_bridge). This page is mainly a package index pointer.
## Why it matters for your testbed

Useful for orienting which package provides which piece of Gazebo↔ROS functionality and keeping dependencies consistent in Jazzy.
## Key takeaways

- ros_gz is an umbrella for ROS 2 Gazebo integration packages.
- It helps you track versions and dependencies when setting up a consistent toolchain.
## What to do in your project (practical checklist)

1. Pin and document the ROS 2 distro and ros_gz versions used in your environment (store in run metadata / thesis appendix).
1. When something breaks after updates, check ros_gz/bridge versions first.
## Common pitfalls

- Mixing packages from different distros or versions can cause subtle bridge issues.
## Copilot-friendly prompt snippet

We use ROS 2 Jazzy and ros_gz for Gazebo integration. Explain which ros_gz packages matter for bridging sensors/state into ROS 2 and how to keep versions consistent.
## Commands, examples, and templates

Baseline sanity checks (use in any terminal):
  ros2 topic list
  ros2 node list
  ros2 service list
  ros2 topic info <topic> -v
  ros2 topic hz <topic>
  ros2 topic echo <topic> --once

TF checks:
  ros2 run tf2_ros tf2_echo <from_frame> <to_frame>
  ros2 run tf2_tools view_frames

Rosbag checks:
  ros2 bag info <bag_dir>
  ros2 bag play <bag_dir>

# ROS 2 QoS Concepts: About Quality of Service settings

Source: https://docs.ros.org/en/galactic/Concepts/About-Quality-of-Service-Settings.html

## What this page is

Conceptual explanation of ROS 2 QoS policies (history/depth, reliability, durability, liveliness, deadline, lifespan, etc.) and how incompatibilities prevent connections.
## Why it matters for your testbed

Once you introduce controlled network impairment (OMNeT++), QoS becomes a major factor affecting delivery ratio, latency, and behavior. Even before OMNeT++, choosing QoS for events vs sensors vs control improves determinism and robustness.
## Key takeaways

- Default QoS often uses keep_last(depth=10), reliable reliability, and volatile durability.
- Sensor data commonly uses best_effort + smaller queues to prefer timeliness over completeness.
- QoS incompatibility can prevent publisher/subscriber from matching; then no messages flow.
- Deadline/liveliness/lease duration affect compatibility and can be used to detect missing publishers.
## What to do in your project (practical checklist)

1. Define QoS profiles for each message class: control, events, sensors, state estimation.
1. Version-control QoS decisions (document in thesis + store in run configuration).
1. When you start impairment experiments, run explicit QoS sensitivity tests (reliable vs best_effort).
## Common pitfalls

- Assuming 'reliable' always gives better results; it may increase latency under loss.
- Publisher/subscriber not matching QoS -> silent no-data unless you check.
## Copilot-friendly prompt snippet

We need QoS profiles for a ROS 2 multi-robot simulation (events, cmd_vel, pose, LiDAR, camera). Propose QoS settings for each (reliability, history, depth, durability) and explain trade-offs for lossy links.
## Commands, examples, and templates

Inspect QoS on a live system:
  ros2 topic info /topic_name -v

Quick rules of thumb:
  - Sensors (camera/LiDAR): best_effort + small depth
  - Events / coordination: reliable + small/moderate depth
  - Static data / late joiners: transient_local durability

# ROS 2 QoS Demo: Using QoS for lossy networks

Source: https://docs.ros.org/en/galactic/Tutorials/Demos/Quality-of-Service.html

## What this page is

Hands-on tutorial demonstrating QoS behavior under lossy or congested networks and how different policies change message delivery.
## Why it matters for your testbed

This is a practical reference for designing your later impairment experiments. It helps interpret results like 'delivery ratio' under different QoS settings and validates that your pipeline can observe changes.
## Key takeaways

- The demo provides a structured way to test QoS settings and observe effects when network traffic increases.
- It illustrates why best_effort vs reliable can behave very differently under loss.
## What to do in your project (practical checklist)

1. Re-run the demo in your environment once to validate you understand QoS and how to measure effects.
1. Reuse the same measurement approach (events + rosbag + analysis) when you start OMNeT++ impairment tests.
## Common pitfalls

- Not controlling background traffic makes results hard to interpret.
## Copilot-friendly prompt snippet

Summarize a repeatable QoS experiment plan based on the ROS 2 lossy network demo: what nodes to run, what traffic to add, which QoS settings to compare, and what metrics to log.
## Commands, examples, and templates

QoS lossy-network demo (from docs):
  ros2 run image_tools showimage
  ros2 run image_tools cam2image

Publisher without camera:
  ros2 run image_tools cam2image --ros-args -p burger_mode:=True

Switch publisher to best-effort reliability:
  ros2 run image_tools cam2image --ros-args -p reliability:=best_effort

# Gazebo: Use ROS 2 to interact with Gazebo (integration overview)

Source: https://gazebosim.org/docs/latest/ros2_integration/

## What this page is

Gazebo documentation describing how ROS 2 interacts with Gazebo via the Gazebo ROS packages (including bridging, simulation interfaces, and typical workflows).
## Why it matters for your testbed

This is the canonical reference for the Gazebo-side of your testbed. It helps you reason about what originates in Gazebo (sensors, entity states) versus what is native ROS, and where bridging is required.
## Key takeaways

- Gazebo provides simulation entities and sensor generation; ROS 2 nodes consume these via integration packages.
- Bridging is a core mechanism for exchanging data between Gazebo and ROS 2.
## What to do in your project (practical checklist)

1. Document which streams are Gazebo-native vs ROS-native in your thesis (helps justify the architecture).
1. Prefer launch-file based wiring to keep experiments reproducible.
## Common pitfalls

- Assuming ROS topic naming/typing matches Gazebo conventions; always verify actual topics.
## Copilot-friendly prompt snippet

Explain the ROS 2 ↔ Gazebo architecture for a simulation testbed. We need to know which parts are Gazebo-native, which are ROS-native, and where ros_gz_bridge fits.
## Commands, examples, and templates

Gazebo topic discovery:
  gz topic -l
  gz topic -e -t /some/topic

Manual bridge example (from docs):
  ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan

# Clearpath: Installing the Clearpath Simulator

Source: https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/install/

## What this page is

Clearpath’s short install guidance for the Clearpath simulator packages on ROS 2 (Jazzy).
## Why it matters for your testbed

Your UGV is a Clearpath platform. Using the vendor-supported simulator packages reduces integration risk and keeps launch/config behavior consistent.
## Key takeaways

- Clearpath provides ROS 2 simulator packages installable via apt for Jazzy.
- Following their install path helps match their expected configuration layout.
## What to do in your project (practical checklist)

1. Record the exact package versions used (apt package list) for reproducibility.
1. Keep the Clearpath robot.yaml location and contents stable across runs.
## Common pitfalls

- Mixing vendor packages with custom configs without recording changes leads to drift across machines.
## Copilot-friendly prompt snippet

We run a Clearpath Husky in simulation. List what to verify after means of installation: robot config location, launch success, topics like /cmd_vel and /odom.
## Commands, examples, and templates

Install ROS↔Gazebo vendor pairing (Jazzy + Harmonic):
  sudo apt-get install ros-jazzy-ros-gz

Install Clearpath simulator:
  sudo apt-get update
  sudo apt-get install ros-jazzy-clearpath-simulator

# Clearpath: Configuration Generators

Source: https://docs.clearpathrobotics.com/docs/ros/config/generators/

## What this page is

Overview of Clearpath configuration generators that produce launch files and parameter YAMLs from the robot configuration YAML.
## Why it matters for your testbed

This is relevant if you want a clean, scalable configuration workflow (especially when you start changing sensors or adding nodes). It also clarifies where platform extras and parameters live.
## Key takeaways

- Launch generator: generates Python launch files (platform and sensor launch) from the robot config YAML.
- Parameter generator: generates ROS parameter YAML files; defaults can be overridden via robot config YAML.
- Platform extras: generator can create launch for extra platform nodes in a standard location.
## What to do in your project (practical checklist)

1. Decide whether you will rely on generators or keep hand-written launch files; document that choice.
1. If using generators, keep robot config YAML versioned and treat generated outputs as build artifacts.
1. When adding sensors (LiDAR/cameras), track which layer owns the config (generator vs custom package).
## Common pitfalls

- Editing generated files directly (they may be overwritten).
- Not recording which robot config YAML produced which generated launch/params.
## Copilot-friendly prompt snippet

Explain how Clearpath generators produce launch and parameter files from robot.yaml. We want a reproducible configuration workflow and to know where to add extra nodes/sensors safely.
## Commands, examples, and templates

Generator workflow rule (from docs):
  - Do NOT edit generated files directly.
  - Edit robot.yaml, then regenerate.

Reproducibility rule:
  - Version-control robot.yaml (and any custom overrides).
  - Treat generated launch/params/setup.bash as build artifacts.

# Nav2 Tuning Guide

Source: https://docs.nav2.org/tuning/index.html

## What this page is

Nav2’s opinionated tuning guidance: how to adjust the most important parameters once your stack is basically working (costmaps, controller behavior, recovery behaviors, etc.).
## Why it matters for your testbed

Once your baseline navigation runs, tuning is the difference between a robot that technically moves and a robot that behaves consistently enough for experiments. It also helps you avoid accidental 'parameter fishing' by keeping changes structured and documented.
## Key takeaways

- Tuning assumes your TF/odom/sensors are correct; fix fundamentals before tuning.
- Costmap settings (resolution, inflation, obstacle sources) heavily shape feasible paths.
- Controller parameters affect smoothness, oscillations, and goal-reaching reliability.
- Recovery behaviors and BT settings can change timings; treat them as experimental factors.
## What to do in your project (practical checklist)

1. Freeze a baseline params YAML before tuning; always version-control changes.
1. Tune in a fixed scenario first (same world + same start pose + same goal set).
1. Log the exact params file and commit hash into your run metadata.
1. Define a small set of tuning metrics (success rate, time-to-goal, path length, oscillations).
## Common pitfalls

- Changing multiple parameter families at once; you won’t know what helped.
- Tuning on one map/world then reporting general conclusions.
- Not recording params versions for each run batch.
## Commands, examples, and templates

Parameter diff workflow (recommended):
  # keep baseline file
  cp nav2_params.yaml nav2_params_baseline.yaml
  # after tuning
  diff -u nav2_params_baseline.yaml nav2_params.yaml > nav2_params_changes.patch

Quick nav sanity checks:
  ros2 topic echo /tf --once
  ros2 topic echo /odom --once
  ros2 lifecycle nodes
  ros2 lifecycle get /controller_server
## Copilot-friendly prompt snippet

We have a working Nav2 baseline in Gazebo. Propose a structured tuning plan that changes one parameter family at a time (costmaps, controller, planner, recoveries), and define what metrics to log for each change.

# Nav2 Setup Guide: robot_localization (EKF)

Source: https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html

## What this page is

Nav2’s guide for using robot_localization (ekf_node) to fuse odometry and publish odom->base_link TF consistently for navigation.
## Why it matters for your testbed

A stable odom->base_link TF and filtered odometry are prerequisites for Nav2 and for repeatable motion metrics. If your odometry is noisy or inconsistent, navigation and SLAM results become hard to interpret.
## Key takeaways

- robot_localization ekf_node can fuse multiple sources and publish filtered odometry.
- Correct TF publication (odom->base_link) is critical; avoid double-publishing TF.
- The EKF is configured via a YAML file; record and version it like other experiment configs.
## What to do in your project (practical checklist)

1. Check whether your sim already provides filtered odometry (you already log /odom/filtered).
1. If you enable robot_localization, ensure only one node publishes odom->base_link TF.
1. Version-control the EKF YAML and store its path/version in run metadata.
1. Validate rate stability with ros2 topic hz on filtered odom.
## Common pitfalls

- Two publishers for the same TF link (e.g., sim + EKF) -> TF fights.
- Incorrect frame IDs or using map frame in the EKF config unintentionally.
- Publishing filtered odom but Nav2 costmaps still reference a different topic/frame.
## Commands, examples, and templates

Install (if needed):
  sudo apt install ros-jazzy-robot-localization

Run EKF (example):
  ros2 run robot_localization ekf_node --ros-args --params-file /path/to/ekf.yaml

Verify TF and topics:
  ros2 run tf2_ros tf2_echo odom base_link
  ros2 topic hz /odometry/filtered
  ros2 topic info /odometry/filtered -v
## Copilot-friendly prompt snippet

Create a minimal robot_localization EKF YAML for a simulated UGV with wheel odometry. Assume we need odom->base_link TF and a filtered odometry topic for Nav2. Also list the verification commands to ensure TF is not double-published.

# rosbag2: Recording with compression and MCAP storage plugin

Source: https://github.com/ros2/rosbag2  +  https://docs.ros.org/en/jazzy/p/rosbag2_storage_mcap/index.html

## What this page is

rosbag2 is the standard ROS 2 logging tool. The MCAP storage plugin provides an efficient bag format; rosbag2 also supports compression options during recording.
## Why it matters for your testbed

Your whole evaluation is based on logged data (WSL2 GUI limitations). Efficient and repeatable logging is essential for scaling experiments, especially when you later add sensors or increase run counts.
## Key takeaways

- Compression can reduce storage footprint during ros2 bag record.
- MCAP is a supported storage backend for rosbag2 and is commonly used for large logs.
- For reproducibility, log bag settings (storage, compression) in metadata.
## What to do in your project (practical checklist)

1. Decide a standard bag format (sqlite3 vs mcap) for the project and stick to it per experiment stage.
1. If enabling compression, keep settings fixed across batches (or record settings per run).
1. Add a quick 'bag integrity check' in your runbook: ros2 bag info + topic list.
1. If you later log images, evaluate storage and compression settings first.
## Common pitfalls

- Changing bag settings between runs without recording it -> hard to compare results.
- Logging heavy topics (images) without a storage plan will explode disk usage.
## Commands, examples, and templates

Record with file-level compression:
  ros2 bag record -a --compression-mode file --compression-format zstd

Record with MCAP storage:
  ros2 bag record -a -s mcap

Verify bag:
  ros2 bag info <bag_dir>
## Copilot-friendly prompt snippet

We log deterministic Gazebo runs with rosbag2. Recommend a repeatable rosbag2 recording command set (topics, storage plugin, compression) and a metadata schema to store these settings per run.

# ROS 2 Launch: Structuring larger projects (namespaces, remaps, parameters)

Source: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html

## What this page is

ROS 2 tutorial on structuring launch files: declaring launch arguments, applying namespaces, remappings, parameter YAML, and composing reusable launch modules.
## Why it matters for your testbed

Your testbed will grow (multi-UAV, Nav2/SLAM stacks, OMNeT++ condition toggles). A clean launch architecture prevents configuration drift and makes experiments reproducible and scriptable.
## Key takeaways

- Prefer launch arguments over hardcoded values (world, namespace, params file, use_sim_time).
- Use namespaces to scale to multiple agents without topic collisions.
- Keep parameter files versioned; pass them explicitly via launch.
- Build a reusable launch stack: bring-up, sensors, nav, logging, experiments.
## What to do in your project (practical checklist)

1. Refactor launch files so that world name, active UAV, and condition are launch arguments.
1. Introduce namespaces per robot when you move to multi-agent runs.
1. Keep a single entrypoint launch that wires: sim + bridges + nodes + logging.
1. Store 'exact launch command + args' in metadata for each run.
## Common pitfalls

- Hardcoding topic names prevents scaling to multiple robots.
- Mixing parameters in code and YAML without a single source of truth.
## Commands, examples, and templates

Launch with args:
  ros2 launch <pkg> <file>.launch.py world:=orchard use_sim_time:=true

Typical remap example:
  ros2 run <pkg> <node> --ros-args -r /cmd_vel:=/a201_0000/cmd_vel
## Copilot-friendly prompt snippet

We need a scalable ROS 2 launch architecture for Gazebo sim + Husky + UAV(s) + bridges + logging. Propose a clean launch file layout with launch arguments for world, namespaces, use_sim_time, and condition.

# ros2_tracing (Jazzy): Trace and analyze ROS 2 timing

Source: https://docs.ros.org/en/jazzy/Tutorials/Advanced/ROS2-Tracing-Trace-and-Analyze.html

## What this page is

Official tutorial for using ros2_tracing to trace callback execution and analyze timing (with tracetools_analysis).
## Why it matters for your testbed

When you start attributing delays to 'network impairment' versus 'ROS scheduling / node behavior', tracing can help you separate those effects. It’s also useful if you need a deeper latency measurement than rosbag timestamps alone.
## Key takeaways

- ros2_tracing can capture callback durations and message flow timing at runtime.
- Trace data can be analyzed with tracetools_analysis to plot callback durations.
- Tracing is best used as a diagnostic tool or in targeted experiments (not necessarily every run).
## What to do in your project (practical checklist)

1. Use tracing selectively to validate where latency is introduced (publisher, executor, subscriber).
1. Keep a simple trace workflow documented in RUNBOOK.md for repeatability.
1. If you later compare baseline vs impaired, use tracing on a small sample to validate interpretation.
## Common pitfalls

- Tracing everything in every experiment run can add overhead; keep it optional.
- Python tracing has limitations compared to C++ in some setups.
## Commands, examples, and templates

Install (if needed):
  sudo apt install ros-jazzy-ros2trace

Example workflow from tutorial:
  ros2 trace -s my_session
  # run your nodes
  ros2 trace stop
  # analyze with tracetools_analysis (see tutorial)
## Copilot-friendly prompt snippet

Show how to use ros2_tracing to measure end-to-end message latency for a topic (publisher -> subscriber) in ROS 2 Jazzy, and how to keep this optional in our experiment scripts.

# performance_test (Jazzy): Middleware / QoS latency and throughput measurement

Source: https://docs.ros.org/en/jazzy/p/performance_test/

## What this page is

A ROS 2 tool to benchmark pub/sub performance across middleware, message sizes, rates, and QoS settings; it records metrics such as latency.
## Why it matters for your testbed

This is useful as a controlled micro-benchmark for your later impairment experiments: you can validate that 'loss/jitter/delay' changes observable metrics, independent of robot control complexity.
## Key takeaways

- You can vary QoS settings and message size/frequency to stress the comm path.
- You can compare different RMW implementations and configurations.
- It provides a controlled baseline to interpret robot-level experiment results.
## What to do in your project (practical checklist)

1. Run a small performance_test baseline to understand your middleware behavior.
1. Reuse the same QoS profiles you plan to use for coordination topics.
1. Only use it as a side experiment; your main evaluation still uses your scripted rounds + rosbag.
## Common pitfalls

- Confusing micro-benchmark metrics with robot-system behavior; keep them separated.
- Running too many variants without a plan; pick a small representative set.
## Commands, examples, and templates

Install (if needed):
  sudo apt install ros-jazzy-performance-test

Basic usage pattern (see package docs for exact flags):
  performance_test --help
  # run pub/sub test with selected QoS, msg size, rate
## Copilot-friendly prompt snippet

Propose a small performance_test experiment plan to quantify QoS effects on latency for a coordination-event-like topic (small messages, moderate rate).

# Transforms and Frame Conventions (REP-105 / Nav2)

Source: https://docs.nav2.org/setup_guides/transformation/setup_transforms.html  +  https://docs.ros.org/en/melodic/api/robot_localization/html/preparing_sensor_data.html

## What this page is

Guidance on ROS mobile-robot frame semantics and the TF tree Nav2 expects: base_link as robot body frame, odom as locally continuous frame (drifts), and map as globally consistent frame (corrects drift). It references REP-105 conventions and explains what each frame means.
## Why it matters for your testbed

Navigation, SLAM, and evaluation metrics can all look 'fine' while being wrong if frames are inconsistent. A stable, correct TF tree is also essential when you later compare baseline vs impaired conditions—otherwise you can’t attribute changes to communication effects.
## Key takeaways

- REP-105 frame semantics: base_link (robot), odom (continuous but drifting), map (globally consistent).
- Nav2 consumes these frames for planning (map) and control (odom/base_link).
- Sensors must have correct static transforms to base_link (e.g., base_link -> laser_frame).
- Avoid double-publishing the same TF link (especially odom->base_link).
## What to do in your project (practical checklist)

1. Write down your canonical frames: base_link, odom, map, plus sensor frames (camera, LiDAR).
1. Verify TF chain in every run (at least once per batch) using tf2_echo and view_frames.
1. Ensure exactly one source publishes odom->base_link TF (either sim or EKF, not both).
1. Store frame names in run metadata or a versioned config so they don’t drift across the project.
## Common pitfalls

- Frame mismatch (map/odom swapped, wrong child_frame_id in odometry) leads to unstable Nav2/SLAM.
- Static transforms not published (sensor frames missing) causes costmap/SLAM failures.
- Multiple TF publishers silently 'fight' and produce jittery transforms.
## Commands, examples, and templates

TF visualization:
  ros2 run tf2_tools view_frames

TF checks (examples):
  ros2 run tf2_ros tf2_echo odom base_link
  ros2 run tf2_ros tf2_echo base_link laser_frame
  ros2 run tf2_ros tf2_echo map odom

Odometry sanity check:
  ros2 topic echo /a201_0000/platform/odom --once
  # check header.frame_id and child_frame_id

Static TF publisher example (if you need a missing sensor frame):
  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_frame
## Copilot-friendly prompt snippet

Summarize the correct REP-105 TF tree for a mobile robot in Nav2 (map/odom/base_link). Then list concrete ROS 2 commands to verify: (1) TF links exist, (2) odometry frame_id/child_frame_id are correct, and (3) no TF link is double-published.

# RMW/DDS Selection and Discovery Configuration (Jazzy)

Source: https://docs.ros.org/en/jazzy/How-To-Guides/Working-with-multiple-RMW-implementations.html  +  https://docs.ros.org/en/jazzy/Tutorials/Advanced/Discovery-Server/Discovery-Server.html

## What this page is

Official instructions for selecting a ROS 2 middleware implementation at runtime via RMW_IMPLEMENTATION, and (for Fast DDS) configuring centralized discovery via Discovery Server.
## Why it matters for your testbed

When you introduce OMNeT++ network impairment (loss/jitter/delay), middleware behavior (discovery, reliability, retransmits) affects the observed results. You need a controlled, documented middleware configuration to avoid confounding factors.
## Key takeaways

- Select middleware at runtime with RMW_IMPLEMENTATION (e.g., rmw_fastrtps_cpp, rmw_cyclonedds_cpp).
- ROS_DOMAIN_ID is a simple isolation mechanism for independent ROS 2 networks.
- Fast DDS Discovery Server can replace multicast discovery (useful in restricted networks).
- Middleware choice + discovery mode should be treated as an experimental configuration variable (document it).
## What to do in your project (practical checklist)

1. Pick a default RMW implementation for the thesis testbed and document it (and why).
1. Add RMW_IMPLEMENTATION and ROS_DOMAIN_ID into your run metadata (per batch is fine).
1. Only use Discovery Server if you actually need it (e.g., multicast issues). If you use it, version its config and document endpoints.
1. Run a small QoS micro-benchmark (optional) after changing RMW to ensure behavior is understood.
## Common pitfalls

- Mixing RMW implementations across machines/containers without noticing.
- Changing discovery mode (multicast vs server) and attributing behavior changes to 'network impairment'.
- Forgetting to record ROS_DOMAIN_ID causing cross-talk between experiments.
## Commands, examples, and templates

Select middleware (runtime):
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  # alternative:
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

Isolate ROS graph:
  export ROS_DOMAIN_ID=7

Fast DDS Discovery Server (per tutorial):
  # start server (example; adjust IP/port)
  fastdds discovery -l 127.0.0.1 -p 11811

  # in each terminal that runs ROS nodes:
  export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
  ros2 run demo_nodes_cpp talker
  ros2 run demo_nodes_cpp listener

Check what RMW is active:
  echo $RMW_IMPLEMENTATION
## Copilot-friendly prompt snippet

We need a reproducible middleware configuration for ROS 2 Jazzy experiments (baseline vs OMNeT++ impaired). Provide a recommended default RMW choice, how to set it via env vars, how to isolate experiments with ROS_DOMAIN_ID, and when to use Fast DDS Discovery Server. Include the exact env vars and verification steps.

# rosbag2 Advanced Recording: Storage plugins (MCAP) and Compression

Source: https://docs.ros.org/en/jazzy/p/rosbag2_storage_mcap/index.html  +  https://github.com/ros2/rosbag2

## What this page is

References for rosbag2 storage backends (including MCAP) and recording options such as compression. Focus is on practical recording commands and how to keep bag settings consistent across runs.
## Why it matters for your testbed

Your evaluation relies on logged data. When run counts increase or you enable sensor topics, storage format and compression become necessary for scaling without losing reproducibility.
## Key takeaways

- rosbag2 supports multiple storage backends (sqlite3 and mcap are common).
- Record settings (storage id, compression mode/format) must be logged as part of experiment metadata.
- Prefer topic allowlists over '-a' when you scale, to keep storage bounded and focused.
## What to do in your project (practical checklist)

1. Standardize on a bag format per experiment stage (keep it fixed during comparisons).
1. If you enable compression, fix mode+format (e.g., file + zstd) across all runs in a batch.
1. Add bag settings (storage id, compression) into meta.yaml for every run.
1. Add a post-run integrity step: ros2 bag info + verify expected topic counts.
## Common pitfalls

- Switching storage/compression between baseline and impaired runs makes comparisons questionable.
- Logging heavy topics without storage planning leads to dropped messages or disk exhaustion.
## Commands, examples, and templates

Record with topic allowlist (recommended pattern):
  ros2 bag record -o <out_dir> \
    /clock /a201_0000/platform/odom /a201_0000/platform/odom/filtered \
    /a201_0000/cmd_vel /a201_0000/tf /a201_0000/tf_static \
    /dji0/pose /coord/events

Select storage backend (examples):
  ros2 bag record ... -s sqlite3
  ros2 bag record ... -s mcap

Compression (examples):
  ros2 bag record ... --compression-mode file --compression-format zstd

Verify:
  ros2 bag info <bag_dir>
  ros2 bag info <bag_dir> --verbose
## Copilot-friendly prompt snippet

We record deterministic ROS 2 runs and later compare baseline vs impaired conditions. Recommend a stable rosbag2 recording configuration (topic allowlist, storage plugin choice, compression), and show how to store those settings into meta.yaml so Copilot can keep runs reproducible.

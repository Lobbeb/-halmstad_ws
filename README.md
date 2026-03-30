# Halmstad ROS 2 + Gazebo Workspace

This workspace contains the current UAV-UGV simulation baseline and supporting thesis tooling.

Current supported baseline:
- 1 UGV + 1 UAV
- Warehouse world
- ROS 2 Jazzy + Gazebo Harmonic

## Start Here

Main references:
- [src/lrs_halmstad/README.md](src/lrs_halmstad/README.md) - package-level runbook, topics, launch arguments, and workflow details
- [THESIS_HANDOFF.md](THESIS_HANDOFF.md) - thesis-specific implementation notes and OMNeT handoff context

## Recommended Bring-Up

Build after code changes:

```bash
cd <workspace_root>
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
```

Recommended tmux workflow:

```bash
cd <workspace_root>
./run.sh tmux_1to1 warehouse
```

Stop the tmux-managed stack:

```bash
cd <workspace_root>
./stop.sh tmux_1to1 warehouse
```

## Manual Baseline Order

If you want to run the current baseline without tmux, use the helper wrappers from the workspace root:

1. `./run.sh gazebo_sim warehouse`
2. `./run.sh spawn_uav warehouse uav_name:=dji0`
3. `./run.sh localization warehouse`
4. `./run.sh nav2`
5. `./run.sh 1to1_follow warehouse`

For the detailed launch-level workflow, parameter notes, and topic contract, use the package README:
- [src/lrs_halmstad/README.md](src/lrs_halmstad/README.md)

## Handoff Notes

- The current baseline should be treated as locked before new feature work.
- The `William/` folder contains thesis material and reference papers; it is not part of the runtime stack.
- Root-level temp files and local experiment leftovers should not be treated as source of truth.

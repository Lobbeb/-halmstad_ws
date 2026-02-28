#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source ~/halmstad_ws/install/setup.bash
source ~/halmstad_ws/src/lrs_halmstad/clearpath/setup.bash
ros2 launch lrs_halmstad run_round_follow_motion.launch.py \
    "$@"

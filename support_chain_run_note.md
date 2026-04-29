# Support-Chain Run Note

This is the practical ROS-level support-chain baseline for the thesis system.

## Run

Baseline support-chain smoke run:

```bash
./run.sh tmux_support_chain warehouse mode:=follow gui:=true
```

Explicit current relation placeholder:

```bash
./run.sh tmux_support_chain warehouse mode:=follow gui:=true support_mux_relation_source:=odom
```

Optional support-camera scan/capture mode:

```bash
./run.sh tmux_support_chain warehouse mode:=follow gui:=true \
  support_camera_scan_enable:=true \
  support_bridge_gimbal:=true \
  support_camera_scan_yaw_amplitude_deg:=35.0 \
  support_camera_scan_period_s:=8.0 \
  support_camera_scan_pitch_deg:=-20.0
```

`support_bridge_gimbal:=true` is intentionally explicit because camera scanning should not alter the accepted support-follow baseline unless requested.

## Proof Topics

Motion/topology:

```bash
ros2 topic echo --once /dji1/pose
ros2 topic echo --once /dji2/pose
ros2 topic echo --once /dji1/pose_cmd
ros2 topic echo --once /dji2/pose_cmd
```

Support observations and aggregation:

```bash
ros2 topic echo --once /coord/support/dji1/leader_detection_status
ros2 topic echo --once /coord/support/dji2/leader_detection_status
ros2 topic echo --once /coord/dji0/support_observation_summary
ros2 topic echo --once /coord/ugv/support_observation_summary
ros2 topic echo --once /coord/ugv/support_awareness_status
```

Camera streams:

```bash
ros2 topic echo --once /dji1/camera0/image_raw
ros2 topic echo --once /dji2/camera0/image_raw
ros2 topic echo --once /dji1/camera0/camera_info
ros2 topic echo --once /dji2/camera0/camera_info
```

Camera scan commands, when enabled:

```bash
ros2 topic echo --once /dji1/update_pan
ros2 topic echo --once /dji2/update_pan
ros2 topic echo --once /dji1/update_tilt
ros2 topic echo --once /dji2/update_tilt
ros2 topic echo --once /coord/support/camera_scan_status
```

## Recording Support Cameras

For later data analysis/training material, record the support camera streams and camera info:

```bash
ros2 bag record \
  /dji1/camera0/image_raw /dji1/camera0/camera_info \
  /dji2/camera0/image_raw /dji2/camera0/camera_info \
  /coord/support/dji1/leader_detection_status \
  /coord/support/dji2/leader_detection_status \
  /coord/dji0/support_observation_summary \
  /coord/ugv/support_observation_summary \
  /coord/ugv/support_awareness_status
```

## Relation Source

Current thesis implementation:

- `relation_source=odom` means the support UAV relation is based on fixed odom support slots around `dji0`.
- `relation_quality=not_evaluated` means no signal/RSSI quality has been computed yet.
- `relation_note=support_slots` identifies the current slot-based behavior.

Future work:

- `relation_source=rssi`, `signal`, `omnet`, or `communication_quality` can later describe a communication-aware relation source.
- RSSI/signal/OMNeT should replace or augment the relation input without changing the support observation summary contract.

## Scope

Current thesis scope:

- `dji1/dji2` support UAVs follow `dji0`.
- Support UAVs publish observation outputs.
- `dji0` aggregates/selects/summarizes support observations.
- The UGV receives structured support information and a human-readable awareness status.
- The UGV does not actively replan or publish motion commands from this support information.

Future work:

- RSSI/signal-aware support positioning.
- OMNeT communication integration.
- Active UGV reaction/replanning.
- Larger support-camera data collection and detector improvement.

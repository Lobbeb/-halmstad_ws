#!/usr/bin/env python3
from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64, String


def _coerce_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


class SupportCameraScanner(Node):
    """Small optional pan/tilt sweep for support-UAV cameras."""

    def __init__(self) -> None:
        super().__init__("support_camera_scanner")

        uav_names_raw = str(self.declare_parameter("uav_names", "dji1,dji2").value)
        self.uav_names = [name.strip() for name in uav_names_raw.split(",") if name.strip()]
        if not self.uav_names:
            self.uav_names = ["dji1", "dji2"]

        self.yaw_center_deg = float(self.declare_parameter("yaw_center_deg", 0.0).value)
        self.yaw_amplitude_deg = max(0.0, float(self.declare_parameter("yaw_amplitude_deg", 35.0).value))
        self.period_s = max(0.5, float(self.declare_parameter("period_s", 8.0).value))
        self.pitch_deg = float(self.declare_parameter("pitch_deg", -20.0).value)
        self.rate_hz = max(0.5, float(self.declare_parameter("rate_hz", 10.0).value))
        self.publish_pitch = _coerce_bool(self.declare_parameter("publish_pitch", True).value)
        self.status_topic = str(
            self.declare_parameter("status_topic", "/coord/support/camera_scan_status").value
        ).strip() or "/coord/support/camera_scan_status"

        self._pan_pubs = {
            name: self.create_publisher(Float64, f"/{name}/update_pan", 10)
            for name in self.uav_names
        }
        self._tilt_pubs = {
            name: self.create_publisher(Float64, f"/{name}/update_tilt", 10)
            for name in self.uav_names
        }
        self._status_pub = self.create_publisher(String, self.status_topic, 10)
        self._timer = self.create_timer(1.0 / self.rate_hz, self._on_timer)

        self.get_logger().info(
            "[support_camera_scanner] Started: "
            f"uavs={','.join(self.uav_names)}, yaw_center_deg={self.yaw_center_deg:.1f}, "
            f"yaw_amplitude_deg={self.yaw_amplitude_deg:.1f}, period_s={self.period_s:.1f}, "
            f"pitch_deg={self.pitch_deg:.1f}, rate_hz={self.rate_hz:.1f}"
        )

    def _on_timer(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        pan_deg = self.yaw_center_deg + self.yaw_amplitude_deg * math.sin(
            (2.0 * math.pi * now_s) / self.period_s
        )

        pan_msg = Float64()
        pan_msg.data = pan_deg
        tilt_msg = Float64()
        tilt_msg.data = self.pitch_deg

        for name in self.uav_names:
            self._pan_pubs[name].publish(pan_msg)
            if self.publish_pitch:
                self._tilt_pubs[name].publish(tilt_msg)

        status = String()
        status.data = (
            "task=support_camera_scan "
            f"state=OK uavs={','.join(self.uav_names)} pan_deg={pan_deg:.2f} "
            f"pitch_deg={self.pitch_deg:.2f} yaw_center_deg={self.yaw_center_deg:.2f} "
            f"yaw_amplitude_deg={self.yaw_amplitude_deg:.2f} period_s={self.period_s:.2f}"
        )
        self._status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SupportCameraScanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

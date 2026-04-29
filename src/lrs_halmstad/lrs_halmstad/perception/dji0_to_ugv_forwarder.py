from __future__ import annotations

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from lrs_halmstad.perception.detection_protocol import decode_detection_payload, encode_detection_payload
from lrs_halmstad.perception.detection_status import parse_status_line


class Dji0ToUgvForwarder(Node):
    def __init__(self) -> None:
        super().__init__("dji0_to_ugv_forwarder")

        self.in_detection_topic = str(
            self.declare_parameter("in_detection_topic", "/coord/dji0/leader_detection").value
        ).strip() or "/coord/dji0/leader_detection"
        self.in_status_topic = str(
            self.declare_parameter("in_status_topic", "/coord/dji0/leader_detection_status").value
        ).strip() or "/coord/dji0/leader_detection_status"

        self.out_detection_topic = str(
            self.declare_parameter("out_detection_topic", "/coord/ugv/leader_detection").value
        ).strip() or "/coord/ugv/leader_detection"
        self.out_status_topic = str(
            self.declare_parameter("out_status_topic", "/coord/ugv/leader_detection_status").value
        ).strip() or "/coord/ugv/leader_detection_status"
        self.in_summary_topic = str(
            self.declare_parameter("in_summary_topic", "/coord/dji0/support_observation_summary").value
        ).strip() or "/coord/dji0/support_observation_summary"
        self.out_summary_topic = str(
            self.declare_parameter("out_summary_topic", "/coord/ugv/support_observation_summary").value
        ).strip() or "/coord/ugv/support_observation_summary"

        self.forward_owner = str(self.declare_parameter("forward_owner", "dji0").value).strip() or "dji0"
        self.forward_stage = str(self.declare_parameter("forward_stage", "dji0_to_ugv").value).strip() or "dji0_to_ugv"

        self._out_detection_pub = self.create_publisher(String, self.out_detection_topic, 10)
        self._out_status_pub = self.create_publisher(String, self.out_status_topic, 10)
        self._out_summary_pub = self.create_publisher(String, self.out_summary_topic, 10)

        self.create_subscription(String, self.in_detection_topic, self._on_detection, 10)
        self.create_subscription(String, self.in_status_topic, self._on_status, 10)
        self.create_subscription(String, self.in_summary_topic, self._on_summary, 10)

        self.get_logger().info(
            "[dji0_to_ugv_forwarder] Started: "
            f"in_detection={self.in_detection_topic}, in_status={self.in_status_topic}, "
            f"out_detection={self.out_detection_topic}, out_status={self.out_status_topic}, "
            f"in_summary={self.in_summary_topic}, out_summary={self.out_summary_topic}, "
            f"forward_owner={self.forward_owner}, forward_stage={self.forward_stage}"
        )

    def _on_detection(self, msg: String) -> None:
        out = String()
        now_ns = int(self.get_clock().now().nanoseconds)

        try:
            det_msg = decode_detection_payload(msg.data)
            metadata = dict(det_msg.metadata or {})
            metadata["forward_stage"] = self.forward_stage
            metadata["forward_owner"] = self.forward_owner
            metadata["forwarded_from_topic"] = self.in_detection_topic
            metadata["forwarded_ros_ns"] = str(now_ns)
            out.data = encode_detection_payload(det_msg.stamp_ns or now_ns, det_msg.detection, metadata=metadata)
        except Exception:
            # Keep forwarding resilient even if upstream payload changes unexpectedly.
            out.data = msg.data

        self._out_detection_pub.publish(out)

    def _on_status(self, msg: String) -> None:
        base = str(msg.data).strip()
        suffix = f" forward_stage={self.forward_stage} forward_owner={self.forward_owner}"
        if not base:
            out_line = f"task=forward state=NO_INPUT reason=empty_upstream_status{suffix}"
        else:
            fields = parse_status_line(base)
            if (
                fields.get("forward_stage", "").strip() == self.forward_stage
                and fields.get("forward_owner", "").strip() == self.forward_owner
            ):
                out_line = base
            else:
                out_line = f"{base}{suffix}"

        out = String()
        out.data = out_line
        self._out_status_pub.publish(out)

    def _on_summary(self, msg: String) -> None:
        now_ns = int(self.get_clock().now().nanoseconds)
        out = String()
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("summary_payload_not_object")
            payload["forward_stage"] = self.forward_stage
            payload["forward_owner"] = self.forward_owner
            payload["forwarded_from_topic"] = self.in_summary_topic
            payload["forwarded_ros_ns"] = now_ns
            out.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        except Exception:
            out.data = msg.data
        self._out_summary_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Dji0ToUgvForwarder()
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

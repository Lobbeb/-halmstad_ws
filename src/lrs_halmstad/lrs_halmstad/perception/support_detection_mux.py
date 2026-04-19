from __future__ import annotations

from dataclasses import dataclass, replace
from functools import partial
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from lrs_halmstad.perception.detection_protocol import Detection2D, DetectionMessage, decode_detection_payload, encode_detection_payload
from lrs_halmstad.perception.detection_status import build_detection_status_line, parse_status_line


@dataclass
class SourceState:
    source_id: str
    detection_topic: str
    status_topic: str
    last_detection: Optional[DetectionMessage] = None
    last_detection_recv_ns: int = 0
    last_status_line: str = ""
    last_status_recv_ns: int = 0


class SupportDetectionMux(Node):
    def __init__(self) -> None:
        super().__init__("support_detection_mux")

        source_a_id = str(self.declare_parameter("source_a_id", "dji1").value).strip() or "dji1"
        source_b_id = str(self.declare_parameter("source_b_id", "dji2").value).strip() or "dji2"

        source_a_detection_topic = str(
            self.declare_parameter(
                "source_a_detection_topic",
                f"/coord/support/{source_a_id}/leader_detection",
            ).value
        ).strip() or f"/coord/support/{source_a_id}/leader_detection"
        source_a_status_topic = str(
            self.declare_parameter(
                "source_a_status_topic",
                f"/coord/support/{source_a_id}/leader_detection_status",
            ).value
        ).strip() or f"/coord/support/{source_a_id}/leader_detection_status"
        source_b_detection_topic = str(
            self.declare_parameter(
                "source_b_detection_topic",
                f"/coord/support/{source_b_id}/leader_detection",
            ).value
        ).strip() or f"/coord/support/{source_b_id}/leader_detection"
        source_b_status_topic = str(
            self.declare_parameter(
                "source_b_status_topic",
                f"/coord/support/{source_b_id}/leader_detection_status",
            ).value
        ).strip() or f"/coord/support/{source_b_id}/leader_detection_status"

        self.out_detection_topic = str(
            self.declare_parameter("out_detection_topic", "/coord/dji0/leader_detection").value
        ).strip() or "/coord/dji0/leader_detection"
        self.out_status_topic = str(
            self.declare_parameter("out_status_topic", "/coord/dji0/leader_detection_status").value
        ).strip() or "/coord/dji0/leader_detection_status"

        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 10.0).value)
        self.source_stale_timeout_s = max(0.05, float(self.declare_parameter("source_stale_timeout_s", 0.75).value))
        self.prefer_current_source_on_tie = bool(
            self.declare_parameter("prefer_current_source_on_tie", True).value
        )

        self._source_a = SourceState(
            source_id=source_a_id,
            detection_topic=source_a_detection_topic,
            status_topic=source_a_status_topic,
        )
        self._source_b = SourceState(
            source_id=source_b_id,
            detection_topic=source_b_detection_topic,
            status_topic=source_b_status_topic,
        )
        self._current_source_id: str = ""
        self._last_decode_warn_ns: int = 0

        self._out_detection_pub = self.create_publisher(String, self.out_detection_topic, 10)
        self._out_status_pub = self.create_publisher(String, self.out_status_topic, 10)

        self.create_subscription(
            String,
            self._source_a.detection_topic,
            partial(self._on_detection, self._source_a),
            10,
        )
        self.create_subscription(
            String,
            self._source_b.detection_topic,
            partial(self._on_detection, self._source_b),
            10,
        )
        self.create_subscription(
            String,
            self._source_a.status_topic,
            partial(self._on_status, self._source_a),
            10,
        )
        self.create_subscription(
            String,
            self._source_b.status_topic,
            partial(self._on_status, self._source_b),
            10,
        )

        period_s = 1.0 / max(0.1, self.publish_rate_hz)
        self._timer = self.create_timer(period_s, self._on_timer)

        self.get_logger().info(
            "[support_detection_mux] Started: "
            f"sources=({self._source_a.source_id},{self._source_b.source_id}), "
            f"source_stale_timeout_s={self.source_stale_timeout_s:.2f}, "
            f"publish_rate_hz={self.publish_rate_hz:.1f}, "
            f"out_detection={self.out_detection_topic}, out_status={self.out_status_topic}"
        )

    def _on_detection(self, source: SourceState, msg: String) -> None:
        recv_ns = int(self.get_clock().now().nanoseconds)
        try:
            source.last_detection = decode_detection_payload(msg.data)
            source.last_detection_recv_ns = recv_ns
        except ValueError as exc:
            if recv_ns - self._last_decode_warn_ns > 2_000_000_000:
                self._last_decode_warn_ns = recv_ns
                self.get_logger().warn(
                    f"[support_detection_mux] Ignoring invalid detection payload from {source.source_id}: {exc}"
                )

    def _on_status(self, source: SourceState, msg: String) -> None:
        source.last_status_line = str(msg.data)
        source.last_status_recv_ns = int(self.get_clock().now().nanoseconds)

    def _on_timer(self) -> None:
        now_ns = int(self.get_clock().now().nanoseconds)
        selected = self._select_source(now_ns)

        if selected is None or selected.last_detection is None:
            self._publish_no_input(now_ns)
            self._current_source_id = ""
            return

        det_msg = selected.last_detection
        det = self._with_mux_source(det_msg.detection, selected.source_id)

        metadata = dict(det_msg.metadata or {})
        metadata["mux_source"] = selected.source_id
        metadata["mux_source_age_ms"] = f"{self._source_age_ms(selected, now_ns):.1f}"
        metadata["mux_strategy"] = "highest_confidence_valid"

        out_msg = String()
        out_msg.data = encode_detection_payload(
            det_msg.stamp_ns or now_ns,
            det,
            metadata=metadata,
        )
        self._out_detection_pub.publish(out_msg)

        status_msg = String()
        status_msg.data = self._build_status_line(selected=selected, det=det, now_ns=now_ns)
        self._out_status_pub.publish(status_msg)

        self._current_source_id = selected.source_id

    def _select_source(self, now_ns: int) -> Optional[SourceState]:
        fresh = [source for source in (self._source_a, self._source_b) if self._is_fresh(source, now_ns)]
        if not fresh:
            return None

        valid = [source for source in fresh if source.last_detection is not None and source.last_detection.detection is not None]
        if valid:
            best_conf = max(float(source.last_detection.detection.conf) for source in valid)
            best = [
                source
                for source in valid
                if abs(float(source.last_detection.detection.conf) - best_conf) <= 1e-6
            ]
            if (
                self.prefer_current_source_on_tie
                and self._current_source_id
                and any(source.source_id == self._current_source_id for source in best)
            ):
                return next(source for source in best if source.source_id == self._current_source_id)
            return sorted(best, key=lambda source: source.source_id)[0]

        return max(fresh, key=lambda source: source.last_detection_recv_ns)

    def _is_fresh(self, source: SourceState, now_ns: int) -> bool:
        if source.last_detection is None or source.last_detection_recv_ns <= 0:
            return False
        max_age_ns = int(self.source_stale_timeout_s * 1_000_000_000.0)
        return (now_ns - source.last_detection_recv_ns) <= max_age_ns

    def _source_age_ms(self, source: SourceState, now_ns: int) -> float:
        if source.last_detection_recv_ns <= 0:
            return -1.0
        return max(0.0, (now_ns - source.last_detection_recv_ns) / 1_000_000.0)

    def _with_mux_source(self, det: Optional[Detection2D], source_id: str) -> Optional[Detection2D]:
        if det is None:
            return None
        if str(det.source).strip():
            return det
        return replace(det, source=f"support_{source_id}")

    def _build_status_line(
        self,
        *,
        selected: SourceState,
        det: Optional[Detection2D],
        now_ns: int,
    ) -> str:
        state = "OK" if det is not None else "NO_DET"
        reason = "none" if det is not None else "no_detection"
        source_fields = parse_status_line(selected.last_status_line)
        if source_fields:
            source_state = source_fields.get("state", "").strip()
            if det is None and source_state:
                state = source_state
            source_reason = source_fields.get("reason", "").strip()
            if source_reason:
                reason = source_reason

        extras = {
            "mux_source": selected.source_id,
            "mux_source_age_ms": f"{self._source_age_ms(selected, now_ns):.1f}",
            "mux_sources": f"{self._source_a.source_id},{self._source_b.source_id}",
        }
        return build_detection_status_line(
            state=state,
            reason=reason,
            task="support_mux",
            det=det,
            extras=extras,
        )

    def _publish_no_input(self, now_ns: int) -> None:
        out_msg = String()
        out_msg.data = encode_detection_payload(
            now_ns,
            None,
            metadata={
                "mux_source": "none",
                "mux_strategy": "highest_confidence_valid",
                "reason": "no_fresh_input",
            },
        )
        self._out_detection_pub.publish(out_msg)

        status_msg = String()
        status_msg.data = build_detection_status_line(
            state="NO_INPUT",
            reason="no_fresh_input",
            task="support_mux",
            det=None,
            extras={
                "mux_source": "none",
                "mux_sources": f"{self._source_a.source_id},{self._source_b.source_id}",
            },
        )
        self._out_status_pub.publish(status_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SupportDetectionMux()
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

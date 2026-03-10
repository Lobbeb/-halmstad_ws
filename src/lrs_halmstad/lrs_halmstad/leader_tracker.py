#!/usr/bin/env python3
from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    cv2 = None

try:
    from ultralytics import YOLO  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    YOLO = None


@dataclass
class Detection2D:
    u: float
    v: float
    conf: float
    bbox: Tuple[float, float, float, float]
    cls_id: Optional[int] = None
    cls_name: str = ""
    track_id: Optional[int] = None
    obb_corners: Optional[Tuple[Tuple[float, float], ...]] = None


def _default_models_root() -> str:
    configured_root = os.environ.get("LRS_HALMSTAD_MODELS_ROOT", "").strip()
    if configured_root:
        return os.path.expanduser(configured_root)
    try:
        pkg_share = get_package_share_path("lrs_halmstad").resolve()
        for parent in pkg_share.parents:
            candidate = parent / "src" / "lrs_halmstad"
            if candidate.is_dir():
                return str(parent / "models")
    except Exception:
        pass
    return str((Path(__file__).resolve().parents[3] / "models").resolve())


def _package_config_root() -> Path:
    try:
        return get_package_share_path("lrs_halmstad").resolve() / "config"
    except Exception:
        return (Path(__file__).resolve().parents[1] / "config").resolve()


class LeaderTracker(Node):
    """Ultralytics track-mode wrapper publishing tracked detections on /coord/leader_detection."""

    def __init__(self):
        super().__init__("leader_tracker")
        dyn_num = ParameterDescriptor(dynamic_typing=True)

        self.declare_parameter("uav_name", "dji0")
        self.declare_parameter("camera_topic", "")
        self.declare_parameter("out_topic", "/coord/leader_detection")
        self.declare_parameter("models_root", _default_models_root())
        self.declare_parameter("yolo_weights", "")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("conf_threshold", 0.05, dyn_num)
        self.declare_parameter("iou_threshold", 0.45, dyn_num)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("target_class_id", -1)
        self.declare_parameter("target_class_name", "")
        self.declare_parameter("predict_hz", 20.0, dyn_num)
        self.declare_parameter("tracker_config", "trackers/botsort.yaml")
        self.declare_parameter("event_topic", "/coord/events")
        self.declare_parameter("publish_events", False)

        self.uav_name = str(self.get_parameter("uav_name").value)
        self.camera_topic = str(self.get_parameter("camera_topic").value) or f"/{self.uav_name}/camera0/image_raw"
        self.out_topic = str(self.get_parameter("out_topic").value)
        self.models_root = os.path.expanduser(str(self.get_parameter("models_root").value).strip())
        self.yolo_weights = self._resolve_yolo_weights_path(str(self.get_parameter("yolo_weights").value).strip())
        self.device = str(self.get_parameter("device").value).strip() or "cpu"
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.target_class_id = int(self.get_parameter("target_class_id").value)
        self.target_class_name = str(self.get_parameter("target_class_name").value).strip()
        self.predict_hz = float(self.get_parameter("predict_hz").value)
        self.tracker_config = self._resolve_tracker_config(str(self.get_parameter("tracker_config").value).strip())
        self.event_topic = str(self.get_parameter("event_topic").value)
        self.publish_events = bool(self.get_parameter("publish_events").value)

        if self.predict_hz <= 0.0:
            raise ValueError("predict_hz must be > 0")
        if not self.yolo_weights:
            raise ValueError("yolo_weights must be set for leader_tracker")
        if not os.path.isfile(self.yolo_weights):
            raise ValueError(f"leader_tracker weights not found: {self.yolo_weights}")
        if not os.path.isfile(self.tracker_config):
            raise ValueError(f"leader_tracker tracker_config not found: {self.tracker_config}")
        if YOLO is None:
            raise ValueError("ultralytics is required for leader_tracker")

        self.model = YOLO(self.yolo_weights)
        self.last_predict_time: Optional[Time] = None
        self.active_track_id: Optional[int] = None

        self.image_sub = self.create_subscription(Image, self.camera_topic, self.on_image, 10)
        self.pub = self.create_publisher(String, self.out_topic, 10)
        self.events_pub = self.create_publisher(String, self.event_topic, 10)

        self.get_logger().info(
            "[leader_tracker] Started: "
            f"image={self.camera_topic}, out={self.out_topic}, "
            f"weights={self.yolo_weights}, tracker={self.tracker_config}, device={self.device}, predict_hz={self.predict_hz}"
        )
        self.emit_event("TRACKER_NODE_START")

    def emit_event(self, name: str) -> None:
        if not self.publish_events:
            return
        msg = String()
        msg.data = str(name)
        self.events_pub.publish(msg)

    def _resolve_yolo_weights_path(self, raw_path: str) -> str:
        if not raw_path:
            return ""
        expanded = os.path.expanduser(raw_path)
        if os.path.isabs(expanded):
            return expanded
        return os.path.join(self.models_root, expanded)

    def _resolve_tracker_config(self, raw_path: str) -> str:
        if not raw_path:
            return str(_package_config_root() / "trackers" / "botsort.yaml")
        expanded = Path(os.path.expanduser(raw_path))
        if expanded.is_absolute():
            return str(expanded)
        config_root = _package_config_root()
        candidates = [
            config_root / expanded,
            config_root / "trackers" / expanded,
        ]
        for candidate in candidates:
            if candidate.is_file():
                return str(candidate)
        return str((config_root / expanded).resolve())

    @staticmethod
    def _image_to_bgr(msg: Image) -> Optional[np.ndarray]:
        if cv2 is None:
            return None
        try:
            if msg.encoding.lower() in ("bgr8", "rgb8"):
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                if msg.encoding.lower() == "rgb8":
                    arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                return arr.copy()
        except Exception:
            return None
        return None

    @staticmethod
    def _stamp_ns(msg: Image) -> int:
        try:
            return int(Time.from_msg(msg.header.stamp).nanoseconds)
        except Exception:
            return 0

    @staticmethod
    def _order_quad(points: np.ndarray) -> np.ndarray:
        center = np.mean(points, axis=0)
        angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
        ordered = points[np.argsort(angles)]
        start = int(np.argmin(ordered[:, 0] + ordered[:, 1]))
        return np.roll(ordered, -start, axis=0)

    @staticmethod
    def _extract_track_id(item) -> Optional[int]:
        raw = getattr(item, "id", None)
        if raw is None:
            return None
        try:
            if hasattr(raw, "tolist"):
                raw = raw.tolist()
            if isinstance(raw, (list, tuple)):
                raw = raw[0]
            return int(raw)
        except Exception:
            return None

    def _candidate_ok(self, cls_id: Optional[int], cls_name: str) -> bool:
        if self.target_class_id >= 0 and cls_id != self.target_class_id:
            return False
        if self.target_class_name and cls_name.lower() != self.target_class_name.lower():
            return False
        return True

    def _collect_obb_candidates(self, result) -> list[Detection2D]:
        obb = getattr(result, "obb", None)
        if obb is None or len(obb) == 0:
            return []
        names = getattr(result, "names", {}) or {}
        candidates: list[Detection2D] = []
        for item in obb:
            try:
                xyxy = item.xyxy[0].tolist()
                x1, y1, x2, y2 = [float(v) for v in xyxy]
                conf = float(item.conf[0]) if getattr(item, "conf", None) is not None else 0.0
                cls_id = int(item.cls[0]) if getattr(item, "cls", None) is not None else None
                cls_name = str(names.get(cls_id, "")) if cls_id is not None else ""
                corners_arr = np.asarray(item.xyxyxyxy[0].tolist(), dtype=np.float64).reshape(4, 2)
                corners = tuple((float(pt[0]), float(pt[1])) for pt in self._order_quad(corners_arr))
                track_id = self._extract_track_id(item)
            except Exception:
                continue
            if not self._candidate_ok(cls_id, cls_name):
                continue
            candidates.append(
                Detection2D(
                    u=0.5 * (x1 + x2),
                    v=0.5 * (y1 + y2),
                    conf=conf,
                    bbox=(x1, y1, x2, y2),
                    cls_id=cls_id,
                    cls_name=cls_name,
                    track_id=track_id,
                    obb_corners=corners,
                )
            )
        return candidates

    def _collect_box_candidates(self, result) -> list[Detection2D]:
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return []
        names = getattr(result, "names", {}) or {}
        candidates: list[Detection2D] = []
        for item in boxes:
            try:
                xyxy = item.xyxy[0].tolist()
                x1, y1, x2, y2 = [float(v) for v in xyxy]
                conf = float(item.conf[0]) if getattr(item, "conf", None) is not None else 0.0
                cls_id = int(item.cls[0]) if getattr(item, "cls", None) is not None else None
                cls_name = str(names.get(cls_id, "")) if cls_id is not None else ""
                track_id = self._extract_track_id(item)
            except Exception:
                continue
            if not self._candidate_ok(cls_id, cls_name):
                continue
            candidates.append(
                Detection2D(
                    u=0.5 * (x1 + x2),
                    v=0.5 * (y1 + y2),
                    conf=conf,
                    bbox=(x1, y1, x2, y2),
                    cls_id=cls_id,
                    cls_name=cls_name,
                    track_id=track_id,
                )
            )
        return candidates

    def _choose_candidate(self, candidates: list[Detection2D]) -> Optional[Detection2D]:
        if not candidates:
            self.active_track_id = None
            return None
        if self.active_track_id is not None:
            matches = [cand for cand in candidates if cand.track_id == self.active_track_id]
            if matches:
                return max(matches, key=lambda cand: cand.conf)
        tracked = [cand for cand in candidates if cand.track_id is not None]
        if tracked:
            best = max(tracked, key=lambda cand: cand.conf)
            self.active_track_id = best.track_id
            return best
        self.active_track_id = None
        return max(candidates, key=lambda cand: cand.conf)

    def _track_detection(self, img_bgr: np.ndarray) -> Optional[Detection2D]:
        try:
            results = self.model.track(
                source=img_bgr,
                persist=True,
                tracker=self.tracker_config,
                verbose=False,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                imgsz=self.imgsz,
                device=self.device,
            )
        except Exception as exc:
            self.get_logger().warn(f"[leader_tracker] Ultralytics track() failed: {exc}")
            return None
        if not results:
            self.active_track_id = None
            return None
        result = results[0]
        candidates = self._collect_obb_candidates(result)
        if not candidates:
            candidates = self._collect_box_candidates(result)
        return self._choose_candidate(candidates)

    def _publish_detection(self, msg: Image, det: Optional[Detection2D]) -> None:
        payload = {
            "stamp_ns": self._stamp_ns(msg),
            "valid": det is not None,
            "u": None if det is None else float(det.u),
            "v": None if det is None else float(det.v),
            "conf": -1.0 if det is None else float(det.conf),
            "bbox": [] if det is None else [float(v) for v in det.bbox],
            "cls_id": None if det is None else det.cls_id,
            "cls_name": "" if det is None else det.cls_name,
            "track_id": None if det is None else det.track_id,
            "obb_corners": [] if det is None or det.obb_corners is None else [[float(x), float(y)] for x, y in det.obb_corners],
        }
        out = String()
        out.data = json.dumps(payload, separators=(",", ":"), ensure_ascii=True)
        self.pub.publish(out)

    def on_image(self, msg: Image) -> None:
        now = self.get_clock().now()
        if self.last_predict_time is not None:
            dt = (now - self.last_predict_time).nanoseconds * 1e-9
            if dt < (1.0 / self.predict_hz):
                return
        self.last_predict_time = now

        img_bgr = self._image_to_bgr(msg)
        if img_bgr is None:
            self._publish_detection(msg, None)
            return
        det = self._track_detection(img_bgr)
        self._publish_detection(msg, det)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.emit_event("TRACKER_NODE_SHUTDOWN")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

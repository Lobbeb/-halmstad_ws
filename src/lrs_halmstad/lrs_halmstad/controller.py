import math
from typing import List, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


def _parse_waypoints(raw: Sequence[float], fallback: List[Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
    values = [float(v) for v in raw]
    if len(values) < 3 or len(values) % 3 != 0:
        return fallback

    out: List[Tuple[float, float, float]] = []
    for i in range(0, len(values), 3):
        out.append((values[i], values[i + 1], values[i + 2]))
    return out


class Controller(Node):
    def __init__(self) -> None:
        super().__init__("example_controller")

        self.declare_parameter("speed_mps", 2.0)
        self.declare_parameter("period_s", 0.1)
        self.declare_parameter("arrival_tolerance_m", 0.2)
        self.declare_parameter("pan_deg", 0.0)
        self.declare_parameter("tilt_deg", -45.0)
        self.declare_parameter("waypoints", [15.0, 15.0, 4.0, 25.0, 5.0, 4.0])

        self.speed = float(self.get_parameter("speed_mps").value)
        self.period_time = float(self.get_parameter("period_s").value)
        self.arrival_tolerance = float(self.get_parameter("arrival_tolerance_m").value)
        self.pan = float(self.get_parameter("pan_deg").value)
        self.tilt = float(self.get_parameter("tilt_deg").value)

        default_waypoints = [(15.0, 15.0, 4.0), (25.0, 5.0, 4.0)]
        raw_waypoints = self.get_parameter("waypoints").value
        self.goal_positions: List[Tuple[float, float, float]] = _parse_waypoints(raw_waypoints, default_waypoints)

        self.world_position: Point | None = None
        self.current_pose: PoseStamped | None = None
        self.goal_position: Point | None = None
        self.goal_yaw = 0.0
        self.state = "idle"
        self.done_logged = False

        self.ctrl_pos_yaw_pub = self.create_publisher(Joy, "psdk_ros2/flight_control_setpoint_ENUposition_yaw", 10)
        self.update_tilt_pub = self.create_publisher(Float64, "update_tilt", 10)
        self.update_pan_pub = self.create_publisher(Float64, "update_pan", 10)
        self.pose_sub = self.create_subscription(PoseStamped, "pose", self.pose_callback, 10)
        self.timer = self.create_timer(self.period_time, self.timer_callback)

        self.get_logger().info(
            (
                "[controller] Started: "
                f"ns={self.get_namespace()} speed={self.speed:.2f} "
                f"period={self.period_time:.2f} waypoints={len(self.goal_positions)}"
            )
        )

    def pose_callback(self, msg: PoseStamped) -> None:
        self.current_pose = msg
        self.world_position = msg.pose.position

    def check_arrived(self, goal_position: Point) -> bool:
        if self.world_position is None:
            return False
        dx = self.world_position.x - goal_position.x
        dy = self.world_position.y - goal_position.y
        dist = math.hypot(dx, dy)
        return dist < self.arrival_tolerance

    def get_goal_yaw(self, goal_position: Point) -> float:
        if self.world_position is None:
            return 0.0
        dx = goal_position.x - self.world_position.x
        dy = goal_position.y - self.world_position.y
        return math.atan2(dy, dx)

    def timer_callback(self) -> None:
        if self.current_pose is None or self.world_position is None:
            return

        if self.state == "flying" and self.goal_position is not None:
            if self.check_arrived(self.goal_position):
                self.state = "idle"
            else:
                self.control_pose(self.goal_position, self.goal_yaw)

        if self.state == "idle":
            if self.goal_positions:
                x, y, z = self.goal_positions.pop(0)
                self.goal_position = Point(x=x, y=y, z=z)
                self.goal_yaw = self.get_goal_yaw(self.goal_position)
                self.state = "flying"
                self.get_logger().info(
                    (
                        "[controller] New goal: "
                        f"({self.goal_position.x:.2f}, {self.goal_position.y:.2f}, {self.goal_position.z:.2f}), "
                        f"yaw={math.degrees(self.goal_yaw):.1f}deg, "
                        f"remaining={len(self.goal_positions)}"
                    )
                )
            elif not self.done_logged:
                self.done_logged = True
                self.get_logger().info("[controller] Waypoint list complete")

    def control_pose(self, goal_position: Point, goal_yaw: float) -> None:
        if self.world_position is None:
            return

        dx = goal_position.x - self.world_position.x
        dy = goal_position.y - self.world_position.y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return

        # Publish incremental ENU position steps each cycle.
        step = min(self.speed * self.period_time, dist)
        x_cmd = (dx / dist) * step
        y_cmd = (dy / dist) * step
        z_cmd = goal_position.z

        msg = Joy()
        msg.axes = [x_cmd, y_cmd, z_cmd, goal_yaw]
        self.ctrl_pos_yaw_pub.publish(msg)

        pan_msg = Float64()
        pan_msg.data = self.pan
        self.update_pan_pub.publish(pan_msg)

        tilt_msg = Float64()
        tilt_msg.data = self.tilt
        self.update_tilt_pub.publish(tilt_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Controller()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

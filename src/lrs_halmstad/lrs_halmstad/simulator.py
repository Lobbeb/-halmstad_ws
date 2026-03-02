import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

from .world_names import gazebo_world_name


def _truthy_value(value: object) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


class Simulator(Node):
    def __init__(self) -> None:
        super().__init__("example_simulator")
        self.group = ReentrantCallbackGroup()

        self.declare_parameter("world", "orchard")
        self.declare_parameter("name", "dji1")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("initial_x", 10.0)
        self.declare_parameter("initial_y", 10.0)
        self.declare_parameter("initial_z", 4.0)
        self.declare_parameter("initial_yaw_deg", 0.0)
        self.declare_parameter("camera_z_offset", 0.27)
        self.declare_parameter("initial_pan_deg", 0.0)
        self.declare_parameter("initial_tilt_deg", -45.0)
        self.declare_parameter("set_camera_on_start", True)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.world = gazebo_world_name(str(self.get_parameter("world").value))
        self.name = str(self.get_parameter("name").value)
        self.z_offset = float(self.get_parameter("camera_z_offset").value)

        self.world_position = Point()
        self.world_position.x = float(self.get_parameter("initial_x").value)
        self.world_position.y = float(self.get_parameter("initial_y").value)
        self.world_position.z = float(self.get_parameter("initial_z").value)
        self.yaw = math.radians(float(self.get_parameter("initial_yaw_deg").value))

        # Tilt is degrees from horizontal plane, pan from UAV forward direction.
        self.tilt = float(self.get_parameter("initial_tilt_deg").value)
        self.pan = float(self.get_parameter("initial_pan_deg").value)

        self.update_msg: Optional[Joy] = None
        self.update_gimbal_flag = _truthy_value(self.get_parameter("set_camera_on_start").value)
        self._bad_joy_warned = False

        self.cli = self.create_client(SetEntityPose, f"/world/{self.world}/set_pose", callback_group=self.group)
        self.get_logger().info(f"Waiting for service: /world/{self.world}/set_pose")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                return
            self.get_logger().info("Service not available, waiting again...")

        self.pose_pub = self.create_publisher(PoseStamped, "pose", 10, callback_group=self.group)
        self.update_sub = self.create_subscription(
            Joy,
            "psdk_ros2/flight_control_setpoint_ENUposition_yaw",
            self.update_callback,
            10,
            callback_group=self.group,
        )
        self.update_tilt_sub = self.create_subscription(
            Float64,
            "update_tilt",
            self.update_tilt_callback,
            10,
            callback_group=self.group,
        )
        self.update_pan_sub = self.create_subscription(
            Float64,
            "update_pan",
            self.update_pan_callback,
            10,
            callback_group=self.group,
        )

        self.timer = self.create_timer(0.1, self.timer_callback, callback_group=self.group)
        self.get_logger().info(
            (
                "[simulator] Started: "
                f"ns={self.get_namespace()} world={self.world} name={self.name} "
                f"start=({self.world_position.x:.2f},{self.world_position.y:.2f},{self.world_position.z:.2f}) "
                f"yaw={math.degrees(self.yaw):.1f}deg"
            )
        )

    def update_callback(self, msg: Joy) -> None:
        self.update_msg = msg

    def update_pan_callback(self, msg: Float64) -> None:
        self.pan = msg.data
        self.update_gimbal_flag = True

    def update_tilt_callback(self, msg: Float64) -> None:
        self.tilt = msg.data
        self.update_gimbal_flag = True

    def update(self) -> None:
        if self.update_msg is None:
            return

        if len(self.update_msg.axes) < 4:
            if not self._bad_joy_warned:
                self.get_logger().warn(
                    "[simulator] Joy command requires 4 axes [dx,dy,z,yaw]; ignoring short message"
                )
                self._bad_joy_warned = True
            self.update_msg = None
            return

        self.world_position.x += float(self.update_msg.axes[0])
        self.world_position.y += float(self.update_msg.axes[1])
        self.world_position.z = float(self.update_msg.axes[2])
        self.yaw = float(self.update_msg.axes[3])
        self.update_msg = None

    def timer_callback(self) -> None:
        self.update()
        x = self.world_position.x
        y = self.world_position.y
        z = self.world_position.z

        self.set_pose(self.name, x, y, z, self.yaw)
        if self.update_gimbal_flag:
            self.set_camera(f"{self.name}_camera0", x, y, z, self.pan, self.tilt)
            self.update_gimbal_flag = False

        quat = _quaternion_from_euler(0.0, 0.0, self.yaw)
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position = self.world_position
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        self.pose_pub.publish(msg)

    def set_pose(self, name: str, x: float, y: float, z: float, yaw: float) -> None:
        request = SetEntityPose.Request()
        quat = _quaternion_from_euler(0.0, 0.0, yaw)
        request.entity.id = 0
        request.entity.name = name
        request.entity.type = request.entity.MODEL
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = z
        request.pose.orientation.x = quat[0]
        request.pose.orientation.y = quat[1]
        request.pose.orientation.z = quat[2]
        request.pose.orientation.w = quat[3]
        self.cli.call_async(request)

    def set_camera(self, name: str, x: float, y: float, z: float, pan_deg: float, tilt_deg: float) -> None:
        camera_yaw = self.yaw + math.radians(pan_deg)
        request = SetEntityPose.Request()
        quat = _quaternion_from_euler(0.0, -math.radians(tilt_deg), camera_yaw)
        request.entity.id = 0
        request.entity.name = name
        request.entity.type = request.entity.MODEL
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = z - self.z_offset
        request.pose.orientation.x = quat[0]
        request.pose.orientation.y = quat[1]
        request.pose.orientation.z = quat[2]
        request.pose.orientation.w = quat[3]
        self.cli.call_async(request)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Simulator()
    executor = MultiThreadedExecutor(num_threads=8)
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

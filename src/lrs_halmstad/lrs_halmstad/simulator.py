import os
import re
import rclpy
from rclpy.node import Node
import math
from typing import Set

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger
from std_msgs.msg import UInt8, Float64
from geometry_msgs.msg import PoseArray, PoseStamped, Vector3Stamped, QuaternionStamped, Quaternion, Point

from tf_transformations import quaternion_from_euler, quaternion_multiply
from sensor_msgs.msg import Joy
from ros_gz_interfaces.srv import SetEntityPose

class Simulator(Node):
    def __init__(self):
        super().__init__('example_simulator')
        self.group = ReentrantCallbackGroup()

        self.declare_parameter("world", "orchard")
        self.declare_parameter("uav_name", "")
        self.declare_parameter("name", "")
        self.declare_parameter("update_rate_hz", 30.0)
        self.declare_parameter("camera_mode", "integrated_joint")
        self.declare_parameter("camera_name", "camera0")
        self.declare_parameter("camera_z_offset", 0.27)
        self.declare_parameter("gimbal_pitch_min_rad", -1.5708)
        self.declare_parameter("gimbal_pitch_max_rad", 0.7854)

        self.frame_id = "odom"
        self.world = self.get_parameter("world").value
        self.uav_name = self._resolve_uav_name()
        self.name = self.uav_name
        self.update_rate_hz = max(1.0, float(self.get_parameter("update_rate_hz").value))
        self.period_time = 1.0 / self.update_rate_hz
        self.camera_mode = str(self.get_parameter("camera_mode").value).strip().lower()
        self.camera_name = str(self.get_parameter("camera_name").value).strip()
        self.camera_z_offset = float(self.get_parameter("camera_z_offset").value)
        self.gimbal_pitch_min = float(self.get_parameter("gimbal_pitch_min_rad").value)
        self.gimbal_pitch_max = float(self.get_parameter("gimbal_pitch_max_rad").value)
        if self.camera_mode == "integrated":
            self.camera_mode = "integrated_joint"
        if self.camera_mode == "detached":
            self.camera_mode = "detached_model"
        if self.camera_mode not in ("integrated_joint", "detached_model"):
            self.get_logger().warn(
                f"Unknown camera_mode='{self.camera_mode}', using 'integrated_joint'"
            )
            self.camera_mode = "integrated_joint"

        default_start_x, default_start_y, default_start_z = self._default_start_xyz(self.uav_name)
        self.declare_parameter("start_x", default_start_x)
        self.declare_parameter("start_y", default_start_y)
        self.declare_parameter("start_z", default_start_z)
        self.declare_parameter("start_yaw_deg", 0.0)

        self.world_position = Point()
        self.world_position.x = float(self.get_parameter("start_x").value)
        self.world_position.y = float(self.get_parameter("start_y").value)
        self.world_position.z = float(self.get_parameter("start_z").value)


        self.ns = self.get_namespace()
        print("NAMESPACE:", self.ns)

        self.yaw = math.radians(float(self.get_parameter("start_yaw_deg").value))
        self.update_msg = None
        self.update_gimbal_flag = False
        self.tilt = -45.0
        self.pan = 0.0
        self.tilt_update_msg = None
        self.pan_update_msg = None
        self.future1 = None
        self.future2 = None

        self.cli = self.create_client(SetEntityPose, f'/world/{self.world}/set_pose', callback_group=self.group)        
        print(f"WAIT for service: /world/{self.world}/set_pose'")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        pose_topic = self._uav_topic("pose")
        cmd_topic = self._uav_topic("psdk_ros2/flight_control_setpoint_ENUposition_yaw")
        tilt_topic = self._uav_topic("update_tilt")
        pan_topic = self._uav_topic("update_pan")
        gimbal_pitch_topic = self._gimbal_topic("pitch")
        gimbal_yaw_topic = self._gimbal_topic("yaw")

        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10, callback_group=self.group)
        self.update_sub = self.create_subscription(
            Joy, cmd_topic, self.update_callback, 10, callback_group=self.group
        )
        self.update_tilt_sub = self.create_subscription(
            Float64, tilt_topic, self.update_tilt_callback, 10, callback_group=self.group
        )
        self.update_pan_sub = self.create_subscription(
            Float64, pan_topic, self.update_pan_callback, 10, callback_group=self.group
        )
        self.gimbal_pitch_pub = self.create_publisher(Float64, gimbal_pitch_topic, 10)
        self.gimbal_yaw_pub = self.create_publisher(Float64, gimbal_yaw_topic, 10)

        self.timer = self.create_timer(self.period_time, self.timer_callback, callback_group=self.group)
        self.get_logger().info(
            f"Using UAV '{self.uav_name}' topics: cmd={cmd_topic}, pose={pose_topic}, "
            f"gimbal_pitch={gimbal_pitch_topic}, camera_mode={self.camera_mode}, "
            f"update_rate_hz={self.update_rate_hz:.1f}"
        )
        self.get_logger().info(
            f"Initial pose x={self.world_position.x:.2f} y={self.world_position.y:.2f} "
            f"z={self.world_position.z:.2f} yaw_deg={math.degrees(self.yaw):.1f}"
        )

    def _uav_topic(self, suffix: str) -> str:
        return f"/{self.uav_name}/{suffix.lstrip('/')}"

    def _gimbal_topic(self, axis: str) -> str:
        return f"/model/{self.uav_name}/joint/{self.uav_name}_gimbal_joint/{axis}/cmd_pos"

    def _clamp(self, value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def _default_start_xyz(self, uav_name: str):
        match = re.match(r"^dji(\d+)$", uav_name)
        if match:
            index = int(match.group(1))
            return 0.0, 0.0, 2.27 + float(index)
        return 0.0, 0.0, 2.27

    def _graph_uav_candidates(self) -> Set[str]:
        patterns = [
            re.compile(r"^/([^/]+)/psdk_ros2/flight_control_setpoint_ENUposition_yaw$"),
            re.compile(r"^/([^/]+)/pose$"),
            re.compile(r"^/([^/]+)/pose_cmd(?:/odom)?$"),
            re.compile(r"^/([^/]+)/camera0/(?:image_raw|camera_info)$"),
        ]
        candidates: Set[str] = set()
        for topic_name, _ in self.get_topic_names_and_types():
            for pattern in patterns:
                match = pattern.match(topic_name)
                if match:
                    candidates.add(match.group(1))
                    break
        return candidates

    def _resolve_uav_name(self) -> str:
        explicit_uav = str(self.get_parameter("uav_name").value).strip()
        explicit_name = str(self.get_parameter("name").value).strip()
        if explicit_uav:
            return explicit_uav
        if explicit_name:
            return explicit_name

        namespace = str(self.get_namespace()).strip("/")
        if namespace:
            return namespace

        candidates = sorted(self._graph_uav_candidates())
        if len(candidates) == 1:
            self.get_logger().info(f"Auto-detected UAV from graph: {candidates[0]}")
            return candidates[0]
        if len(candidates) > 1:
            self.get_logger().warn(
                f"Multiple UAVs discovered {candidates}; using '{candidates[0]}'. "
                "Set parameter 'uav_name' to choose explicitly."
            )
            return candidates[0]

        self.get_logger().warn("Could not auto-detect UAV name; defaulting to 'dji0'.")
        return "dji0"

    def update_callback(self, msg):
        # print("update_callback:", msg)
        self.update_msg = msg

    def update_pan_callback(self, msg):
        self.pan_update_msg = msg
        self.pan = msg.data
        
    def update_tilt_callback(self, msg):
        self.tilt_update_msg = msg
        self.tilt = msg.data

    def update(self):
        if self.update_msg:
            self.world_position.x += self.update_msg.axes[0]
            self.world_position.y += self.update_msg.axes[1]
            self.world_position.z = self.update_msg.axes[2]
            self.yaw = self.update_msg.axes[3]
            self.update_msg = None
        if self.tilt_update_msg and self.pan_update_msg:
            self.update_gimbal_flag = True
            
    def timer_callback(self):
        try:
            self.update()
            x = self.world_position.x
            y = self.world_position.y
            z = self.world_position.z
            self.set_pose(self.name, x, y, z, self.yaw)
            if self.update_gimbal_flag:
                if self.camera_mode == "detached_model":
                    self.set_camera_model_pose(x, y, z, self.pan, self.tilt)
                else:
                    pitchmsg = Float64()
                    pitchmsg.data = self._clamp(
                        -math.radians(self.tilt), self.gimbal_pitch_min, self.gimbal_pitch_max
                    )
                    self.gimbal_pitch_pub.publish(pitchmsg)
                    yawmsg = Float64()
                    yawmsg.data = math.radians(self.pan)
                    self.gimbal_yaw_pub.publish(yawmsg)
            quat = quaternion_from_euler(0.0, 0.0, self.yaw)
            msg = PoseStamped()
            msg.header.frame_id = self.frame_id
            msg.pose.position = self.world_position
            msg.pose.orientation.x = quat[0]
            msg.pose.orientation.y = quat[1]
            msg.pose.orientation.z = quat[2]
            msg.pose.orientation.w = quat[3]
            self.pose_pub.publish(msg)
        except Exception as ex:
            print("Exception timer_callback:", ex, type(ex))


    def set_pose(self, name, x, y, z, yaw):
        try:
            if self.future1 is not None and not self.future1.done():
                return
            robot_request = SetEntityPose.Request()
            quat1 = quaternion_from_euler(0.0, 0.0, self.yaw)
            robot_request.entity.id = 0
            robot_request.entity.name = name
            robot_request.entity.type = robot_request.entity.MODEL
            robot_request.pose.position.x = x
            robot_request.pose.position.y = y
            robot_request.pose.position.z = z
            robot_request.pose.orientation.x = quat1[0]
            robot_request.pose.orientation.y = quat1[1]
            robot_request.pose.orientation.z = quat1[2]
            robot_request.pose.orientation.w = quat1[3]
            ## print(robot_request)
            self.future1 = self.cli.call_async(robot_request)
            #rclpy.spin_until_future_complete(self, future1)
            #print(future1.result())
        except Exception as ex:
            print("Exception set_pose:", ex, type(ex))

    def set_camera_model_pose(self, x, y, z, pan_deg, tilt_deg):
        try:
            if self.future2 is not None and not self.future2.done():
                return
            camera_request = SetEntityPose.Request()
            yaw = self.yaw + math.radians(pan_deg)
            quat2 = quaternion_from_euler(0.0, -math.radians(tilt_deg), yaw)
            camera_request.entity.id = 0
            camera_request.entity.name = f"{self.name}_{self.camera_name}"
            camera_request.entity.type = camera_request.entity.MODEL
            camera_request.pose.position.x = x
            camera_request.pose.position.y = y
            camera_request.pose.position.z = z - self.camera_z_offset
            camera_request.pose.orientation.x = quat2[0]
            camera_request.pose.orientation.y = quat2[1]
            camera_request.pose.orientation.z = quat2[2]
            camera_request.pose.orientation.w = quat2[3]
            self.future2 = self.cli.call_async(camera_request)
        except Exception as ex:
            print("Exception set_camera_model_pose:", ex, type(ex))


def main(args=None):
    rclpy.init(args=args)
    node = Simulator()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    print("Spinning simple simulator")
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    executor.shutdown()    
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
            

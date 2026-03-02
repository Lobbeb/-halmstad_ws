from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    world_arg = DeclareLaunchArgument("world", default_value="orchard")
    uav_name_arg = DeclareLaunchArgument(
        "uav_name",
        default_value="dji1",
        description="Gazebo model name and ROS namespace for simulator/controller",
    )
    frame_id_arg = DeclareLaunchArgument("frame_id", default_value="odom")
    run_controller_arg = DeclareLaunchArgument(
        "run_controller",
        default_value="true",
        description="Start example waypoint controller together with simulator",
    )
    initial_x_arg = DeclareLaunchArgument("initial_x", default_value="10.0")
    initial_y_arg = DeclareLaunchArgument("initial_y", default_value="10.0")
    initial_z_arg = DeclareLaunchArgument("initial_z", default_value="4.0")
    initial_yaw_deg_arg = DeclareLaunchArgument("initial_yaw_deg", default_value="0.0")
    initial_pan_deg_arg = DeclareLaunchArgument("initial_pan_deg", default_value="0.0")
    initial_tilt_deg_arg = DeclareLaunchArgument("initial_tilt_deg", default_value="-45.0")

    simulator_node = Node(
        package="lrs_halmstad",
        executable="simulator",
        name="simulator",
        namespace=LaunchConfiguration("uav_name"),
        output="screen",
        parameters=[
            {
                "world": LaunchConfiguration("world"),
                "name": LaunchConfiguration("uav_name"),
                "frame_id": LaunchConfiguration("frame_id"),
                "initial_x": LaunchConfiguration("initial_x"),
                "initial_y": LaunchConfiguration("initial_y"),
                "initial_z": LaunchConfiguration("initial_z"),
                "initial_yaw_deg": LaunchConfiguration("initial_yaw_deg"),
                "initial_pan_deg": LaunchConfiguration("initial_pan_deg"),
                "initial_tilt_deg": LaunchConfiguration("initial_tilt_deg"),
            }
        ],
    )

    controller_node = Node(
        package="lrs_halmstad",
        executable="controller",
        name="controller",
        namespace=LaunchConfiguration("uav_name"),
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_controller")),
    )

    return LaunchDescription(
        [
            world_arg,
            uav_name_arg,
            frame_id_arg,
            run_controller_arg,
            initial_x_arg,
            initial_y_arg,
            initial_z_arg,
            initial_yaw_deg_arg,
            initial_pan_deg_arg,
            initial_tilt_deg_arg,
            simulator_node,
            controller_node,
        ]
    )

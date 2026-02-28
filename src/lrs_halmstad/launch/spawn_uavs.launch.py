#import logging
#logging.root.setLevel(logging.DEBUG)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression


def _gazebo_world_name(world_sub):
    return PythonExpression([
        "'office_construction' if '",
        world_sub,
        "' == 'construction' else '",
        world_sub,
        "'",
    ])


world_arg = DeclareLaunchArgument('world', default_value='orchard',
                      choices=[
                          'construction',
                          'office',
                          'orchard',
                          'pipeline',
                          'solar_farm',
                          'warehouse',
                      ],
                      description='Gazebo World')
uav_mode_arg = DeclareLaunchArgument('uav_mode', default_value='teleport',
                      description='UAV mode: teleport (deterministic) or physics')

def generate_launch_description():
    gz_world = _gazebo_world_name(LaunchConfiguration('world'))

    dji0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": "dji0",
                          "type": "m100",
                          "uav_mode": LaunchConfiguration('uav_mode'),
                          "with_camera": "true",
                          "camera_name": "camera0",
                          "z": "2.27",
                          "world": LaunchConfiguration('world')
                          }.items(),
    )


    dji1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": "dji1",
                          "type": "m100",
                          "uav_mode": LaunchConfiguration('uav_mode'),
                          "with_camera": "true",
                          "camera_name": "camera0",
                          "z": "3.27",
                          "world": LaunchConfiguration('world'),
                          }.items(),
    )

    dji2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": "dji2",
                          "type": "m100",
                          "uav_mode": LaunchConfiguration('uav_mode'),
                          "with_camera": "true",
                          "camera_name": "camera0",
                          "z": "4.27",
                          "world": LaunchConfiguration('world'),
                          }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
#        arguments= ['/world/orchard/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
        arguments=[
            ['/world/', gz_world, '/set_pose@ros_gz_interfaces/srv/SetEntityPose']
        ],
        output='screen'
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/dji0/camera0/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/dji0/camera0/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/dji1/camera0/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/dji1/camera0/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/dji2/camera0/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/dji2/camera0/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
        ],
        output='screen',
    )
    
    
    return LaunchDescription([
        world_arg,
        uav_mode_arg,
        bridge,
        camera_bridge,
        dji0,
        dji1,
        dji2,
    ])

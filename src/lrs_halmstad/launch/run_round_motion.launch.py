from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_default = PathJoinSubstitution(
        [FindPackageShare('lrs_halmstad'), 'config', 'run_round_follow_defaults.yaml']
    )
    launch_file = PathJoinSubstitution(
        [FindPackageShare('lrs_halmstad'), 'launch', 'run_round_follow_motion.launch.py']
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_default,
        description='Parameter YAML for leader_estimator, follow_uav, and UGV motion driver',
    )
    world_arg = DeclareLaunchArgument('world', default_value='orchard')
    uav_name_arg = DeclareLaunchArgument('uav_name', default_value='dji0')
    leader_mode_arg = DeclareLaunchArgument('leader_mode', default_value='odom')
    leader_perception_enable_arg = DeclareLaunchArgument('leader_perception_enable', default_value='false')
    start_estimator_arg = DeclareLaunchArgument('start_leader_estimator', default_value='auto')
    uav_start_x_arg = DeclareLaunchArgument('uav_start_x', default_value='-2.0')
    uav_start_y_arg = DeclareLaunchArgument('uav_start_y', default_value='0.0')
    uav_start_yaw_deg_arg = DeclareLaunchArgument('uav_start_yaw_deg', default_value='0.0')
    ugv_namespace_arg = DeclareLaunchArgument('ugv_namespace', default_value='a201_0000')
    ugv_cmd_topic_arg = DeclareLaunchArgument('ugv_cmd_topic', default_value='cmd_vel')
    ugv_odom_topic_arg = DeclareLaunchArgument(
        'ugv_odom_topic',
        default_value=['/', LaunchConfiguration('ugv_namespace'), '/platform/odom'],
    )
    leader_image_topic_arg = DeclareLaunchArgument(
        'leader_image_topic',
        default_value=['/', LaunchConfiguration('uav_name'), '/camera0/image_raw'],
    )
    leader_camera_info_topic_arg = DeclareLaunchArgument(
        'leader_camera_info_topic',
        default_value=['/', LaunchConfiguration('uav_name'), '/camera0/camera_info'],
    )
    leader_depth_topic_arg = DeclareLaunchArgument('leader_depth_topic', default_value='')
    leader_uav_pose_topic_arg = DeclareLaunchArgument(
        'leader_uav_pose_topic',
        default_value=['/', LaunchConfiguration('uav_name'), '/pose_cmd'],
    )
    yolo_weights_arg = DeclareLaunchArgument(
        'yolo_weights',
        default_value='/home/ruben/halmstad_ws/models/yolo26/yolo26n.pt',
    )
    yolo_device_arg = DeclareLaunchArgument('yolo_device', default_value='cpu')
    event_topic_arg = DeclareLaunchArgument('event_topic', default_value='/coord/events')
    ugv_start_delay_arg = DeclareLaunchArgument('ugv_start_delay_s', default_value='0.0')

    forward_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'world': LaunchConfiguration('world'),
            'uav_name': LaunchConfiguration('uav_name'),
            'leader_mode': LaunchConfiguration('leader_mode'),
            'leader_perception_enable': LaunchConfiguration('leader_perception_enable'),
            'start_leader_estimator': LaunchConfiguration('start_leader_estimator'),
            'uav_start_x': LaunchConfiguration('uav_start_x'),
            'uav_start_y': LaunchConfiguration('uav_start_y'),
            'uav_start_yaw_deg': LaunchConfiguration('uav_start_yaw_deg'),
            'ugv_namespace': LaunchConfiguration('ugv_namespace'),
            'ugv_cmd_topic': LaunchConfiguration('ugv_cmd_topic'),
            'ugv_odom_topic': LaunchConfiguration('ugv_odom_topic'),
            'leader_image_topic': LaunchConfiguration('leader_image_topic'),
            'leader_camera_info_topic': LaunchConfiguration('leader_camera_info_topic'),
            'leader_depth_topic': LaunchConfiguration('leader_depth_topic'),
            'leader_uav_pose_topic': LaunchConfiguration('leader_uav_pose_topic'),
            'yolo_weights': LaunchConfiguration('yolo_weights'),
            'yolo_device': LaunchConfiguration('yolo_device'),
            'event_topic': LaunchConfiguration('event_topic'),
            'ugv_start_delay_s': LaunchConfiguration('ugv_start_delay_s'),
        }.items(),
    )

    return LaunchDescription([
        params_file_arg,
        world_arg,
        uav_name_arg,
        leader_mode_arg,
        leader_perception_enable_arg,
        start_estimator_arg,
        uav_start_x_arg,
        uav_start_y_arg,
        uav_start_yaw_deg_arg,
        ugv_namespace_arg,
        ugv_cmd_topic_arg,
        ugv_odom_topic_arg,
        leader_image_topic_arg,
        leader_camera_info_topic_arg,
        leader_depth_topic_arg,
        leader_uav_pose_topic_arg,
        yolo_weights_arg,
        yolo_device_arg,
        event_topic_arg,
        ugv_start_delay_arg,
        forward_launch,
    ])

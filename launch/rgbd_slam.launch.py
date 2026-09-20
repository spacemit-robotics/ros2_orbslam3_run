import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('orbslam3_run')
    dependency_prefix = os.environ.get('ORB_SLAM3_PREFIX', '/opt/orbslam3')

    arguments = [
        DeclareLaunchArgument(
            'vocabulary',
            default_value=os.path.join(
                dependency_prefix, 'share', 'orbslam3', 'ORBvoc.txt')),
        DeclareLaunchArgument(
            'settings',
            default_value=os.path.join(
                package_share, 'config', 'd455_rgbd_slam.yaml')),
        DeclareLaunchArgument(
            'save_directory',
            default_value=os.path.expanduser('~/.ros/orbslam3_slam')),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('map_frame', default_value='orb_map'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('map_publish_period', default_value='1.0'),
        DeclareLaunchArgument('max_path_poses', default_value='10000'),
        DeclareLaunchArgument(
            'color_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('pose_topic', default_value='/orbslam3_slam/pose'),
        DeclareLaunchArgument('path_topic', default_value='/orbslam3_slam/path'),
        DeclareLaunchArgument(
            'map_points_topic', default_value='/orbslam3_slam/map_points'),
        DeclareLaunchArgument(
            'tracking_state_topic',
            default_value='/orbslam3_slam/tracking_state'),
        DeclareLaunchArgument(
            'finish_mapping_service',
            default_value='/orbslam3_slam/finish_mapping'),
        DeclareLaunchArgument(
            'reset_map_service', default_value='/orbslam3_slam/reset_map'),
    ]

    node = Node(
        package='orbslam3_run',
        executable='orbslam3_slam_node',
        name='orbslam3_slam',
        output='screen',
        parameters=[{
            'vocabulary': LaunchConfiguration('vocabulary'),
            'settings': LaunchConfiguration('settings'),
            'save_directory': LaunchConfiguration('save_directory'),
            'publish_tf': ParameterValue(
                LaunchConfiguration('publish_tf'), value_type=bool),
            'map_frame': LaunchConfiguration('map_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'map_publish_period': ParameterValue(
                LaunchConfiguration('map_publish_period'), value_type=float),
            'max_path_poses': ParameterValue(
                LaunchConfiguration('max_path_poses'), value_type=int),
        }],
        remappings=[
            ('/camera/color/image_raw', LaunchConfiguration('color_topic')),
            ('/camera/aligned_depth_to_color/image_raw',
             LaunchConfiguration('depth_topic')),
            ('pose', LaunchConfiguration('pose_topic')),
            ('path', LaunchConfiguration('path_topic')),
            ('map_points', LaunchConfiguration('map_points_topic')),
            ('tracking_state', LaunchConfiguration('tracking_state_topic')),
            ('finish_mapping', LaunchConfiguration('finish_mapping_service')),
            ('reset_map', LaunchConfiguration('reset_map_service')),
        ],
    )
    return LaunchDescription(arguments + [node])

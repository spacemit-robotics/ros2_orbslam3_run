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

    return LaunchDescription([
        DeclareLaunchArgument(
            'vocabulary',
            default_value=os.path.join(
                dependency_prefix, 'share', 'orbslam3', 'ORBvoc.txt')),
        DeclareLaunchArgument(
            'settings',
            default_value=os.path.join(package_share, 'config', 'd455_rgbd.yaml')),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='false',
            description='是否发布 odom_frame 到 base_frame 的 TF'),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='orb_odom',
            description='里程计坐标系名称'),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_footprint',
            description='机器人底盘坐标系名称'),
        DeclareLaunchArgument(
            'color_topic',
            default_value='/camera/color/image_raw',
            description='彩色图像输入话题'),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/aligned_depth_to_color/image_raw',
            description='对齐到彩色图像的深度输入话题'),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/color/camera_info',
            description='彩色相机标定信息话题'),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/orbslam3/odom',
            description='里程计输出话题'),
        DeclareLaunchArgument(
            'odom_2d_topic',
            default_value='/orbslam3/odom_2d',
            description='二维平面里程计输出话题'),
        DeclareLaunchArgument(
            'tracking_state_topic',
            default_value='/orbslam3/tracking_state',
            description='跟踪状态输出话题'),
        Node(
            package='orbslam3_run',
            executable='orbslam3_odometry_node',
            name='orbslam3_odometry',
            output='screen',
            parameters=[{
                'vocabulary': LaunchConfiguration('vocabulary'),
                'settings': LaunchConfiguration('settings'),
                'publish_tf': ParameterValue(
                    LaunchConfiguration('publish_tf'), value_type=bool),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
            }],
            remappings=[
                ('/camera/color/image_raw', LaunchConfiguration('color_topic')),
                ('/camera/aligned_depth_to_color/image_raw',
                 LaunchConfiguration('depth_topic')),
                ('/camera/color/camera_info',
                 LaunchConfiguration('camera_info_topic')),
                ('odom', LaunchConfiguration('odom_topic')),
                ('odom_2d', LaunchConfiguration('odom_2d_topic')),
                ('tracking_state', LaunchConfiguration('tracking_state_topic')),
            ],
        )
    ])

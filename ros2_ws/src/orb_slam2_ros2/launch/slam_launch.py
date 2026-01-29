from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('orb_slam2_ros2')
    
    # Paths to ORB-SLAM2 files
    vocabulary_file = '/home/jetson/capstone/ORB_SLAM2/Vocabulary/ORBvoc.txt'
    config_file = PathJoinSubstitution([pkg_share, 'config', 'realsense_d435.yaml'])
    
    # Declare launch arguments
    use_viewer_arg = DeclareLaunchArgument(
        'use_viewer',
        default_value='false',
        description='Enable ORB-SLAM2 visualization (set to true for GUI)'
    )
    
    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/camera/realsense/color/image_raw',
        description='RGB image topic'
    )
    
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/realsense/aligned_depth_to_color/image_raw',
        description='Depth image topic (aligned to color for better accuracy)'
    )
    
    # ORB-SLAM2 RGBD node
    slam_node = Node(
        package='orb_slam2_ros2',
        executable='rgbd_slam_node',
        name='rgbd_slam_node',
        output='screen',
        arguments=[
            vocabulary_file,
            config_file,
            LaunchConfiguration('use_viewer')
        ],
        parameters=[{
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic')
        }]
    )
    
    return LaunchDescription([
        use_viewer_arg,
        rgb_topic_arg,
        depth_topic_arg,
        slam_node
    ])

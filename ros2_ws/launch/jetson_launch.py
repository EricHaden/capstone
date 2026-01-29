from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense',
            namespace='camera',
            parameters=[
                {"enable_depth": True},
                {"enable_color": True},
                {"enable_infra": True},
                {"publish_tf": False},  # Disable transform publishing
                {"enable_gyro": False},  # Disable IMU (gyro)
                {"enable_accel": False},  # Disable IMU (accelerometer)
                {"enable_sync": True},  # Sync depth and color frames
                {"depth_module.depth_profile": "640x480x30"},
                {"rgb_camera.color_profile": "640x480x30"},
                {"rgb_camera.color_format": "RGB8"},  # Force RGB8 format
                {"depth_module.infra_profile": "640x480x30"},
                {"align_depth.enable": True},  # Align depth to color
                ],
        ),
    ])

# ORB-SLAM2 ROS2 Workspace

This workspace contains the ROS2 wrapper for ORB-SLAM2, integrated with the Intel RealSense camera.

## Directory Structure

```
/home/jetson/capstone/
├── ORB_SLAM2/              # Core ORB-SLAM2 library
│   ├── lib/                # Compiled libraries
│   ├── include/            # Header files
│   ├── Vocabulary/         # ORB vocabulary file
│   └── Examples/           # Original examples (ROS1, Monocular, etc.)
│
├── ros2_ws/                # ROS2 workspace
│   ├── src/
│   │   └── orb_slam2_ros2/ # ROS2 wrapper package
│   ├── build/              # Build artifacts
│   └── install/            # Installed files
│
├── datasets/               # Test datasets
└── Pangolin/               # Visualization library
```

## Quick Start

### 1. Build the Workspace

```bash
cd /home/jetson/capstone/ros2_ws
colcon build --packages-select orb_slam2_ros2 --symlink-install
```

### 2. Source the Workspace

```bash
source /home/jetson/capstone/ros2_ws/install/setup.bash
```

Or add to your `~/.bashrc`:

```bash
echo "source /home/jetson/capstone/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 3. Launch the SLAM System

**Terminal 1** - Start camera:
```bash
ros2 launch /home/jetson/capstone/ros2_ws/launch/jetson_launch.py
```

**Terminal 2** - Start SLAM:
```bash
source /home/jetson/capstone/ros2_ws/install/setup.bash
ros2 launch orb_slam2_ros2 slam_launch.py
```

## Available Packages

### orb_slam2_ros2

ROS2 wrapper for ORB-SLAM2 RGB-D SLAM.

**Documentation:**
- [README.md](src/orb_slam2_ros2/README.md) - Full documentation
- [QUICKSTART.md](src/orb_slam2_ros2/QUICKSTART.md) - Step-by-step guide
- [MIGRATION.md](src/orb_slam2_ros2/MIGRATION.md) - Migration notes

**Executables:**
- `rgbd_slam_node` - Main SLAM node

**Launch Files:**
- `slam_launch.py` - Launch SLAM with default parameters

**Configuration:**
- `config/realsense_d435.yaml` - Camera parameters and ORB settings

## Topics

### Subscribed
- `/camera/realsense/color/image_raw` - RGB image
- `/camera/realsense/depth/image_rect_raw` - Depth image

### Published
- `/orb_slam2/camera_pose` - Camera pose (geometry_msgs/PoseStamped)
- `/tf` - TF transform (map → camera_link)

## Dependencies

- ROS2 Humble
- OpenCV
- Eigen3
- Pangolin
- ORB-SLAM2 core library
- realsense2_camera (for camera driver)

## Building Additional Packages

To add more packages to this workspace:

```bash
cd /home/jetson/capstone/ros2_ws/src
# Create or copy your package here
cd ..
colcon build --packages-select <your_package_name>
source install/setup.bash
```

## Troubleshooting

### Package not found after building

Make sure to source the workspace:
```bash
source /home/jetson/capstone/ros2_ws/install/setup.bash
```

### Build errors

Clean and rebuild:
```bash
rm -rf build install
colcon build --packages-select orb_slam2_ros2
```

### Runtime errors

Check that ORB-SLAM2 libraries exist:
```bash
ls -l /home/jetson/capstone/ORB_SLAM2/lib/
ls -l /home/jetson/capstone/ORB_SLAM2/Vocabulary/ORBvoc.txt
```

## Resources

- [ORB-SLAM2 GitHub](https://github.com/raulmur/ORB_SLAM2)
- [RealSense ROS2 GitHub](https://github.com/IntelRealSense/realsense-ros)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)

## License

This wrapper is GPLv3, compatible with ORB-SLAM2's license.

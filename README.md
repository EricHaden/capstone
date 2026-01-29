# ORB-SLAM2 ROS2 Integration for Intel RealSense

A ROS2 wrapper for ORB-SLAM2 RGB-D SLAM, designed for real-time visual SLAM on NVIDIA Jetson with Intel RealSense cameras.

## Overview

This project integrates ORB-SLAM2 with ROS2 for real-time camera pose estimation and 3D mapping. It's optimized for the Jetson platform with OpenCV 4 compatibility patches.

## Repository Structure

```
capstone/
├── ORB_SLAM2/          # ORB-SLAM2 core library (submodule, OpenCV 4 compatible)
├── Pangolin/           # Visualization library (submodule)
├── ros2_ws/            # ROS2 workspace
│   ├── src/
│   │   └── orb_slam2_ros2/   # ROS2 wrapper package
│   └── launch/         # System launch files
└── VERIFICATION.md     # Setup verification checklist
```

## Prerequisites

- NVIDIA Jetson (tested on Jetson Nano/Xavier)
- Ubuntu 20.04+
- ROS2 Humble
- Intel RealSense camera (D435/D435i)
- OpenCV 4.x
- Eigen3

## Quick Start

### 1. Clone with Submodules

```bash
git clone --recursive https://github.com/EricHaden/capstone.git
cd capstone
```

If you already cloned without `--recursive`:
```bash
git submodule update --init --recursive
```

### 2. Build Dependencies

#### Build Pangolin
```bash
cd Pangolin
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
cd ../..
```

#### Build ORB-SLAM2
```bash
cd ORB_SLAM2
./build.sh
cd ..
```

### 3. Build the ROS2 Package

```bash
cd ros2_ws
colcon build --packages-select orb_slam2_ros2 --symlink-install
source install/setup.bash
```

### 4. Run SLAM

**Terminal 1** - Start RealSense camera:
```bash
ros2 launch realsense2_camera rs_launch.py enable_depth:=true enable_color:=true
```

**Terminal 2** - Start SLAM:
```bash
source ros2_ws/install/setup.bash
ros2 launch orb_slam2_ros2 slam_launch.py
```

## Topics

### Subscribed
- `/camera/realsense/color/image_raw` - RGB image
- `/camera/realsense/depth/image_rect_raw` - Depth image

### Published
- `/orb_slam2/camera_pose` - Camera pose (PoseStamped)
- `/tf` - Transform: map → camera_link

## Configuration

Camera parameters are in `ros2_ws/src/orb_slam2_ros2/config/realsense_d435.yaml`.

Key parameters:
- Camera intrinsics (fx, fy, cx, cy)
- ORBextractor.nFeatures (reduce for better Jetson performance)
- DepthMapFactor

## Documentation

- [ROS2 Package README](ros2_ws/src/orb_slam2_ros2/README.md) - Detailed usage
- [Quick Start Guide](ros2_ws/src/orb_slam2_ros2/QUICKSTART.md) - Step-by-step setup
- [Migration Notes](ros2_ws/src/orb_slam2_ros2/MIGRATION.md) - Porting notes

## Modifications from Upstream

### ORB-SLAM2
- OpenCV 4 compatibility patches
- Build system updates for modern CMake
- Jetson ARM64 compatibility

### This Wrapper
- Native ROS2 Humble integration (no bridge)
- RealSense D435 configuration
- TF2 broadcasting
- Optimized for Jetson performance

## Troubleshooting

### SLAM not initializing
- Ensure good lighting and textured surfaces
- Move camera slowly during initialization
- Check that both RGB and depth topics are publishing

### Performance issues
- Reduce `ORBextractor.nFeatures` (e.g., 500-600)
- Lower camera resolution in launch file
- Disable viewer: `use_viewer:=false`

## License

This project is GPLv3, compatible with ORB-SLAM2's license.

## Acknowledgments

- [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) by Raúl Mur-Artal
- [Pangolin](https://github.com/stevenlovegrove/Pangolin) by Steven Lovegrove
- [RealSense ROS2](https://github.com/IntelRealSense/realsense-ros)


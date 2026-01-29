# ORB-SLAM2 ROS2 Wrapper for Intel RealSense

ROS2 wrapper for ORB-SLAM2 RGB-D SLAM system, designed to work with Intel RealSense cameras on the Jetson platform. 


## Overview

This package subscribes to RGB and depth image topics from a RealSense camera and runs ORB-SLAM2 to estimate camera pose and build a 3D map in real-time.

## Dependencies

- ROS2 (Foxy or later)
- OpenCV
- Eigen3
- Pangolin
- ORB-SLAM2 core library (located at `/home/jetson/capstone/ORB_SLAM2`)
- realsense2_camera ROS2 package

## Building

```bash
cd /home/jetson/capstone/ros2_ws
colcon build --packages-select orb_slam2_ros2 --symlink-install
source ./setup_workspace.sh   # use this for both bash and zsh
```

## Usage

### 1. Launch the RealSense Camera

First, start the RealSense camera node:

```bash
# Using the jetson_launch.py file (configured for your setup)
ros2 launch /home/jetson/capstone/ros2_ws/launch/jetson_launch.py
```

Or using the standard RealSense launch file:
```bash
ros2 launch realsense2_camera rs_launch.py enable_depth:=true enable_color:=true
```

### 2. Verify Camera Topics

In a new terminal, verify the camera is publishing:

```bash
source /home/jetson/capstone/ros2_ws/setup_workspace.sh
ros2 topic list | grep camera
ros2 topic hz /camera/realsense/color/image_raw
ros2 topic hz /camera/realsense/depth/image_rect_raw
```

Both topics should be publishing at ~30 Hz.

### 3. Launch ORB-SLAM2

In another terminal, launch the SLAM node:

```bash
source /home/jetson/capstone/ros2_ws/setup_workspace.sh
ros2 launch orb_slam2_ros2 slam_launch.py
```

### 4. Run SLAM with Custom Topics (Optional)

If your topics have different names:

```bash
ros2 launch orb_slam2_ros2 slam_launch.py \
  rgb_topic:=/your/rgb/topic \
  depth_topic:=/your/depth/topic
```

### 5. Run with Visualization (Optional)

To enable the ORB-SLAM2 GUI viewer:

```bash
ros2 launch orb_slam2_ros2 slam_launch.py use_viewer:=true
```

**Note:** Visualization requires a display and may impact performance on Jetson.

## Topics

### Subscribed Topics

- `/camera/realsense/color/image_raw` (sensor_msgs/Image): RGB image from camera
- `/camera/realsense/depth/image_rect_raw` (sensor_msgs/Image): Depth image from camera

### Published Topics

- `/orb_slam2/camera_pose` (geometry_msgs/PoseStamped): Current camera pose in map frame
- `/tf` (tf2_msgs/TFMessage): Transform from map to camera_link

## Output Files

When you stop the SLAM node (Ctrl+C), it will automatically save:

- `KeyFrameTrajectory.txt`: Camera trajectory in TUM format (timestamp x y z qx qy qz qw)

This file is saved in the directory where you launched the node.

## Configuration

Camera parameters are configured in `config/realsense_d435.yaml`. You may need to adjust:

- **Camera intrinsics** (fx, fy, cx, cy): Extract from RealSense camera_info topic for your specific camera
- **ORBextractor.nFeatures**: Reduce if performance is slow (current: 800)
- **DepthMapFactor**: Should be 1.0 (depth conversion handled in node)

### Extracting Camera Intrinsics

To get accurate intrinsics for your specific RealSense camera:

```bash
ros2 topic echo /camera/realsense/color/camera_info
```

Update the config file with the K matrix values:
- K[0] = fx
- K[4] = fy
- K[2] = cx
- K[5] = cy

## Troubleshooting

### SLAM Not Initializing

ORB-SLAM2 needs sufficient motion and features to initialize. Try:
1. Move the camera slowly with good lighting
2. Ensure the scene has texture (not blank walls)
3. Check that depth and RGB images are synchronized

### Tracking Lost

If tracking is frequently lost:
1. Move the camera more slowly
2. Improve lighting conditions
3. Reduce ORBextractor.nFeatures in config for better performance
4. Check depth image quality

### Performance Issues on Jetson

To improve performance:
1. Reduce image resolution in jetson_launch.py (e.g., 424x240)
2. Lower ORBextractor.nFeatures (e.g., 500-600)
3. Disable visualization (use_viewer:=false)
4. Close other applications

### Depth Format Issues

If you see errors about depth image format:
- The node expects 16-bit depth (CV_16UC1) in millimeters (default for RealSense)
- Or 32-bit float depth (CV_32FC1) in meters
- Automatic conversion is performed in the node

## Example: Complete Workflow

Terminal 1 - Launch camera:
```bash
# Launch RealSense camera with your configured launch file
ros2 launch /home/jetson/capstone/ros2_ws/launch/jetson_launch.py
```

Terminal 2 - Launch SLAM:
```bash
cd /home/jetson/capstone/ros2_ws
source ./setup_workspace.sh
ros2 launch orb_slam2_ros2 slam_launch.py
```

Terminal 3 - Monitor pose output:
```bash
source /home/jetson/capstone/ros2_ws/setup_workspace.sh
ros2 topic echo /orb_slam2/camera_pose
```

## License

This wrapper is provided under GPLv3 license, compatible with ORB-SLAM2.

## References

- [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)
- [RealSense ROS2 Wrapper](https://github.com/IntelRealSense/realsense-ros)

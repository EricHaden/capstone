# Quick Start Guide

## Prerequisites Check

Before running ORB-SLAM2, verify all dependencies are in place:

```bash
# Check ORB-SLAM2 library
ls -l /home/jetson/capstone/ORB_SLAM2/lib/libORB_SLAM2.so

# Check vocabulary file
ls -l /home/jetson/capstone/ORB_SLAM2/Vocabulary/ORBvoc.txt

# Check third-party libraries
ls -l /home/jetson/capstone/ORB_SLAM2/Thirdparty/DBoW2/lib/libDBoW2.so
ls -l /home/jetson/capstone/ORB_SLAM2/Thirdparty/g2o/lib/libg2o.so
```

All files should exist. If any are missing, you need to build ORB-SLAM2 first.

## Step-by-Step Testing

### Step 1: Start RealSense Camera

Open a terminal and run:

```bash
# Launch RealSense camera with your configured launch file
ros2 launch /home/jetson/capstone/ros2_ws/launch/jetson_launch.py
```

Keep this terminal open.

### Step 2: Verify Camera is Working

Open a **new** terminal and run:

```bash
source /home/jetson/capstone/ros2_ws/setup_workspace.sh
ros2 topic list
```

You should see:
- `/camera/realsense/color/image_raw`
- `/camera/realsense/depth/image_rect_raw`

Check the frame rate:

```bash
ros2 topic hz /camera/realsense/color/image_raw
```

Should show ~30 Hz.

### Step 3: Launch ORB-SLAM2

In the **same** terminal (or a new one with sourced environment):

```bash
ros2 launch orb_slam2_ros2 slam_launch.py
```

You should see output like:
```
[INFO] [rgbd_slam_node]: Subscribing to:
[INFO] [rgbd_slam_node]:   RGB topic: /camera/realsense/color/image_raw
[INFO] [rgbd_slam_node]:   Depth topic: /camera/realsense/depth/image_rect_raw
[INFO] [rgbd_slam_node]: ORB-SLAM2 RGBD node initialized and ready!
```

### Step 4: Initialize SLAM

ORB-SLAM2 needs sufficient motion to initialize. With the camera running:

1. **Slowly** move the camera left and right
2. Ensure good lighting and textured surfaces in view
3. Keep movements smooth and not too fast

Watch the terminal for messages about tracking state. When initialized, you should see frame processing messages.

### Step 5: Monitor Pose Output

In a **new** terminal:

```bash

source /home/jetson/capstone/ros2_ws/setup_workspace.sh
ros2 topic echo /orb_slam2/camera_pose
```

Once tracking is working, you'll see pose updates with position (x, y, z) and orientation (quaternion).

### Step 6: Stop and Save

When done, press **Ctrl+C** in the ORB-SLAM2 terminal.

The trajectory will be saved to `KeyFrameTrajectory.txt` in the directory where you launched the node.

## Viewing Results

The trajectory file is in TUM format:
```
timestamp x y z qx qy qz qw
```

You can visualize it with tools like:
- Python matplotlib
- ROS bag tools
- Online TUM trajectory viewers

## Common Issues

### Issue: "Cannot find vocabulary file"

**Solution:** Make sure the vocabulary file path is correct:
```bash
ls -l /home/jetson/capstone/ORB_SLAM2/Vocabulary/ORBvoc.txt
```

### Issue: "No messages on topics"

**Solution:**
1. Verify camera node is running: `ros2 node list`
2. Check topics: `ros2 topic list`
3. Restart camera node if needed

### Issue: "SLAM not initializing"

**Solution:**
1. Move camera more slowly
2. Point at textured surfaces (not blank walls)
3. Ensure good lighting
4. Try smaller movements initially

### Issue: "Tracking lost frequently"

**Solution:**
1. Move camera more slowly
2. Improve lighting
3. Reduce feature count in config (edit `config/realsense_d435.yaml`, lower `ORBextractor.nFeatures` to 500)

### Issue: "Poor performance / lag"

**Solution:**
1. Close other applications on Jetson
2. Reduce camera resolution in `jetson_launch.py` (e.g., 424x240)
3. Lower `ORBextractor.nFeatures` in config file
4. Ensure Jetson is in max performance mode

## Next Steps

After successful testing:
1. Calibrate your specific RealSense camera for best results
2. Tune ORB parameters for your environment
3. Integrate trajectory output with your robotics application
4. Consider adding loop closure detection optimization

## Need Help?

Check the full README.md for detailed configuration options and troubleshooting.

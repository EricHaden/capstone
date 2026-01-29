cmake_minimum_required(VERSION 3.8)
project(orb_slam2_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++14 required for ORB-SLAM2
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Find Pangolin
find_package(Pangolin REQUIRED)

# ORB-SLAM2 paths
set(ORB_SLAM2_ROOT_DIR "/home/jetson/capstone/ORB_SLAM2")

# Include directories
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${ORB_SLAM2_ROOT_DIR}
  ${ORB_SLAM2_ROOT_DIR}/include
)

# ORB-SLAM2 libraries
set(ORB_SLAM2_LIBRARIES
  ${ORB_SLAM2_ROOT_DIR}/lib/libORB_SLAM2.so
  ${ORB_SLAM2_ROOT_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM2_ROOT_DIR}/Thirdparty/g2o/lib/libg2o.so
)

# Create executable
add_executable(rgbd_slam_node
  src/rgbd_slam_node.cpp
)

# Dependencies for the executable
ament_target_dependencies(rgbd_slam_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_filters
  tf2_ros
  tf2_geometry_msgs
)

# Link libraries
target_link_libraries(rgbd_slam_node
  ${ORB_SLAM2_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
)

# Install targets
install(TARGETS
  rgbd_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

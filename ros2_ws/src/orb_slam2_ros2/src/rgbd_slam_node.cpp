/**
* ROS2 wrapper for ORB-SLAM2 RGB-D SLAM system
* Adapted from ORB-SLAM2 ROS1 wrapper
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <opencv2/core/core.hpp>

#include "System.h"

using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;

class RGBDSlamNode : public rclcpp::Node
{
public:
    RGBDSlamNode(const string &strVocFile, const string &strSettingsFile, bool bUseViewer)
        : Node("rgbd_slam_node")
    {
        // Create SLAM system. It initializes all system threads and gets ready to process frames.
        mpSLAM = new ORB_SLAM2::System(strVocFile, strSettingsFile, ORB_SLAM2::System::RGBD, bUseViewer);

        // Declare parameters
        this->declare_parameter<std::string>("rgb_topic", "/camera/realsense/color/image_raw");
        this->declare_parameter<std::string>("depth_topic", "/camera/realsense/depth/image_rect_raw");
        
        // Get parameters
        std::string rgb_topic = this->get_parameter("rgb_topic").as_string();
        std::string depth_topic = this->get_parameter("depth_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "Subscribing to:");
        RCLCPP_INFO(this->get_logger(), "  RGB topic: %s", rgb_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Depth topic: %s", depth_topic.c_str());

        // Create subscribers with message_filters for synchronization
        rgb_sub_.subscribe(this, rgb_topic);
        depth_sub_.subscribe(this, depth_topic);

        // Create synchronizer with ApproximateTime policy
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_pol;
        sync_ = std::make_shared<message_filters::Synchronizer<sync_pol>>(sync_pol(10), rgb_sub_, depth_sub_);
        sync_->registerCallback(std::bind(&RGBDSlamNode::GrabRGBD, this, _1, _2));

        // Create publisher for camera pose
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/orb_slam2/camera_pose", 10);

        // Create TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "ORB-SLAM2 RGBD node initialized and ready!");
    }

    ~RGBDSlamNode()
    {
        // Stop all threads
        mpSLAM->Shutdown();

        // Save camera trajectory
        mpSLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
        RCLCPP_INFO(this->get_logger(), "Trajectory saved to KeyFrameTrajectory.txt");

        delete mpSLAM;
    }

private:
    void GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr& msgRGB,
                  const sensor_msgs::msg::Image::ConstSharedPtr& msgD)
    {
        // Copy the ROS image message to cv::Mat
        cv_bridge::CvImageConstPtr cv_ptrRGB;
        try
        {
            cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv_bridge::CvImageConstPtr cv_ptrD;
        try
        {
            cv_ptrD = cv_bridge::toCvShare(msgD);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Convert depth image to float if needed
        cv::Mat depth_float;
        if (cv_ptrD->image.type() == CV_16UC1)
        {
            // RealSense depth is in millimeters (16-bit), convert to meters (32-bit float)
            cv_ptrD->image.convertTo(depth_float, CV_32F, 0.001);
        }
        else if (cv_ptrD->image.type() == CV_32FC1)
        {
            depth_float = cv_ptrD->image;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unsupported depth image type: %d", cv_ptrD->image.type());
            return;
        }

        // Track RGBD frame
        double timestamp = msgRGB->header.stamp.sec + msgRGB->header.stamp.nanosec * 1e-9;
        cv::Mat Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image, depth_float, timestamp);

        // Publish camera pose if tracking successful
        if (!Tcw.empty())
        {
            PublishPose(Tcw, msgRGB->header);
        }
    }

    void PublishPose(const cv::Mat &Tcw, const std_msgs::msg::Header &header)
    {
        // Tcw is camera-to-world transformation (4x4 matrix)
        // Extract rotation and translation
        cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

        // Create pose message
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = header;
        pose_msg.header.frame_id = "map";

        // Set position
        pose_msg.pose.position.x = twc.at<float>(0);
        pose_msg.pose.position.y = twc.at<float>(1);
        pose_msg.pose.position.z = twc.at<float>(2);

        // Convert rotation matrix to quaternion
        Eigen::Matrix3f rotation_matrix;
        rotation_matrix << Rwc.at<float>(0, 0), Rwc.at<float>(0, 1), Rwc.at<float>(0, 2),
                          Rwc.at<float>(1, 0), Rwc.at<float>(1, 1), Rwc.at<float>(1, 2),
                          Rwc.at<float>(2, 0), Rwc.at<float>(2, 1), Rwc.at<float>(2, 2);
        
        Eigen::Quaternionf q(rotation_matrix);
        
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        // Publish pose
        pose_pub_->publish(pose_msg);

        // Publish TF transform
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header = pose_msg.header;
        transform_stamped.child_frame_id = "camera_link";
        
        transform_stamped.transform.translation.x = pose_msg.pose.position.x;
        transform_stamped.transform.translation.y = pose_msg.pose.position.y;
        transform_stamped.transform.translation.z = pose_msg.pose.position.z;
        
        transform_stamped.transform.rotation = pose_msg.pose.orientation;
        
        tf_broadcaster_->sendTransform(transform_stamped);
    }

    ORB_SLAM2::System* mpSLAM;
    
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Need at least vocab + settings (2 args). Launch may add --ros-args/--params-file, so argc can be > 4.
    if(argc < 3)
    {
        cerr << endl << "Usage: ros2 run orb_slam2_ros2 rgbd_slam_node path_to_vocabulary path_to_settings [use_viewer]" << endl;
        cerr << "Example: ros2 run orb_slam2_ros2 rgbd_slam_node /path/to/ORBvoc.txt /path/to/config.yaml false" << endl;
        return 1;
    }

    bool bUseViewer = false;
    if(argc >= 4 && argv[3][0] != '-')  // 4th arg is use_viewer only if not a ROS 2 option (e.g. --ros-args)
    {
        string strViewer(argv[3]);
        bUseViewer = (strViewer == "true" || strViewer == "1");
    }

    // Create node
    auto node = std::make_shared<RGBDSlamNode>(argv[1], argv[2], bUseViewer);

    // Spin
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

#ifndef APRILTAG_LOCALIZATION_NODE_HPP_
#define APRILTAG_LOCALIZATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "apriltag_ros/tag_functions.hpp"

class AprilTagNode : public rclcpp::Node {
public:
    AprilTagNode(const rclcpp::NodeOptions& options);

    ~AprilTagNode() override;

private:
    const OnSetParametersCallbackHandle::SharedPtr cb_parameter;

    apriltag_family_t* tf;
    apriltag_detector_t* const td;

    // parameter
    std::mutex mutex;
    double tag_edge_size;
    std::atomic<int> max_hamming;
    std::atomic<bool> profile;

    std::function<void(apriltag_family_t*)> tf_destructor;

    const image_transport::CameraSubscriber sub_cam;
    const rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::msg::TransformStamped tag_pose;

    void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci);

    rcl_interfaces::msg::SetParametersResult onParameter(const std::vector<rclcpp::Parameter>& parameters);
};

#endif
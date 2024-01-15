#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <apriltag.h>

#include <Eigen/Dense>

#include "apriltag_ros/apriltag_node.hpp"
#include "apriltag/common/homography.h"

#define IF(N, V) \
    if(assign_check(parameter, N, V)) continue;

template<typename T>
void assign(const rclcpp::Parameter& parameter, T& var)
{
    var = parameter.get_value<T>();
}

template<typename T>
void assign(const rclcpp::Parameter& parameter, std::atomic<T>& var)
{
    var = parameter.get_value<T>();
}

template<typename T>
bool assign_check(const rclcpp::Parameter& parameter, const std::string& name, T& var)
{
    if(parameter.get_name() == name) {
        assign(parameter, var);
        return true;
    }
    return false;
}

typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Mat3;

rcl_interfaces::msg::ParameterDescriptor
descr(const std::string& description, const bool& read_only = false)
{
    rcl_interfaces::msg::ParameterDescriptor descr;

    descr.description = description;
    descr.read_only = read_only;

    return descr;
}

void getPose(const matd_t& H,
             const Mat3& Pinv,
             geometry_msgs::msg::Transform& t,
             const double size)
{
    // compute extrinsic camera parameter
    // https://dsp.stackexchange.com/a/2737/31703
    // H = K * T  =>  T = K^(-1) * H
    const Mat3 T = Pinv * Eigen::Map<const Mat3>(H.data);
    Mat3 R;
    R.col(0) = T.col(0).normalized();
    R.col(1) = T.col(1).normalized();
    R.col(2) = R.col(0).cross(R.col(1));

    // rotate by half rotation about x-axis to have z-axis
    // point upwards orthogonal to the tag plane
    R.col(1) *= -1;
    R.col(2) *= -1;

    // the corner coordinates of the tag in the canonical frame are (+/-1, +/-1)
    // hence the scale is half of the edge size
    Eigen::Vector3d tt = T.rightCols<1>() / ((T.col(0).norm() + T.col(0).norm()) / 2.0) * (size / 2.0);
    // tt *= -1; // inverse the translation

    Eigen::Quaterniond q(R);
    // q = q.conjugate(); // inverse the rotation

    t.translation.x = tt.x();
    t.translation.y = tt.y();
    t.translation.z = tt.z();
    t.rotation.w = q.w();
    t.rotation.x = q.x();
    t.rotation.y = q.y();
    t.rotation.z = q.z();
}

AprilTagNode::AprilTagNode(const rclcpp::NodeOptions& options)
  : Node("apriltag", options),
    // parameter
    cb_parameter(add_on_set_parameters_callback(std::bind(&AprilTagNode::onParameter, this, std::placeholders::_1))),
    td(apriltag_detector_create()),
    // topics
    sub_cam(image_transport::create_camera_subscription(this, "image_rect", std::bind(&AprilTagNode::onCamera, this, std::placeholders::_1, std::placeholders::_2), declare_parameter("image_transport", "raw", descr({}, true)), rmw_qos_profile_sensor_data)),
    pub_detections(create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("apriltag/detections", rclcpp::QoS(1))),
    tf_broadcaster(this)
{
    // read-only parameters
    const std::string tag_family = declare_parameter("family", "36h11", descr("tag family", true));
    tag_edge_size = declare_parameter("size", 1.0, descr("default tag size", true));

    // detector parameters in "detector" namespace
    declare_parameter("detector.threads", td->nthreads, descr("number of threads"));
    declare_parameter("detector.decimate", td->quad_decimate, descr("decimate resolution for quad detection"));
    declare_parameter("detector.blur", td->quad_sigma, descr("sigma of Gaussian blur for quad detection"));
    declare_parameter("detector.refine", td->refine_edges, descr("snap to strong gradients"));
    declare_parameter("detector.sharpening", td->decode_sharpening, descr("sharpening of decoded images"));
    declare_parameter("detector.debug", td->debug, descr("write additional debugging images to working directory"));

    declare_parameter("max_hamming", 0, descr("reject detections with more corrected bits than allowed"));
    declare_parameter("profile", false, descr("print profiling information to stdout"));

    if(tag_fun.count(tag_family)) {
        tf = tag_fun.at(tag_family).first();
        tf_destructor = tag_fun.at(tag_family).second;
        apriltag_detector_add_family(td, tf);
    }
    else {
        throw std::runtime_error("Unsupported tag family: " + tag_family);
    }

    tag_pose.header.frame_id = "camera_optical_link";
    tag_pose.child_frame_id = "apriltag";
}

AprilTagNode::~AprilTagNode()
{
    apriltag_detector_destroy(td);
    tf_destructor(tf);
}

void AprilTagNode::onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
                            const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci)
{
    // precompute inverse projection matrix
    const Mat3 Pinv = Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(msg_ci->p.data()).leftCols<3>().inverse();

    // convert to 8bit monochrome image
    const cv::Mat img_uint8 = cv_bridge::toCvShare(msg_img, "mono8")->image;

    image_u8_t im{img_uint8.cols, img_uint8.rows, img_uint8.cols, img_uint8.data};

    // detect tags
    mutex.lock();
    zarray_t* detections = apriltag_detector_detect(td, &im);
    mutex.unlock();

    if(profile)
        timeprofile_display(td->tp);

    apriltag_msgs::msg::AprilTagDetectionArray msg_detections;
    msg_detections.header = msg_img->header;
    
    if (zarray_size(detections) == 0)
    {
        return;
    }

    apriltag_detection_t* det;
    zarray_get(detections, 0, &det);

    RCLCPP_DEBUG(get_logger(),
                 "detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
                 0, det->family->nbits, det->family->h, det->id,
                 det->hamming, det->decision_margin);

    // reject detections with more corrected bits than allowed
    if(det->hamming > max_hamming) { return; }

    // detection
    apriltag_msgs::msg::AprilTagDetection msg_detection;
    msg_detection.family = std::string(det->family->name);
    msg_detection.id = det->id;
    msg_detection.hamming = det->hamming;
    msg_detection.decision_margin = det->decision_margin;
    msg_detection.centre.x = det->c[0];
    msg_detection.centre.y = det->c[1];
    std::memcpy(msg_detection.corners.data(), det->p, sizeof(double) * 8);
    std::memcpy(msg_detection.homography.data(), det->H->data, sizeof(double) * 9);
    msg_detections.detections.push_back(msg_detection);
    
    // 3D orientation and position
    tag_pose.header.stamp = this->now();
    getPose(*(det->H), Pinv, tag_pose.transform, tag_edge_size);

    pub_detections->publish(msg_detections);
    tf_broadcaster.sendTransform(tag_pose);

    apriltag_detections_destroy(detections);
}

rcl_interfaces::msg::SetParametersResult
AprilTagNode::onParameter(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;

    mutex.lock();

    for(const rclcpp::Parameter& parameter : parameters) {
        RCLCPP_DEBUG_STREAM(get_logger(), "setting: " << parameter);

        IF("detector.threads", td->nthreads)
        IF("detector.decimate", td->quad_decimate)
        IF("detector.blur", td->quad_sigma)
        IF("detector.refine", td->refine_edges)
        IF("detector.sharpening", td->decode_sharpening)
        IF("detector.debug", td->debug)
        IF("max_hamming", max_hamming)
        IF("profile", profile)
    }

    mutex.unlock();

    result.successful = true;

    return result;
}

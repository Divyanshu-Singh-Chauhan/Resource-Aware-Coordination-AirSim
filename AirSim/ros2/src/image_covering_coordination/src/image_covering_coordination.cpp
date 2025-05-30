#include <memory>
#include <string>
#include <unordered_map>
#include <chrono>
#include <vector>

// ROS 2 headers
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "airsim_ros_pkgs/msg/neighbors_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

// Airsim library
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "common/AirSimSettings.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "sensors/lidar/LidarSimpleParams.hpp"
#include "sensors/imu/ImuBase.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "yaml-cpp/yaml.h"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Using declarations
using namespace std::chrono_literals;
using namespace msr::airlib;
typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;
typedef common_utils::FileSystem FileSystem;

class ImageCoveringCoordination
{

private:
    void connectivityMeshCallback(const airsim_ros_pkgs::msg::NeighborsArray::SharedPtr msg) // Save connectivity data
    {
        // TODO: implement
    }

    void collectRotatedCameraImages(const airsim_ros_pkgs::msg::NeighborsArray::SharedPtr msg) 
    {
        // TODO: implement
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) // Get pose
    {
        // TODO: implement
    }

    // ROS 2 publishers & subscribers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Subscription<airsim_ros_pkgs::msg::NeighborsArray>::SharedPtr connectivity_mesh_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    // Class variables
    std::string drone_name_self_;
    std::string camera_name_;
    double communication_range_;

public:
    ImageCoveringCoordination()
    : Node("image_covering_coordination_node")
    {
        // Get ROS params
        declare_parameter<std::string>("drone_name", "");
        declare_parameter<std::string>("camera_name", "");
        declare_parameter<double>("communication_range", 0.0);
        get_parameter("drone_name", drone_name_self_);
        get_parameter("camera_name", camera_name_);
        get_parameter("communication_range", communication_range_);

        // ROS publishers
        image_pub_ = create_publisher<sensor_msgs::msg::Image>("/covered_image", 10);

        // ROS subscribers
        connectivity_mesh_sub_ = create_subscription<airsim_ros_pkgs::msg::NeighborsArray>(
            "/neighbors_data", 10,
            std::bind(&ImageCoveringCoordination::connectivityMeshCallback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&ImageCoveringCoordination::odomCallback, this, std::placeholders::_1));

        // TF2 listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    // Callback functions
    void connectivityMeshCallback(const airsim_ros_pkgs::NeighborsArray& msg); // Save connectivity data

    void collectRotatedCameraImages(const airsim_ros_pkgs::NeighborsArray& msg); 

    void odomCallback(const nav_msgs::Odometry& msg); // Get pose

};



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageCoveringCoordination>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#ifndef IMAGE_COVERING_COORD_RAG_HPP_
#define IMAGE_COVERING_COORD_RAG_HPP_

// =================================================================
// ROS 2 HEADERS
// =================================================================
// The main rclcpp header for creating nodes, publishers, subscribers, etc.
#include "rclcpp/rclcpp.hpp"

// Message and Service Headers - note the '/msg/' and '/srv/' subdirectories
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_srvs/srv/empty.hpp"

// TF2 Headers for ROS2
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // Note .hpp

// Image Transport for ROS2
#include "image_transport/image_transport.hpp"

// Custom message headers - they must be generated in your ROS2 workspace
#include "airsim_ros_pkgs/msg/neighbors.hpp"
#include "airsim_ros_pkgs/msg/neighbors_array.hpp"
#include "multi_target_tracking/msg/marginal_gain_rag.hpp"
#include "image_covering_coordination/msg/image_covering.hpp"

// CV Bridge for ROS2
#include "cv_bridge/cv_bridge.hpp"

// =================================================================
// EXTERNAL & C++ STANDARD HEADERS (Largely Unchanged)
// =================================================================
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
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <math.h>
#include <unordered_map>
#include <map>
#include <memory>
#include <random>
#include <chrono>
#include <thread>
#include <algorithm>
#include <limits>
#include <vector>

// Airsim specific namespaces and typedefs (Unchanged)
using namespace msr::airlib;
typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;
typedef common_utils::FileSystem FileSystem;


// In ROS2, your node class inherits from rclcpp::Node
class ImageCoveringCoordRAG : public rclcpp::Node
{
public:
    // The constructor no longer takes a NodeHandle.
    // It calls the parent rclcpp::Node constructor with the node name.
    explicit ImageCoveringCoordRAG(const rclcpp::NodeOptions & options);
    ~ImageCoveringCoordRAG(); // Destructor remains the same

private:
    // Structs using ROS messages must be updated to use ROS2 message types
    struct ImageInfoStruct {
        geometry_msgs::msg::Pose pose;
        int camera_orientation;
        sensor_msgs::msg::Image image;
        double best_marginal_gain_val;
    };

    // --- ROS 2 Member Variables ---
    // Publishers and Subscribers now use rclcpp types and are stored as Shared Pointers.
    rclcpp::Publisher<multi_target_tracking::msg::MarginalGainRAG>::SharedPtr mg_rag_msg_pub_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr marginal_gain_pub_clock_sub_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr check_drones_selection_sub_;

    // The TF2 listener is now composed of a buffer and a listener.
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // --- Callback function signatures must be updated ---
    void clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);
    void marginalGainPublisherclockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);
    void updateAllRobotsSelectionStatusCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);

    // --- Function signatures using ROS types must be updated ---
    void objectiveFunctionRAG(double& obj_func_val, std::vector<int>& in_neigh_ids_select_actions, bool& include_self_robot, int& self_robot_action_id);
    void marginalGainFunctionRAG(double& best_marginal_gain, int& optimal_action_id, geometry_msgs::msg::Pose latest_pose, std::map<int, ImageInfoStruct>& prev_selected_action_inneigh_ids_img);
    // ... other non-ROS-API function declarations remain the same

    // This function signature and return type is updated for ROS2
    std::unique_ptr<sensor_msgs::msg::Image> get_img_msg_from_response(const ImageResponse& img_response);
    
    // Functions returning ros::Time now return rclcpp::Time
    rclcpp::Time airsim_timestamp_to_ros(const msr::airlib::TTimePoint& stamp) const;
    rclcpp::Time chrono_timestamp_to_ros(const std::chrono::system_clock::time_point& stamp) const;
    
    // ... function declarations ...
    double computeWeightedScoreSingleImage(const sensor_msgs::msg::Image& image,  bool convertBGR2RGB);
    std::vector<cv::Point2f> getRectangleFromPose(const geometry_msgs::msg::Pose& pose, int orientation);
    sensor_msgs::msg::Image findNonOverlappingRegions(const sensor_msgs::msg::Image& curr_image, const geometry_msgs::msg::Pose& curr_pose,
                                                  int curr_orientation, std::map<int, std::pair<geometry_msgs::msg::Pose, 
                                                  int>>& in_neighbor_poses_and_orientations, bool convertBGR2RGB);
    std::vector<cv::Point2f> getRectangleFromPoseInNeighbor(const geometry_msgs::msg::Pose& pose_self_robot, const geometry_msgs::msg::Pose& pose_inn_robot,
                                                            int self_rob_img_orient, int inn_img_orient);
    geometry_msgs::msg::Pose convertToGeometryPose(const msr::airlib::MultirotorState& pose1);
    double PoseEuclideanDistance(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2, int cam_orient_1, int cam_orient_2);
    // ... other non-ROS-API function declarations remain the same ...


    // --- Class Member Variables (Non-ROS or data) ---
    // ros::NodeHandle node_handle_ is removed.
    // ros::AsyncSpinner is removed. Executor logic is handled in main().
    
    // This buffer now holds ROS2 messages
    airsim_ros_pkgs::msg::NeighborsArray neighbors_data_;

    // This vector now holds ROS2 image messages
    std::vector<sensor_msgs::msg::Image> camera_rotated_imgs_vec_;
    sensor_msgs::msg::Image best_self_image_;
    
    // All other non-ROS variables remain unchanged
    int curr_time_step_samp_;
    int num_robots_;
    // ... all other variables like Eigen matrices, strings, doubles, etc.
};

#endif // IMAGE_COVERING_COORD_RAG_HPP_
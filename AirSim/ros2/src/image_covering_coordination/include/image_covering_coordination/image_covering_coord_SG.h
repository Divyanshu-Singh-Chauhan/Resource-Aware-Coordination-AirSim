/*
NOTE: This header contains the algorithms RBG and BSG only; for RAG-H view multi_target_tracking_coord_RAG.cpp
*/
#ifndef IMAGE_COVERING_COORD_SG_H
#define IMAGE_COVERING_COORD_SG_H

// C++ headers
#include <memory>
#include <map>
#include <unordered_map>
#include <vector>
#include <string>
#include <random>
#include <chrono>
#include <thread>
#include <algorithm>

// ROS 2 headers
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

// Custom message headers
#include "airsim_ros_pkgs/msg/neighbors.hpp"
#include "airsim_ros_pkgs/msg/neighbors_array.hpp"
// #include <multi_target_tracking/PursuerEvaderData.h>
// #include <multi_target_tracking/PursuerEvaderDataArray.h>
#include "multi_target_tracking/msg/marginal_gain_rag.hpp"
#include "image_covering_coordination/msg/image_covering.hpp"

// Airsim library
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

// #include "airsim_settings_parser.h"
#include "common/AirSimSettings.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "sensors/lidar/LidarSimpleParams.hpp"
#include "sensors/imu/ImuBase.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "yaml-cpp/yaml.h"
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>

// Airsim specific namespaces and typedefs
using namespace msr::airlib;
typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;
typedef common_utils::FileSystem FileSystem;

class ImageCoveringCoordSG : public rclcpp::Node
{
public:
    ImageCoveringCoordSG();
    ~ImageCoveringCoordSG(); // destructor

private:

    // Structure to represent a graph
    struct Graph {
        std::unordered_map<int, std::vector<int>> adjList;

        void addEdge(int u, int v) {
            adjList[u].push_back(v);
            adjList[v].push_back(u); // Assuming undirected graph
        }
    };

    struct ImageInfoStruct {
        geometry_msgs::msg::Pose pose;
        int camera_orientation;
        sensor_msgs::msg::Image image;
        double best_marginal_gain_val;
    };

    // Node parameters and state
    int curr_time_step_samp_;
    int num_robots_;
    int num_evaders_;
    int robot_idx_self_;
    double total_data_msg_size_; // in KB
    double communication_data_rate_; // in KB/s
    bool takeoff_wait_flag_;
    double max_sensing_range_;  // max sensing range among all robots
    double altitude_flight_level_;
    double flight_level_;
    double drone_lin_vel_factor_;
    double yaw_rate_;
    double yaw_duration_;
    int communication_round_;

    std::chrono::system_clock::time_point start_;

    // ROS 2 subscriptions & publishers
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr marginal_gain_pub_clock_sub_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr check_drones_selection_sub_;
    rclcpp::Publisher<image_covering_coordination::msg::ImageCovering>::SharedPtr mg_rag_msg_pub_;

    // TF2 listener
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    // Class variables
    std::string drone_name_self_;
    std::string camera_name_;
    double communication_range_;
    double objective_function_value_;
    Eigen::MatrixXd reward_;
    bool PEdata_new_msg_received_flag_;
    double fov_x_;
    double fov_y_;
    double best_marginal_gain_self_;
    std::map<int, bool> all_robots_selected_map_;
    bool terminate_decision_making_;

    // Callback functions
    void clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);
    void marginalGainPublisherClockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);
    void updateAllRobotsSelectionStatusCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);

    void marginalGainFunctionSG_LP(
        double& best_marginal_gain,
        int& optimal_action_id,
        const geometry_msgs::msg::Pose& latest_pose,
        bool& got_prec_data_flag);
    void marginalGainFunctionSG_DFS(
        double& best_marginal_gain,
        int& optimal_action_id,
        const geometry_msgs::msg::Pose& latest_pose,
        bool& got_prec_data_flag);

    void collectRotatedCameraImages();

    void simulatedCommTimeDelaySG_LP();  // for SG
    void simulatedCommTimeDelaySG_LP(double& processed_time);
    void simulatedCommTimeDelaySG_DFSv1();
    void simulatedCommTimeDelaySG_DFSv2();

    std::pair<double, double> computeRangeBearing(const std::vector<double>& pose1, const std::vector<double>& pose2);

    void getSelectedActionsInNeighborIDs(std::map<int, double>& selected_action_inneigh_ids_mg);
    void getSelectedActionsPrecNeighborIDImages(
        std::map<int, ImageInfoStruct>& selected_action_precneigh_ids_mg,
        bool& got_prec_data_flag);

    void getNonSelectedActionsInNeighborIDs(std::map<int, double>& non_selected_action_inneigh_ids_mg);
    void isMyMarginalGainBest(bool& is_my_mg_best, std::map<int, double>& non_selected_action_inneigh_ids_mg);
    void takeUnionOfMapsDataStruct(
        std::map<int, double>& selected_action_inneigh_ids_mg,
        std::map<int, double>& new_selected_action_inneigh_ids_mg,
        std::map<int, double>& combined_map);
    void takeUnionOfMapsDataStruct(
        std::map<int, double>& selected_action_inneigh_ids_mg,
        std::map<int, double>& new_selected_action_precneigh_ids_mg);

    sensor_msgs::msg::Image get_img_msg_from_response(const ImageResponse& img_response);
    rclcpp::Time airsim_timestamp_to_ros(const msr::airlib::TTimePoint& stamp) const;
    rclcpp::Time chrono_timestamp_to_ros(const std::chrono::system_clock::time_point& stamp) const;

    bool areAllValuesTrue();

    msr::airlib::Pose giveGoalPoseFormat(
        msr::airlib::MultirotorState& curr_pose,
        float disp_x, float disp_y, float disp_z, float new_yaw);
    Eigen::Quaternionf quaternionFromEuler(float roll, float pitch, float yaw);
    Eigen::Vector3f eulerFromQuaternion(const Eigen::Quaternionf& q);
    msr::airlib::Pose giveCameraGoalPoseFormat(
        msr::airlib::MultirotorState& curr_pose,
        float disp_x, float disp_y, float disp_z,
        float new_pitch, float curr_roll, float curr_yaw);

    std::vector<double> computeWeightedScores(const std::vector<sensor_msgs::msg::Image>& images);
    double computeWeightedScoreSingleImage(const sensor_msgs::msg::Image& image, bool invertBGR2RGB);
    double computeWeightedScoreSingleImage(const cv::Mat& img);

    std::vector<cv::Point> toCvPointVector(const std::vector<cv::Point2f>& points);
    sensor_msgs::msg::Image findNonOverlappingRegions(
        const sensor_msgs::msg::Image& curr_image,
        const geometry_msgs::msg::Pose& curr_pose,
        int curr_orientation,
        std::map<int, std::pair<geometry_msgs::msg::Pose, int>>& in_neighbor_poses_and_orientations,
        bool convertBGR2RGB);

    geometry_msgs::msg::Pose convertToGeometryPose(const msr::airlib::MultirotorState& pose1);
    double PoseEuclideanDistance(
        const geometry_msgs::msg::Pose& pose1,
        const geometry_msgs::msg::Pose& pose2,
        int cam_orient_1, int cam_orient_2);
    void saveImage(std::vector<ImageResponse>& response);
    void viewImage(std::vector<ImageResponse>& response);
    void viewImageCvMat(cv::Mat& image);
    void viewImageCvMat(cv::Mat& image, bool switchBGR_RGB_flag);
    void viewImageCvMat(cv::Mat& image, bool switchBGR_RGB_flag, const std::string filename);
    void viewImageFromSensorMsg(const sensor_msgs::msg::Image& msg);

    void rotateImagesInVector();
    double calculateAngleFromOrientation(int cam_orient);

    // graph-related functions
    int findNumEdgesBetweenIDs(Graph& graph, int currentID);
    void graphDecisionIDHops(
        const airsim_ros_pkgs::msg::NeighborsArray& neighbors_array,
        Graph& graph);
    void getSelectedActionsPrecNeighborIDImages_DFS(
        std::map<int, ImageInfoStruct>& selected_action_precneigh_ids_mg,
        bool& got_prec_data_flag);
    std::unordered_map<int, int> mapDecisionIDToDroneID(
        const airsim_ros_pkgs::msg::NeighborsArray& neighbors_array);
    int graphDFSdelayFactor(Graph& graph);
    void printMap(const std::map<int, double>& map);

    // CAMERA PARAMETERS******
    double horizon_;
    double n_time_steps_;
    int selected_action_ind_; // a_i,t_RAG-H
    bool finished_action_selection_;
    bool waiting_for_robots_to_finish_selection_;
    std::vector<int> in_neigh_ids_select_actions_; // I_i,t ; agents in N^-_i,t that have already selected an action to execute
    Eigen::MatrixXd actions_;
    Eigen::MatrixXd actions_unit_vec_;
    Eigen::Matrix<double, 3, 1> last_selected_displacement_;
    float client_timeout_sec_;
    const msr::airlib::YawMode client_default_yaw_mode_;
    float client_lookahead_;
    float client_adaptive_lookahead_;
    float client_margin_;
    double yaw_rate_timestep_;
    double drone_linear_velocity_;
    msr::airlib::DrivetrainType client_drivetrain_;
    msr::airlib::MultirotorRpcLibClient client_;
    airsim_ros_pkgs::msg::NeighborsArray neighbors_data_;
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> async_spinner_;
    std::vector<sensor_msgs::msg::Image> camera_rotated_imgs_vec_;
    double camera_pitch_theta_;
    double camera_downward_pitch_angle_;
    std::vector<ImageRequest> current_image_requests_vec_;
    msr::airlib::ImageCaptureBase::ImageType camera_image_type_;
    sensor_msgs::msg::Image best_self_image_;
    int best_image_cam_orientation_;
    std::string img_save_path_;
    double y_half_side_len_m_;
    double x_half_side_len_m_;
    double y_dir_px_m_scale_;
    double x_dir_px_m_scale_;
    double delta_C_; // displacement in center due to viewing angle
    double diagonal_frame_dist_;
    double diagonal_px_scale_;
    int pasted_rect_height_pix_;
    int pasted_rect_width_pix_;
    bool take_new_pics_;
    double road_pixel_weight_;
    std::string algorithm_to_run_;
    std::string graph_topology_SG_;
    double control_action_disp_scale_;
    int random_seed_;
    int is_server_experiment_;
};

#endif // IMAGE_COVERING_COORD_SG_H

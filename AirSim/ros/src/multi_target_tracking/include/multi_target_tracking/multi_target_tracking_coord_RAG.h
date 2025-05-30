/*
NOTE: This header contains the algorithms RBG and BSG only; for RAG-H view multi_target_tracking_coord_RAG.cpp
 */
#ifndef MULTI_TARGET_TRACKING_COORD_RAG_H
#define MULTI_TARGET_TRACKING_COORD_RAG_H

// C++ headers
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

// Custom message headers
#include <airsim_ros_pkgs/Neighbors.h>
#include <airsim_ros_pkgs/NeighborsArray.h>
#include <multi_target_tracking/PursuerEvaderData.h>
#include <multi_target_tracking/PursuerEvaderDataArray.h>
#include <multi_target_tracking/MarginalGainRAG.h>

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
#include "ros/ros.h"
#include "sensors/imu/ImuBase.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "yaml-cpp/yaml.h"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <math.h>
#include <math_common.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <rosgraph_msgs/Clock.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <unordered_map>
#include<map>
#include <memory>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <rosgraph_msgs/Clock.h>
#include <random>
#include <chrono>
#include <thread>
#include <algorithm>

#include <ros/spinner.h>

// Airsim specific namespaces and typedefs
using namespace msr::airlib;
typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;
typedef common_utils::FileSystem FileSystem;

class MultiTargetTrackingCoordRAG
{

public:
    MultiTargetTrackingCoordRAG(ros::NodeHandle& nh);
    ~MultiTargetTrackingCoordRAG(); // destructor

private:
    // ROS node handle
    ros::NodeHandle node_handle_;
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

    std::chrono::time_point<std::chrono::_V2::system_clock> start_;

    // ROS subscribers
    ros::Subscriber pursuer_evader_data_sub_;
    ros::Subscriber clock_sub_;
    ros::Subscriber marginal_gain_pub_clock_sub_;
    ros::Subscriber check_pursuers_selection_sub_;

    // ROS publishers
    ros::Publisher depth_image_pub_;
    ros::Publisher mg_rag_msg_pub_;

    // TF listener
    tf::TransformListener* tf_listener_;

    // Class variables
    std::string drone_name_self_;
    std::string camera_name_;
    double communication_range_;
    double objective_function_value_;
    // double previous_robots_obj_func_val_; // normalized reward not used in RAG-H
    Eigen::MatrixXd reward_;
    bool PEdata_new_msg_received_flag_;
    double fov_x_;
    double fov_y_;
    double best_marginal_gain_self_;
    std::map<int, bool> all_robots_selected_map_;

    // Callback functions
    // void collectRotatedCameraImages(const multi_target_tracking::PursuerEvaderDataArray& msg);
    void clockCallback(const rosgraph_msgs::Clock& msg);
    void marginalGainPublisherclockCallback(const rosgraph_msgs::Clock& msg);
    void updateAllRobotsSelectionStatusCallback(const rosgraph_msgs::Clock& msg);

    void objectiveFunctionRAG(double& obj_func_val, std::vector<int>& in_neigh_ids_select_actions, bool& include_self_robot, int& self_robot_action_id);
    void marginalGainFunctionRAG(double& best_marginal_gain, int& optimal_action_id, std::vector<int>& inneighs_selected_action);
    void bestInitActionObjFunctionRAG(double& best_marginal_gain, int& optimal_action_id, std::vector<int>& inneighs_selected_action);

    void simulatedCommTimeDelayRAG();  // for RAG-H
    void simulatedCommTimeDelayRAG(double& processed_time);

    bool isEvaderInFov(const std::vector<double>& pursuer_pose, 
                      const std::vector<double>& evader_pose,
                      double max_dist);

    std::pair<double, double> computeRangeBearing(const std::vector<double>& pose1, const std::vector<double>& pose2);

    void getSelectedActionsInNeighborIDs(std::map<int, double>& selected_action_inneigh_ids_mg);
    void getNonSelectedActionsInNeighborIDs(std::map<int, double>& non_selected_action_inneigh_ids_mg);
    void isMyMarginalGainBest(bool& is_my_mg_best, std::map<int, double>& non_selected_action_inneigh_ids_mg);
    void takeUnionOfMaps(std::map<int, double>& selected_action_inneigh_ids_mg, std::map<int, double>& new_selected_action_inneigh_ids_mg, std::map<int, double>& combined_map);
    void takeUnionOfMaps(std::map<int, double>& selected_action_inneigh_ids_mg, std::map<int, double>& new_selected_action_inneigh_ids_mg);
    bool areAllValuesTrue();

    double horizon_;
    double n_time_steps_;

    // actions set
    int selected_action_ind_; // a_i,t_RAG-H
    bool finished_action_selection_;
    bool waiting_for_robots_to_finish_selection_;
    std::vector<int> in_neigh_ids_select_actions_; // I_i,t ; agents in N^-_i,t (in-neighbors) that have already selected an action to execute
    Eigen::MatrixXd actions_;
    Eigen::Matrix<double, 3, 1> last_selected_displacement_;

    // Airsim client and constants
    float client_timeout_sec_;
    const msr::airlib::YawMode client_default_yaw_mode_;
    float client_lookahead_;
    float client_adaptive_lookahead_ ;
    float client_margin_;
    double yaw_rate_timestep_;
    double drone_linear_velocity_;
    msr::airlib::DrivetrainType client_drivetrain_;
    msr::airlib::MultirotorRpcLibClient client_;

    // to store pursuer evader data in a buffer
    multi_target_tracking::PursuerEvaderDataArray pursuer_evader_data_;


    std::unique_ptr<ros::AsyncSpinner> async_spinner_;

};

#endif // MULTI_TARGET_TRACKING_COORD_RAG_H
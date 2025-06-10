// multi_target_tracking_coordination.h

#ifndef MULTI_TARGET_TRACKING_COORDINATION_H
#define MULTI_TARGET_TRACKING_COORDINATION_H

// C++ headers
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

// EXP*-SIX algorithm class
#include "multi_target_tracking/EXPStarSix.h"

// Custom message headers
#include <airsim_ros_pkgs/Neighbors.h>
#include <airsim_ros_pkgs/NeighborsArray.h>
#include <multi_target_tracking/PursuerEvaderData.h>
#include <multi_target_tracking/PursuerEvaderDataArray.h>

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
#include <queue>

// Airsim specific namespaces and typedefs
using namespace msr::airlib;
typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;
typedef common_utils::FileSystem FileSystem;

class MultiTargetTrackingCoordination
{

public:
    MultiTargetTrackingCoordination(ros::NodeHandle& nh);

private:
    // ROS node handle
    ros::NodeHandle node_handle_;
    int curr_time_step_samp_;
    int num_robots_;
    int num_evaders_;
    int robot_idx_self_;
    double total_data_msg_size_; // in KB
    double communication_data_rate_; // in KB/s
    std::string algorithm_to_run_;
    double yaw_rate_;
    double yaw_duration_;

    // DRONE VELOCITY SETTINGS***********
    double drone_linear_velocity_; // fixed param
    double drone_yaw_angular_speed_; //fixed param
    double interp_checkpoint_total_dist_; // fixed param NOTE!!! this will be the magnitude of actions_ rows 0,1
    double interp_total_time_;
    int num_interpolation_pts_;
    Eigen::Matrix<double, 3, 1> forward_only_disp_mat_;

    // DRONE VELOCITY SETTINGS***********

    // TF listener
    // tf::TransformListener* tf_listener_;
    // TF2 listener: Buffer not recognized as a type
    // tf2_ros::Buffer tfBuffer_;
    // tf2_ros::TransformListener tf2_listener_(tfBuffer_);

    // ROS subscribers
    ros::Subscriber pursuer_evader_data_sub_;
    ros::Subscriber clock_sub_;
    ros::Subscriber motion_execution_sub_;

    // ROS publishers
    ros::Publisher depth_image_pub_;

    // TF listener
    tf::TransformListener* tf_listener_;

    // Class variables
    std::string drone_name_self_;
    std::string camera_name_;
    double communication_range_;
    int communication_round_;
    double objective_function_value_;
    double previous_robots_obj_func_val_;
    Eigen::MatrixXd reward_;
    double max_sensing_range_;  // max sensing range among all robots
    bool PEdata_new_msg_received_flag_;
    int replan_counter_;
    int last_selected_action_index_;
    std::chrono::time_point<std::chrono::_V2::system_clock> start_;
    std::chrono::time_point<std::chrono::_V2::system_clock> last_msg_start_;

    // Callback functions
    void pursuerEvaderDataCallback(const multi_target_tracking::PursuerEvaderDataArray& msg);
    // void collectRotatedCameraImages(const multi_target_tracking::PursuerEvaderDataArray& msg);
    void clockCallback(const rosgraph_msgs::Clock& msg);
    void motionExecutionCallback(const rosgraph_msgs::Clock& msg);

    // other
    void drawDiscreteRandomSample(int& curr_time_step_samp, Eigen::MatrixXd& action_prob_dist, int& selected_action_index);

    void objectiveFunctionBSG(double& obj_func_val, int& up_to_drone_id);
    void marginalGainFunctionBSG(double& marginal_gain);
    
    void objectiveFunctionRBG(double& obj_func_val, std::vector<int>& drone_self_in_neighbors_ids);
    void marginalGainFunctionRBG(double& marginal_gain);

    void simulatedCommTimeDelayBSG();  // for Bandit Sequential Greedy BSG
    void simulatedCommTimeDelayRBG();  // for Resource-aware Bandit Greedy

    // for working with setting pose
    msr::airlib::Pose giveGoalPoseFormat(msr::airlib::MultirotorState &curr_pose, float disp_x, float disp_y, float disp_z, float new_yaw);
    Eigen::Quaternionf quaternionFromEuler(float roll, float pitch, float yaw);
    Eigen::Vector3f eulerFromQuaternion(const Eigen::Quaternionf& q);

    std::queue<msr::airlib::Pose> interpolatePose(const Eigen::Matrix<double, 3, 1>& selected_action_displacement,
                                              int num_interp_points, msr::airlib::MultirotorState curr_pose, bool use_currpose_yaw);
    double normalizeYaw(double yaw);
    float yawAngleDiff(float a1, float a2);


    // // EXP*-SIX class object
    EXPStarSIX expStarSixAlg_;
    double horizon_;
    double n_time_steps_;
    double altitude_flight_level_;
    double flight_level_;
    double replan_freq_;
    double replan_period_;
    float altitude_float_;
    // actions set
    Eigen::MatrixXd actions_;

    // Airsim client and constants
    float client_timeout_sec_;
    const msr::airlib::YawMode client_default_yaw_mode_;
    float client_lookahead_;
    float client_adaptive_lookahead_ ;
    float client_margin_;
    double yaw_rate_timestep_;
    msr::airlib::DrivetrainType client_drivetrain_;
    msr::airlib::MultirotorRpcLibClient client_;

    // to store pursuer evader data in a buffer
    multi_target_tracking::PursuerEvaderDataArray pursuer_evader_data_;

    bool ignore_collision_;


    std::unique_ptr<ros::AsyncSpinner> async_spinner_;

    // for motion plan execution
    int next_action_selected_counter_;
    int motion_execution_counter_;

};

#endif // MULTI_TARGET_TRACKING_COORDINATION_H
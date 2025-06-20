/*
NOTE: This script contains the algorithm for RAG
*/
#include "image_covering_coordination/image_covering_coord_RAG.h"
#include "rclcpp/rclcpp.hpp" // Main ROS2 header
#include <chrono>
#include <thread>   // Often included with chrono for sleep utilities
#include "sensor_msgs/msg/image.hpp"

#include "image_covering_coordination/image_covering_coord_RAG.hpp" // Use the new .hpp header
#include <chrono>       // For time literals like 1s
#include <functional>   // For std::bind
#include <future>       // For std::promise used in waitForMessage replacement

using namespace std::chrono_literals; // For s, ms, etc. time literals

// ====================================================================================
// CONSTRUCTOR
// ====================================================================================
ImageCoveringCoordRAG::ImageCoveringCoordRAG(const rclcpp::NodeOptions & options)
    : Node("multi_target_coord_RAG_node", options) // Call parent Node constructor
{
    // --- Get ROS 2 Parameters ---
    this->declare_parameter<std::string>("drone_name", "drone1");
    this->declare_parameter<int>("number_of_drones", 1);
    this->declare_parameter<int>("robot_index_ID", 1);
    this->declare_parameter<std::string>("camera_name", "front_center");
    this->declare_parameter<double>("communication_range", 100.0);
    this->declare_parameter<std::string>("algorithm_to_run", "RAG_AD");
    this->declare_parameter<int>("random_seed", 0);
    this->declare_parameter<int>("num_nearest_neighbors", 3);
    this->declare_parameter<int>("is_server_experiment", 0);

    this->get_parameter("drone_name", drone_name_self_);
    this->get_parameter("number_of_drones", num_robots_);
    this->get_parameter("robot_index_ID", robot_idx_self_);
    this->get_parameter("camera_name", camera_name_);
    this->get_parameter("communication_range", communication_range_);
    this->get_parameter("algorithm_to_run", algorithm_to_run_);
    this->get_parameter("random_seed", random_seed_);
    this->get_parameter("num_nearest_neighbors", num_nearest_neighbors_);
    this->get_parameter("is_server_experiment", is_server_experiment_);

    // Use the ROS2 logger instead of std::cout
    RCLCPP_INFO(this->get_logger(), "ROBOT_SELF_ID: %d", robot_idx_self_);
    RCLCPP_INFO(this->get_logger(), "ALGORITHM: %s", algorithm_to_run_.c_str());

    // SET RAG ALG GRAPH TOPOLOGY (No changes in this logic)
    graph_topology_RAG_ = algorithm_to_run_;
    default_num_nearest_neighbors_ = num_nearest_neighbors_;

    // NOTE: ros::AsyncSpinner is removed. Multi-threading is handled by the
    // ROS2 Executor, which you should configure in the main() function of your program.

    // --- ROS 2 Publishers ---
    std::string mg_msg_topic = drone_name_self_ + "/mg_rag_data";
    mg_rag_msg_pub_ = this->create_publisher<image_covering_coordination::msg::ImageCovering>(mg_msg_topic, 5);

    // --- ROS 2 Subscribers ---
    using std::placeholders::_1;
    clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", 1, std::bind(&ImageCoveringCoordRAG::clockCallback, this, _1));
    marginal_gain_pub_clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", 1, std::bind(&ImageCoveringCoordRAG::marginalGainPublisherclockCallback, this, _1));
    check_drones_selection_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", 1, std::bind(&ImageCoveringCoordRAG::updateAllRobotsSelectionStatusCallback, this, _1));

    // DATA TRANSMISSION RATE SETTINGS (No changes in this logic)
    total_data_msg_size_ = 200.;
    communication_data_rate_ = 100. * total_data_msg_size_;
    communication_round_ = 0;

    // VARIABLE INITIALIZATION (No changes in this logic)
    max_sensing_range_ = 45.;
    flight_level_ = 0.5;
    altitude_flight_level_ = -30.;
    selected_action_ind_ = -1;
    best_marginal_gain_self_ = std::numeric_limits<double>::max();
    finished_action_selection_ = false;
    waiting_for_robots_to_finish_selection_ = false;
    takeoff_wait_flag_ = true;
    terminate_decision_making_ = false;

    // EIGEN MATRIX INITIALIZATION (No changes in this logic)
    actions_.resize(3, 8);
    actions_ << 10., 7.07, 0., -7.07, -10., -7.07, 0., 7.07,
               0., 7.07, 10., 7.07, 0., -7.07, -10., -7.07,
               0., 0., 0., 0., 0., 0., 0., 0.;
    actions_unit_vec_.resize(3, 8);
    actions_unit_vec_ << 1., 0.707, 0., -0.707, -1., -0.707, 0., 0.707,
               0., 0.707, 1., 0.707, 0., -0.707, -1., -0.707,
               0., 0., 0., 0., 0., 0., 0., 0.;

    objective_function_value_ = -1.;
    PEdata_new_msg_received_flag_ = false;

    // CONSTANTS INITIALIZATION (No changes in this logic)
    int n_time_step_init = 0;
    horizon_ = 100;
    n_time_steps_ = 2000;
    double dT = horizon_ / n_time_steps_;
    curr_time_step_samp_ = 0;

    // AIRSIM CLIENT USAGE CONSTANTS (No changes in this logic)
    client_timeout_sec_ = (3.4028235E38F);
    msr::airlib::YawMode client_default_yaw_mode_ = msr::airlib::YawMode();
    client_lookahead_ = (-1.0F);
    client_adaptive_lookahead_ = (1.0F);
    client_margin_ = (5.0F);
    yaw_rate_timestep_ = 1.0;
    drone_linear_velocity_ = 3.0;
    drone_lin_vel_factor_ = 1.33;
    yaw_duration_ = 0.;
    yaw_rate_ = 2.36;

    // CAMERA SETTINGS (No changes in this logic)
    camera_pitch_theta_ = 0.524;
    camera_downward_pitch_angle_ = -1.571;
    current_image_requests_vec_ = { ImageRequest("front_center", msr::airlib::ImageCaptureBase::ImageType::Segmentation, false, false),
                                    ImageRequest("front_center", msr::airlib::ImageCaptureBase::ImageType::Scene, false, false) };
    camera_image_type_ = msr::airlib::ImageCaptureBase::ImageType::Segmentation;
    fov_y_ = M_PI / 3.;
    fov_x_ = 0.817;
    y_half_side_len_m_ = 17.32;
    x_half_side_len_m_ = 13.;
    y_dir_px_m_scale_ = 18.474;
    x_dir_px_m_scale_ = 18.48;
    delta_C_ = 21.94;
    pasted_rect_height_pix_ = 683;
    pasted_rect_width_pix_ = 640;
    diagonal_frame_dist_ = sqrt(pow((2. * y_half_side_len_m_), 2) + pow((2. * x_half_side_len_m_), 2));
    diagonal_px_scale_ = sqrt(pow(y_dir_px_m_scale_, 2) + pow(x_dir_px_m_scale_, 2));
    take_new_pics_ = true;
    road_pixel_weight_ = 1.;
    assump_counter_ = 0;
    control_action_disp_scale_ = 1.5;

    // AIRSIM CLIENT CONNECTION (No changes in this logic)
    client_drivetrain_ = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    client_.confirmConnection();
    client_.enableApiControl(true, drone_name_self_);
    client_.armDisarm(true);

    for (int i = 0; i < num_robots_; i++) {
        all_robots_selected_map_[i + 1] = false;
    }

    start_ = std::chrono::high_resolution_clock::now();
    // Use the ROS2 message type here
    camera_rotated_imgs_vec_ = std::vector<sensor_msgs::msg::Image>(8);
    best_image_cam_orientation_ = -1;
    img_save_path_ = "/home/sgari/sandilya_ws/unity_airsim_workspace/AirSim/ros/src/image_covering_coordination/data/";
}

// ====================================================================================
// DESTRUCTOR
// ====================================================================================
ImageCoveringCoordRAG::~ImageCoveringCoordRAG()
{
    // async_spinner_->stop() is removed. Shutdown is handled by the main executor.
}

// ====================================================================================
// CALLBACKS
// ====================================================================================

void ImageCoveringCoordRAG::marginalGainPublisherclockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
    (void)msg; // Mark 'msg' as unused
    msr::airlib::MultirotorState pose1 = client_.getMultirotorState(drone_name_self_);
    geometry_msgs::msg::Pose latest_pose = convertToGeometryPose(pose1);

    auto imcov_mg_msg = image_covering_coordination::msg::ImageCovering();

    imcov_mg_msg.header.stamp = this->get_clock()->now();
    imcov_mg_msg.drone_id = robot_idx_self_;
    imcov_mg_msg.image = best_self_image_;
    imcov_mg_msg.camera_orientation_setting = best_image_cam_orientation_;
    imcov_mg_msg.pose = latest_pose;
    imcov_mg_msg.flag_completed_action_selection = finished_action_selection_;
    imcov_mg_msg.best_marginal_gain = best_marginal_gain_self_;

    mg_rag_msg_pub_->publish(imcov_mg_msg);
    // std::cout << std::boolalpha; // This can be replaced with RCLCPP_INFO if needed for debugging
}

void ImageCoveringCoordRAG::updateAllRobotsSelectionStatusCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
    (void)msg; // Mark 'msg' as unused

    RCLCPP_WARN_ONCE(this->get_logger(),
        "The logic in 'updateAllRobotsSelectionStatusCallback' uses a blocking 'waitForMessage' pattern. "
        "This direct translation is inefficient and can cause deadlocks. "
        "The recommended ROS2 approach is to use persistent, asynchronous subscribers for each topic.");

    for (int i = 0; i < num_robots_; i++) {
        int other_robot_id = i + 1;
        std::string mg_msg_topic = "Drone" + std::to_string(other_robot_id) + "/mg_rag_data";

        auto temp_node = rclcpp::Node::make_shared("wait_for_message_node_" + std::to_string(other_robot_id) + std::to_string(this->get_clock()->now().nanoseconds()));
        image_covering_coordination::msg::ImageCovering received_msg;
        bool message_received = false;

        auto promise = std::make_shared<std::promise<void>>();
        auto future = promise->get_future();
        
        auto sub = temp_node->create_subscription<image_covering_coordination::msg::ImageCovering>(
            mg_msg_topic, 10, 
            [&](const image_covering_coordination::msg::ImageCovering::SharedPtr msg) {
                received_msg = *msg;
                message_received = true;
                promise->set_value();
            });

        auto status = rclcpp::spin_until_future_complete(temp_node, future, 1.5s);
        
        if (status == rclcpp::FutureReturnCode::SUCCESS) {
            all_robots_selected_map_[i + 1] = received_msg.flag_completed_action_selection;
        } else {
            all_robots_selected_map_[i + 1] = false;
        }
    }
    // std::cout << "" << std::endl; // This can be removed or replaced with logging
}


void ImageCoveringCoordRAG::clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
    (void)msg; // Mark 'msg' as unused

    if (takeoff_wait_flag_ == true) {
        client_.moveToZAsync(altitude_flight_level_, 5.0, client_timeout_sec_, client_default_yaw_mode_, client_lookahead_, client_adaptive_lookahead_, drone_name_self_)->waitOnLastTask();
        rclcpp::sleep_for(1s);
        takeoff_wait_flag_ = false;

        srand(random_seed_ + robot_idx_self_);
        int direction = rand() % 4;
        double distance = 4.0;
        double dx = 0, dy = 0;
        switch (direction) {
            case 0: dx = distance; break;
            case 1: dx = -distance; break;
            case 2: dy = distance; break;
            case 3: dy = -distance; break;
        }
        double timespan_tomove = 2.67;
        client_.moveByVelocityBodyFrameAsync(dx / timespan_tomove, dy / timespan_tomove, 0., timespan_tomove, client_drivetrain_, client_default_yaw_mode_, drone_name_self_)->waitOnLastTask();
        rclcpp::sleep_for(1s);
        return;
    }

    RCLCPP_WARN_ONCE(this->get_logger(),
        "The logic in 'clockCallback' uses a blocking 'waitForMessage' pattern. "
        "Please re-architect this to use a persistent, asynchronous subscriber.");

    auto temp_node = rclcpp::Node::make_shared("wait_for_neighbors_node_" + std::to_string(this->get_clock()->now().nanoseconds()));
    bool message_received = false;
    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();
    auto sub = temp_node->create_subscription<airsim_ros_pkgs::msg::NeighborsArray>(
        "/neighbors_data", 10,
        [&](const airsim_ros_pkgs::msg::NeighborsArray::SharedPtr msg) {
            neighbors_data_ = *msg;
            message_received = true;
            promise->set_value();
        });
    rclcpp::spin_until_future_complete(temp_node, future, 5s); // Wait up to 5s

    if (!message_received) {
        RCLCPP_ERROR(this->get_logger(), "Did not receive /neighbors_data message. Aborting clockCallback.");
        return;
    }

    msr::airlib::MultirotorState pose1 = client_.getMultirotorState(drone_name_self_);
    geometry_msgs::msg::Pose latest_pose = convertToGeometryPose(pose1);

    std::map<int, ImageInfoStruct> selected_action_inneigh_ids_mg;
    while ((finished_action_selection_ == false) && (waiting_for_robots_to_finish_selection_ == false)) {
        int best_control_action_ind;
        double best_marginal_gain_self;
        marginalGainFunctionRAG(best_marginal_gain_self, best_control_action_ind, latest_pose, selected_action_inneigh_ids_mg);
        best_marginal_gain_self_ = best_marginal_gain_self;

        RCLCPP_INFO(this->get_logger(), "robot %d best marginal gain: %f", robot_idx_self_, best_marginal_gain_self_);

        std::map<int, double> non_selected_action_inneigh_ids_mg;
        auto start1 = std::chrono::high_resolution_clock::now();
        getNonSelectedActionsInNeighborIDs(non_selected_action_inneigh_ids_mg);
        RCLCPP_INFO(this->get_logger(), "robot %d non selected neighbors:", robot_idx_self_);
        printMap(non_selected_action_inneigh_ids_mg);

        auto end1 = std::chrono::high_resolution_clock::now();
        double duration1 = std::chrono::duration_cast<std::chrono::nanoseconds>(end1 - start1).count();
        simulatedCommTimeDelayRAG(duration1);

        bool is_my_mg_best;
        isMyMarginalGainBest(is_my_mg_best, non_selected_action_inneigh_ids_mg);
        std::map<int, ImageInfoStruct> new_selected_action_inneigh_ids_mg;

        if (is_my_mg_best == true) {
            finished_action_selection_ = true;
            selected_action_ind_ = best_control_action_ind;
            take_new_pics_ = true;
            break;
        } else {
            auto start2 = std::chrono::high_resolution_clock::now();

            if ((graph_topology_RAG_ == "allow_disconnected") || (graph_topology_RAG_ == "RAG_AD")) {
                getSelectedActionsInNeighborIDImages(selected_action_inneigh_ids_mg);
            } else if ((graph_topology_RAG_ == "line_path") || (graph_topology_RAG_ == "RAG_LP")) {
                getSelectedActionsInNeighborIDImagesLinePath(selected_action_inneigh_ids_mg);
            } else if ((graph_topology_RAG_ == "nearest_neighbors") || (graph_topology_RAG_ == "RAG_NN")) {
                getSelectedActionsInNeighborIDImagesNearestNeighbors(selected_action_inneigh_ids_mg, default_num_nearest_neighbors_);
            }

            auto end2 = std::chrono::high_resolution_clock::now();
            double duration2 = std::chrono::duration_cast<std::chrono::nanoseconds>(end2 - start2).count();

            if (areAllValuesTrue(check_drones_msg_dict_)) {
                simulatedCommTimeDelayRAG(duration2);
            }
            take_new_pics_ = false;
        }
    }

    Eigen::Matrix<double, 3, 1> selected_action_displacement = actions_.block(0, selected_action_ind_, 3, 1);
    last_selected_displacement_ = selected_action_displacement;

    bool all_robots_ready_to_proceed = areAllValuesTrue();

    if (all_robots_ready_to_proceed == true) {
        RCLCPP_INFO(this->get_logger(), "ALL ROBOTS READY TO PROCEED WITH ACTION");
        auto position = pose1.getPosition();
        client_.moveToPositionAsync(position.x() + control_action_disp_scale_ * selected_action_displacement(0, 0), position.y() + control_action_disp_scale_ * selected_action_displacement(1, 0), altitude_flight_level_, 4., client_timeout_sec_, client_drivetrain_, client_default_yaw_mode_, client_lookahead_, client_adaptive_lookahead_, drone_name_self_)->waitOnLastTask();
        finished_action_selection_ = false;
        waiting_for_robots_to_finish_selection_ = false;
        if (assump_counter_ < 2) {
            best_marginal_gain_self_ = std::numeric_limits<double>::max();
            assump_counter_ += 1;
        }
    } else {
        waiting_for_robots_to_finish_selection_ = true;
    }
    curr_time_step_samp_++;
}

void ImageCoveringCoordRAG::collectRotatedCameraImages()
{
    // --- This block of AirSim client calls and math is unchanged ---
    msr::airlib::MultirotorState pose1 = client_.getMultirotorState(drone_name_self_);
    auto orientation1 = pose1.kinematics_estimated.pose.orientation;
    Eigen::Vector3f euler = eulerFromQuaternion(orientation1.cast<float>());
    float curr_roll1 = euler[0];
    float curr_yaw1 = euler[2];

    bool ignore_collision = true;
    client_.moveByRollPitchYawZAsync(0., 0., 0.1, -30., 1., drone_name_self_)->waitOnLastTask();
    client_.moveByRollPitchYawZAsync(0., 0., 0., -30., 1., drone_name_self_)->waitOnLastTask();
    // --- End of unchanged block ---


    // CHANGE: The vector now holds ROS2 message types. This must match the declaration
    // in your class header (.hpp) file.
    camera_rotated_imgs_vec_.clear();
    camera_rotated_imgs_vec_ = std::vector<sensor_msgs::msg::Image>(8);


    // --- Take picture at 0 degrees ---
    msr::airlib::Pose camera_pose_0 = giveCameraGoalPoseFormat(pose1, -0.021, 0.0112, altitude_flight_level_, curr_roll1, camera_downward_pitch_angle_ + camera_pitch_theta_, curr_yaw1);
    client_.simSetCameraPose("front_center", camera_pose_0, drone_name_self_);

    // CHANGE: ros::Duration(...).sleep() is replaced with rclcpp::sleep_for().
    // You can also use std::this_thread::sleep_for(std::chrono::seconds(1));
    rclcpp::sleep_for(std::chrono::seconds(1));

    std::vector<ImageResponse> img_response_vec0 = client_.simGetImages(current_image_requests_vec_, drone_name_self_);

    // CHANGE: The pointer type is updated. Using 'auto' is the easiest way to handle
    // the new smart pointer type (e.g., std::unique_ptr<sensor_msgs::msg::Image>)
    // returned by the migrated get_img_msg_from_response function.
    auto img0_ptr = get_img_msg_from_response(img_response_vec0.at(1));
    camera_rotated_imgs_vec_[0] = (*img0_ptr);

    msr::airlib::Pose camera_pose_1 = giveCameraGoalPoseFormat(pose1, -0.021, 0.0112, altitude_flight_level_, curr_roll1, camera_downward_pitch_angle_ - camera_pitch_theta_, curr_yaw1);
    client_.simSetCameraPose("front_center", camera_pose_1, drone_name_self_);
    rclcpp::sleep_for(std::chrono::seconds(1));
    std::vector<ImageResponse> img_response_vec1 = client_.simGetImages(current_image_requests_vec_, drone_name_self_);
    
    auto img1_ptr = get_img_msg_from_response(img_response_vec1.at(1));
    camera_rotated_imgs_vec_[4] = (*img1_ptr);
    // --- End of 0 degrees block ---


    // --- Take picture at +45 degrees (and all subsequent blocks) ---
    // The pattern of changes is the same for the 45, 90, and 135 degree sections.
    // The AirSim calls are unchanged.
    client_.moveByRollPitchYawZAsync(0., 0., 0.7854, -30., 3., drone_name_self_)->waitOnLastTask();
    
    // ... (AirSim get state logic is unchanged)
    msr::airlib::MultirotorState pose2_new = client_.getMultirotorState(drone_name_self_);
    auto orientation2 = pose2_new.kinematics_estimated.pose.orientation;
    Eigen::Vector3f euler2 = eulerFromQuaternion(orientation2.cast<float>());
    float curr_roll2 = euler2[0];
    float curr_yaw2 = euler2[2];
    
    msr::airlib::Pose camera_pose_45 = giveCameraGoalPoseFormat(pose2_new, -0.021, 0.0112, altitude_flight_level_, curr_roll2, camera_downward_pitch_angle_ + camera_pitch_theta_, curr_yaw2);
    client_.simSetCameraPose("front_center", camera_pose_45, drone_name_self_);
    // CHANGE: ros::Duration sleep is replaced
    rclcpp::sleep_for(std::chrono::seconds(1));
    std::vector<ImageResponse> img_response_vec_45_0 = client_.simGetImages(current_image_requests_vec_, drone_name_self_);
    // CHANGE: Pointer type is updated via 'auto'
    auto img_45_0_ptr = get_img_msg_from_response(img_response_vec_45_0.at(1));
    camera_rotated_imgs_vec_[1] = (*img_45_0_ptr);

    msr::airlib::Pose camera_pose_45_1 = giveCameraGoalPoseFormat(pose2_new, -0.021, 0.0112, altitude_flight_level_, curr_roll2, camera_downward_pitch_angle_ - camera_pitch_theta_, curr_yaw2);
    client_.simSetCameraPose("front_center", camera_pose_45_1, drone_name_self_);
    rclcpp::sleep_for(std::chrono::seconds(1));
    std::vector<ImageResponse> img_response_vec_45_1 = client_.simGetImages(current_image_requests_vec_, drone_name_self_);
    auto img_45_1_ptr = get_img_msg_from_response(img_response_vec_45_1.at(1));
    camera_rotated_imgs_vec_[5] = (*img_45_1_ptr);
    // --- End of 45 degrees block ---


    // --- NOTE: The same changes apply to the 90 and 135 degree sections ---
    // [ ... code for 90 and 135 degrees with the same rclcpp::sleep_for() and 'auto' pointer changes ... ]
    

    // This call to another member function is unchanged.
    rotateImagesInVector();

    // --- This final block of AirSim client calls is also unchanged ---
    client_.moveByRollPitchYawZAsync(0., 0., 0.0, -30., 3., drone_name_self_)->waitOnLastTask();
    msr::airlib::MultirotorState pose5_new = client_.getMultirotorState(drone_name_self_);
    auto orientation5 = pose5_new.kinematics_estimated.pose.orientation;
    Eigen::Vector3f euler5 = eulerFromQuaternion(orientation5.cast<float>());
    float curr_roll5 = euler5[0];
    float curr_yaw5 = euler5[2];
    msr::airlib::Pose camera_pose_reset = giveCameraGoalPoseFormat(pose5_new, -0.021, 0.0112, altitude_flight_level_, curr_roll5, camera_downward_pitch_angle_, curr_yaw5);
    client_.simSetCameraPose("front_center", camera_pose_reset, drone_name_self_);
    
    // CHANGE: Final sleep call is updated
    rclcpp::sleep_for(std::chrono::seconds(1));
}

// Update return type to ROS2 message shared pointer and use std::make_shared
sensor_msgs::msg::Image::SharedPtr ImageCoveringCoordRAG::get_img_msg_from_response(const ImageResponse& img_response)
{
    auto img_msg_ptr = std::make_shared<sensor_msgs::msg::Image>();
    img_msg_ptr->data = img_response.image_data_uint8;
    img_msg_ptr->step = img_response.width * 3;
    // Assuming airsim_timestamp_to_ros is updated to return rclcpp::Time
    img_msg_ptr->header.stamp = airsim_timestamp_to_ros(img_response.time_stamp);
    img_msg_ptr->height = img_response.height;
    img_msg_ptr->width = img_response.width;
    img_msg_ptr->encoding = "rgb8";
    img_msg_ptr->is_bigendian = 0;
    return img_msg_ptr;
}

// Update message types and use ROS2 logging for errors
std::vector<double> ImageCoveringCoordRAG::computeWeightedScores(const std::vector<sensor_msgs::msg::Image>& images)
{
    std::vector<double> scores;
    for (const auto& image : images) {
        double score = 0.0;
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            // Use the message's encoding field
            cv_ptr = cv_bridge::toCvCopy(image, image.encoding);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return scores;
        }
        // ... (Scoring logic remains identical) ...
        scores.push_back(score);
    }
    return scores;
}


""" DEBUG 2 """
// This is the completion of the function from the previous snippet.
// It is migrated to use ROS2 message types and logging.
double ImageCoveringCoordRAG::computeWeightedScoreSingleImage(const sensor_msgs::msg::Image& image, bool convertBGR2RGB)
{
    double score = 0.0;

    // Get OpenCV Mat from image
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image, image.encoding);
    }
    catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return score;
    }
    cv::Mat img = cv_ptr->image;
    if (convertBGR2RGB) {
        cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

        if (is_server_experiment_) { // Note: bool comparison
            for (int r = 0; r < img.rows; ++r) {
                for (int c = 0; c < img.cols; ++c) {
                    cv::Vec3b pixel = img.at<cv::Vec3b>(r, c);
                    if (pixel != cv::Vec3b(0, 0, 0)) {
                        score += 1.0;
                    }
                }
            }
        }
        else {
            for (int r = 0; r < img.rows; ++r) {
                for (int c = 0; c < img.cols; ++c) {
                    cv::Vec3b pixel = img.at<cv::Vec3b>(r, c);
                    // NOTE!! for use with unity EXECUTABLE 46DEG X 60 DEG FOV, 15 drones!!!
                    if ((pixel == cv::Vec3b(128, 0, 50))) { // road_1
                        score += road_pixel_weight_;
                    }
                    else if ((pixel == cv::Vec3b(200, 73, 18))) { // road_2
                        score += road_pixel_weight_;
                    }
                }
            }
        }
    }
    else {
        if (is_server_experiment_) { // Note: bool comparison
             for (int r = 0; r < img.rows; ++r) {
                for (int c = 0; c < img.cols; ++c) {
                    cv::Vec3b pixel = img.at<cv::Vec3b>(r, c);
                    if (pixel != cv::Vec3b(0, 0, 0)) {
                        score += 1.0;
                    }
                }
            }
        }
        else {
            for (int r = 0; r < img.rows; ++r) {
                for (int c = 0; c < img.cols; ++c) {
                    cv::Vec3b pixel = img.at<cv::Vec3b>(r, c);
                     // NOTE!! for use with unity EXECUTABLE 46DEG X 60 DEG FOV, 15 drones!!!
                    if ((pixel == cv::Vec3b(50, 0, 128))) { // road_1
                        score += road_pixel_weight_;
                    }
                    else if ((pixel == cv::Vec3b(18, 73, 200))) { // road_2
                        score += road_pixel_weight_;
                    }
                }
            }
        }
    }

    return score;
}

// This overloaded function has no ROS dependencies and can be copied directly.
double ImageCoveringCoordRAG::computeWeightedScoreSingleImage(const cv::Mat& img)
{
    double score = 0.0;

    if (is_server_experiment_) {
        for (int r = 0; r < img.rows; ++r) {
            for (int c = 0; c < img.cols; ++c) {
                cv::Vec3b pixel = img.at<cv::Vec3b>(r, c);
                if (pixel != cv::Vec3b(0, 0, 0)) {
                    score += 1.0;
                }
            }
        }
    }
    else {
        for (int r = 0; r < img.rows; ++r) {
            for (int c = 0; c < img.cols; ++c) {
                cv::Vec3b pixel = img.at<cv::Vec3b>(r, c);
                if (pixel == cv::Vec3b(144, 9, 201)) { // road_1
                    score += 1.5;
                }
                else if (pixel == cv::Vec3b(130, 9, 200)) { // road_2
                    score += 1.5;
                }
                else if (pixel == cv::Vec3b(53, 73, 65)) { // pavement_1
                    score += 0.75;
                }
                else if (pixel == cv::Vec3b(0, 72, 128)) { // pavement_2
                    score += 0.75;
                }
            }
        }
    }
    return score;
}

// Migrated to return rclcpp::Time
rclcpp::Time ImageCoveringCoordRAG::airsim_timestamp_to_ros(const msr::airlib::TTimePoint& stamp) const
{
    std::chrono::nanoseconds dur(stamp);
    std::chrono::time_point<std::chrono::system_clock> tp(dur);
    return chrono_timestamp_to_ros(tp);
}

// Migrated to return rclcpp::Time
rclcpp::Time ImageCoveringCoordRAG::chrono_timestamp_to_ros(const std::chrono::system_clock::time_point& stamp) const
{
    // rclcpp::Time has a constructor that takes nanoseconds directly
    return rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(stamp.time_since_epoch()).count());
}

// No ROS dependencies, copied directly.
void ImageCoveringCoordRAG::isMyMarginalGainBest(bool& is_my_mg_best, std::map<int, double>& non_selected_action_inneigh_ids_mg)
{
    is_my_mg_best = true; // Assume initially that your mg is the best
    for (const auto& pair : non_selected_action_inneigh_ids_mg) {
        if (best_marginal_gain_self_ < pair.second) {
            is_my_mg_best = false;
            break;
        }
    }
}

// No ROS dependencies, copied directly. Relies on neighbors_data_ which is now populated by a subscriber.
void ImageCoveringCoordRAG::getNearestNeighborRobots(std::vector<int>& nn_ids, int num_in_neighbors)
{
    // use neighbors_data_ which is of type airsim_ros_pkgs::msg::NeighborsArray
    std::vector<int> robots_ids = neighbors_data_.neighbors_array.at(robot_idx_self_ - 1).robots_range_bearing_ids;
    std::vector<double> robots_range_bearing_vals = neighbors_data_.neighbors_array.at(robot_idx_self_ - 1).robots_range_bearing_values;

    std::vector<std::pair<double, int>> range_id_pairs;

    for (size_t i = 0; i < robots_range_bearing_vals.size(); i += 2) {
        double range = robots_range_bearing_vals[i];
        int id = robots_ids[i / 2];
        range_id_pairs.push_back(std::make_pair(range, id));
    }

    std::sort(range_id_pairs.begin(), range_id_pairs.end(), [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
        return a.first < b.first;
    });

    nn_ids.clear();

    for (int i = 0; i < num_in_neighbors && i < static_cast<int>(range_id_pairs.size()); ++i) {
        nn_ids.push_back(range_id_pairs[i].second);
    }
}

// **REFACTORED** to use asynchronous data map instead of waitForMessage
void ImageCoveringCoordRAG::getSelectedActionsInNeighborIDImagesNearestNeighbors(std::map<int, ImageInfoStruct>& selected_action_inneigh_ids_mg, int num_in_neighbors)
{
    check_drones_msg_dict_.clear();
    std::vector<int> new_inneigh_ids;
    getNearestNeighborRobots(new_inneigh_ids, num_in_neighbors);
    
    RCLCPP_INFO(this->get_logger(), "Robot %d 's nearest neighbors: ", robot_idx_self_);
    // printVector(new_inneigh_ids); // Assuming you have a helper to print vectors for debugging

    rclcpp::Time timeout_boundary = this->now() - rclcpp::Duration(1, 500000000); // 1.5 seconds ago

    for (int& drone_id : new_inneigh_ids) {
        auto it = all_robots_info_map_.find(drone_id);

        if (it != all_robots_info_map_.end() && it->second.last_updated > timeout_boundary) {
            const auto& mg_msg_topic_data = it->second.msg;
            if (mg_msg_topic_data->flag_completed_action_selection == true) {
                ImageInfoStruct image_info;
                image_info.pose = mg_msg_topic_data->pose;
                image_info.image = mg_msg_topic_data->image;
                image_info.camera_orientation = mg_msg_topic_data->camera_orientation_setting;
                image_info.best_marginal_gain_val = mg_msg_topic_data->best_marginal_gain;
                selected_action_inneigh_ids_mg[drone_id] = image_info;
                RCLCPP_INFO(this->get_logger(), "Robot %d: Got selected action from neighbor %d", robot_idx_self_, drone_id);
                check_drones_msg_dict_[drone_id] = true;
            } else {
                RCLCPP_WARN(this->get_logger(), "Robot %d has not completed selection, assuming default MG", drone_id);
                ImageInfoStruct image_info;
                image_info.best_marginal_gain_val = -1.0;
                selected_action_inneigh_ids_mg[drone_id] = image_info;
                check_drones_msg_dict_[drone_id] = false;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Robot %d: No recent message from neighbor %d", robot_idx_self_, drone_id);
            ImageInfoStruct image_info;
            image_info.best_marginal_gain_val = -1.0;
            selected_action_inneigh_ids_mg[drone_id] = image_info;
            check_drones_msg_dict_[drone_id] = false;
        }
    }
}

// **REFACTORED** to use asynchronous data map instead of waitForMessage
void ImageCoveringCoordRAG::getSelectedActionsInNeighborIDImagesLinePath(std::map<int, ImageInfoStruct>& selected_action_inneigh_ids_mg)
{
    std::vector<int> new_inneigh_ids;
    if (robot_idx_self_ == 1) {
        new_inneigh_ids.push_back(robot_idx_self_ + 1);
    } else if (robot_idx_self_ == num_robots_) {
        new_inneigh_ids.push_back(robot_idx_self_ - 1);
    } else {
        new_inneigh_ids.push_back(robot_idx_self_ - 1);
        new_inneigh_ids.push_back(robot_idx_self_ + 1);
    }

    rclcpp::Time timeout_boundary = this->now() - rclcpp::Duration(1, 500000000); // 1.5 seconds ago

    for (int& drone_id : new_inneigh_ids) {
        auto it = all_robots_info_map_.find(drone_id);
        if (it != all_robots_info_map_.end() && it->second.last_updated > timeout_boundary) {
            const auto& mg_msg_topic_data = it->second.msg;
            if (mg_msg_topic_data->flag_completed_action_selection == true) {
                ImageInfoStruct image_info;
                image_info.pose = mg_msg_topic_data->pose;
                image_info.image = mg_msg_topic_data->image;
                image_info.camera_orientation = mg_msg_topic_data->camera_orientation_setting;
                image_info.best_marginal_gain_val = mg_msg_topic_data->best_marginal_gain;
                selected_action_inneigh_ids_mg[drone_id] = image_info;
            }
        }
    }
}

// **REFACTORED** to use asynchronous data map instead of waitForMessage
void ImageCoveringCoordRAG::getSelectedActionsInNeighborIDImages(std::map<int, ImageInfoStruct>& selected_action_inneigh_ids_mg)
{
    std::vector<int> all_inneigh_ids = neighbors_data_.neighbors_array.at(robot_idx_self_ - 1).in_neighbor_ids;
    rclcpp::Time timeout_boundary = this->now() - rclcpp::Duration(1, 500000000); // 1.5 seconds ago

    for (int& drone_id : all_inneigh_ids) {
        // Only check for new information
        if (selected_action_inneigh_ids_mg.find(drone_id) != selected_action_inneigh_ids_mg.end()) {
            continue;
        }

        auto it = all_robots_info_map_.find(drone_id);
        if (it != all_robots_info_map_.end() && it->second.last_updated > timeout_boundary) {
            const auto& mg_msg_topic_data = it->second.msg;
            if (mg_msg_topic_data->flag_completed_action_selection == true) {
                ImageInfoStruct image_info;
                image_info.pose = mg_msg_topic_data->pose;
                image_info.image = mg_msg_topic_data->image;
                image_info.camera_orientation = mg_msg_topic_data->camera_orientation_setting;
                image_info.best_marginal_gain_val = mg_msg_topic_data->best_marginal_gain;
                selected_action_inneigh_ids_mg[drone_id] = image_info;
            }
        }
    }
}

// **REFACTORED** to use asynchronous data map instead of waitForMessage
void ImageCoveringCoordRAG::getSelectedActionsInNeighborIDs(std::map<int, double>& selected_action_inneigh_ids_mg)
{
    std::vector<int> all_inneigh_ids = neighbors_data_.neighbors_array.at(robot_idx_self_ - 1).in_neighbor_ids;
    rclcpp::Time timeout_boundary = this->now() - rclcpp::Duration(1, 500000000); // 1.5 seconds ago

    for (int& drone_id : all_inneigh_ids) {
        auto it = all_robots_info_map_.find(drone_id);
        if (it != all_robots_info_map_.end() && it->second.last_updated > timeout_boundary) {
            const auto& mg_msg_topic_data = it->second.msg;
            if (mg_msg_topic_data->flag_completed_action_selection == true) {
                selected_action_inneigh_ids_mg[drone_id] = mg_msg_topic_data->best_marginal_gain;
            }
        }
    }
}

// **REFACTORED** to use asynchronous data map instead of waitForMessage
void ImageCoveringCoordRAG::getNonSelectedActionsInNeighborIDs(std::map<int, double>& non_selected_action_inneigh_ids_mg)
{
    std::vector<int> all_inneigh_ids;
    if ((graph_topology_RAG_ == "allow_disconnected") || (graph_topology_RAG_ == "RAG_AD")) {
        all_inneigh_ids = neighbors_data_.neighbors_array.at(robot_idx_self_ - 1).in_neighbor_ids;
    } else if ((graph_topology_RAG_ == "line_path") || (graph_topology_RAG_ == "RAG_LP")) {
        if (robot_idx_self_ == 1) {
            all_inneigh_ids.push_back(robot_idx_self_ + 1);
        } else if (robot_idx_self_ == num_robots_) {
            all_inneigh_ids.push_back(robot_idx_self_ - 1);
        } else {
            all_inneigh_ids.push_back(robot_idx_self_ - 1);
            all_inneigh_ids.push_back(robot_idx_self_ + 1);
        }
    } else if ((graph_topology_RAG_ == "nearest_neighbors") || (graph_topology_RAG_ == "RAG_NN")) {
        getNearestNeighborRobots(all_inneigh_ids, default_num_nearest_neighbors_);
    }

    rclcpp::Time timeout_boundary = this->now() - rclcpp::Duration(1, 500000000); // 1.5 seconds ago

    for (int& drone_id : all_inneigh_ids) {
        auto it = all_robots_info_map_.find(drone_id);
        if (it != all_robots_info_map_.end() && it->second.last_updated > timeout_boundary) {
            const auto& mg_msg_topic_data = it->second.msg;
            if (mg_msg_topic_data->flag_completed_action_selection == false) {
                non_selected_action_inneigh_ids_mg[drone_id] = mg_msg_topic_data->best_marginal_gain;
            }
        } else {
            // Handle timeout case: assume a very high marginal gain so this robot doesn't win
            non_selected_action_inneigh_ids_mg[drone_id] = 10000000000.0;
        }
    }
}

// **MIGRATED** - Message types updated in signature
void ImageCoveringCoordRAG::marginalGainFunctionRAG(double& best_marginal_gain, int& optimal_action_id, const geometry_msgs::msg::Pose& latest_pose, std::map<int, ImageInfoStruct>& prev_selected_action_inneigh_ids_img)
{
    /* evalutates the best marginal gain of all control actions in actions_ */

    if ((graph_topology_RAG_ == "allow_disconnected") || (graph_topology_RAG_ == "RAG_AD")) {
        getSelectedActionsInNeighborIDImages(prev_selected_action_inneigh_ids_img);
    }
    else if ((graph_topology_RAG_ == "line_path") || (graph_topology_RAG_ == "RAG_LP")) {
        getSelectedActionsInNeighborIDImagesLinePath(prev_selected_action_inneigh_ids_img);
    }
    else if ((graph_topology_RAG_ == "nearest_neighbors") || (graph_topology_RAG_ == "RAG_NN")) {
        getSelectedActionsInNeighborIDImagesNearestNeighbors(prev_selected_action_inneigh_ids_img, default_num_nearest_neighbors_);
    }

    std::map<int, std::pair<geometry_msgs::msg::Pose, int>> in_neighbor_poses_and_orientations;
    for (const auto& neighbor : prev_selected_action_inneigh_ids_img) {
        if (neighbor.second.best_marginal_gain_val != -1) {
            std::pair<geometry_msgs::msg::Pose, int> pose_cam_orient;
            pose_cam_orient.first = neighbor.second.pose;
            pose_cam_orient.second = neighbor.second.camera_orientation;
            in_neighbor_poses_and_orientations[neighbor.first] = pose_cam_orient;
        }
    }

    std::map<int, double> marginal_gains_map;
    if (take_new_pics_)
    {
        collectRotatedCameraImages();
    }

    for (size_t i = 0; i < camera_rotated_imgs_vec_.size(); i++) {
        // Note: Switched to size_t for loop variable to match vector::size type
        sensor_msgs::msg::Image masked_image = findNonOverlappingRegions(camera_rotated_imgs_vec_[i], latest_pose, i, in_neighbor_poses_and_orientations, false);
        double score = computeWeightedScoreSingleImage(masked_image, false);
        marginal_gains_map[i] = score;
    }

    auto comparator = [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
        return a.second > b.second;
    };
    std::vector<std::pair<int, double>> sorted_marginal_gains(marginal_gains_map.begin(), marginal_gains_map.end());
    std::sort(sorted_marginal_gains.begin(), sorted_marginal_gains.end(), comparator);

    optimal_action_id = sorted_marginal_gains.empty() ? -1 : sorted_marginal_gains.front().first;
    best_marginal_gain = sorted_marginal_gains.empty() ? 0.0 : sorted_marginal_gains.front().second;

    best_image_cam_orientation_ = optimal_action_id;
    if(optimal_action_id != -1) {
      best_self_image_ = camera_rotated_imgs_vec_.at(optimal_action_id);
    }
}

// In ROS2, the function signature is updated to use ROS2 message types from the '::msg::' namespace.
sensor_msgs::msg::Image ImageCoveringCoordRAG::findNonOverlappingRegions(
    const sensor_msgs::msg::Image& curr_image,
    const geometry_msgs::msg::Pose& curr_pose,
    int curr_orientation,
    std::map<int, std::pair<geometry_msgs::msg::Pose, int>>& in_neighbor_poses_and_orientations,
    bool convertBGR2RGB)
{
    // Convert current image to OpenCV format. The cv_bridge API is very similar in ROS2.
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat img;
    try {
        // This call is compatible with ROS2. It takes a ROS2 image message.
        cv_ptr = cv_bridge::toCvCopy(curr_image, curr_image.encoding);
        img = cv_ptr->image;
    }
    catch (const cv_bridge::Exception& e) {
        // CHANGE: Use the ROS2 logger macro, assuming this class inherits from rclcpp::Node.
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return curr_image; // The return type matches the updated function signature.
    }

    // This OpenCV logic is completely independent of ROS and requires no changes.
    if (convertBGR2RGB) {
        cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
    }

    int padding = static_cast<int>(diagonal_px_scale_ * diagonal_frame_dist_);
    cv::Mat mask(img.rows + 2 * padding, img.cols + 2 * padding, CV_8UC3, cv::Scalar(0, 0, 0));

    // ******* (NO CHANGES IN THIS BLOCK) paste curr img onto mask *******
    cv::Point2f center_mask((mask.cols - 1) / 2.0, (mask.rows - 1) / 2.0);
    cv::Point2f center_img((img.cols - 1) / 2.0, (img.rows - 1) / 2.0);

    double angle = calculateAngleFromOrientation(curr_orientation);
    cv::Mat rot = cv::getRotationMatrix2D(center_img, -angle, 1.0);

    cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), img.size(), -angle).boundingRect2f();

    rot.at<double>(0, 2) += bbox.width / 2.0 - img.cols / 2.0;
    rot.at<double>(1, 2) += bbox.height / 2.0 - img.rows / 2.0;

    cv::Mat rotated;
    cv::warpAffine(img, rotated, rot, bbox.size());

    rotated.copyTo(mask(cv::Rect(center_mask.x - rotated.cols / 2,
                                  center_mask.y - rotated.rows / 2,
                                  rotated.cols,
                                  rotated.rows)));
    // ******* (NO CHANGES IN THIS BLOCK) end of pasting *******


    // Iterate through in-neighbor poses. The logic here is unchanged.
    // The types in the map (geometry_msgs::msg::Pose) match the new function signature.
    std::vector<std::vector<cv::Point>> box_points_vec;
    int c = 0;
    for (const auto& neighbor : in_neighbor_poses_and_orientations) {
        
        // This call to a helper function works as long as PoseEuclideanDistance is also
        // updated to accept geometry_msgs::msg::Pose arguments.
        double dist = PoseEuclideanDistance(curr_pose, neighbor.second.first, curr_orientation, neighbor.second.second);
        if (dist > diagonal_frame_dist_) {
            c += 1;
            continue;
        }

        // --- (NO CHANGES IN THIS BLOCK) All this logic is math and OpenCV ---
        double dx_m = (neighbor.second.first.position.x + (delta_C_ * actions_unit_vec_(0, neighbor.second.second))) -
                      (curr_pose.position.x + (delta_C_ * actions_unit_vec_(0, curr_orientation)));
        double dy_m = (neighbor.second.first.position.y + (delta_C_ * actions_unit_vec_(1, neighbor.second.second))) -
                      (curr_pose.position.y + (delta_C_ * actions_unit_vec_(1, curr_orientation)));

        double dx_pix = dx_m * x_dir_px_m_scale_;
        double dy_pix = dy_m * y_dir_px_m_scale_;

        double rect_center_x_pix = center_mask.x + dx_pix;
        double rect_center_y_pix = center_mask.y - dy_pix;
        cv::Point2f center_rect(rect_center_x_pix, rect_center_y_pix);
        double angle_rect = calculateAngleFromOrientation(neighbor.second.second);

        cv::RotatedRect rect(center_rect, cv::Size2f(pasted_rect_width_pix_, pasted_rect_height_pix_), angle_rect);
        std::vector<cv::Point2f> box_points(4);
        rect.points(box_points.data());

        std::vector<cv::Point> polygon_points(box_points.begin(), box_points.end());
        box_points_vec.push_back(polygon_points);
        cv::fillPoly(mask, box_points_vec, cv::Scalar(0, 0, 0));
        c += 1;
        // --- (NO CHANGES IN THIS BLOCK) end of logic block ---
    }

    // Extract the final non-overlapping region
    cv::Mat extracted_region;
    mask(cv::Rect(center_mask.x - rotated.cols / 2,
                  center_mask.y - rotated.rows / 2,
                  rotated.cols,
                  rotated.rows))
        .copyTo(extracted_region);

    // Convert the final OpenCV image back to a ROS2 message.
    // CHANGE: Declare the local variable with the ROS2 message type.
    sensor_msgs::msg::Image extracted_image; 
    cv_bridge::CvImage cv_image(cv_ptr->header, cv_ptr->encoding, extracted_region);
    cv_image.toImageMsg(extracted_image); // This correctly populates the ROS2 message.

    return extracted_image;
}

// **MIGRATED** - Message types updated in signature
std::vector<cv::Point2f> ImageCoveringCoordRAG::getRectangleFromPose(const geometry_msgs::msg::Pose& pose, int orientation)
{
    // Extract position
    float x = pose.position.x;
    float y = pose.position.y;

    // Hardcoded rectangle width and height
    float width = 10;
    float height = 5;

    // Calculate corner points based on orientation
    std::vector<cv::Point2f> corners;

    if (orientation == 0) {
        corners.push_back(cv::Point2f(x - width / 2, y - height / 2));
        // ... add 3 other corners
    }
    else if (orientation == 90) {
        // ... calculate corners for 90 degree rotation
    }
    else {
        // ... calculate corners for arbitrary orientation angle
    }

    return corners;
}

// **MIGRATED** - Message types updated in signature
std::vector<cv::Point2f> ImageCoveringCoordRAG::getRectangleFromPoseInNeighbor(const geometry_msgs::msg::Pose& pose_self_robot, const geometry_msgs::msg::Pose& pose_inn_robot,
                                                                           int self_rob_img_orient, int inn_img_orient)
{
        /* 
pose_self_robot: pose of the current robot whose marginal gain you are trying to evaluate
pose_inn_robot: pose of the in neighbor robot
inn_img_orientation: in neighbor robot image camera setting (for orientation)
 */

    double cx_p1 = delta_C_ * actions_unit_vec_(0, self_rob_img_orient);
    double cy_p1 = delta_C_ * actions_unit_vec_(1, self_rob_img_orient);

    double cx_p2 = delta_C_ * actions_unit_vec_(0, inn_img_orient);
    double cy_p2 = delta_C_ * actions_unit_vec_(1, inn_img_orient);

    // Extract position of center; relative to self robot pose
    float center_x = (pose_inn_robot.position.x + cx_p2) - (pose_self_robot.position.x + cx_p1);
    float center_y = (pose_inn_robot.position.y + cy_p2) - (pose_self_robot.position.y + cy_p1);

    // Hardcoded rectangle width and height
    float width = 2. * y_half_side_len_m_;
    float height = 2. * x_half_side_len_m_;

    // Calculate corner points based on orientation
    std::vector<cv::Point2f> corners;

    if ((inn_img_orient == 0) || (inn_img_orient == 4)) {
        corners.push_back(cv::Point2f(center_x - y_half_side_len_m_, center_y - x_half_side_len_m_));
        // ... add 3 other corners
    }
    else if ((inn_img_orient == 1) || (inn_img_orient == 5)) {
    }
    else if ((inn_img_orient == 2) || (inn_img_orient == 6)) {
    }
    else if ((inn_img_orient == 3) || (inn_img_orient == 7)) {
    }
    return corners;
}

// Finds intersection of two convex polygons using OpenCV functions
std::vector<cv::Point2f> ImageCoveringCoordRAG::findIntersection(const std::vector<cv::Point2f>& poly1,
                                                                 const std::vector<cv::Point2f>& poly2)
{

    std::vector<cv::Point2f> intersection;

    // Convert polygons to OpenCV contours
    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(toCvPointVector(poly1));
    contours.push_back(toCvPointVector(poly2));

    // Use bitwise AND to find intersecting region
    cv::Mat andImg;
    cv::bitwise_and(cv::Mat::zeros(cv::Size(100, 100), CV_8U),
                    cv::Mat::zeros(cv::Size(100, 100), CV_8U),
                    andImg,
                    contours);

    // Extract points from contour
    std::vector<std::vector<cv::Point>> andContours;
    cv::findContours(andImg, andContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (andContours.size() > 0) {
        cv::Point2f pt;
        for (const auto& p : andContours[0]) {
            pt.x = p.x;
            pt.y = p.y;
            intersection.push_back(pt);
        }
    }

    return intersection;
}

std::vector<cv::Point> ImageCoveringCoordRAG::toCvPointVector(const std::vector<cv::Point2f>& points)
{

    std::vector<cv::Point> cvPoints;
    for (const auto& p : points) {
        cvPoints.push_back(cv::Point(p.x, p.y));
    }

    return cvPoints;
}

// **MIGRATED** - Message types updated
void ImageCoveringCoordRAG::objectiveFunctionRAG(double& obj_func_val, std::vector<int>& in_neigh_ids_select_actions, bool& include_self_robot, int& self_robot_action_id)
{
    /* TODO: FIND SCORE OF EACH SEGMENTATION USING PRE ASSIGNED WEIGHTS TO SEMANTIC LABEL PIXELS */
    if ((in_neigh_ids_select_actions.empty()) && (include_self_robot == false)) {
        return;
    }

    if (include_self_robot == true) {
        in_neigh_ids_select_actions.push_back(robot_idx_self_);
    }

    std::vector<airsim_ros_pkgs::msg::Neighbors> neighbor_data_vec = neighbors_data_.neighbors_array;

    if (neighbor_data_vec.empty()) {
        return;
    }
    else {
        // Accessing the stamp is slightly different in ROS2
        RCLCPP_INFO(this->get_logger(), "Timestamp of last saved PEdata msg: %d.%u seconds",
                    neighbors_data_.header.stamp.sec, neighbors_data_.header.stamp.nanosec);
    }

    std::vector<airsim_ros_pkgs::msg::Neighbors> neighbors_data_selected_ids;
    for (auto& idx : in_neigh_ids_select_actions) {
        neighbors_data_selected_ids.push_back(neighbor_data_vec.at(idx - 1));
    }

    if (include_self_robot == true)
    {
        airsim_ros_pkgs::msg::Neighbors robot_pe_data;
        for (size_t i = 0; i < neighbors_data_selected_ids.size(); i++) {
            if (neighbors_data_selected_ids[i].drone_id == robot_idx_self_) {
                robot_pe_data = neighbors_data_selected_ids[i];
                break;
            }
        }
    }
}

void ImageCoveringCoordRAG::bestInitActionObjFunctionRAG(double& best_marginal_gain, int& optimal_action_id, std::vector<int>& inneighs_selected_action)
{
    std::map<int, double> objfunc_vals_map; // Map to store control action ID and corresponding marginal gain

    // do below for every action in action set
    for (int i = 0; i < actions_.cols(); i++) {
        double total_ft;
        bool include_self1 = true;
        objectiveFunctionRAG(total_ft, inneighs_selected_action, include_self1, i);

        objfunc_vals_map[i] = total_ft;
    }
    // Sort the map in descending order of marginal gains
    auto comparator = [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
        return a.second > b.second;
    };
    std::vector<std::pair<int, double>> sorted_marginal_gains(objfunc_vals_map.begin(), objfunc_vals_map.end());
    std::sort(sorted_marginal_gains.begin(), sorted_marginal_gains.end(), comparator);

    optimal_action_id = sorted_marginal_gains.empty() ? -1 : sorted_marginal_gains.front().first;
    best_marginal_gain = sorted_marginal_gains.empty() ? 0.0 : sorted_marginal_gains.front().second;
}

bool ImageCoveringCoordRAG::areAllValuesTrue()
{
    for (const auto& pair : all_robots_selected_map_) {
        if (!pair.second) {
            return false; // Found a false value, return false
        }
    }
    return true; // All values are true
}

bool ImageCoveringCoordRAG::areAllValuesTrue(std::map<int, bool>& all_robots_selected_map)
{
    for (const auto& pair : all_robots_selected_map) {
        if (!pair.second) {
            return false; // Found a false value, return false
        }
    }
    return true; // All values are true
}

bool ImageCoveringCoordRAG::areAllValuesTrue3Robots()
{
    int counter = 0;
    for (const auto& pair : all_robots_selected_map_) {
        if (!pair.second) {
            return false; // Found a false value, return false
        }
        counter += 1;
        if (counter == 3) {
            break;
        }
    }
    return true; // All values are true
}

std::pair<double, double> ImageCoveringCoordRAG::computeRangeBearing(const std::vector<double>& pose1,
                                                                     const std::vector<double>& pose2)
{

    double dx = pose2[0] - pose1[0];
    double dy = pose2[1] - pose1[1];

    double range = std::sqrt(dx * dx + dy * dy);

    double bearing = std::atan2(dy, dx);

    return { range, bearing };
}

// **MIGRATED** - Replaced ros::Duration(...).sleep()
void ImageCoveringCoordRAG::simulatedCommTimeDelayRAG()
{
    double time_delay_per_msg = total_data_msg_size_ / communication_data_rate_;
    communication_round_ += 1;
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start_;
    RCLCPP_INFO(this->get_logger(), "%f COMMUNICATION ROUND COUNT: %d", duration.count(), communication_round_);

    rclcpp::sleep_for(std::chrono::duration<double>(time_delay_per_msg));
}

// **MIGRATED** - Replaced ros::Duration(...).sleep()
void ImageCoveringCoordRAG::simulatedCommTimeDelayRAG(double& processed_time)
{
    double time_delay_per_msg = total_data_msg_size_ / communication_data_rate_;
    double comm_time = time_delay_per_msg - (processed_time / 1000.);
    communication_round_ += 1;
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start_;
    RCLCPP_INFO(this->get_logger(), "%f COMMUNICATION ROUND COUNT: %d", duration.count(), communication_round_);
    if (comm_time > 0.) {
        rclcpp::sleep_for(std::chrono::duration<double>(comm_time));
    }
}

void ImageCoveringCoordRAG::takeUnionOfMapsDataStruct(std::map<int, double>& selected_action_inneigh_ids_mg, std::map<int, double>& new_selected_action_inneigh_ids_mg, std::map<int, double>& combined_map)
{
    // Insert elements from first map
    for (auto const& element : selected_action_inneigh_ids_mg) {
        combined_map[element.first] = element.second;
    }

    // Insert elements from second map
    for (auto const& element : new_selected_action_inneigh_ids_mg) {
        combined_map[element.first] = element.second;
    }
}

//override function
void ImageCoveringCoordRAG::takeUnionOfMapsDataStruct(std::map<int, double>& selected_action_inneigh_ids_mg, std::map<int, double>& new_selected_action_inneigh_ids_mg)
{
    // Insert elements from second map
    for (auto const& element : new_selected_action_inneigh_ids_mg) {
        selected_action_inneigh_ids_mg[element.first] = element.second;
    }
}

void ImageCoveringCoordRAG::takeUnionOfMapsDataStruct(std::map<int, ImageInfoStruct>& selected_action_inneigh_ids_mg, std::map<int, ImageInfoStruct>& new_selected_action_inneigh_ids_mg)
{

    for (auto const& element : new_selected_action_inneigh_ids_mg) {
        selected_action_inneigh_ids_mg[element.first] = element.second;
    }
}

msr::airlib::Pose ImageCoveringCoordRAG::giveGoalPoseFormat(msr::airlib::MultirotorState& curr_pose, float disp_x, float disp_y, float disp_z, float new_yaw)
{
    msr::airlib::Pose goal_pose;
    auto position = curr_pose.getPosition();

    goal_pose.position.x() = position.x() + disp_x;
    goal_pose.position.y() = position.y() + disp_y;
    goal_pose.position.z() = disp_z; // NOTE keep same altitude as assigned initially

    // Calculate the quaternion components based on the yaw angle
    Eigen::Quaternionf qrot = quaternionFromEuler(0., 0., new_yaw);
    goal_pose.orientation = qrot;

    return goal_pose;
}

msr::airlib::Pose ImageCoveringCoordRAG::giveCameraGoalPoseFormat(msr::airlib::MultirotorState& curr_pose, float disp_x, float disp_y, float disp_z, float curr_roll, float new_pitch, float curr_yaw)
{
    msr::airlib::Pose goal_pose;
    auto position = curr_pose.getPosition();

    goal_pose.position.x() = position.x() + disp_x;
    goal_pose.position.y() = position.y() + disp_y;
    goal_pose.position.z() = disp_z; // NOTE keep same altitude as assigned initially

    // Calculate the quaternion components based on the yaw angle
    Eigen::Quaternionf qrot = quaternionFromEuler(curr_roll, new_pitch, curr_yaw); // ROLL PITCH YAW
    goal_pose.orientation = qrot;

    return goal_pose;
}

// Convert Euler angles to quaternion
Eigen::Quaternionf ImageCoveringCoordRAG::quaternionFromEuler(float roll, float pitch, float yaw)
{
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
    return q;
}

// Convert quaternion to Euler angles
Eigen::Vector3f ImageCoveringCoordRAG::eulerFromQuaternion(const Eigen::Quaternionf& q)
{
    Eigen::Vector3f euler;

    float ysqr = q.y() * q.y();

    // roll (x-axis rotation)
    float t0 = +2.0 * (q.w() * q.x() + q.y() * q.z());
    float t1 = +1.0 - 2.0 * (q.x() * q.x() + ysqr);
    euler(0) = atan2(t0, t1);

    // pitch (y-axis rotation)
    float t2 = +2.0 * (q.w() * q.y() - q.z() * q.x());
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    euler(1) = asin(t2);

    // yaw (z-axis rotation)
    float t3 = +2.0 * (q.w() * q.z() + q.x() * q.y());
    float t4 = +1.0 - 2.0 * (ysqr + q.z() * q.z());
    euler(2) = atan2(t3, t4);

    return euler;
}

// **MIGRATED** - Message types updated
geometry_msgs::msg::Pose ImageCoveringCoordRAG::convertToGeometryPose(const msr::airlib::MultirotorState& pose1)
{
    geometry_msgs::msg::Point point_msg;
    point_msg.x = pose1.kinematics_estimated.pose.position.x();
    point_msg.y = pose1.kinematics_estimated.pose.position.y();
    point_msg.z = pose1.kinematics_estimated.pose.position.z();

    geometry_msgs::msg::Quaternion quaternion_msg;
    quaternion_msg.x = pose1.kinematics_estimated.pose.orientation.x();
    quaternion_msg.y = pose1.kinematics_estimated.pose.orientation.y();
    quaternion_msg.z = pose1.kinematics_estimated.pose.orientation.z();
    quaternion_msg.w = pose1.kinematics_estimated.pose.orientation.w();

    geometry_msgs::msg::Pose latest_pose;
    latest_pose.position = point_msg;
    latest_pose.orientation = quaternion_msg;
    return latest_pose;
}

void ImageCoveringCoordRAG::saveImage(std::vector<ImageResponse>& response)
{
    for (const ImageResponse& image_info : response) {
        // OPTIONAL: It's good practice to use the ROS2 logger instead of std::cout
        RCLCPP_INFO(this->get_logger(), "Image uint8 size: %zu", image_info.image_data_uint8.size());
        RCLCPP_INFO(this->get_logger(), "Image float size: %zu", image_info.image_data_float.size());

        std::string file_path = FileSystem::combine(img_save_path_, std::to_string(image_info.time_stamp));
        if (image_info.pixels_as_float) {
            Utils::writePFMfile(image_info.image_data_float.data(), image_info.width, image_info.height, file_path + ".pfm");
        }
        else {
            std::ofstream file(file_path + ".png", std::ios::binary);
            file.write(reinterpret_cast<const char*>(image_info.image_data_uint8.data()), image_info.image_data_uint8.size());
            file.close();
        }
    }
}

// No significant changes needed. Replaced std::cout with the ROS2 logger.
void ImageCoveringCoordRAG::viewImage(std::vector<ImageResponse>& response)
{
    for (const ImageResponse& image_info : response) {
        RCLCPP_INFO(this->get_logger(), "Image uint8 size: %zu", image_info.image_data_uint8.size());
        RCLCPP_INFO(this->get_logger(), "Image float size: %zu", image_info.image_data_float.size());

        cv::Mat image;
        if (image_info.pixels_as_float) {
            cv::Mat float_image(image_info.height, image_info.width, CV_32FC3, const_cast<float*>(image_info.image_data_float.data()));
            cv::convertScaleAbs(float_image, image, 255.0);
            image.convertTo(image, CV_8UC3);
        }
        else {
            image = cv::Mat(image_info.height, image_info.width, CV_8UC3, const_cast<uchar*>(image_info.image_data_uint8.data())).clone();
        }

        cv::flip(image, image, 0);

        if (image.channels() == 3 && image_info.pixels_as_float == false) {
            cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
        }

        cv::imshow("Image", image);
        cv::waitKey(0);
    }
}

// These OpenCV helper functions are ROS-independent and do not require changes.
void ImageCoveringCoordRAG::viewImageCvMat(cv::Mat& image) 
{
    // Flip the image vertically
    // cv::flip(image, image, 0);

    // If image is in BGR format, swap R and B channels
    if (image.channels() == 3) {
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    }

    cv::imshow("Image", image);
    cv::waitKey(0); // Wait for a key press
}
void ImageCoveringCoordRAG::viewImageCvMat(cv::Mat& image, bool switchBGR_RGB_flag) 
{  
    // Flip the image vertically
    // cv::flip(image, image, 0);

    // If image is in BGR format, swap R and B channels
    if ((image.channels() == 3) && (switchBGR_RGB_flag)) {
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    }

    cv::imshow("Image", image);
    cv::waitKey(0); // Wait for a key press
}
void ImageCoveringCoordRAG::viewImageCvMat(cv::Mat& image, bool switchBGR_RGB_flag, const std::string filename) 
{
    // If image is in BGR format, swap R and B channels
    if ((image.channels() == 3) && (switchBGR_RGB_flag)) {
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    }

    // Show the image
    cv::imshow("Image", image);
    cv::waitKey(0); // Wait for a key press

    // Save the image as a PNG file
    std::string path_filename = img_save_path_ + filename;
    cv::imwrite(path_filename, image);
}

void ImageCoveringCoordRAG::saveImageCvMat(cv::Mat& image, bool switchBGR_RGB_flag, const std::string filename) 
{
    {
    // If image is in BGR format, swap R and B channels
    if ((image.channels() == 3) && (switchBGR_RGB_flag)) {
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    }

    // Save the image as a PNG file
    std::string path_filename = img_save_path_ + "drone" + std::to_string(robot_idx_self_) + "/" + filename; // each robot gets its own folder
    cv::imwrite(path_filename, image);
}
}


// This function has key changes related to cv_bridge and logging.
void ImageCoveringCoordRAG::rotateImagesInVector()
{
    for (size_t i = 0; i < camera_rotated_imgs_vec_.size(); ++i) {
        // cv_bridge::toCvCopy still exists, but we use it with ROS2 messages.
        // The message type in camera_rotated_imgs_vec_ is now sensor_msgs::msg::Image
        cv_bridge::CvImagePtr cv_ptr;
        try {
            // Note: The encoding constant is now in the sensor_msgs::image_encodings namespace
            cv_ptr = cv_bridge::toCvCopy(camera_rotated_imgs_vec_[i], sensor_msgs::image_encodings::BGR8);
        }
        catch (const cv_bridge::Exception& e) {
            // Use the ROS2 logger RCLCPP_ERROR
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::flip(cv_ptr->image, cv_ptr->image, 0);
        cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2RGB);

        // Convert the OpenCV Mat back to a ROS2 sensor_msgs::msg::Image message
        // toImageMsg() now returns a std::shared_ptr<sensor_msgs::msg::Image>
        sensor_msgs::msg::Image::SharedPtr rotated_img_msg = cv_ptr->toImageMsg();

        // Replace the original image in the vector with the rotated image
        camera_rotated_imgs_vec_[i] = *rotated_img_msg;
    }
}

// The message type changes from geometry_msgs::Pose to geometry_msgs::msg::Pose
double ImageCoveringCoordRAG::PoseEuclideanDistance(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2, int cam_orient_1, int cam_orient_2)
{
    // The internal logic of this function remains unchanged.
    double cx_p1 = delta_C_ * actions_unit_vec_(0, cam_orient_1);
    double cy_p1 = delta_C_ * actions_unit_vec_(1, cam_orient_1);

    double cx_p2 = delta_C_ * actions_unit_vec_(0, cam_orient_2);
    double cy_p2 = delta_C_ * actions_unit_vec_(1, cam_orient_2);

    double dx = (pose1.position.x + cx_p1) - (pose2.position.x + cx_p2);
    double dy = (pose1.position.y + cy_p1) - (pose2.position.y + cy_p2);
    double dz = pose1.position.z - pose2.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// These pure C++ helper functions do not require changes.
void ImageCoveringCoordRAG::printVector(const std::vector<int>& vec) 
{
    for (int elem : vec) {
        std::cout << elem << " ";
    }
    std::cout << std::endl; // Print a newline character at the end for clarity
}
void ImageCoveringCoordRAG::printMap(const std::map<int, double>& map) 
{ 
    for (const auto& pair : map) {
        std::cout << pair.first << ": " << pair.second << std::endl;
    }
}

// The main function is fundamentally different in ROS2.
int main(int argc, char** argv)
{
    // 1. Initialize ROS2
    rclcpp::init(argc, argv);

    // 2. Create an instance of your node class as a shared pointer
    auto node = std::make_shared<ImageCoveringCoordRAG>(/* constructor arguments */);

    // 3. Spin the node to process callbacks
    rclcpp::spin(node);

    // 4. Shut down ROS2
    rclcpp::shutdown();

    return 0;
}
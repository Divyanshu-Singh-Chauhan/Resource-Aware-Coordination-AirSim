/*
NOTE: This script contains the algorithm for RAG
 */
#include "image_covering_coordination/src/image_covering_coord_SG.hpp"
 
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "image_covering_coordination/msg/image_covering.hpp"
 
using std::placeholders::_1;

ImageCoveringCoordSG::ImageCoveringCoordSG(ros::NodeHandle& nh)
    : node_handle_(nh)
{
    // Get ROS params
    declare_parameter<std::string>("drone_name", "");
    declare_parameter<int>("number_of_drones", 1);
    declare_parameter<int>("robot_index_ID", 0);
    declare_parameter<std::string>("camera_name", ""); // need to set default here if not mentioned
    declare_parameter<double>("communication_range", 0.0); // need to set default here if not mentioned
    declare_parameter<std::string>("algorithm_to_run", "SG_LP");
    declare_parameter<int>("random_seed", 0); // Default to 0 if not set
    declare_parameter<bool>("is_server_experiment", false); // treat as bool; 0 is false, 1 is true

    get_parameter("drone_name",         drone_name_self_);
    get_parameter("number_of_drones",   num_robots_);
    get_parameter("robot_index_ID",     robot_idx_self_);
    get_parameter("camera_name",        camera_name_);
    get_parameter("communication_range",communication_range_);
    get_parameter("algorithm_to_run",   algorithm_to_run_);
    get_parameter("random_seed",        random_seed_);
    get_parameter("is_server_experiment", is_server_experiment_);

    RCLCPP_INFO(this->get_logger(), "ROBOT_SELF_ID: %d", robot_idx_self_);
    RCLCPP_INFO(this->get_logger(), "ALGORITHM: %s", algorithm_to_run_.c_str());

    // SET SG ALG GRAPH TOPOLOGY************
    graph_topology_SG_ = algorithm_to_run_; // can be either SG_LP or SG_DFS
    // graph_topology_SG_ = "line_path";
    // graph_topology_SG_ = "DFS";
    // SET SG ALG GRAPH TOPOLOGY************end

    // async_spinner_.reset(new ros::AsyncSpinner(3));
    // async_spinner_->start();

    // ROS publishers
    std::string mg_msg_topic = drone_name_self_ + "/mg_rag_data";
    mg_rag_msg_pub_ = this->create_publisher<image_covering_coordination::msg::ImageCovering>(
      mg_msg_topic,
      rclcpp::QoS(5)
    );

    // ROS subscribers
    // NOTE!!! Subscribe to /clock topic for putting a callback which checks whether n_time_step_ amount of time has passed
    clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::QoS(1),
        std::bind(&ImageCoveringCoordSG::clockCallback, this, _1)
      );
    
    marginal_gain_pub_clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::QoS(1),
        std::bind(&ImageCoveringCoordSG::marginalGainPublisherclockCallback, this, _1)
    );
    
    check_drones_selection_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::QoS(1),
        std::bind(&ImageCoveringCoordSG::updateAllRobotsSelectionStatusCallback, this, _1)
    );
    // DATA TRANSMISSION RATE SETTINGS
    // total_data_msg_size_ = 1024.; // depth frame+pose will be 1024 KB
    // communication_data_rate_ = 10. * total_data_msg_size_; // KB/s;
    // total_data_msg_size_ = 200.;
    // communication_data_rate_ = 0.25 * total_data_msg_size_; // KB/s; multiplying factor
    total_data_msg_size_ = 200.;
    communication_data_rate_ = 100. * total_data_msg_size_; // KB/s; multiplying factor

    communication_round_ = 0;

    // set actions x,y,theta. Note x,y are independent of theta
    flight_level_ = 0.5;
    // altitude_flight_level_ = -40. - 0.5 * (num_robots_ * flight_level_) + double(robot_idx_self_) * flight_level_;
    altitude_flight_level_ = -30.; // altitude for image covering will be slightly lower

    selected_action_ind_ = -1; // initialized to be -1 indicating no action has been selected yet
    best_marginal_gain_self_ = -1.;
    finished_action_selection_ = false;
    waiting_for_robots_to_finish_selection_ = false;
    takeoff_wait_flag_ = true;
    terminate_decision_making_ = false;

    fov_y_ = M_PI / 2.; // in rads; 90 deg horizontal fov
    fov_x_ = 1.093; // in rads; 62.6 deg vertical fov

    actions_.resize(3, 8);
    actions_ << 10., 7.07, 0., -7.07, -10., -7.07, 0., 7.07,
        0., 7.07, 10., 7.07, 0., -7.07, -10., -7.07,
        0., 0., 0., 0., 0., 0., 0., 0.;

    actions_unit_vec_.resize(3, 8);
    actions_unit_vec_ << 1., 0.707, 0., -0.707, -1., -0.707, 0., 0.707,
        0., 0.707, 1., 0.707, 0., -0.707, -1., -0.707,
        0., 0., 0., 0., 0., 0., 0., 0.;

    control_action_disp_scale_ = 1.5;

    objective_function_value_ = -1.;
    PEdata_new_msg_received_flag_ = false;

    // consts initialization
    int n_time_step_init = 0;
    horizon_ = 100;
    n_time_steps_ = 2000;
    double dT = horizon_ / n_time_steps_;
    curr_time_step_samp_ = 0; // this is "t" passed into expstarsix alg; initially 0

    reward_ = Eigen::MatrixXd(int(n_time_steps_), 1);

    // initialize Airsim client
    // AIRSIM CLIENT USAGE CONSTANTS
    client_timeout_sec_ = (3.4028235E38F);
    msr::airlib::YawMode client_default_yaw_mode_ = msr::airlib::YawMode();
    client_lookahead_ = (-1.0F);
    client_adaptive_lookahead_ = (1.0F);
    client_margin_ = (5.0F);
    yaw_rate_timestep_ = 1.0; // will use at most 1.5 seconds to reach goal yaw state
    drone_linear_velocity_ = 3.0;
    drone_lin_vel_factor_ = 1.33;
    yaw_duration_ = 0.;
    yaw_rate_ = 2.36; // rad/s

    // CAMERA SETTINGS*******
    camera_pitch_theta_ = 0.2182; // 12.5 deg
    camera_downward_pitch_angle_ = -1.571; // DO NOT CHANGE! -90 deg
    current_image_requests_vec_ = { ImageRequest("front_center", msr::airlib::ImageCaptureBase::ImageType::Segmentation, false, false),
                                    ImageRequest("front_center", msr::airlib::ImageCaptureBase::ImageType::Scene, false, false) };
    camera_image_type_ = msr::airlib::ImageCaptureBase::ImageType::Segmentation;

    // NOTE!!! below settings are for 62.7deg x 90 deg FOV, 10 robots!!!
    /* y_half_side_len_m_ = 30.; //m
    x_half_side_len_m_ = 18.25; //m
    y_dir_px_m_scale_ = 10.67; // pixels per m in the y dir
    x_dir_px_m_scale_ = 13.15; // pixels per m in the x dir
    delta_C_ = 10.44; // m, displacement in center due to viewing angle when delta = 0.2182 rads
    diagonal_frame_dist_ = sqrt(pow((2. * y_half_side_len_m_), 2) + pow((2. * x_half_side_len_m_), 2)); // + 2.*delta_C_;  // shortest distance needed to consider overlap from in-neighbors
    diagonal_px_scale_ = sqrt(pow(y_dir_px_m_scale_, 2) + pow(x_dir_px_m_scale_, 2));
    pasted_rect_height_pix_ = 513; 
    pasted_rect_width_pix_ = 640;
    take_new_pics_ = true;
    road_pixel_weight_= 1.; */

    // NOTE!!! below settings are for 46.83deg x 60 deg FOV, 10 robots and 15 robots!!!
    fov_y_ = M_PI / 3.; // in rads; 60 deg horizontal fov
    fov_x_ = 0.817; // in rads; 46.83 deg vertical fov
    y_half_side_len_m_ = 17.32; //m
    x_half_side_len_m_ = 13.; //m
    y_dir_px_m_scale_ = 18.474; // pixels per m in the y dir NOTE this is for unstretched frame
    x_dir_px_m_scale_ = 18.48; // pixels per m in the x dir
    delta_C_ = 21.94; // m, displacement in center due to viewing angle when delta = 0.2182 rads
    // delta_C_ = 0.;
    pasted_rect_height_pix_ = 683;
    pasted_rect_width_pix_ = 640;
    diagonal_frame_dist_ = sqrt(pow((2. * y_half_side_len_m_), 2) + pow((2. * x_half_side_len_m_), 2)); // + 2.*delta_C_;  // shortest distance needed to consider overlap from in-neighbors
    diagonal_px_scale_ = sqrt(pow(y_dir_px_m_scale_, 2) + pow(x_dir_px_m_scale_, 2));
    take_new_pics_ = true;
    road_pixel_weight_ = 1.;
    // CAMERA SETTINGS*******end

    client_drivetrain_ = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    client_.confirmConnection(); // may need to put this in callback
    client_.enableApiControl(true, drone_name_self_);
    client_.armDisarm(true);

    for (int i = 0; i < num_robots_; i++) {
        all_robots_selected_map_[i + 1] = false; // initialized all to be false, no drone has initially selected an action
    }

    start_ = std::chrono::high_resolution_clock::now();
    camera_rotated_imgs_vec_ = std::vector<sensor_msgs::Image>(8); // initialize with 8 image objects first, then overwrite
    best_image_cam_orientation_ = -1; // just for initialization
    img_save_path_ = "/home/sgari/sandilya_ws/unity_airsim_workspace/AirSim/ros/src/image_covering_coordination/data/";
}

// destructor
ImageCoveringCoordSG::~ImageCoveringCoordSG() = default;

void ImageCoveringCoordSG::marginalGainPublisherclockCallback(
    const rosgraph_msgs::msg::Clock::SharedPtr /*msg*/)
{
    /* PUBLISHES best marginal gain value */
    msr::airlib::MultirotorState pose1 = client_.getMultirotorState(drone_name_self_);
    geometry_msgs::Pose latest_pose = convertToGeometryPose(pose1);

    // access last set best marginal gain from clock callback and continuously publish it
    image_covering_coordination::ImageCovering imcov_mg_msg;

    imcov_mg_msg.header.stamp = this->get_clock()->now();
    imcov_mg_msg.drone_id = robot_idx_self_;
    imcov_mg_msg.image = best_self_image_; // NOTE!!! Not publishing real image
    imcov_mg_msg.camera_orientation_setting = best_image_cam_orientation_;
    imcov_mg_msg.pose = latest_pose;
    imcov_mg_msg.flag_completed_action_selection = finished_action_selection_; // set this to false as soon as you begin planning stage
    imcov_mg_msg.best_marginal_gain = best_marginal_gain_self_;
    // imcov_mg_msg.terminate_decision_making_flag = terminate_decision_making_;

    mg_rag_msg_pub_->publish(imcov_mg_msg);

    RCLCPP_INFO(this->get_logger(),
                "Published mg_rag msg: gain=%.3f  completed=%s",
                imcov_mg_msg.best_marginal_gain,
                imcov_mg_msg.flag_completed_action_selection ? "true" : "false");
    // std::cout << "Published mg_rag msg with best marginal gain: " <<  imcov_mg_msg.best_marginal_gain << " and action selection flag: "<< std::to_string(imcov_mg_msg.flag_completed_action_selection) << std::endl;
}
//--------------------------------------------------------
void ImageCoveringCoordSG::updateAllRobotsSelectionStatusCallback(
    const rosgraph_msgs::msg::Clock::SharedPtr /*msg*/)
{
    /* SUBSCRIBES to marginal gain message which contains action selection flag;
    to check whether all robots have selected actions so that execution occurs simultaneously */

    auto timeout = rclcpp::Duration::from_seconds(3.0); // Timeout seconds
    for (int i = 0; i < num_robots_; i++) {
        int other_robot_id = i + 1;
        std::string mg_msg_topic = "Drone" + std::to_string(other_robot_id) + "/mg_rag_data";
        // std::cout << "MG MSG TOPIC: " << mg_msg_topic << std::endl;

        image_covering_coordination::msg::ImageCovering tmp_msg;
        bool got = rclcpp::wait_for_message<image_covering_coordination::msg::ImageCovering>(
          tmp_msg,
          this->shared_from_this(),
          mg_msg_topic,
          timeout
        );
        all_robots_selected_map_[other_robot_id] =
          got ? tmp_msg.flag_completed_action_selection : false;
    }
    RCLCPP_INFO(
        this->get_logger(),
        ""
      );
}

// Get wall clock time; algorithm continues from here
void ImageCoveringCoordSG::clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr /*msg*/)
{
    /* MAIN callback for SG decision making */

    // wait to proceed until drone has taken off
    if (takeoff_wait_flag_ == true) {
        client_.moveToZAsync(altitude_flight_level_, 5.0, client_timeout_sec_, client_default_yaw_mode_, client_lookahead_, client_adaptive_lookahead_, drone_name_self_)->waitOnLastTask();
        rclcpp::sleep_for(std::chrono::seconds(1));
        takeoff_wait_flag_ = false;

        // go to random location; when generating movements, each drone will
        // have its own sequence based on the modified seed
        srand(random_seed_ + robot_idx_self_);
        int direction = rand() % 4; // 0=right, 1=left, 2=forward, 3=backward
        double distance = 4.0; // Example fixed distance
        double dx = 0, dy = 0;
        switch (direction) {
        case 0:
            dx = distance;
            break; // Move forward
        case 1:
            dx = -distance;
            break; // Move backward
        case 2:
            dy = distance;
            break; // Move right
        case 3:
            dy = -distance;
            break; // Move left
        }
        double timespan_tomove = 2.67;
        client_.moveByVelocityBodyFrameAsync(dx / timespan_tomove, dy / timespan_tomove, 0., timespan_tomove, client_drivetrain_, client_default_yaw_mode_, drone_name_self_)->waitOnLastTask();
        rclcpp::sleep_for(std::chrono::seconds(1));

        return;
    }

    // get a single NeighborsArray message
    airsim_ros_pkgs::msg::NeighborsArray neighbors_data_msg;
    bool got_neighbors = rclcpp::wait_for_message<airsim_ros_pkgs::msg::NeighborsArray>(
        neighbors_data_msg,
        this->shared_from_this(),
        "/neighbors_data",
        std::chrono::duration<double>(3.0)
    );
    if (got_neighbors) {
        neighbors_data_ = neighbors_data_msg;
    }

    // get pose
    msr::airlib::MultirotorState pose1 = client_.getMultirotorState(drone_name_self_);
    geometry_msgs::Pose latest_pose = convertToGeometryPose(pose1);

    // for self robot, do as many rounds of RAG-H needed to find the actions each will execute. Program communication delays where needed.
    std::map<int, double> selected_action_precneigh_ids_mg; // int inneigh id : marginal gain value
    if ((finished_action_selection_ == false) && (waiting_for_robots_to_finish_selection_ == false)) // need to have flag here that indicates all robots have executed actions
    {
        // subscribe to the gain topic of preceding drone, while continuously publishing your own best gain

        int best_control_action_ind;
        double best_marginal_gain_self;
        bool got_prec_data = true;

        if ((graph_topology_SG_ == "SG_LP") || (graph_topology_SG_ == "line_path")) {
            marginalGainFunctionSG_LP(best_marginal_gain_self, best_control_action_ind, latest_pose, got_prec_data); // wrt only to in-neighbors that have selected actions
        }
        else if ((graph_topology_SG_ == "SG_DFS") || (graph_topology_SG_ == "DFS")) {
            marginalGainFunctionSG_DFS(best_marginal_gain_self, best_control_action_ind, latest_pose, got_prec_data); // wrt only to in-neighbors that have selected actions
        }

        if (got_prec_data == false) {
            return; // exit and wait for next message
        }

        best_marginal_gain_self_ = best_marginal_gain_self; // will publish this in the mg gain publisher callback thread

        if ((graph_topology_SG_ == "SG_LP") || (graph_topology_SG_ == "line_path")) {
            simulatedCommTimeDelaySG_LP(); // for use with line path graph
        }
        else if ((graph_topology_SG_ == "SG_DFS") || (graph_topology_SG_ == "DFS")) {
            // simulatedCommTimeDelaySG_DFSv2(); // for use with DFS
            simulatedCommTimeDelaySG_DFSv1();
        }

        finished_action_selection_ = true;
        RCLCPP_INFO(this->get_logger(), "my marginal gain is best");
        selected_action_ind_ = best_control_action_ind;
        take_new_pics_ = false; // for next round
    }

    // Once every drone has selected its action, execute this self robot's action. NOTE: We assume all robots execute actions at the same time,
    // until then they execute zero-order hold on their previously selected control action.
    Eigen::Matrix<double, 3, 1> selected_action_displacement = actions_.block(0, selected_action_ind_, 3, 1);
    last_selected_displacement_ = selected_action_displacement;

    // !!!CHECK if all drones have already selected actions
    bool all_robots_ready_to_proceed = areAllValuesTrue(); // NOTE!!! TEMPORARILY COMMENTED OUT FOR DEBUGGING
    if (all_robots_ready_to_proceed == true) {
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start_;
        RCLCPP_INFO(this->get_logger(), "%f ALL ROBOTS READY TO PROCEED WITH ACTION", duration.count());
        // execute selected action using airsim client and action table________
        // get current position, then add the x,y to it; ensure drone does not yaw
        auto position = pose1.getPosition();

        client_.moveToPositionAsync(position.x() + control_action_disp_scale_ * selected_action_displacement(0, 0),
                                    position.y() + control_action_disp_scale_ * selected_action_displacement(1, 0),
                                    altitude_flight_level_,
                                    4.,
                                    client_timeout_sec_,
                                    client_drivetrain_,
                                    client_default_yaw_mode_,
                                    client_lookahead_,
                                    client_adaptive_lookahead_,
                                    drone_name_self_)
            ->waitOnLastTask();

        client_.hoverAsync(drone_name_self_)->waitOnLastTask();
        rclcpp::sleep_for(std::chrono::seconds(1));

        finished_action_selection_ = false; // reset this flag for next timestep
        waiting_for_robots_to_finish_selection_ = false;
        take_new_pics_ = true;
    }
    else {
        waiting_for_robots_to_finish_selection_ = true;
    }

    curr_time_step_samp_++; // increment by 1 for each t in T
}

void ImageCoveringCoordSG::collectRotatedCameraImages()
{
    // ensure drone is hovering stably by yawing by small amount and yawing back
    msr::airlib::MultirotorState pose1 = client_.getMultirotorState(drone_name_self_);
    auto orientation1 = pose1.kinematics_estimated.pose.orientation;
    Eigen::Vector3f euler = eulerFromQuaternion(orientation1.cast<float>());
    float curr_roll1 = euler[0];
    float curr_yaw1 = euler[2];

    bool ignore_collision = true;
    client_.moveByRollPitchYawZAsync(0., 0., 0.1, -30., 1., drone_name_self_)->waitOnLastTask();

    // auto newpose_0 = giveGoalPoseFormat(pose1, 0., 0., 0., 0.);
    client_.moveByRollPitchYawZAsync(0., 0., 0., -30., 1., drone_name_self_)->waitOnLastTask();

    // ensure class priv vector is empty. take 8 images in total
    camera_rotated_imgs_vec_.clear();
    camera_rotated_imgs_vec_ = std::vector<sensor_msgs::msg::Image>(8);

    // first take picture in 0deg yaw orientation: 2 images, first forward and then backward camera pitch. Save to class priv vector
    // msr::airlib::Pose camera_pose_0 = giveCameraGoalPoseFormat(pose1, -0.021, 0.0112, altitude_flight_level_, camera_downward_pitch_angle_ + camera_pitch_theta_,
    //     curr_roll1, curr_yaw1);
    msr::airlib::Pose camera_pose_0 = giveCameraGoalPoseFormat(pose1, -0.021, 0.0112, altitude_flight_level_, curr_roll1, camera_downward_pitch_angle_ + camera_pitch_theta_, curr_yaw1);
    client_.simSetCameraPose("front_center", camera_pose_0, drone_name_self_); // set camera pose
    rclcpp::sleep_for(std::chrono::seconds(1)); // sleep to let camera change pose
    std::vector<ImageResponse> img_response_vec0 = client_.simGetImages(current_image_requests_vec_, drone_name_self_);

    // viewImage(img_response_vec0);
    sensor_msgs::msg::Image img0 = *get_img_msg_from_response(img_response_vec0.at(1));
    camera_rotated_imgs_vec_[0] = img0;

    msr::airlib::Pose camera_pose_1 = giveCameraGoalPoseFormat(pose1, -0.021, 0.0112, altitude_flight_level_, curr_roll1, camera_downward_pitch_angle_ - camera_pitch_theta_, curr_yaw1);
    client_.simSetCameraPose("front_center", camera_pose_1, drone_name_self_);
    rclcpp::sleep_for(std::chrono::seconds(1));
    std::vector<ImageResponse> img_response_vec1 =
      client_.simGetImages(current_image_requests_vec_, drone_name_self_);
    // viewImage(img_response_vec1);
    sensor_msgs::msg::Image img1 = *get_img_msg_from_response(img_response_vec1.at(1));
    camera_rotated_imgs_vec_[4] = img1;

    // _______________________________________________________________ 0 deg end

    // take picture in +45deg yaw orientation: 2 images, first forward and then backward camera pitch. Save to class priv vector
    client_.moveByRollPitchYawZAsync(0., 0., 0.7854, -30., 3., drone_name_self_)->waitOnLastTask();

    msr::airlib::MultirotorState pose2_new = client_.getMultirotorState(drone_name_self_);
    auto orientation2 = pose2_new.kinematics_estimated.pose.orientation;
    Eigen::Vector3f euler2 = eulerFromQuaternion(orientation2.cast<float>());
    float curr_roll2 = euler2[0];
    float curr_yaw2 = euler2[2];

    msr::airlib::Pose camera_pose_45 = giveCameraGoalPoseFormat(pose2_new, -0.021, 0.0112, altitude_flight_level_, curr_roll2, camera_downward_pitch_angle_ + camera_pitch_theta_, curr_yaw2);
    client_.simSetCameraPose("front_center", camera_pose_45, drone_name_self_); // set camera pose
    rclcpp::sleep_for(std::chrono::seconds(1)); // sleep to let camera change pose
    std::vector<ImageResponse> img_response_vec_45_0 = client_.simGetImages(current_image_requests_vec_, drone_name_self_);
    // viewImage(img_response_vec_45_0);
    sensor_msgs::msg::Image img45_0 = *get_img_msg_from_response(img_response_vec_45_0.at(1));
    camera_rotated_imgs_vec_[1] = img45_0;

    msr::airlib::Pose camera_pose_45_1 = giveCameraGoalPoseFormat(pose2_new, -0.021, 0.0112, altitude_flight_level_, curr_roll2, camera_downward_pitch_angle_ - camera_pitch_theta_, curr_yaw2);
    client_.simSetCameraPose("front_center", camera_pose_45_1, drone_name_self_); // set camera pose
    rclcpp::sleep_for(std::chrono::seconds(1)); // sleep to let camera change pose
    std::vector<ImageResponse> img_response_vec_45_1 = client_.simGetImages(current_image_requests_vec_, drone_name_self_);
    // viewImage(img_response_vec_45_1);
    sensor_msgs::msg::Image img45_1 = *get_img_msg_from_response(img_response_vec_45_1.at(1));
    camera_rotated_imgs_vec_[5] = img45_1;

    // _______________________________________________________________ 45 deg end

    // take picture in +90 deg yaw orientation: 2 images, first forward and then backward camera pitch. Save to class priv vector
    client_.moveByRollPitchYawZAsync(0., 0., 1.571, -30., 3., drone_name_self_)->waitOnLastTask();

    msr::airlib::MultirotorState pose3_new = client_.getMultirotorState(drone_name_self_);
    auto orientation3 = pose3_new.kinematics_estimated.pose.orientation;
    Eigen::Vector3f euler3 = eulerFromQuaternion(orientation3.cast<float>());
    float curr_roll3 = euler3[0];
    float curr_yaw3 = euler3[2];

    msr::airlib::Pose camera_pose_90 = giveCameraGoalPoseFormat(pose3_new, -0.021, 0.0112, altitude_flight_level_, curr_roll3, camera_downward_pitch_angle_ + camera_pitch_theta_, curr_yaw3);
    client_.simSetCameraPose("front_center", camera_pose_90, drone_name_self_); // set camera pose
    rclcpp::sleep_for(std::chrono::seconds(1)); // sleep to let camera change pose
    std::vector<ImageResponse> img_response_vec_90_0 = client_.simGetImages(current_image_requests_vec_, drone_name_self_);
    // viewImage(img_response_vec_90_0);
    sensor_msgs::msg::Image img90_0 = *get_img_msg_from_response(img_response_vec_90_0.at(1));
    camera_rotated_imgs_vec_[2] = img90_0;

    msr::airlib::Pose camera_pose_90_1 = giveCameraGoalPoseFormat(pose3_new, -0.021, 0.0112, altitude_flight_level_, curr_roll3, camera_downward_pitch_angle_ - camera_pitch_theta_, curr_yaw3);
    client_.simSetCameraPose("front_center", camera_pose_90_1, drone_name_self_); // set camera pose
    rclcpp::sleep_for(std::chrono::seconds(1)); // sleep to let camera change pose
    std::vector<ImageResponse> img_response_vec_90_1 = client_.simGetImages(current_image_requests_vec_, drone_name_self_);
    // viewImage(img_response_vec_90_1);
    sensor_msgs::msg::Image img90_1 = *get_img_msg_from_response(img_response_vec_90_1.at(1));
    camera_rotated_imgs_vec_[6] = img90_1;

    // _______________________________________________________________ 90 deg end

    // take picture in +135 deg yaw orientation: 2 images, first forward and then backward camera pitch. Save to class priv vector
    client_.moveByRollPitchYawZAsync(0., 0., 2.36, -30., 3., drone_name_self_)->waitOnLastTask();

    msr::airlib::MultirotorState pose4_new = client_.getMultirotorState(drone_name_self_);
    auto orientation4 = pose4_new.kinematics_estimated.pose.orientation;
    Eigen::Vector3f euler4 = eulerFromQuaternion(orientation4.cast<float>());
    float curr_roll4 = euler4[0];
    float curr_yaw4 = euler4[2];

    msr::airlib::Pose camera_pose_135 = giveCameraGoalPoseFormat(pose4_new, -0.021, 0.0112, altitude_flight_level_, curr_roll4, camera_downward_pitch_angle_ + camera_pitch_theta_, curr_yaw4);
    client_.simSetCameraPose("front_center", camera_pose_135, drone_name_self_); // set camera pose
    rclcpp::sleep_for(std::chrono::seconds(1)); // sleep to let camera change pose
    std::vector<ImageResponse> img_response_vec_135_0 = client_.simGetImages(current_image_requests_vec_, drone_name_self_);
    // viewImage(img_response_vec_135_0);
    sensor_msgs::msg::Image img135_0 = *get_img_msg_from_response(img_response_vec_135_0.at(1));
    camera_rotated_imgs_vec_[3] = img135_0;

    msr::airlib::Pose camera_pose_135_1 = giveCameraGoalPoseFormat(pose4_new, -0.021, 0.0112, altitude_flight_level_, curr_roll4, camera_downward_pitch_angle_ - camera_pitch_theta_, curr_yaw4);
    client_.simSetCameraPose("front_center", camera_pose_135_1, drone_name_self_); // set camera pose
    rclcpp::sleep_for(std::chrono::seconds(1)); // sleep to let camera change pose
    std::vector<ImageResponse> img_response_vec_135_1 = client_.simGetImages(current_image_requests_vec_, drone_name_self_);
    // viewImage(img_response_vec_135_1);
    sensor_msgs::msg::Image img135_1 = *get_img_msg_from_response(img_response_vec_135_1.at(1));
    camera_rotated_imgs_vec_[7] = img135_1;

    // rotate the images by 180 degree (equivalent to vertical flip)
    rotateImagesInVector();

    // rotate back to 0deg yaw world frame; reset camera
    client_.moveByRollPitchYawZAsync(0., 0., 0.0, -30., 3., drone_name_self_)->waitOnLastTask();
    msr::airlib::MultirotorState pose5_new = client_.getMultirotorState(drone_name_self_);
    auto orientation5 = pose5_new.kinematics_estimated.pose.orientation;
    Eigen::Vector3f euler5 = eulerFromQuaternion(orientation5.cast<float>());
    float curr_roll5 = euler5[0];
    float curr_yaw5 = euler5[2];
    msr::airlib::Pose camera_pose_reset = giveCameraGoalPoseFormat(pose5_new, -0.021, 0.0112, altitude_flight_level_, curr_roll5, camera_downward_pitch_angle_, curr_yaw5);
    client_.simSetCameraPose("front_center", camera_pose_reset, drone_name_self_);
    rclcpp::sleep_for(std::chrono::seconds(1)); // sleep to let camera change pose
}

sensor_msgs::ImagePtr ImageCoveringCoordSG::get_img_msg_from_response(const ImageResponse& img_response)
{
    sensor_msgs::ImagePtr img_msg_ptr = boost::make_shared<sensor_msgs::msg::Image>();
    img_msg_ptr->data = img_response.image_data_uint8;
    img_msg_ptr->step = img_response.width * 3; // AirSim gives RGBA images on linux;
    img_msg_ptr->header.stamp = airsim_timestamp_to_ros(img_response.time_stamp);
    // img_msg_ptr->header.frame_id = frame_id;
    img_msg_ptr->height = img_response.height;
    img_msg_ptr->width = img_response.width;
    // img_msg_ptr->encoding = "bgr8";
    // if (is_vulkan_)
    img_msg_ptr->encoding = "rgb8"; // vulkan drivers by default
    img_msg_ptr->is_bigendian = 0;
    return img_msg_ptr;
}

std::vector<double> ImageCoveringCoordSG::computeWeightedScores(const std::vector<sensor_msgs::msg::Image>& images)
{

    std::vector<double> scores;

    for (const auto& image : images) {

        double score = 0.0;

        // Get OpenCV Mat from image
        auto img_ptr = std::make_shared<sensor_msgs::msg::Image>(image);

        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(img_ptr, image.encoding);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 异常：%s", e.what());
            return scores;
        }
        cv::Mat img = cv_ptr->image;

        if (is_server_experiment_ == 1) {
            for (int r = 0; r < img.rows; ++r) {
                for (int c = 0; c < img.cols; ++c) {
                    cv::Vec3b pixel = img.at<cv::Vec3b>(r, c);

                    // Check if the pixel is not black
                    if (pixel != cv::Vec3b(0, 0, 0)) {
                        score += 1.0; // Increment the score for each non-black pixel
                    }
                }
            }
        }

        else {
            // Iterate through pixels
            for (int r = 0; r < img.rows; ++r) {
                for (int c = 0; c < img.cols; ++c) {

                    cv::Vec3b pixel = img.at<cv::Vec3b>(r, c);

                    // Check pixel color and add weighted score
                    if (pixel == cv::Vec3b(144, 9, 201)) { // road_1
                        score += road_pixel_weight_;
                    }
                    else if (pixel == cv::Vec3b(130, 9, 200)) { // road_2
                        score += road_pixel_weight_;
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

        scores.push_back(score);
    }
    return scores;
}

double ImageCoveringCoordSG::computeWeightedScoreSingleImage(const sensor_msgs::msg::Image & image, bool invertBGR2RGB)
{
    double score = 0.0;

    // Get OpenCV Mat from image
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image, image.encoding);
    }
    catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return score;
    }
    cv::Mat img = cv_ptr->image;

    if (invertBGR2RGB) {
        cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

        if (is_server_experiment_ == 1) {
            for (int r = 0; r < img.rows; ++r) {
                for (int c = 0; c < img.cols; ++c) {
                    cv::Vec3b pixel = img.at<cv::Vec3b>(r, c);

                    // Check if the pixel is not black
                    if (pixel != cv::Vec3b(0, 0, 0)) {
                        score += 1.0; // Increment the score for each non-black pixel
                    }
                }
            }
        }

        else {
            for (int r = 0; r < img.rows; ++r) {
                for (int c = 0; c < img.cols; ++c) {

                    cv::Vec3b pixel = img.at<cv::Vec3b>(r, c);

                    // for use with unity executable RGB!!!
                    // if (pixel == cv::Vec3b(144, 9, 201)) { // road_1
                    //     score += road_pixel_weight_;
                    // }
                    // else if (pixel == cv::Vec3b(130, 9, 200)) { // road_2
                    //     score += road_pixel_weight_;
                    // }
                    // // else if (pixel == cv::Vec3b(53, 73, 65)) { // pavement_1
                    // //     score += 0.05;
                    // // }
                    // // else if (pixel == cv::Vec3b(0, 72, 128)) { // pavement_2
                    // //     score += 0.15;
                    // // }

                    // for use with unity editor 46DEG X 60 DEG FOV, 10 drones!!!
                    // if ((pixel == cv::Vec3b(32, 97, 80))) { // road_1
                    //     score += road_pixel_weight_;
                    // }
                    // else if ((pixel == cv::Vec3b(176, 97, 24))) { // road_2
                    //     score += road_pixel_weight_;
                    // }
                    // // else if ((pixel == cv::Vec3b(0, 73, 194))) { // pavement_1
                    // //     score += 0.05;
                    // // }
                    // // else if ((pixel == cv::Vec3b(34, 97, 16))) { // pavement_2
                    // //     score += 0.05;
                    // // }

                    // NOTE!! for use with unity editor 46DEG X 60 DEG FOV, 15 drones!!!
                    // if ((pixel == cv::Vec3b(25, 97, 134))) { // road_1
                    //     score += road_pixel_weight_;
                    // }
                    // else if ((pixel == cv::Vec3b(16, 41, 4))) { // road_2
                    //     score += road_pixel_weight_;
                    // }
                    // else if ((pixel == cv::Vec3b(194, 73, 0))) { // pavement_1
                    //     score += 0.05;
                    // }
                    // else if ((pixel == cv::Vec3b(25, 97, 132))) { // pavement_2
                    //     score += 0.05;
                    // }

                    // NOTE!! for use with unity EXECUTABLE 46DEG X 60 DEG FOV, 15 drones!!!
                    if ((pixel == cv::Vec3b(128, 0, 50))) { // road_1
                        score += road_pixel_weight_;
                    }
                    else if ((pixel == cv::Vec3b(200, 73, 18))) { // road_2
                        score += road_pixel_weight_;
                    }
                    // else if ((pixel == cv::Vec3b(128, 9, 18))) { // pavement_1
                    //     score += 0.05;
                    // }
                    // else if ((pixel == cv::Vec3b(128, 72, 128))) { // pavement_2
                    //     score += 0.05;
                    // }
                }
            }
        }
    }

    else {

        if (is_server_experiment_ == 1) {
            for (int r = 0; r < img.rows; ++r) {
                for (int c = 0; c < img.cols; ++c) {
                    cv::Vec3b pixel = img.at<cv::Vec3b>(r, c);

                    // Check if the pixel is not black
                    if (pixel != cv::Vec3b(0, 0, 0)) {
                        score += 1.0; // Increment the score for each non-black pixel
                    }
                }
            }
        }

        else {
            // for use with unity executable RGB
            for (int r = 0; r < img.rows; ++r) {
                for (int c = 0; c < img.cols; ++c) {

                    cv::Vec3b pixel = img.at<cv::Vec3b>(r, c);

                    // for use with unity executable RGB, 10 drones
                    // if (pixel == cv::Vec3b(201, 9, 144)) { // road_1
                    //     score += road_pixel_weight_;
                    // }
                    // else if (pixel == cv::Vec3b(200, 9, 130)) { // road_2
                    //     score += road_pixel_weight_;
                    // }
                    // // else if (pixel == cv::Vec3b(65, 73, 53)) { // pavement_1
                    // //     score += 0.05;
                    // // }
                    // // else if (pixel == cv::Vec3b(128, 72, 0)) { // pavement_2
                    // //     score += 0.15;
                    // // }

                    // for use with unity editor 46DEG X 60 DEG FOV, 10 drones!!!
                    // if ((pixel == cv::Vec3b(80, 97, 32))) { // road_1
                    //     score += road_pixel_weight_;
                    // }
                    // else if ((pixel == cv::Vec3b(24, 97, 176))) { // road_2
                    //     score += road_pixel_weight_;
                    // }
                    // // else if ((pixel == cv::Vec3b(194, 73, 0))) { // pavement_1
                    // //     score += 0.05;
                    // // }
                    // // else if ((pixel == cv::Vec3b(16, 97, 34))) { // pavement_2
                    // //     score += 0.05;
                    // // }

                    // NOTE!! for use with unity editor 46DEG X 60 DEG FOV, 15 drones!!!
                    // if ((pixel == cv::Vec3b(134, 97, 25))) { // road_1
                    //     score += road_pixel_weight_;
                    // }
                    // else if ((pixel == cv::Vec3b(4, 41, 16))) { // road_2
                    //     score += road_pixel_weight_;
                    // }
                    // else if ((pixel == cv::Vec3b(0, 73, 194))) { // pavement_1
                    //     score += 0.05;
                    // }
                    // else if ((pixel == cv::Vec3b(132, 97, 25))) { // pavement_2
                    //     score += 0.05;
                    // }

                    // NOTE!! for use with unity EXECUTABLE 46DEG X 60 DEG FOV, 15 drones!!!
                    if ((pixel == cv::Vec3b(50, 0, 128))) { // road_1
                        score += road_pixel_weight_;
                    }
                    else if ((pixel == cv::Vec3b(18, 73, 200))) { // road_2
                        score += road_pixel_weight_;
                    }
                    // else if ((pixel == cv::Vec3b(18, 9, 128))) { // pavement_1
                    //     score += 0.05;
                    // }
                    // else if ((pixel == cv::Vec3b(128, 72, 128))) { // pavement_2
                    //     score += 0.05;
                    // }
                }
            }
        }
    }

    return score;
}

// override
double ImageCoveringCoordSG::computeWeightedScoreSingleImage(const cv::Mat& img)
{
    double score = 0.0;

    if (is_server_experiment_ == 1) {
        for (int r = 0; r < img.rows; ++r) {
            for (int c = 0; c < img.cols; ++c) {
                cv::Vec3b pixel = img.at<cv::Vec3b>(r, c);

                // Check if the pixel is not black
                if (pixel != cv::Vec3b(0, 0, 0)) {
                    score += 1.0; // Increment the score for each non-black pixel
                }
            }
        }
    }

    else {
        for (int r = 0; r < img.rows; ++r) {
            for (int c = 0; c < img.cols; ++c) {

                cv::Vec3b pixel = img.at<cv::Vec3b>(r, c);

                // Check pixel color and add weighted score
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

    // Iterate through pixels

    return score;
}

rclcpp::Time ImageCoveringCoordSG::airsim_timestamp_to_ros(
    const msr::airlib::TTimePoint & stamp) const
{
    // airsim appears to use chrono::system_clock with nanosecond precision
    std::chrono::nanoseconds dur(stamp);
    std::chrono::system_clock::time_point tp(dur);
    return chrono_timestamp_to_ros(tp);
}

rclcpp::Time ImageCoveringCoordSG::chrono_timestamp_to_ros(
    const std::chrono::system_clock::time_point & stamp) const
{
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                  stamp.time_since_epoch()).count();
    return rclcpp::Time(ns);
}

void ImageCoveringCoordSG::getSelectedActionsPrecNeighborIDImages(std::map<int, ImageInfoStruct>& selected_action_precneigh_ids_mg, bool& got_prec_data_flag)
{
    // std::vector<int> all_inneigh_ids = neighbors_data_.neighbors_array.at(robot_idx_self_ - 1).in_neighbor_ids;
    int prec_neighbor_id = robot_idx_self_ - 1;
    if (prec_neighbor_id < 1) {
        return;
    }

    // Creating a vector of ints from 1 to prec_neighbor_id
    std::vector<int> range_ids;
    for (int i = 1; i <= prec_neighbor_id; ++i) {
        range_ids.push_back(i);
    }

    rclcpp::sleep_for(std::chrono::duration<double>(0.6)); // sleep thread to get latest message
    // poll topic of preceding drones
    for (auto drone_id : range_ids) {
        std::chrono::duration<double> timeout(3.0); // Timeout seconds
        std::string mg_msg_topic = "Drone" + std::to_string(drone_id) + "/mg_rag_data";
        image_covering_coordination::ImageCoveringConstPtr mg_data_msg = ros::topic::waitForMessage<image_covering_coordination::ImageCovering>(mg_msg_topic, timeout);
        // Check if the pointer is not null before dereferencing
        if (mg_data_msg) {
            image_covering_coordination::ImageCovering mg_msg_topic_data = *mg_data_msg;
            if (mg_msg_topic_data.flag_completed_action_selection == true) {
                ImageInfoStruct image_info;
                image_info.pose = mg_msg_topic_data.pose;
                image_info.image = mg_msg_topic_data.image;
                image_info.camera_orientation = mg_msg_topic_data.camera_orientation_setting;
                image_info.best_marginal_gain_val = mg_msg_topic_data.best_marginal_gain;
                selected_action_precneigh_ids_mg[drone_id] = image_info;
            }
        }
        else {
            // Handle the case where no message was received within the timeout
            // std::cout << "could not get mg msg from robot " << drone_id << std::endl;
            // selected_action_precneigh_ids_mg[drone_id] = -1.; // setting a default value if no message received from other drone within 5 seconds
            got_prec_data_flag = false;
            return;
        }
    }
}

void ImageCoveringCoordSG::getSelectedActionsPrecNeighborIDImages_DFS(std::map<int, ImageInfoStruct>& selected_action_precneigh_ids_mg, bool& got_prec_data_flag)
{
    // get the selected actions of drones of all preceding decision making IDs.
    std::unordered_map<int, int> decIdtodronId = mapDecisionIDToDroneID(neighbors_data_);

    int curr_drone_dec_id = decIdtodronId[robot_idx_self_];

    // std::vector<int> all_inneigh_ids = neighbors_data_.neighbors_array.at(robot_idx_self_ - 1).in_neighbor_ids;
    int prec_neighbor_decid = curr_drone_dec_id - 1; // drone id
    if (prec_neighbor_decid < 1) {
        return;
    }

    // Creating a vector of ints from 1 to prec_neighbor_id
    std::vector<int> range_ids;
    for (int i = 1; i <= prec_neighbor_decid; ++i) {
        int droneid = decIdtodronId[i];
        range_ids.push_back(droneid);
    }

    rclcpp::sleep_for(std::chrono::duration<double>(0.6));
    // sleep thread to get latest message
    // poll topic of preceding drones
    for (int drone_id : range_ids) {
        std::string mg_msg_topic =
            "Drone" + std::to_string(drone_id) + "/mg_rag_data";
        std::chrono::duration<double> timeout(3.0);
        image_covering_coordination::msg::ImageCovering tmp_msg;
        bool got = rclcpp::wait_for_message<image_covering_coordination::msg::ImageCovering>(
            tmp_msg,
            this->shared_from_this(),
            mg_msg_topic,
            timeout
        );

        if (got) {
            if (tmp_msg.flag_completed_action_selection) {
                ImageInfoStruct info;
                info.pose                    = tmp_msg.pose;
                info.image                   = tmp_msg.image;
                info.camera_orientation      = tmp_msg.camera_orientation_setting;
                info.best_marginal_gain_val  = tmp_msg.best_marginal_gain;
                selected_action_precneigh_ids_mg[drone_id] = info;
            }
        } else {
            got_prec_data_flag = false;
            return;
        }
    }
}

void ImageCoveringCoordSG::marginalGainFunctionSG_LP(double& best_marginal_gain, int& optimal_action_id, const geometry_msgs::msg::Pose & latest_pose, bool& got_prec_data_flag)
{
    /* evalutates the best marginal gain of all control actions in actions_ 
    */

    /* 
   pseudocode:
   Given the preceding neighbor's ID which has already selected an action, you want to choose your best image out of 8 images.
   step 1: get all images of preceding neighbor and project all those images onto the world frame; find overlap and reduce to a polygon;
   step 2: get all 8 images of your own
   step 3: for each of 8 images of self, evaluate marginal gain by removing overlap between self image and projected combined images from preceding neighbor
   step 4: find the self image that has the best marginal gain in terms of the score from semantic segmentation frames; this is also the corresponding action set
    */

    std::map<int, ImageInfoStruct> selected_action_inneigh_ids_img; // will contain robot pose, camera orientation setting, and image for preceding neighbor only
    bool got_prec_data = true;
    getSelectedActionsPrecNeighborIDImages(selected_action_inneigh_ids_img, got_prec_data); // NOTE!!! MIGHT BE TEMPORARILY COMMENTED OUT FOR DEBUGGING

    // do printmap here
    std::map<int, double> in_neighbor_ids_and_bmg; // ids and best marginal gain
    for (const auto& neighbor : selected_action_inneigh_ids_img) {
        in_neighbor_ids_and_bmg[neighbor.first] = neighbor.second.best_marginal_gain_val;
    }

    // NOTE!!! FOR LOGGING MEMORY USAGE
    printMap(in_neighbor_ids_and_bmg); // will log robot id and corresponding score, post-processing parses this

    if (got_prec_data == false) {
        got_prec_data_flag = false;
        return;
    }

    std::map<int, std::pair<geometry_msgs::Pose, int>> in_neighbor_poses_and_orientations;
    for (const auto& neighbor : selected_action_inneigh_ids_img) {
        std::pair<geometry_msgs::Pose, int> pose_cam_orient;
        pose_cam_orient.first = neighbor.second.pose;
        pose_cam_orient.second = neighbor.second.camera_orientation;
        in_neighbor_poses_and_orientations[neighbor.first] = pose_cam_orient;

        RCLCPP_INFO(this->get_logger(), "SELECTED ACTION NEIGHBOR ID SGLP: %d", neighbor.first);
    }

    // FOR TESTING AND DEBUGGING: push back temporarily in-neighbors info
    /* geometry_msgs::Pose dummy_pose1;
    dummy_pose1.position.x = -1.09;
    dummy_pose1.position.y = 18.12;
    dummy_pose1.position.z = -30.;
    in_neighbor_poses_and_orientations[2] = std::make_pair(dummy_pose1, 0);

    geometry_msgs::Pose dummy_pose2;
    dummy_pose2.position.x = -13.07;
    dummy_pose2.position.y = -3.51;
    dummy_pose2.position.z = -30.;
    in_neighbor_poses_and_orientations[4] = std::make_pair(dummy_pose2, 5); */

    std::map<int, double> marginal_gains_map; // Map to store control action ID and corresponding marginal gain
    if (take_new_pics_) // if false, uses the same pictures taken at previous round
    {
        collectRotatedCameraImages(); // collects 8 images
    }

    for (int i = 0; i < camera_rotated_imgs_vec_.size(); i++) {
        // Find non-overlapping regions
        sensor_msgs::Image masked_image;
        double score;
        if (robot_idx_self_ > 1) {
            masked_image = findNonOverlappingRegions(camera_rotated_imgs_vec_[i], latest_pose, i, in_neighbor_poses_and_orientations, false);
            score = computeWeightedScoreSingleImage(masked_image, false); // Compute score on masked image
        }
        else if (robot_idx_self_ == 1) {
            masked_image = camera_rotated_imgs_vec_.at(i);
            // viewImageFromSensorMsg(masked_image);
            score = computeWeightedScoreSingleImage(masked_image, false); // Compute score on masked image
        }

        marginal_gains_map[i] = score; // placeholder until above function is completed
    }

    // Sort the map in descending order of marginal gains
    auto comparator = [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
        return a.second > b.second;
    };
    std::vector<std::pair<int, double>> sorted_marginal_gains(marginal_gains_map.begin(), marginal_gains_map.end());
    std::sort(sorted_marginal_gains.begin(), sorted_marginal_gains.end(), comparator);

    // Retrieve the highest marginal gain and its corresponding control action ID
    optimal_action_id = sorted_marginal_gains.empty() ? -1 : sorted_marginal_gains.front().first;
    best_marginal_gain = sorted_marginal_gains.empty() ? 0.0 : sorted_marginal_gains.front().second;
    // std::cout << "optimal action id: " << optimal_action_id << " best marginal gain: " << best_marginal_gain << std::endl;

    // set these here for publisher in separate thread
    best_image_cam_orientation_ = optimal_action_id;
    best_self_image_ = camera_rotated_imgs_vec_.at(optimal_action_id);

    got_prec_data_flag = true;
}

void ImageCoveringCoordSG::marginalGainFunctionSG_DFS(double& best_marginal_gain, int& optimal_action_id, geometry_msgs::msg::Pose latest_pose, bool& got_prec_data_flag)
{
    /* evalutates the best marginal gain of all control actions in actions_ 
    */

    /* 
   pseudocode:
   Given the preceding neighbor's ID which has already selected an action, you want to choose your best image out of 8 images.
   step 1: get all images of preceding neighbor and project all those images onto the world frame; find overlap and reduce to a polygon;
   step 2: get all 8 images of your own
   step 3: for each of 8 images of self, evaluate marginal gain by removing overlap between self image and projected combined images from preceding neighbor
   step 4: find the self image that has the best marginal gain in terms of the score from semantic segmentation frames; this is also the corresponding action set
    */

    std::map<int, ImageInfoStruct> selected_action_inneigh_ids_img; // will contain robot pose, camera orientation setting, and image for preceding neighbor only
    bool got_prec_data = true;
    getSelectedActionsPrecNeighborIDImages_DFS(selected_action_inneigh_ids_img, got_prec_data);

    // do printmap here
    std::map<int, double> in_neighbor_ids_and_bmg; // ids and best marginal gain
    for (const auto& neighbor : selected_action_inneigh_ids_img) {
        in_neighbor_ids_and_bmg[neighbor.first] = neighbor.second.best_marginal_gain_val;
    }

    // NOTE!!! FOR LOGGING MEMORY USAGE
    printMap(in_neighbor_ids_and_bmg);

    if (got_prec_data == false) {
        got_prec_data_flag = false;
        return;
    }

    std::map<int, std::pair<geometry_msgs::Pose, int>> in_neighbor_poses_and_orientations;
    for (const auto& neighbor : selected_action_inneigh_ids_img) {
        std::pair<geometry_msgs::Pose, int> pose_cam_orient;
        pose_cam_orient.first = neighbor.second.pose;
        pose_cam_orient.second = neighbor.second.camera_orientation;
        in_neighbor_poses_and_orientations[neighbor.first] = pose_cam_orient;

        RCLCPP_INFO(this->get_logger(), "SELECTED ACTION NEIGHBOR ID SGDFS: %d", neighbor.first);
    }

    // FOR TESTING AND DEBUGGING: push back temporarily in-neighbors info
    /* geometry_msgs::Pose dummy_pose1;
    dummy_pose1.position.x = -1.09;
    dummy_pose1.position.y = 18.12;
    dummy_pose1.position.z = -30.;
    in_neighbor_poses_and_orientations[2] = std::make_pair(dummy_pose1, 0);

    geometry_msgs::Pose dummy_pose2;
    dummy_pose2.position.x = -13.07;
    dummy_pose2.position.y = -3.51;
    dummy_pose2.position.z = -30.;
    in_neighbor_poses_and_orientations[4] = std::make_pair(dummy_pose2, 5); */

    std::map<int, double> marginal_gains_map; // Map to store control action ID and corresponding marginal gain
    if (take_new_pics_) // if false, uses the same pictures taken at previous round
    {
        collectRotatedCameraImages(); // collects 8 images
    }

    for (int i = 0; i < camera_rotated_imgs_vec_.size(); i++) {
        // Find non-overlapping regions
        sensor_msgs::Image masked_image;
        double score;
        if (robot_idx_self_ > 1) {
            masked_image = findNonOverlappingRegions(camera_rotated_imgs_vec_[i], latest_pose, i, in_neighbor_poses_and_orientations, false);
            score = computeWeightedScoreSingleImage(masked_image, false); // Compute score on masked image
        }
        else if (robot_idx_self_ == 1) {
            masked_image = camera_rotated_imgs_vec_.at(i);
            // viewImageFromSensorMsg(masked_image);
            score = computeWeightedScoreSingleImage(masked_image, false); // Compute score on masked image
        }

        marginal_gains_map[i] = score; // placeholder until above function is completed
    }

    // Sort the map in descending order of marginal gains
    auto comparator = [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
        return a.second > b.second;
    };
    std::vector<std::pair<int, double>> sorted_marginal_gains(marginal_gains_map.begin(), marginal_gains_map.end());
    std::sort(sorted_marginal_gains.begin(), sorted_marginal_gains.end(), comparator);

    // Retrieve the highest marginal gain and its corresponding control action ID
    optimal_action_id = sorted_marginal_gains.empty() ? -1 : sorted_marginal_gains.front().first;
    best_marginal_gain = sorted_marginal_gains.empty() ? 0.0 : sorted_marginal_gains.front().second;
    // std::cout << "optimal action id: " << optimal_action_id << " best marginal gain: " << best_marginal_gain << std::endl;

    // set these here for publisher in separate thread
    best_image_cam_orientation_ = optimal_action_id;
    best_self_image_ = camera_rotated_imgs_vec_.at(optimal_action_id);
}

sensor_msgs::msg::Image ImageCoveringCoordSG::findNonOverlappingRegions(
    const sensor_msgs::msg::Image& curr_image,
    const geometry_msgs::msg::Pose& curr_pose,
    int curr_orientation,
    std::map<int, std::pair<geometry_msgs::msg::Pose, int>>& in_neighbor_poses_and_orientations,
    bool convertBGR2RGB)
{

    // Convert current image to OpenCV format; NOTE: convert BGR to RGB
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat img;
    try {
        cv_ptr = cv_bridge::toCvCopy(curr_image, curr_image.encoding);
        img = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return curr_image;
    }

    // swap BGR with RGB NOTE!!! double check whether you need to do this
    if (convertBGR2RGB) {
        cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
    }

    // Mask to store overlapping regions
    // int padding = static_cast<int>(4. * diagonal_frame_dist_);
    int padding = static_cast<int>(diagonal_px_scale_ * diagonal_frame_dist_); // ?frame too large to view on screen
    cv::Mat mask(img.rows + 2 * padding, img.cols + 2 * padding, CV_8UC3, cv::Scalar(0, 0, 0));

    // *******paste curr img onto mask at correct angle; center of image will be center of mask frame******
    cv::Point2f center_mask((mask.cols - 1) / 2.0, (mask.rows - 1) / 2.0);
    cv::Point2f center_img((img.cols - 1) / 2.0, (img.rows - 1) / 2.0);

    double angle = calculateAngleFromOrientation(curr_orientation);
    cv::Mat rot = cv::getRotationMatrix2D(center_img, -angle, 1.0);

    // determine bounding rectangle, center not relevant
    cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), img.size(), -angle).boundingRect2f();

    // adjust transformation matrix
    rot.at<double>(0, 2) += bbox.width / 2.0 - img.cols / 2.0;
    rot.at<double>(1, 2) += bbox.height / 2.0 - img.rows / 2.0;

    cv::Mat rotated;
    cv::warpAffine(img, rotated, rot, bbox.size());
    // viewImageCvMat(rotated, false);

    rotated.copyTo(mask(cv::Rect(center_mask.x - rotated.cols / 2,
                                 center_mask.y - rotated.rows / 2,
                                 rotated.cols,
                                 rotated.rows)));

    // viewImageCvMat(mask, false);
    // *******pasting img at angle END********

    // Iterate through in-neighbor poses
    std::vector<std::vector<cv::Point>> box_points_vec;
    // int c=0; // counter for naming png image during tests
    for (const auto& neighbor : in_neighbor_poses_and_orientations) {

        double dist = PoseEuclideanDistance(curr_pose, neighbor.second.first, curr_orientation, neighbor.second.second);
        if (dist > diagonal_frame_dist_) {
            continue; // skip this in neighbor because there will be no overlap
        }

        // get pixel coordinates of the inn rectangle relative to center_mask as center
        double dx_m = (neighbor.second.first.position.x + (delta_C_ * actions_unit_vec_(0, neighbor.second.second))) -
                      (curr_pose.position.x + (delta_C_ * actions_unit_vec_(0, curr_orientation))); // in m
        double dy_m = (neighbor.second.first.position.y + (delta_C_ * actions_unit_vec_(1, neighbor.second.second))) -
                      (curr_pose.position.y + (delta_C_ * actions_unit_vec_(1, curr_orientation))); // in m

        double dx_pix = dx_m * x_dir_px_m_scale_;
        double dy_pix = dy_m * y_dir_px_m_scale_;

        double rect_center_x_pix = center_mask.x + dx_pix; // dx_pix has to be negative because +ve x-axis in image frame is downwards
        double rect_center_y_pix = center_mask.y - dy_pix;
        cv::Point2f center_rect(rect_center_x_pix, rect_center_y_pix);
        double angle_rect = calculateAngleFromOrientation(neighbor.second.second); // remember to negate; this is in deg

        // get a BoxPoints or RotatedRect object based on the neighbor's pose and camera orientation in pixel coords
        // Create the rotated rectangle
        cv::RotatedRect rect(center_rect, cv::Size2f(pasted_rect_width_pix_, pasted_rect_height_pix_), angle_rect);
        std::vector<cv::Point2f> box_points(4);
        rect.points(box_points.data());

        // convert Point2f to Point
        std::vector<cv::Point> polygon_points(box_points.begin(), box_points.end());
        box_points_vec.push_back(polygon_points);
        cv::fillPoly(mask, box_points_vec, cv::Scalar(0, 0, 0)); // NOTE!!! can directly take all polygons so it is faster
        // std::string name = std::to_string(c) + ".png";
        // viewImageCvMat(mask, false, name);
        // c++;
    }

    // fill polygons here
    // cv::fillPoly(mask, box_points_vec, cv::Scalar(0,0,0));  // NOTE!!! can directly take all polygons so it is faster
    // viewImageCvMat(mask, false, "masktest1.png");

    // extract the image from the mask frame so you have lesser pixels to search for score computation
    cv::Mat extracted_region;
    mask(cv::Rect(center_mask.x - rotated.cols / 2,
                  center_mask.y - rotated.rows / 2,
                  rotated.cols,
                  rotated.rows))
        .copyTo(extracted_region);

    // viewImageCvMat(extracted_region, false, "extracted_region.png");

    // Convert img back to sensor_msgs::Image and return
    sensor_msgs::msg::Image extracted_image;
    cv_bridge::CvImage cv_image{
        cv_ptr->header,
        cv_ptr->encoding,
        extracted_region};
    cv_image.toImageMsg(extracted_image);

    return extracted_image;
}

std::vector<cv::Point> ImageCoveringCoordSG::toCvPointVector(const std::vector<cv::Point2f>& points)
{

    std::vector<cv::Point> cvPoints;
    for (const auto& p : points) {
        cvPoints.push_back(cv::Point(p.x, p.y));
    }

    return cvPoints;
}

bool ImageCoveringCoordSG::areAllValuesTrue()
{
    for (const auto& pair : all_robots_selected_map_) {
        if (!pair.second) {
            return false; // Found a false value, return false
        }
    }
    return true; // All values are true
}

std::pair<double, double> ImageCoveringCoordSG::computeRangeBearing(const std::vector<double>& pose1,
                                                                    const std::vector<double>& pose2)
{

    double dx = pose2[0] - pose1[0];
    double dy = pose2[1] - pose1[1];

    double range = std::sqrt(dx * dx + dy * dy);

    double bearing = std::atan2(dy, dx);

    return { range, bearing };
}

void ImageCoveringCoordSG::simulatedCommTimeDelaySG_LP()
{
    // NOTE!!! This is for Sequential Greedy
    double time_delay_per_msg = total_data_msg_size_ / communication_data_rate_;
    double delay_factor = robot_idx_self_ * (robot_idx_self_ - 1) / 2; // arithmetic series
    double total_time_delay = delay_factor * time_delay_per_msg; // seconds

    communication_round_ += 1;
    // std::cout << "Communication delay of " << total_time_delay << " s" << std::endl;
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start_;
    RCLCPP_INFO(
        this->get_logger(),
        "%f COMMUNICATION ROUND COUNT: %d",
        duration.count(),
        communication_round_);

    if (total_time_delay != 0) {
        rclcpp::sleep_for(std::chrono::duration<double>(total_time_delay));
    }
}

void ImageCoveringCoordSG::simulatedCommTimeDelaySG_DFSv1()
{
    double time_delay_per_msg = total_data_msg_size_ / communication_data_rate_;
    double delay_factor = robot_idx_self_ * (robot_idx_self_ - 1) / 2; // arithmetic series
    double total_time_delay = delay_factor * time_delay_per_msg; // seconds

    // Existing random number generation setup
    std::random_device rd; // Obtain a random number from hardware
    std::mt19937 gen(rd()); // Seed the generator

    // Generate a random number between 0 and 6 for the communication round increment
    std::uniform_int_distribution<> comm_round_distr(3, 10);
    int random_comm_round_increment = comm_round_distr(gen);

    communication_round_ += random_comm_round_increment; // Add the random increment

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start_;

    RCLCPP_INFO(
        this->get_logger(),
        "%f ADDITIONAL MEMORY STORAGE DUE TO RELAY: %d",
        duration.count(),
        random_comm_round_increment);
    RCLCPP_INFO(
        this->get_logger(),
        "%f COMMUNICATION ROUND COUNT: %d",
        duration.count(),
        communication_round_);

    // Random delay functionality (unchanged)
    std::uniform_real_distribution<> distr(total_time_delay, 10); // Define the range for random delay
    double random_delay = distr(gen); // Generate the random delay
    RCLCPP_INFO(
        this->get_logger(),
        "Applying random delay of %f seconds.",
        random_delay);

    rclcpp::sleep_for(std::chrono::duration<double>(random_delay)); // Apply the random delay
}

void ImageCoveringCoordSG::simulatedCommTimeDelaySG_DFSv2()
{
    // this function uses DFS graph to find hops according to decision making
    // ID to decide communication delay duration

    double time_delay_per_msg = total_data_msg_size_ / communication_data_rate_;

    communication_round_ += 1;
    // auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = std::chrono::high_resolution_clock::now() - start_;
    RCLCPP_INFO(
        this->get_logger(),
        "%f COMMUNICATION ROUND COUNT: %d",
        duration.count(),
        communication_round_);

    // graph DFS hop counting
    Graph graph;
    graphDecisionIDHops(neighbors_data_, graph); // graph here has updated edges and nodes according to decision making id
    int delay_factor = graphDFSdelayFactor(graph);
    double total_time_delay = delay_factor * time_delay_per_msg; // seconds

    if (total_time_delay != 0) {
        rclcpp::sleep_for(std::chrono::duration<double>(total_time_delay)); // Your existing delay
    }
}

void ImageCoveringCoordSG::simulatedCommTimeDelaySG_LP(double& processed_time)
{
    // NOTE!!! This is for Sequential Greedy
    double time_delay_per_msg = total_data_msg_size_ / communication_data_rate_;
    double delay_factor = robot_idx_self_ * (robot_idx_self_ - 1) / 2;
    double total_time_delay = delay_factor * time_delay_per_msg; // seconds
    double comm_time = total_time_delay - processed_time;

    communication_round_ += 1;
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start_;

    RCLCPP_INFO(
        this->get_logger(),
        "%f COMMUNICATION ROUND COUNT: %d",
        duration.count(),
        communication_round_);

    if (comm_time > 0.0) {
        rclcpp::sleep_for(std::chrono::duration<double>(comm_time));
    }
}

void ImageCoveringCoordSG::takeUnionOfMapsDataStruct(std::map<int, double>& selected_action_precneigh_ids_mg, std::map<int, double>& new_selected_action_precneigh_ids_mg, std::map<int, double>& combined_map)
{
    // Insert elements from first map
    for (auto const& element : selected_action_precneigh_ids_mg) {
        combined_map[element.first] = element.second;
    }

    // Insert elements from second map
    for (auto const& element : new_selected_action_precneigh_ids_mg) {
        combined_map[element.first] = element.second;
    }
}

//override function
void ImageCoveringCoordSG::takeUnionOfMapsDataStruct(std::map<int, double>& selected_action_precneigh_ids_mg, std::map<int, double>& new_selected_action_precneigh_ids_mg)
{
    // Insert elements from second map
    for (auto const& element : new_selected_action_precneigh_ids_mg) {
        selected_action_precneigh_ids_mg[element.first] = element.second;
    }
}

msr::airlib::Pose ImageCoveringCoordSG::giveGoalPoseFormat(msr::airlib::MultirotorState& curr_pose, float disp_x, float disp_y, float disp_z, float new_yaw)
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

msr::airlib::Pose ImageCoveringCoordSG::giveCameraGoalPoseFormat(msr::airlib::MultirotorState& curr_pose, float disp_x, float disp_y, float disp_z, float curr_roll, float new_pitch, float curr_yaw)
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
Eigen::Quaternionf ImageCoveringCoordSG::quaternionFromEuler(float roll, float pitch, float yaw)
{
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
    return q;
}

// Convert quaternion to Euler angles
Eigen::Vector3f ImageCoveringCoordSG::eulerFromQuaternion(const Eigen::Quaternionf& q)
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

geometry_msgs::Pose ImageCoveringCoordSG::convertToGeometryPose(const msr::airlib::MultirotorState& pose1)
{
    // Convert Eigen::Vector3f to geometry_msgs::Point
    geometry_msgs::Point point_msg;
    point_msg.x = pose1.kinematics_estimated.pose.position.x();
    point_msg.y = pose1.kinematics_estimated.pose.position.y();
    point_msg.z = pose1.kinematics_estimated.pose.position.z();

    // Convert Eigen::Quaternion<float, 2> to geometry_msgs::Quaternion
    geometry_msgs::Quaternion quaternion_msg;
    quaternion_msg.x = pose1.kinematics_estimated.pose.orientation.x();
    quaternion_msg.y = pose1.kinematics_estimated.pose.orientation.y();
    quaternion_msg.z = pose1.kinematics_estimated.pose.orientation.z();
    quaternion_msg.w = pose1.kinematics_estimated.pose.orientation.w();

    // Assign to the geometry_msgs::Pose
    geometry_msgs::Pose latest_pose;
    latest_pose.position = point_msg;
    latest_pose.orientation = quaternion_msg;

    return latest_pose;
}

void ImageCoveringCoordSG::saveImage(std::vector<ImageResponse>& response)
{
    for (const ImageResponse& image_info : response) {
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

void ImageCoveringCoordSG::viewImage(std::vector<ImageResponse>& response)
{
    for (const ImageResponse& image_info : response) {
        RCLCPP_INFO(this->get_logger(), "Image uint8 size: %zu", image_info.image_data_uint8.size());
        RCLCPP_INFO(this->get_logger(), "Image float size: %zu", image_info.image_data_float.size());

        cv::Mat image;
        if (image_info.pixels_as_float) {
            // Allocate memory for uint8 image
            cv::Mat float_image(image_info.height, image_info.width, CV_32FC3, const_cast<float*>(image_info.image_data_float.data()));

            // Convert to uint8 range [0, 255] and CV_8UC3 format
            cv::convertScaleAbs(float_image, image, 255.0);
            image.convertTo(image, CV_8UC3);
        }
        else {
            // Assuming image_data_uint8 stores uint8 data in BGR format
            image = cv::Mat(image_info.height, image_info.width, CV_8UC3, const_cast<uchar*>(image_info.image_data_uint8.data())).clone();
        }

        // Flip the image vertically
        cv::flip(image, image, 0);

        // If image is in BGR format, swap R and B channels
        if (image.channels() == 3 && image_info.pixels_as_float == false) {
            cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
        }

        cv::imshow("Image", image);
        cv::waitKey(0); // Wait for a key press
    }
}

void ImageCoveringCoordSG::viewImageFromSensorMsg(const sensor_msgs::msg::Image& msg)
{
    try {
        // Convert the sensor_msgs::Image to cv::Mat using cv_bridge
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // Ensure correct encoding

        cv::Mat image = cv_ptr->image;

        // If image is in BGR format, swap R and B channels
        if (image.channels() == 3) {
            cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
        }

        // Display the image
        cv::imshow("Image", image);
        cv::waitKey(0); // Wait for a key press
    }
    catch (cv_bridge::Exception& e) {
        // Handle potential conversion errors
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
}

void ImageCoveringCoordSG::viewImageCvMat(cv::Mat& image)
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

// override
void ImageCoveringCoordSG::viewImageCvMat(cv::Mat& image, bool switchBGR_RGB_flag)
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

void ImageCoveringCoordSG::viewImageCvMat(cv::Mat& image, bool switchBGR_RGB_flag, const std::string filename)
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

void ImageCoveringCoordSG::rotateImagesInVector()
{
    /*WARN needs testing 
     */
    for (size_t i = 0; i < camera_rotated_imgs_vec_.size(); ++i) {
        // Convert sensor_msgs::Image to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(camera_rotated_imgs_vec_[i], sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Rotate the image by 180 degrees (vertical flip)
        cv::flip(cv_ptr->image, cv_ptr->image, 0);
        cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2RGB);

        // Convert the OpenCV Mat back to sensor_msgs::Image
        sensor_msgs::ImagePtr rotated_img_msg = cv_ptr->toImageMsg();

        // Replace the original image in the vector with the rotated image
        camera_rotated_imgs_vec_[i] = *rotated_img_msg;
    }
}

double ImageCoveringCoordSG::PoseEuclideanDistance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2, int cam_orient_1, int cam_orient_2)
{
    /* 
    cam_orient_1: camera orientation of pose1
    cam_orient_2: camera orientation of pose2
     */
    // has to account for delta C with image orient

    double cx_p1 = delta_C_ * actions_unit_vec_(0, cam_orient_1);
    double cy_p1 = delta_C_ * actions_unit_vec_(1, cam_orient_1);

    double cx_p2 = delta_C_ * actions_unit_vec_(0, cam_orient_2);
    double cy_p2 = delta_C_ * actions_unit_vec_(1, cam_orient_2);

    double dx = (pose1.position.x + cx_p1) - (pose2.position.x + cx_p2);
    double dy = (pose1.position.y + cy_p1) - (pose2.position.y + cy_p2);
    double dz = pose1.position.z - pose2.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double ImageCoveringCoordSG::calculateAngleFromOrientation(int cam_orient)
{

    double angle_deg;
    if ((cam_orient == 0) || (cam_orient == 4)) {
        angle_deg = 0.;
    }
    else if ((cam_orient == 1) || (cam_orient == 5)) {
        angle_deg = 45.;
    }
    else if ((cam_orient == 2) || (cam_orient == 6)) {
        angle_deg = 90.;
    }
    else if ((cam_orient == 3) || (cam_orient == 7)) {
        angle_deg = 135.;
    }

    return angle_deg;
}

// graph functions
// Function to find number of edges between current decision making ID and the previous decision making ID
int ImageCoveringCoordSG::findNumEdgesBetweenIDs(Graph& graph, int currentID)
{
    if (graph.adjList.find(currentID) == graph.adjList.end() || graph.adjList.find(currentID - 1) == graph.adjList.end())
        return 0; // IDs do not exist in the graph

    // Check if direct edge exists
    auto& edges = graph.adjList[currentID];
    if (std::find(edges.begin(), edges.end(), currentID - 1) != edges.end())
        return 1; // Direct edge exists

    return 0; // No direct edge
}

void ImageCoveringCoordSG::graphDecisionIDHops(const airsim_ros_pkgs::NeighborsArray neighbors_array, Graph& graph)
{

    // Assuming drone_id 1 contains the complete graph information
    for (const auto& neighbor : neighbors_array.neighbors_array) {
        if (neighbor.drone_id == 1) {
            // Reconstruct the graph using decision-making IDs; only edges are needed

            for (size_t i = 0; i < neighbor.graph_edges.size(); i += 2) {
                int u = neighbor.graph_edges[i];
                int v = neighbor.graph_edges[i + 1];
                graph.addEdge(u, v);
            }

            break; // Assuming only drone_id 1 publishes the complete graph
        }
    }
}

// Function to find drone ID for every decision-making ID
std::unordered_map<int, int> ImageCoveringCoordSG::mapDecisionIDToDroneID(const airsim_ros_pkgs::NeighborsArray neighbors_array)
{

    std::unordered_map<int, int> decisionIDtoDroneID;

    // NOTE!!! drone_id 1 contains the complete graph information
    for (const auto& neighbor : neighbors_array.neighbors_array) {
        decisionIDtoDroneID[neighbor.decision_making_id] = neighbor.drone_id;
    }
    return decisionIDtoDroneID;
}

int ImageCoveringCoordSG::graphDFSdelayFactor(Graph& graph)
{

    std::map<int, int> decisionIDtoHops; // maps the decision id (key) to the number of hops to the preceding decision id
    std::unordered_map<int, int> decIdtodronId = mapDecisionIDToDroneID(neighbors_data_);
    int curr_drone_dec_id = decIdtodronId[robot_idx_self_];

    for (int i = curr_drone_dec_id; i > 0; i--) {
        int numhops = findNumEdgesBetweenIDs(graph, i);
        decisionIDtoHops[i] = numhops;
    }

    int sum = 0; // Initialize sum of multiplied values
    for (auto& pair : decisionIDtoHops) {
        sum += pair.first * pair.second; // Multiply key by value and add to sum
    }

    return sum; // Return the final sum
}

void ImageCoveringCoordSG::printMap(const std::map<int, double>& map)
{
    for (const auto& pair : map) {
        RCLCPP_INFO(this->get_logger(), "%d: %f", pair.first, pair.second);
    }
}

int main(int argc, char** argv)
{
    // Initialize ROS node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageCoveringCoordSG>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
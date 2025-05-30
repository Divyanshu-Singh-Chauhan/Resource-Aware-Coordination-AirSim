/*
NOTE: This script contains the algorithm for RAG
 */
#include "image_covering_coordination/image_covering_coord_RAG.h"

ImageCoveringCoordRAG::ImageCoveringCoordRAG(ros::NodeHandle& nh)
    : node_handle_(nh)
{
    // Get ROS params
    ros::param::get("~drone_name", drone_name_self_);
    ros::param::get("~number_of_drones", num_robots_);
    ros::param::get("~robot_index_ID", robot_idx_self_);
    ros::param::get("~camera_name", camera_name_); // need to set default here if not mentioned
    ros::param::get("~communication_range", communication_range_); // need to set default here if not mentioned
    ros::param::get("~algorithm_to_run", algorithm_to_run_);
    ros::param::get("~random_seed", random_seed_); // Default to 0 if not set
    ros::param::get("~num_nearest_neighbors", num_nearest_neighbors_);
    ros::param::get("~is_server_experiment", is_server_experiment_); // treat as bool; 0 is false, 1 is true

    std::cout << "ROBOT_SELF_ID: " << robot_idx_self_ << std::endl;

    // std::cout << "ALGORITHM: RAG_AD" << std::endl;
    std::cout << "ALGORITHM: " << algorithm_to_run_ << std::endl;
    // std::cout << "ALGORITHM: RAG_NN" << std::endl;

    // SET RAG ALG GRAPH TOPOLOGY************
    graph_topology_RAG_ = algorithm_to_run_;
    // graph_topology_RAG_ = "allow_disconnected";
    // graph_topology_RAG_ = "line_path";
    // graph_topology_RAG_ = "nearest_neighbors"; // note! set number of in neighbors to have within mg functions
    // default_num_nearest_neighbors_ = 3;
    default_num_nearest_neighbors_ = num_nearest_neighbors_;
    // SET RAG ALG GRAPH TOPOLOGY************end

    async_spinner_.reset(new ros::AsyncSpinner(3));
    async_spinner_->start();

    // ROS publishers
    std::string mg_msg_topic = drone_name_self_ + "/mg_rag_data";
    mg_rag_msg_pub_ = node_handle_.advertise<image_covering_coordination::ImageCovering>(mg_msg_topic, 5); //

    // ROS subscribers
    // NOTE!!! Subscribe to /clock topic for putting a callback which checks whether n_time_step_ amount of time has passed
    clock_sub_ = node_handle_.subscribe("/clock", 1, &ImageCoveringCoordRAG::clockCallback, this); // for action selection frequency
    marginal_gain_pub_clock_sub_ = node_handle_.subscribe("/clock", 1, &ImageCoveringCoordRAG::marginalGainPublisherclockCallback, this); // for action selection frequency
    check_drones_selection_sub_ = node_handle_.subscribe("/clock", 1, &ImageCoveringCoordRAG::updateAllRobotsSelectionStatusCallback, this);

    // DATA TRANSMISSION RATE SETTINGS
    // total_data_msg_size_ = 1024.; // depth frame+pose will be 1024 KB
    // communication_data_rate_ = 10. * total_data_msg_size_; // KB/s; multiplying factor 
    // total_data_msg_size_ = 200.;
    // communication_data_rate_ = 0.25 * total_data_msg_size_; // KB/s; multiplying factor
    total_data_msg_size_ = 200.;
    communication_data_rate_ = 100. * total_data_msg_size_; // KB/s; multiplying factor

    communication_round_ = 0;

    // set actions x,y,theta. Note x,y are independent of theta
    max_sensing_range_ = 45.;
    flight_level_ = 0.5;
    // altitude_flight_level_ = -40. - 0.5 * (num_robots_ * flight_level_) + double(robot_idx_self_) * flight_level_;
    altitude_flight_level_ = -30.; // altitude for image covering will be slightly lower

    selected_action_ind_ = -1; // initialized to be -1 indicating no action has been selected yet

    // best_marginal_gain_self_ = -1.;
    best_marginal_gain_self_ = std::numeric_limits<double>::max();

    finished_action_selection_ = false;
    waiting_for_robots_to_finish_selection_ = false;
    takeoff_wait_flag_ = true;
    terminate_decision_making_ = false;

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

    // consts initialization
    int n_time_step_init = 0;
    horizon_ = 100;
    n_time_steps_ = 2000;
    double dT = horizon_ / n_time_steps_;
    curr_time_step_samp_ = 0; // this is "t" passed into expstarsix alg; initially 0

    // reward_ = Eigen::MatrixXd(int(n_time_steps_), 1);

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
    // camera_pitch_theta_ = 0.2182; // 12.5 deg
    camera_pitch_theta_ = 0.524; // 30 deg
    camera_downward_pitch_angle_ = -1.571; // DO NOT CHANGE! -90 deg
    current_image_requests_vec_ = { ImageRequest("front_center", msr::airlib::ImageCaptureBase::ImageType::Segmentation, false, false),
                                    ImageRequest("front_center", msr::airlib::ImageCaptureBase::ImageType::Scene, false, false) };

    camera_image_type_ = msr::airlib::ImageCaptureBase::ImageType::Segmentation;

    // NOTE!!! below settings are for 62.7deg x 90 deg FOV, 10 robots!!!
    // fov_y_ = M_PI / 2.; // in rads; 90 deg horizontal fov
    // fov_x_ = 1.093; // in rads; 62.6 deg vertical fov
    // y_half_side_len_m_ = 30.; //m
    // x_half_side_len_m_ = 18.25; //m
    // y_dir_px_m_scale_ = 10.67; // pixels per m in the y dir
    // x_dir_px_m_scale_ = 13.15; // pixels per m in the x dir
    // delta_C_ = 10.44; // m, displacement in center due to viewing angle when delta = 0.2182 rads
    // pasted_rect_height_pix_ = 513;
    // pasted_rect_width_pix_ = 640;

    // NOTE!!! below settings are for 46.83deg x 60 deg FOV, 10 robots and 15 robots!!!
    fov_y_ = M_PI / 3.; // in rads; 60 deg horizontal fov
    fov_x_ = 0.817; // in rads; 46.83 deg vertical fov
    y_half_side_len_m_ = 17.32; //m
    x_half_side_len_m_ = 13.; //m
    y_dir_px_m_scale_ = 18.474; // pixels per m in the y dir NOTE this is for unstretched frame
    x_dir_px_m_scale_ = 18.48; // pixels per m in the x dir
    delta_C_ = 21.94; // m, displacement in center due to viewing angle when delta = 0.2182 rads
    // delta_C_ = 0.;
    // pasted_rect_height_pix_ = 480;
    pasted_rect_height_pix_ = 683;
    pasted_rect_width_pix_ = 640;

    diagonal_frame_dist_ = sqrt(pow((2. * y_half_side_len_m_), 2) + pow((2. * x_half_side_len_m_), 2)); // + 2.*delta_C_;  // shortest distance needed to consider overlap from in-neighbors
    diagonal_px_scale_ = sqrt(pow(y_dir_px_m_scale_, 2) + pow(x_dir_px_m_scale_, 2));

    take_new_pics_ = true;
    road_pixel_weight_ = 1.;
    assump_counter_ = 0;
    // CAMERA SETTINGS*******end

    control_action_disp_scale_ = 1.5;
    // control_action_disp_scale_ = delta_C_ / 10.;

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
ImageCoveringCoordRAG::~ImageCoveringCoordRAG()
{
    async_spinner_->stop();
}

void ImageCoveringCoordRAG::marginalGainPublisherclockCallback(const rosgraph_msgs::Clock& msg)
{
    /* PUBLISHES best marginal gain value */
    msr::airlib::MultirotorState pose1 = client_.getMultirotorState(drone_name_self_);
    geometry_msgs::Pose latest_pose = convertToGeometryPose(pose1);

    // access last set best marginal gain from clock callback and continuously publish it
    image_covering_coordination::ImageCovering imcov_mg_msg;

    imcov_mg_msg.header.stamp = ros::Time::now();
    imcov_mg_msg.drone_id = robot_idx_self_;
    imcov_mg_msg.image = best_self_image_; // NOTE!!! publishing real image
    imcov_mg_msg.camera_orientation_setting = best_image_cam_orientation_;
    imcov_mg_msg.pose = latest_pose;
    imcov_mg_msg.flag_completed_action_selection = finished_action_selection_; // set this to false as soon as you begin planning stage
    imcov_mg_msg.best_marginal_gain = best_marginal_gain_self_;
    // imcov_mg_msg.terminate_decision_making_flag = terminate_decision_making_;

    mg_rag_msg_pub_.publish(imcov_mg_msg);
    std::cout << std::boolalpha;
    // std::cout << "Published mg_rag msg with best marginal gain: " <<  imcov_mg_msg.best_marginal_gain << " and action selection flag: "<< std::to_string(imcov_mg_msg.flag_completed_action_selection) << std::endl;
}

void ImageCoveringCoordRAG::updateAllRobotsSelectionStatusCallback(const rosgraph_msgs::Clock& msg)
{
    /* SUBSCRIBES to marginal gain message which contains action selection flag */

    ros::Duration timeout(1.5); // Timeout seconds
    for (int i = 0; i < num_robots_; i++) {
        int other_robot_id = i + 1;
        std::string mg_msg_topic = "Drone" + std::to_string(other_robot_id) + "/mg_rag_data";
        // std::cout << "MG MSG TOPIC: " << mg_msg_topic << std::endl;

        image_covering_coordination::ImageCoveringConstPtr mg_data_msg = ros::topic::waitForMessage<image_covering_coordination::ImageCovering>(mg_msg_topic, timeout);

        if (mg_data_msg) {
            image_covering_coordination::ImageCovering mg_msg_topic_data = *mg_data_msg;
            all_robots_selected_map_[i + 1] = mg_msg_topic_data.flag_completed_action_selection;
            // std::cout <<  "got mg msg from robot " + std::to_string(other_robot_id) << std::endl;
        }
        else {
            // Handle the case where no message was received within the timeout
            all_robots_selected_map_[i + 1] = false;
            //    std::cout << "could not get mg msg from robot " + std::to_string(other_robot_id) << std::endl;
        }
    }
    std::cout << "" << std::endl;
}

void ImageCoveringCoordRAG::clockCallback(const rosgraph_msgs::Clock& msg)
{
    /* MAIN callback for RAG decision making */

    // wait to proceed until drone has taken off
    if (takeoff_wait_flag_ == true) {
        client_.moveToZAsync(altitude_flight_level_, 5.0, client_timeout_sec_, client_default_yaw_mode_, client_lookahead_, client_adaptive_lookahead_, drone_name_self_)->waitOnLastTask();
        ros::Duration(1.).sleep();
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
        ros::Duration(1.).sleep();

        return;
    }

    // get a single NeighborsArray message
    airsim_ros_pkgs::NeighborsArrayConstPtr neighbors_data_msg = ros::topic::waitForMessage<airsim_ros_pkgs::NeighborsArray>("/neighbors_data");
    neighbors_data_ = *neighbors_data_msg;

    // get pose
    msr::airlib::MultirotorState pose1 = client_.getMultirotorState(drone_name_self_);
    geometry_msgs::Pose latest_pose = convertToGeometryPose(pose1);

    // for self robot, do as many rounds of RAG-H needed to find the actions each will execute. Program communication delays where needed.
    std::map<int, ImageInfoStruct> selected_action_inneigh_ids_mg; // int inneigh id : marginal gain value
    while ((finished_action_selection_ == false) && (waiting_for_robots_to_finish_selection_ == false)) // need to have flag here that indicates all robots have executed actions
    {
        // for each drone in the in-neighbors set, subscribe to their gain topic, while continuously publishing your own best gain (out neighbors will subscribe to this anyway)
        // subscribe to all in neighbors' mg rag msgs and collect ids of those in-neighbors that have selected ids. pass this into the marginal gain function

        int best_control_action_ind;
        double best_marginal_gain_self;
        marginalGainFunctionRAG(best_marginal_gain_self, best_control_action_ind, latest_pose, selected_action_inneigh_ids_mg); // wrt only to in-neighbors that have selected actions
        best_marginal_gain_self_ = best_marginal_gain_self; // will publish this in the mg gain publisher callback thread

        std::cout << "robot " << robot_idx_self_ << " best marginal gain: " << best_marginal_gain_self_ << std::endl;

        std::map<int, double> non_selected_action_inneigh_ids_mg;
        auto start1 = std::chrono::high_resolution_clock::now();
        getNonSelectedActionsInNeighborIDs(non_selected_action_inneigh_ids_mg); // key id : best mg value  // NOTE!!! TEMPORARILY COMMENTED OUT FOR DEBUGGING
        std::cout << "robot " << robot_idx_self_ << " non selected neighbors: " << std::endl;
        printMap(non_selected_action_inneigh_ids_mg);

        auto end1 = std::chrono::high_resolution_clock::now();
        double duration1 = (end1 - start1).count(); // milliseconds
        // std::cout << "duration1: " << duration1 << std::endl;
        simulatedCommTimeDelayRAG(duration1); // delay 1

        // compare your best mg with those of in neighbors that have not selected an action yet
        bool is_my_mg_best;
        isMyMarginalGainBest(is_my_mg_best, non_selected_action_inneigh_ids_mg);
        // std::map<int, double> new_selected_action_inneigh_ids_mg;
        std::map<int, ImageInfoStruct> new_selected_action_inneigh_ids_mg;

        if (is_my_mg_best == true) {
            finished_action_selection_ = true;

            // std::cout << "my marginal gain is best" <<std::endl;
            selected_action_ind_ = best_control_action_ind;
            take_new_pics_ = true; // for next round
            break;
        }
        else {
            //    std::cout << "my marginal gain is NOT best, waiting for others to select action" << std::endl;
            auto start2 = std::chrono::high_resolution_clock::now();

            if ((graph_topology_RAG_ == "allow_disconnected") || (graph_topology_RAG_ == "RAG_AD")) {
                getSelectedActionsInNeighborIDImages(selected_action_inneigh_ids_mg); // NOTE!!! TEMPORARILY COMMENTED OUT FOR DEBUGGING
            }
            else if ((graph_topology_RAG_ == "line_path") || (graph_topology_RAG_ == "RAG_LP")) {
                getSelectedActionsInNeighborIDImagesLinePath(selected_action_inneigh_ids_mg);
            }
            else if ((graph_topology_RAG_ == "nearest_neighbors") || (graph_topology_RAG_ == "RAG_NN")) {
                getSelectedActionsInNeighborIDImagesNearestNeighbors(selected_action_inneigh_ids_mg, default_num_nearest_neighbors_);
            }

            auto end2 = std::chrono::high_resolution_clock::now();
            double duration2 = (end2 - start2).count();

            if (areAllValuesTrue(check_drones_msg_dict_)) {
                simulatedCommTimeDelayRAG(duration2); // delay 2
            }

            take_new_pics_ = false;
        }
    }

    // Once every drone has selected its action, execute this self robot's action. NOTE: We assume all robots execute actions at the same time,
    // until then they execute zero-order hold on their previously selected control action.
    Eigen::Matrix<double, 3, 1> selected_action_displacement = actions_.block(0, selected_action_ind_, 3, 1);
    last_selected_displacement_ = selected_action_displacement;

    // CHECK if all drones have already selected actions
    bool all_robots_ready_to_proceed = areAllValuesTrue(); // NOTE!!! MAY BE TEMPORARILY COMMENTED OUT FOR DEBUGGING
    // bool all_robots_ready_to_proceed = areAllValuesTrue3Robots();
    // bool all_robots_ready_to_proceed = true;

    if (all_robots_ready_to_proceed == true) {
        std::cout << "ALL ROBOTS READY TO PROCEED WITH ACTION" << std::endl;
        // execute selected action using airsim client and action table________
        // get current position, then add the x,y to it; ensure drone does not yaw
        auto position = pose1.getPosition();

        client_.moveToPositionAsync(position.x() + control_action_disp_scale_ * selected_action_displacement(0, 0), position.y() + control_action_disp_scale_ * selected_action_displacement(1, 0), altitude_flight_level_, 4., client_timeout_sec_, client_drivetrain_, client_default_yaw_mode_, client_lookahead_, client_adaptive_lookahead_, drone_name_self_)->waitOnLastTask();

        finished_action_selection_ = false; // reset this flag for next timestep
        waiting_for_robots_to_finish_selection_ = false;

        // also reset the best marginal gain here; this is AFTER the drone moves and has completed RAG round
        // NOTE!! make this assumption once or twice only
        if (assump_counter_ < 2) {
            best_marginal_gain_self_ = std::numeric_limits<double>::max();
            assump_counter_ += 1;
        }
    }
    else {

        // check again here whether your marginal gain is best
        // std::map<int, double> non_selected_inn;
        // getNonSelectedActionsInNeighborIDs(non_selected_inn);
        // bool bestmgflag;
        // isMyMarginalGainBest(bestmgflag, non_selected_inn);

        waiting_for_robots_to_finish_selection_ = true;
    }

    curr_time_step_samp_++; // increment by 1 for each t in T
}

void ImageCoveringCoordRAG::collectRotatedCameraImages()
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
    camera_rotated_imgs_vec_ = std::vector<sensor_msgs::Image>(8);

    // first take picture in 0deg yaw orientation: 2 images, first forward and then backward camera pitch. Save to class priv vector
    // msr::airlib::Pose camera_pose_0 = giveCameraGoalPoseFormat(pose1, -0.021, 0.0112, altitude_flight_level_, camera_downward_pitch_angle_ + camera_pitch_theta_,
    //     curr_roll1, curr_yaw1);
    msr::airlib::Pose camera_pose_0 = giveCameraGoalPoseFormat(pose1, -0.021, 0.0112, altitude_flight_level_, curr_roll1, camera_downward_pitch_angle_ + camera_pitch_theta_, curr_yaw1);
    client_.simSetCameraPose("front_center", camera_pose_0, drone_name_self_); // set camera pose
    ros::Duration(1.).sleep(); // sleep to let camera change pose
    std::vector<ImageResponse> img_response_vec0 = client_.simGetImages(current_image_requests_vec_, drone_name_self_);

    // viewImage(img_response_vec0); // for debugging only!!

    sensor_msgs::ImagePtr img0_ptr = get_img_msg_from_response(img_response_vec0.at(1)); // 1st elem is segmentation, airsim bug swaps seg and scene

    // sensor_msgs::ImagePtr img0_ptr = get_img_msg_from_response(img_response_vec0.at(0)); // 1st elem is segmentation, airsim bug swaps seg and scene

    camera_rotated_imgs_vec_[0] = (*img0_ptr);

    msr::airlib::Pose camera_pose_1 = giveCameraGoalPoseFormat(pose1, -0.021, 0.0112, altitude_flight_level_, curr_roll1, camera_downward_pitch_angle_ - camera_pitch_theta_, curr_yaw1);
    client_.simSetCameraPose("front_center", camera_pose_1, drone_name_self_); // set camera pose
    ros::Duration(1.).sleep(); // sleep to let camera change pose
    std::vector<ImageResponse> img_response_vec1 = client_.simGetImages(current_image_requests_vec_, drone_name_self_);
    // viewImage(img_response_vec1);
    sensor_msgs::ImagePtr img1_ptr = get_img_msg_from_response(img_response_vec1.at(1));

    // sensor_msgs::ImagePtr img1_ptr = get_img_msg_from_response(img_response_vec1.at(0));

    camera_rotated_imgs_vec_[4] = (*img1_ptr);

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
    ros::Duration(1.).sleep(); // sleep to let camera change pose
    std::vector<ImageResponse> img_response_vec_45_0 = client_.simGetImages(current_image_requests_vec_, drone_name_self_);
    // viewImage(img_response_vec_45_0);
    sensor_msgs::ImagePtr img_45_0_ptr = get_img_msg_from_response(img_response_vec_45_0.at(1));

    // sensor_msgs::ImagePtr img_45_0_ptr = get_img_msg_from_response(img_response_vec_45_0.at(0));

    camera_rotated_imgs_vec_[1] = (*img_45_0_ptr);

    msr::airlib::Pose camera_pose_45_1 = giveCameraGoalPoseFormat(pose2_new, -0.021, 0.0112, altitude_flight_level_, curr_roll2, camera_downward_pitch_angle_ - camera_pitch_theta_, curr_yaw2);
    client_.simSetCameraPose("front_center", camera_pose_45_1, drone_name_self_); // set camera pose
    ros::Duration(1.).sleep(); // sleep to let camera change pose
    std::vector<ImageResponse> img_response_vec_45_1 = client_.simGetImages(current_image_requests_vec_, drone_name_self_);
    // viewImage(img_response_vec_45_1);
    sensor_msgs::ImagePtr img_45_1_ptr = get_img_msg_from_response(img_response_vec_45_1.at(1));

    // sensor_msgs::ImagePtr img_45_1_ptr = get_img_msg_from_response(img_response_vec_45_1.at(0));

    camera_rotated_imgs_vec_[5] = (*img_45_1_ptr);

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
    ros::Duration(1.).sleep(); // sleep to let camera change pose
    std::vector<ImageResponse> img_response_vec_90_0 = client_.simGetImages(current_image_requests_vec_, drone_name_self_);
    // viewImage(img_response_vec_90_0);
    sensor_msgs::ImagePtr img_90_0_ptr = get_img_msg_from_response(img_response_vec_90_0.at(1));

    // sensor_msgs::ImagePtr img_90_0_ptr = get_img_msg_from_response(img_response_vec_90_0.at(0));

    camera_rotated_imgs_vec_[2] = (*img_90_0_ptr);

    msr::airlib::Pose camera_pose_90_1 = giveCameraGoalPoseFormat(pose3_new, -0.021, 0.0112, altitude_flight_level_, curr_roll3, camera_downward_pitch_angle_ - camera_pitch_theta_, curr_yaw3);
    client_.simSetCameraPose("front_center", camera_pose_90_1, drone_name_self_); // set camera pose
    ros::Duration(1.).sleep(); // sleep to let camera change pose
    std::vector<ImageResponse> img_response_vec_90_1 = client_.simGetImages(current_image_requests_vec_, drone_name_self_);
    // viewImage(img_response_vec_90_1);
    sensor_msgs::ImagePtr img_90_1_ptr = get_img_msg_from_response(img_response_vec_90_1.at(1));

    // sensor_msgs::ImagePtr img_90_1_ptr = get_img_msg_from_response(img_response_vec_90_1.at(0));

    camera_rotated_imgs_vec_[6] = (*img_90_1_ptr);

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
    ros::Duration(1.).sleep(); // sleep to let camera change pose
    std::vector<ImageResponse> img_response_vec_135_0 = client_.simGetImages(current_image_requests_vec_, drone_name_self_);
    // viewImage(img_response_vec_135_0);
    sensor_msgs::ImagePtr img_135_0_ptr = get_img_msg_from_response(img_response_vec_135_0.at(1));

    // sensor_msgs::ImagePtr img_135_0_ptr = get_img_msg_from_response(img_response_vec_135_0.at(0));

    camera_rotated_imgs_vec_[3] = (*img_135_0_ptr);

    msr::airlib::Pose camera_pose_135_1 = giveCameraGoalPoseFormat(pose4_new, -0.021, 0.0112, altitude_flight_level_, curr_roll4, camera_downward_pitch_angle_ - camera_pitch_theta_, curr_yaw4);
    client_.simSetCameraPose("front_center", camera_pose_135_1, drone_name_self_); // set camera pose
    ros::Duration(1.).sleep(); // sleep to let camera change pose
    std::vector<ImageResponse> img_response_vec_135_1 = client_.simGetImages(current_image_requests_vec_, drone_name_self_);
    // viewImage(img_response_vec_135_1);
    sensor_msgs::ImagePtr img_135_1_ptr = get_img_msg_from_response(img_response_vec_135_1.at(1));

    // sensor_msgs::ImagePtr img_135_1_ptr = get_img_msg_from_response(img_response_vec_135_1.at(0));

    camera_rotated_imgs_vec_[7] = (*img_135_1_ptr);

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
    ros::Duration(1.).sleep(); // sleep to let camera change pose
}

sensor_msgs::ImagePtr ImageCoveringCoordRAG::get_img_msg_from_response(const ImageResponse& img_response)
{
    sensor_msgs::ImagePtr img_msg_ptr = boost::make_shared<sensor_msgs::Image>();
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

std::vector<double> ImageCoveringCoordRAG::computeWeightedScores(const std::vector<sensor_msgs::Image>& images)
{

    std::vector<double> scores;

    for (const auto& image : images) {

        double score = 0.0;

        // Get OpenCV Mat from image
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(image, image.encoding);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
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

double ImageCoveringCoordRAG::computeWeightedScoreSingleImage(const sensor_msgs::Image& image, bool convertBGR2RGB)
{
    double score = 0.0;

    // Get OpenCV Mat from image
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image, image.encoding);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return score;
    }
    cv::Mat img = cv_ptr->image;
    if (convertBGR2RGB) {
        cv::cvtColor(img, img, cv::COLOR_BGR2RGB); // comverts RGB to BGR

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

                    // for use with unity executable RGB , 10 drones
                    // if ((pixel == cv::Vec3b(201, 9, 144))) { // road_1
                    //     score += road_pixel_weight_;
                    // }
                    // else if ((pixel == cv::Vec3b(200, 9, 130))) { // road_2
                    //     score += road_pixel_weight_;
                    // }
                    // else if ((pixel == cv::Vec3b(65, 73, 53))) { // pavement_1
                    //     score += 0.15;
                    // }
                    // else if ((pixel == cv::Vec3b(128, 72, 0))) { // pavement_2
                    //     score += 0.15;
                    // }

                    // for use with unity editor 63DEG X 90 DEG FOV, 10 drones!!!
                    // if ((pixel == cv::Vec3b(64, 40, 182))) { // road_1
                    //     score += road_pixel_weight_;
                    // }
                    // else if ((pixel == cv::Vec3b(72, 40, 52))) { // road_2
                    //     score += road_pixel_weight_;
                    // }
                    // else if ((pixel == cv::Vec3b(194, 73, 0))) { // pavement_1
                    //     score += 0.15;
                    // }
                    // else if ((pixel == cv::Vec3b(64, 40, 180))) { // pavement_2
                    //     score += 0.15;
                    // }

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
            for (int r = 0; r < img.rows; ++r) {
                for (int c = 0; c < img.cols; ++c) {

                    cv::Vec3b pixel = img.at<cv::Vec3b>(r, c);

                    // for use with unity executable BGR
                    // if ((pixel == cv::Vec3b(144, 9, 201))) { // road_1
                    //     score += road_pixel_weight_;
                    // }
                    // else if ((pixel == cv::Vec3b(130, 9, 200))) { // road_2
                    //     score += road_pixel_weight_;
                    // }
                    // else if ((pixel == cv::Vec3b(53, 73, 65))) { // pavement_1
                    //     score += 0.15;
                    // }
                    // else if ((pixel == cv::Vec3b(0, 72, 128))) { // pavement_2
                    //     score += 0.15;
                    // }

                    // for use with unity editor 63DEG X 90 DEG FOV, 10 drones!!!
                    // if ((pixel == cv::Vec3b(182, 40, 64))) { // road_1
                    //     score += road_pixel_weight_;
                    // }
                    // else if ((pixel == cv::Vec3b(52, 40, 72))) { // road_2
                    //     score += road_pixel_weight_;
                    // }
                    // else if ((pixel == cv::Vec3b(0, 73, 194))) { // pavement_1
                    //     score += 0.15;
                    // }
                    // else if ((pixel == cv::Vec3b(180, 40, 64))) { // pavement_2
                    //     score += 0.15;
                    // }

                    // for use with unity editor 46DEG X 60 DEG FOV , 10 drones!!!
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
                    // if ((pixel == cv::Vec3b(134, 97, 25))) { // road_1
                    //     score += road_pixel_weight_;
                    // }
                    // else if ((pixel == cv::Vec3b(4, 41, 16))) { // road_2
                    //     score += road_pixel_weight_;
                    // }
                    // // else if ((pixel == cv::Vec3b(0, 73, 194))) { // pavement_1
                    // //     score += 0.05;
                    // // }
                    // // else if ((pixel == cv::Vec3b(132, 97, 25))) { // pavement_2
                    // //     score += 0.05;
                    // // }

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
double ImageCoveringCoordRAG::computeWeightedScoreSingleImage(const cv::Mat& img)
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
        // Iterate through pixels
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

    return score;
}

ros::Time ImageCoveringCoordRAG::airsim_timestamp_to_ros(const msr::airlib::TTimePoint& stamp) const
{
    // airsim appears to use chrono::system_clock with nanosecond precision
    std::chrono::nanoseconds dur(stamp);
    std::chrono::time_point<std::chrono::system_clock> tp(dur);
    ros::Time cur_time = chrono_timestamp_to_ros(tp);
    return cur_time;
}

ros::Time ImageCoveringCoordRAG::chrono_timestamp_to_ros(const std::chrono::system_clock::time_point& stamp) const
{
    auto dur = std::chrono::duration<double>(stamp.time_since_epoch());
    ros::Time cur_time;
    cur_time.fromSec(dur.count());
    return cur_time;
}

void ImageCoveringCoordRAG::isMyMarginalGainBest(bool& is_my_mg_best, std::map<int, double>& non_selected_action_inneigh_ids_mg)
{
    is_my_mg_best = true; // Assume initially that your mg is the best

    for (const auto& pair : non_selected_action_inneigh_ids_mg) {
        // Compare your mg value with each value in the map
        if (best_marginal_gain_self_ < pair.second) {
            is_my_mg_best = false; // Your mg is not the best
            break; // No need to continue checking
        }
    }
}

void ImageCoveringCoordRAG::getNearestNeighborRobots(std::vector<int>& nn_ids, int num_in_neighbors)
{
    // use neighbors_data_ which is of type airsim_ros_pkgs::NeighborsArray to get num_in_neighbors nearest neighbors
    std::vector<int> robots_ids = neighbors_data_.neighbors_array.at(robot_idx_self_ - 1).robots_range_bearing_ids;
    std::vector<double> robots_range_bearing_vals = neighbors_data_.neighbors_array.at(robot_idx_self_ - 1).robots_range_bearing_values; // contains flattened 2d array of range,bearing

    // Vector to store ranges with their corresponding robot IDs
    std::vector<std::pair<double, int>> range_id_pairs;

    // robots_range_bearing_vals stores range and bearing alternately
    for (size_t i = 0; i < robots_range_bearing_vals.size(); i += 2) {
        double range = robots_range_bearing_vals[i];
        int id = robots_ids[i / 2];
        range_id_pairs.push_back(std::make_pair(range, id));
    }

    // Sort pairs based on range
    std::sort(range_id_pairs.begin(), range_id_pairs.end(), [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
        return a.first < b.first;
    });

    // Clear nn_ids to ensure it's empty before pushing new IDs
    nn_ids.clear();

    // Push IDs of the num_in_neighbors nearest neighbors
    for (int i = 0; i < num_in_neighbors && i < static_cast<int>(range_id_pairs.size()); ++i) {
        nn_ids.push_back(range_id_pairs[i].second);
    }
    // std::cout << "robot " << robot_idx_self_ << "'s inn vec: " << nn_ids << std::endl;
}

void ImageCoveringCoordRAG::getSelectedActionsInNeighborIDImagesNearestNeighbors(std::map<int, ImageInfoStruct>& selected_action_inneigh_ids_mg, int num_in_neighbors)
{
    /* 
    Let self robot index be i; this function counts in-neighbors as the nearest N robots among in-neighbors from neighbors data
     */

    check_drones_msg_dict_.clear(); // empty the dict each call

    std::vector<int> new_inneigh_ids;

    // function to get vector of ints of nearest n neighbors from neighbors data
    getNearestNeighborRobots(new_inneigh_ids, num_in_neighbors);

    std::cout << "robot " << robot_idx_self_ << " inn vec: " << std::endl;
    printVector(new_inneigh_ids);

    // poll other drone drone topics
    ros::Duration timeout(1.5); // Timeout seconds
    // ros::Duration(0.03).sleep();  // sleep thread for 30ms to get latest message
    for (int& drone_id : new_inneigh_ids) {
        std::string mg_msg_topic = "Drone" + std::to_string(drone_id) + "/mg_rag_data";
        image_covering_coordination::ImageCoveringConstPtr mg_data_msg = ros::topic::waitForMessage<image_covering_coordination::ImageCovering>(mg_msg_topic, timeout);

        if (mg_data_msg) { // Check if the pointer is not null before dereferencing
            image_covering_coordination::ImageCovering mg_msg_topic_data = *mg_data_msg;
            if (mg_msg_topic_data.flag_completed_action_selection == true) {
                ImageInfoStruct image_info;
                image_info.pose = mg_msg_topic_data.pose;
                image_info.image = mg_msg_topic_data.image;
                image_info.camera_orientation = mg_msg_topic_data.camera_orientation_setting;
                image_info.best_marginal_gain_val = mg_msg_topic_data.best_marginal_gain; // NOTE this is from prev iteration
                selected_action_inneigh_ids_mg[drone_id] = image_info;
                std::cout << "robot " << robot_idx_self_ << " RECEIVED msg from robot " << drone_id << std::endl;
                check_drones_msg_dict_[drone_id] = true;
            }
            else {
                //   std::cout << "robot " << robot_idx_self_ << " did not receive msg from robot " << drone_id << " which has not completed action selection"
                //               << std::endl;
                std::cout << "robot " << drone_id << " has not completed action selection, assuming marginal gain is -1" << std::endl;
                ImageInfoStruct image_info;
                image_info.best_marginal_gain_val = -1.; // NOTE!!! assuming this marginal gain will always be worse in comparison
                selected_action_inneigh_ids_mg[drone_id] = image_info;

                check_drones_msg_dict_[drone_id] = false;
            }
        }
        else {
            // Handle the case where no message was received within the timeout
            ImageInfoStruct image_info;
            image_info.best_marginal_gain_val = -1.; // NOTE!!! assuming this marginal gain will always be worse in comparison
            selected_action_inneigh_ids_mg[drone_id] = image_info;
            std::cout << "robot " << robot_idx_self_ << " did not receive msg from robot " << drone_id << std::endl;

            check_drones_msg_dict_[drone_id] = false;
            continue;
        }
    }
}

void ImageCoveringCoordRAG::getSelectedActionsInNeighborIDImagesLinePath(std::map<int, ImageInfoStruct>& selected_action_inneigh_ids_mg)
{
    /* 
    Let self robot index be i; this function counts in-neighbors as robots (i-1) and (i+1)
     */
    std::vector<int> new_inneigh_ids;
    if (robot_idx_self_ == 1) {
        new_inneigh_ids.push_back(robot_idx_self_ + 1);
    }
    else if (robot_idx_self_ == num_robots_) {
        new_inneigh_ids.push_back(robot_idx_self_ - 1);
    }
    else {
        new_inneigh_ids.push_back(robot_idx_self_ - 1);
        new_inneigh_ids.push_back(robot_idx_self_ + 1);
    }

    // poll other drone drone topics
    ros::Duration timeout(1.5); // Timeout seconds
    ros::Duration(0.03).sleep(); // sleep thread for 30ms to get latest message
    for (int& drone_id : new_inneigh_ids) {
        std::string mg_msg_topic = "Drone" + std::to_string(drone_id) + "/mg_rag_data";
        image_covering_coordination::ImageCoveringConstPtr mg_data_msg = ros::topic::waitForMessage<image_covering_coordination::ImageCovering>(mg_msg_topic, timeout);

        if (mg_data_msg) { // Check if the pointer is not null before dereferencing
            image_covering_coordination::ImageCovering mg_msg_topic_data = *mg_data_msg;
            if (mg_msg_topic_data.flag_completed_action_selection == true) {
                ImageInfoStruct image_info;
                image_info.pose = mg_msg_topic_data.pose;
                image_info.image = mg_msg_topic_data.image;
                image_info.camera_orientation = mg_msg_topic_data.camera_orientation_setting;
                image_info.best_marginal_gain_val = mg_msg_topic_data.best_marginal_gain; // NOTE this is from prev iteration
                selected_action_inneigh_ids_mg[drone_id] = image_info;
            }
        }
        else {
            // Handle the case where no message was received within the timeout
            // std::cout << "could not get mg msg from robot " << drone_id << std::endl;
            // selected_action_inneigh_ids_mg[drone_id] = -1.; // setting a default value if no message received from other drone within 5 seconds
            continue;
        }
    }
}

void ImageCoveringCoordRAG::getSelectedActionsInNeighborIDImages(std::map<int, ImageInfoStruct>& selected_action_inneigh_ids_mg)
{
    /* 
    FOR ALL IN-NEIGHBORS (based on communication range) THAT HAVE SELECTED ACTIONS 
     */
    std::vector<int> all_inneigh_ids = neighbors_data_.neighbors_array.at(robot_idx_self_ - 1).in_neighbor_ids;

    // find the ids from all_inneigh_ids that are NOT already in selected_action_inneigh_ids_mg
    // This vector will hold the ids from all_inneigh_ids that are NOT already in selected_action_inneigh_ids_mg
    std::vector<int> new_inneigh_ids;

    // Iterate through all_inneigh_ids to find new IDs
    for (int& id : all_inneigh_ids) {
        if (selected_action_inneigh_ids_mg.find(id) == selected_action_inneigh_ids_mg.end()) {
            // If the id is not found in selected_action_inneigh_ids_mg, it's a new ID, so add it to the vector
            new_inneigh_ids.push_back(id);
        }
    }

    // poll other drone drone topics here
    ros::Duration timeout(1.5); // Timeout seconds
    ros::Duration(0.03).sleep(); // sleep thread for 30ms to get latest message
    for (int& drone_id : new_inneigh_ids) {
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
                image_info.best_marginal_gain_val = mg_msg_topic_data.best_marginal_gain; // NOTE this is from prev iteration
                selected_action_inneigh_ids_mg[drone_id] = image_info;
            }
        }
        else {
            // Handle the case where no message was received within the timeout
            // std::cout << "could not get mg msg from robot " << drone_id << std::endl;
            // selected_action_inneigh_ids_mg[drone_id] = -1.; // setting a default value if no message received from other drone within 5 seconds
            continue;
        }
    }
}

void ImageCoveringCoordRAG::getSelectedActionsInNeighborIDs(std::map<int, double>& selected_action_inneigh_ids_mg)
{
    std::vector<int> all_inneigh_ids = neighbors_data_.neighbors_array.at(robot_idx_self_ - 1).in_neighbor_ids;

    // poll other drone drone topics here
    ros::Duration timeout(1.5); // Timeout seconds
    for (int& drone_id : all_inneigh_ids) {
        std::string mg_msg_topic = "Drone" + std::to_string(drone_id) + "/mg_rag_data";
        image_covering_coordination::ImageCoveringConstPtr mg_data_msg = ros::topic::waitForMessage<image_covering_coordination::ImageCovering>(mg_msg_topic, timeout);
        // Check if the pointer is not null before dereferencing
        if (mg_data_msg) {
            image_covering_coordination::ImageCovering mg_msg_topic_data = *mg_data_msg;
            if (mg_msg_topic_data.flag_completed_action_selection == true) {
                selected_action_inneigh_ids_mg[drone_id] = mg_msg_topic_data.best_marginal_gain;
            }
        }
        else {
            // Handle the case where no message was received within the timeout
            // std::cout << "could not get mg msg from robot " << drone_id << std::endl;
            // selected_action_inneigh_ids_mg[drone_id] = -1.; // setting a default value if no message received from other drone within 5 seconds
            continue;
        }
    }
}

void ImageCoveringCoordRAG::getNonSelectedActionsInNeighborIDs(std::map<int, double>& non_selected_action_inneigh_ids_mg)
{
    std::vector<int> all_inneigh_ids;
    if ((graph_topology_RAG_ == "allow_disconnected") || (graph_topology_RAG_ == "RAG_AD")) {
        all_inneigh_ids = neighbors_data_.neighbors_array.at(robot_idx_self_ - 1).in_neighbor_ids;
    }
    else if ((graph_topology_RAG_ == "line_path") || (graph_topology_RAG_ == "RAG_LP")) {
        if (robot_idx_self_ == 1) {
            all_inneigh_ids.push_back(robot_idx_self_ + 1);
        }
        else if (robot_idx_self_ == num_robots_) {
            all_inneigh_ids.push_back(robot_idx_self_ - 1);
        }
        else {
            all_inneigh_ids.push_back(robot_idx_self_ - 1);
            all_inneigh_ids.push_back(robot_idx_self_ + 1);
        }
    }
    else if ((graph_topology_RAG_ == "nearest_neighbors") || (graph_topology_RAG_ == "RAG_NN")) {
        getNearestNeighborRobots(all_inneigh_ids, default_num_nearest_neighbors_);
    }

    // poll other drone drone topics here
    ros::Duration timeout(1.5); // Timeout seconds
    for (int& drone_id : all_inneigh_ids) {
        std::string mg_msg_topic = "Drone" + std::to_string(drone_id) + "/mg_rag_data";
        image_covering_coordination::ImageCoveringConstPtr mg_data_msg = ros::topic::waitForMessage<image_covering_coordination::ImageCovering>(mg_msg_topic, timeout);

        // Check if the pointer is not null before dereferencing
        if (mg_data_msg) {
            image_covering_coordination::ImageCovering mg_msg_topic_data = *mg_data_msg;
            if (mg_msg_topic_data.flag_completed_action_selection == false) {
                non_selected_action_inneigh_ids_mg[drone_id] = mg_msg_topic_data.best_marginal_gain;
            }
        }
        else {
            // Handle the case where no message was received within the timeout
            // std::cout << "could not get mg msg from robot " << drone_id << std::endl;
            // non_selected_action_inneigh_ids_mg[drone_id] = -1.; // setting a default value if no message received from other drone within 5 seconds
            non_selected_action_inneigh_ids_mg[drone_id] = 10000000000.;
        }
    }
}

void ImageCoveringCoordRAG::marginalGainFunctionRAG(double& best_marginal_gain, int& optimal_action_id, geometry_msgs::Pose latest_pose, std::map<int, ImageInfoStruct>& prev_selected_action_inneigh_ids_img)
{
    /* evalutates the best marginal gain of all control actions in actions_ 
   Given the in neighbors' IDs that have already selected an action, you want to choose your best image out of 8 images.
   step 1: get all images of in-neighbors that have already selected actions and project all those images onto the world frame; find overlap and reduce to a polygon;
   step 2: get all 8 images of your own
   step 3: for each of 8 images of self, evaluate marginal gain by removing overlap between self image and projected combined images from all in neighbors that have selected action
   step 4: find the self image that has the best marginal gain in terms of the score from semantic segmentation frames; this is also the corresponding action set
    */

    if ((graph_topology_RAG_ == "allow_disconnected") || (graph_topology_RAG_ == "RAG_AD")) {
        getSelectedActionsInNeighborIDImages(prev_selected_action_inneigh_ids_img); // NOTE!!! TEMPORARILY COMMENTED OUT FOR DEBUGGING
    }
    else if ((graph_topology_RAG_ == "line_path") || (graph_topology_RAG_ == "RAG_LP")) {
        getSelectedActionsInNeighborIDImagesLinePath(prev_selected_action_inneigh_ids_img);
    }
    else if ((graph_topology_RAG_ == "nearest_neighbors") || (graph_topology_RAG_ == "RAG_NN")) {
        getSelectedActionsInNeighborIDImagesNearestNeighbors(prev_selected_action_inneigh_ids_img, default_num_nearest_neighbors_);
    }

    std::map<int, std::pair<geometry_msgs::Pose, int>> in_neighbor_poses_and_orientations;
    for (const auto& neighbor : prev_selected_action_inneigh_ids_img) {
        if (neighbor.second.best_marginal_gain_val != -1) {
            std::pair<geometry_msgs::Pose, int> pose_cam_orient;
            pose_cam_orient.first = neighbor.second.pose;
            pose_cam_orient.second = neighbor.second.camera_orientation;
            in_neighbor_poses_and_orientations[neighbor.first] = pose_cam_orient;
        }
    }

    // FOR TESTING AND DEBUGGING: push back temporarily in-neighbors info
    // geometry_msgs::Pose dummy_pose1;
    // dummy_pose1.position.x = 28;
    // dummy_pose1.position.y = -21;
    // dummy_pose1.position.z = -30.;
    // in_neighbor_poses_and_orientations[2] = std::make_pair(dummy_pose1, 4);

    // geometry_msgs::Pose dummy_pose2;
    // dummy_pose2.position.x = -13.07;
    // dummy_pose2.position.y = -3.51;
    // dummy_pose2.position.z = -30.;
    // in_neighbor_poses_and_orientations[4] = std::make_pair(dummy_pose2, 5);

    std::map<int, double> marginal_gains_map; // Map to store control action ID and corresponding marginal gain
    if (take_new_pics_) // if false, uses the same pictures taken at previous round
    {
        collectRotatedCameraImages(); // collects 8 images
    }

    for (int i = 0; i < camera_rotated_imgs_vec_.size(); i++) {
        // Get polygon for this image
        // std::vector<cv::Point2f> my_poly = getRectangleFromPose(latest_pose, i);

        // Find non-overlapping regions
        sensor_msgs::Image masked_image = findNonOverlappingRegions(camera_rotated_imgs_vec_[i], latest_pose, i, in_neighbor_poses_and_orientations, false); // note this is in BGR!

        // Compute score on masked image
        double score = computeWeightedScoreSingleImage(masked_image, false);

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

sensor_msgs::Image ImageCoveringCoordRAG::findNonOverlappingRegions(const sensor_msgs::Image& curr_image,
                                                                    const geometry_msgs::Pose& curr_pose,
                                                                    int curr_orientation,
                                                                    std::map<int, std::pair<geometry_msgs::Pose, int>>& in_neighbor_poses_and_orientations,
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
        ROS_ERROR("cv_bridge exception: %s", e.what());
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
    int c = 0; // counter for naming png image during tests
    for (const auto& neighbor : in_neighbor_poses_and_orientations) {

        double dist = PoseEuclideanDistance(curr_pose, neighbor.second.first, curr_orientation, neighbor.second.second);
        if (dist > diagonal_frame_dist_) {
            std::string name = "round" + std::to_string(curr_time_step_samp_) + "_" + std::to_string(c) + ".png";
            // saveImageCvMat(mask, false, name); // will save to png file
            c += 1;
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
        cv::RotatedRect rect(center_rect, cv::Size2f(pasted_rect_width_pix_, pasted_rect_height_pix_), angle_rect); // angle here is clockwise +ve
        std::vector<cv::Point2f> box_points(4);
        rect.points(box_points.data());

        // convert Point2f to Point
        std::vector<cv::Point> polygon_points(box_points.begin(), box_points.end());
        box_points_vec.push_back(polygon_points);
        cv::fillPoly(mask, box_points_vec, cv::Scalar(0, 0, 0)); // NOTE!!! can directly take all polygons so it is faster
        std::string name = "round" + std::to_string(curr_time_step_samp_) + "_" + std::to_string(c) + ".png";
        // saveImageCvMat(mask, false, name); // will save to png file
        c += 1;
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
    // viewImageCvMat(extracted_region, false);

    // Convert img back to sensor_msgs::Image and return
    sensor_msgs::Image extracted_image;
    cv_bridge::CvImage cv_image{ cv_ptr->header, cv_ptr->encoding, extracted_region };
    cv_image.toImageMsg(extracted_image);

    return extracted_image;
}

// Helper function to get rectangle corners from pose + orientation
std::vector<cv::Point2f> ImageCoveringCoordRAG::getRectangleFromPose(const geometry_msgs::Pose& pose, int orientation)
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

std::vector<cv::Point2f> ImageCoveringCoordRAG::getRectangleFromPoseInNeighbor(const geometry_msgs::Pose& pose_self_robot, const geometry_msgs::Pose& pose_inn_robot,
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

void ImageCoveringCoordRAG::objectiveFunctionRAG(double& obj_func_val, std::vector<int>& in_neigh_ids_select_actions, bool& include_self_robot, int& self_robot_action_id)
{
    /* TODO: FIND SCORE OF EACH SEGMENTATION USING PRE ASSIGNED WEIGHTS TO SEMANTIC LABEL PIXELS */

    if ((in_neigh_ids_select_actions.size() == 0) && (include_self_robot == false)) {
        return;
    }

    if (include_self_robot == true) {
        in_neigh_ids_select_actions.push_back(robot_idx_self_); // will include self robot index in computing f_(t-1)
    }

    std::vector<airsim_ros_pkgs::Neighbors> neighbor_data_vec = neighbors_data_.neighbors_array;

    if (neighbor_data_vec.empty()) {
        return;
    }
    else {
        std::cout << "Timestamp of last saved PEdata msg: " << neighbors_data_.header.stamp.sec << "." << neighbors_data_.header.stamp.nsec << " seconds" << std::endl;
    }

    std::vector<airsim_ros_pkgs::Neighbors> neighbors_data_selected_ids;
    for (auto& idx : in_neigh_ids_select_actions) {
        neighbors_data_selected_ids.push_back(neighbor_data_vec.at(idx - 1));
    }

    // if include_self_robot is true then use self_robot_action_id to select the control action displacement from actions_ Eigen 3x5 matrix (cols are ids)
    // and virtually move the self robots pose.
    if (include_self_robot == true) //start if cond___________________________
    {
        airsim_ros_pkgs::Neighbors robot_pe_data;
        for (int i = 0; i < neighbors_data_selected_ids.size(); i++) {
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

void ImageCoveringCoordRAG::simulatedCommTimeDelayRAG()
{
    // NOTE!!! This is for Resource-aware Distributed Greedy
    double time_delay_per_msg = total_data_msg_size_ / communication_data_rate_;

    communication_round_ += 1;
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start_;
    std::cout << duration.count() << " COMMUNICATION ROUND COUNT: " << communication_round_ << std::endl;

    ros::Duration(time_delay_per_msg).sleep();
}

void ImageCoveringCoordRAG::simulatedCommTimeDelayRAG(double& processed_time)
{
    // NOTE!!! This is for Resource-aware Distributed Greedy
    double time_delay_per_msg = total_data_msg_size_ / communication_data_rate_;
    double comm_time = time_delay_per_msg - (processed_time / 1000.);

    communication_round_ += 1;
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start_;

    std::cout << duration.count() << " COMMUNICATION ROUND COUNT: " << communication_round_ << std::endl;

    if (comm_time > 0.) {
        ros::Duration(comm_time).sleep();
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

geometry_msgs::Pose ImageCoveringCoordRAG::convertToGeometryPose(const msr::airlib::MultirotorState& pose1)
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

void ImageCoveringCoordRAG::saveImage(std::vector<ImageResponse>& response)
{
    for (const ImageResponse& image_info : response) {
        std::cout << "Image uint8 size: " << image_info.image_data_uint8.size() << std::endl;
        std::cout << "Image float size: " << image_info.image_data_float.size() << std::endl;

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

void ImageCoveringCoordRAG::viewImage(std::vector<ImageResponse>& response)
{
    for (const ImageResponse& image_info : response) {
        std::cout << "Image uint8 size: " << image_info.image_data_uint8.size() << std::endl;
        std::cout << "Image float size: " << image_info.image_data_float.size() << std::endl;

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

// override
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
    // If image is in BGR format, swap R and B channels
    if ((image.channels() == 3) && (switchBGR_RGB_flag)) {
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    }

    // Save the image as a PNG file
    std::string path_filename = img_save_path_ + "drone" + std::to_string(robot_idx_self_) + "/" + filename; // each robot gets its own folder
    cv::imwrite(path_filename, image);
}

void ImageCoveringCoordRAG::rotateImagesInVector()
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
            ROS_ERROR("cv_bridge exception: %s", e.what());
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

double ImageCoveringCoordRAG::PoseEuclideanDistance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2, int cam_orient_1, int cam_orient_2)
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

double ImageCoveringCoordRAG::calculateAngleFromOrientation(int cam_orient)
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

int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "multi_target_coord_RAG_node");
    ros::NodeHandle nh_mtt;

    ImageCoveringCoordRAG node(nh_mtt);

    // ros::spin();
    ros::waitForShutdown(); // use with asyncspinner

    return 0;
}
/*
NOTE: This script contains the algorithms RBG and BSG only; for RAG-H view multi_target_tracking_coord_RAG.cpp
 */
#include "multi_target_tracking/multi_target_tracking_coordination.h"
// #include "multi_target_tracking/EXPStarSix.h"
using namespace std;

MultiTargetTrackingCoordination::MultiTargetTrackingCoordination(ros::NodeHandle& nh)
    : node_handle_(nh)
{
    // Get ROS params
    ros::param::get("~drone_name", drone_name_self_);
    ros::param::get("~number_of_pursuers", num_robots_);
    ros::param::get("~robot_index_ID", robot_idx_self_);
    ros::param::get("~number_of_evaders", num_evaders_);
    ros::param::get("~algorithm_to_run", algorithm_to_run_);
    ros::param::get("~camera_name", camera_name_); // need to set default here if not mentioned
    ros::param::get("~communication_range", communication_range_); // need to set default here if not mentioned
    ros::param::get("~replanning_frequency", replan_freq_);

    std::cout << "ALGORITHM: " << algorithm_to_run_ << std::endl;

    async_spinner_.reset(new ros::AsyncSpinner(2)); // use 2 threads
    async_spinner_->start();

    // ROS publishers
    std::string depth_img_pub_topic = drone_name_self_ + "/depth_image";
    depth_image_pub_ = node_handle_.advertise<sensor_msgs::Image>(depth_img_pub_topic, 10);

    total_data_msg_size_ = 1024.; // depth frame+pose will be 1024 KB
    communication_data_rate_ = 50. * total_data_msg_size_; // KB/s; now it is 50MB/s
    communication_round_ = 0;
    // set actions x,y,theta. Note x,y are independent of theta
    max_sensing_range_ = 45.;
    flight_level_ = 0.5;
    altitude_flight_level_ = -40. - 0.5 * (num_robots_ * flight_level_) + double(robot_idx_self_) * flight_level_;

    // DRONE CONTROL ACTION SETTINGS***********
    actions_.resize(3, 7); // note that these are purely displacements
    actions_ << 3.0, 2.12, 0., -2.12, -2.12, 0, 2.12,
        0, 2.12, 3.0, 2.12, -2.12, -3., -2.12,
        0., 0.7854, 1.571, 2.36, -2.36, -1.571, -0.7854;

    interp_checkpoint_total_dist_ = 3.; // fixed param NOTE!!! this will be the magnitude of actions_ rows 0,1
    drone_linear_velocity_ = 1.5;
    interp_total_time_ = interp_checkpoint_total_dist_ / drone_linear_velocity_; // seconds from curr pose to next furthest checkpoint
    drone_yaw_angular_speed_ = 22.69; // rad/s; 1300 deg/s
    // drone_yaw_angular_speed_ = 2.*M_PI; // rad/s; 2pi deg/s
    num_interpolation_pts_ = 50;
    forward_only_disp_mat_ << 3.0, 0., 0.; // should move forward only NOTE!!! only x,y are displacements; yaw theta is absolute
    // DRONE CONTROL ACTION SETTINGS END***********

    objective_function_value_ = -4.0 * max_sensing_range_; // initialized to be 4x max sensing range
    previous_robots_obj_func_val_ = -4.0 * 1000. * double(num_robots_);
    PEdata_new_msg_received_flag_ = false;
    replan_counter_ = 0;

    // EXP*-SIX class object
    int n_time_step_init = 0;
    horizon_ = 100;
    n_time_steps_ = 2000;
    double dT = horizon_ / n_time_steps_;
    curr_time_step_samp_ = 0; // this is "t" passed into expstarsix alg; initially 0
    EXPStarSIX expStarSixAlg(num_robots_, robot_idx_self_, actions_, n_time_steps_, n_time_step_init * dT); // NOTE!!! Initially, the pd of actions must be uniform distribution
    expStarSixAlg_ = expStarSixAlg; // reassigning the object for use in other class methods

    reward_ = Eigen::MatrixXd(int(n_time_steps_), 1);

    // initialize Airsim client
    // AIRSIM CLIENT USAGE CONSTANTS
    client_timeout_sec_ = (3.4028235E38F);
    msr::airlib::YawMode client_default_yaw_mode_ = msr::airlib::YawMode();
    client_lookahead_ = (-1.0F);
    client_adaptive_lookahead_ = (1.0F);
    client_margin_ = (5.0F);
    yaw_rate_timestep_ = -1.; // will use at most 1 seconds to reach goal yaw state

    replan_period_ = 1. / replan_freq_;
    last_selected_action_index_ = 2;

    client_drivetrain_ = msr::airlib::DrivetrainType::ForwardOnly;
    client_.confirmConnection();
    client_.enableApiControl(true, drone_name_self_);
    client_.armDisarm(true);
    ignore_collision_ = true;

    // takeoff pursuer drone to 40m altitude
    auto pose = client_.getMultirotorState(drone_name_self_);
    float delx = 0.;
    float dely = 0.;
    float nyaw = 0.;
    altitude_float_ = static_cast<float>(altitude_flight_level_);

    start_ = std::chrono::high_resolution_clock::now();
    last_msg_start_ = std::chrono::high_resolution_clock::now();
    ros::Duration(7.0).sleep(); // sleep seconds for takeoff
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start_;
    std::cout << "Takeoff sleep time: " << duration.count() << " seconds." << std::endl;

    msr::airlib::Pose takeoff_goal_pose = giveGoalPoseFormat(pose, delx, dely, altitude_float_, nyaw);
    client_.simSetVehiclePose(takeoff_goal_pose, ignore_collision_, drone_name_self_);

    // related to motion execution
    next_action_selected_counter_ = 0;
    motion_execution_counter_ = 0;

    // ROS subscribers
    // pursuer_evader_data_sub_ = node_handle_.subscribe("/pursuer_evader_data", 5, &MultiTargetTrackingCoordination::pursuerEvaderDataCallback, this);
    // NOTE!!! Subscribe to /clock topic for putting a callback which checks whether n_time_step_ amount of time has passed
    clock_sub_ = node_handle_.subscribe("/clock", 1, &MultiTargetTrackingCoordination::clockCallback, this); // for action selection frequency
    motion_execution_sub_ = node_handle_.subscribe("/clock", 1, &MultiTargetTrackingCoordination::motionExecutionCallback, this); // for executing the motion plan in a separate thread
}

// Save connectivity data
void MultiTargetTrackingCoordination::pursuerEvaderDataCallback(const multi_target_tracking::PursuerEvaderDataArray& msg)
{
    // set class flag saying msg was saved
    PEdata_new_msg_received_flag_ = true;

    // store last received message in a private var buffer to access asynchronously
    pursuer_evader_data_ = msg; //message contains header and PursuerEvaderData[] pursuer_evader_info_array
    // std::cout << "received and saved PE data msg in PE callback!" << std::endl;
}

void MultiTargetTrackingCoordination::motionExecutionCallback(const rosgraph_msgs::Clock& msg)
{

    int loop_initial_nasc = next_action_selected_counter_; // for forward motion checking

    if ((next_action_selected_counter_ != 0)) // ie for the first time if next action was not selected
    {
        // multi_target_tracking::PursuerEvaderDataArrayConstPtr pe_data_msg = ros::topic::waitForMessage<multi_target_tracking::PursuerEvaderDataArray>("/pursuer_evader_data");
        // multi_target_tracking::PursuerEvaderDataArray pe_data = *pe_data_msg;
        // std::vector<double> curr_pose = pe_data.pursuer_evader_info_array.at(robot_idx_self_ - 1).self_drone_pose; //pose of self drone x, y, z, theta

        msr::airlib::MultirotorState pose = client_.getMultirotorState(drone_name_self_);

        Eigen::Matrix<double, 3, 1> selected_action_displacement = actions_.block(0, last_selected_action_index_, 3, 1); // access latest set goal direction action index; also includes x,y displacement for interpolation

        // <function that takes curr_position, selected_action_displacement, update rate of motion (number of interpolated points) and returns FIFO queue of interpolated points as Pose objects>
        std::queue<msr::airlib::Pose> interpolated_points_queue = interpolatePose(selected_action_displacement, num_interpolation_pts_, pose, false);
        int initial_queue_size = interpolated_points_queue.size();

        // TODO: for loop through queue, pop first pose, then setting it, sleep this thread
        // float duration = 1. / num_interpolation_pts_; // Get interpolation time step
        float total_duration = interp_checkpoint_total_dist_ / drone_linear_velocity_;
        float checkpoint_step_duration = total_duration / num_interpolation_pts_;

        for (int i = 0; i < initial_queue_size; i++) {

            msr::airlib::Pose next_pose = interpolated_points_queue.front(); // Get next pose
            interpolated_points_queue.pop(); //remove the first element; note that queue size changes

            bool ignore_collision = true;
            client_.simSetVehiclePose(next_pose, ignore_collision, drone_name_self_); // Set vehicle pose

            // std::this_thread::sleep_for(std::chrono::duration<double>(checkpoint_step_duration)); // Sleep thread after setting pose
            ros::Duration(checkpoint_step_duration).sleep();

            motion_execution_counter_ += 1;

            int diff1 = next_action_selected_counter_ - motion_execution_counter_;
            if (diff1 == 0) {
                continue; // continue executing current action
            }
            else if (diff1 > 0) {
                return; // break out of this function and go to next selected action TODO could be an issue if action selection rate is > 50Hz (high)
            }
            else if (diff1 < 0) {
                motion_execution_counter_ += diff1; // reset the me counter; this is edge case
                break; // go to the forward only motion execution
            }

            // check queue size
            if (interpolated_points_queue.size() == 0) // execute given action for as long as possible
            {
                int diff2 = next_action_selected_counter_ - motion_execution_counter_;
                if (diff2 == 0) {
                    break;
                }
                else if (diff1 > 0) {
                    return;
                }
                else if (diff1 < 0) {
                    motion_execution_counter_ += diff1; // reset the me counter
                    break; // go to the forward only motion execution
                }
            }
        }

        msr::airlib::MultirotorState pose_latest = client_.getMultirotorState(drone_name_self_);
        std::queue<msr::airlib::Pose> interpolated_forward_points_queue = interpolatePose(forward_only_disp_mat_, num_interpolation_pts_, pose_latest, true);
        int initial_forward_queue_size = interpolated_forward_points_queue.size();

        for (int i = 0; i < initial_forward_queue_size; i++) {
            msr::airlib::Pose next_pose = interpolated_forward_points_queue.front(); // Get next pose
            interpolated_forward_points_queue.pop(); //remove the first element; note that queue size changes

            bool ignore_collision = true;
            client_.simSetVehiclePose(next_pose, ignore_collision, drone_name_self_); // Set vehicle pose

            // std::this_thread::sleep_for(std::chrono::duration<double>(checkpoint_step_duration)); // Sleep thread after setting pose
            ros::Duration(checkpoint_step_duration).sleep();
            if (loop_initial_nasc != next_action_selected_counter_) {
                return;
            }

            if (interpolated_forward_points_queue.size() == 0) {
                return;
            }
        }
    }
}


// Get wall clock time; algorithm continues from here
void MultiTargetTrackingCoordination::clockCallback(const rosgraph_msgs::Clock& msg)
{
    // NOTE!!! USE .self_drone_pose in pe data to get self drone's pose

    replan_counter_ += 1;
    auto time_elapsed = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = time_elapsed - last_msg_start_;

    if (duration.count() < replan_period_) {
        std::cout << "duration: " << duration.count() << " replan_period: " << replan_period_ << std::endl;
        // continue executing the last selected action in an infinite-horizon MPC
        Eigen::Matrix<double, 3, 1> selected_action_displacement = actions_.block(0, last_selected_action_index_, 3, 1);

        if (last_selected_action_index_ != -1) {
            if (algorithm_to_run_ == "BSG") {

                std::cout << "waiting for replan time to pass" << std::endl;
                // client_.moveByVelocityZBodyFrameAsync(3.0, 0, altitude_flight_level_, 1.0, client_drivetrain_, client_default_yaw_mode_, drone_name_self_);
            }

            else if (algorithm_to_run_ == "RBG") {

                std::cout << "waiting for replan time to pass" << std::endl;
                // client_.moveByVelocityZBodyFrameAsync(2.5, 0, altitude_flight_level_, 2., client_drivetrain_, client_default_yaw_mode_, drone_name_self_);
            }
        }

        return;
    }

    // draw an action from EXPStarSIX probability distribution______
    expStarSixAlg_.update_action_prob_dist(curr_time_step_samp_);
    int selected_action_ind;
    Eigen::MatrixXd action_pd;
    expStarSixAlg_.getActionProbDist(action_pd);
    drawDiscreteRandomSample(curr_time_step_samp_, action_pd, selected_action_ind);
    expStarSixAlg_.setSelectedActionIndex(curr_time_step_samp_, selected_action_ind); // set selected action index in expstarsix class
    last_selected_action_index_ = selected_action_ind;
    std::cout << "curr time step: " << curr_time_step_samp_ << ", selected action index: " << last_selected_action_index_ << endl;

    Eigen::Matrix<double, 3, 1> selected_action_displacement = actions_.block(0, last_selected_action_index_, 3, 1);

    next_action_selected_counter_ += 1; // incrementing this is going to tell motion Execution Callback to execute this last selected action
    last_msg_start_ = std::chrono::high_resolution_clock::now();

    double finish_execution_dur = 1. / 25.; // sleep shortly so the drone can finish executing action
    // std::this_thread::sleep_for(std::chrono::duration<double>(finish_execution_dur)); // Sleep thread after setting pose

    // multi_target_tracking::PursuerEvaderDataArrayConstPtr pe_data_msg = ros::topic::waitForMessage<multi_target_tracking::PursuerEvaderDataArray>("/pursuer_evader_data");
    // pursuer_evader_data_ = *pe_data_msg; // get a single PursuerEvaderDataArray message

    // evaluate rewards and loss from objective function and reward for EXPStarSIX______
    auto start = std::chrono::high_resolution_clock::now();
    double obj_function_marginal_gain;

    if (algorithm_to_run_ == "BSG") {
        simulatedCommTimeDelayBSG();
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        // std::cout << "Communication delay time: " << duration.count() << " seconds." << std::endl;

        marginalGainFunctionBSG(obj_function_marginal_gain);
        std::cout << "BSG objective function MARGINAL GAIN value: " << obj_function_marginal_gain << std::endl;
    }
    else if (algorithm_to_run_ == "RBG") {
        simulatedCommTimeDelayRBG();
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        // std::cout << "Communication delay time: " << duration.count() << " seconds." << std::endl;

        //use the in-neighbors array for this drone in the marginal gain function
        marginalGainFunctionRBG(obj_function_marginal_gain);
        std::cout << "curr time step: " << curr_time_step_samp_ << ", RBG objective function MARGINAL GAIN value: " << obj_function_marginal_gain << std::endl;
    }

    // calculate reward and loss, assign to EXPStarSIX object
/*     if (curr_time_step_samp_ == 0) // encourage first action as good if there are drones in view
    {
        reward_(curr_time_step_samp_, 0) = 1.;  // FOR TESTING
    }
    else {
        reward_(curr_time_step_samp_, 0) = obj_function_marginal_gain / (0. - previous_robots_obj_func_val_);
    }
 */
    reward_(curr_time_step_samp_, 0) = obj_function_marginal_gain / (0. - previous_robots_obj_func_val_);

    std::cout << "curr time step: " << curr_time_step_samp_ << ", normalized REWARD: " << reward_(curr_time_step_samp_, 0) << std::endl;

    if ((reward_(curr_time_step_samp_, 0) < 0.) || (reward_(curr_time_step_samp_, 0) > 1.)) {
        std::cout << "WARN: WRONG REWARD VALUE COMPUTED !(0. < reward < 1.)" << std::endl;
    }
    double loss_normalized = 1. - reward_(curr_time_step_samp_, 0);
    std::cout << "curr time step: " << curr_time_step_samp_ << ", normalized LOSS: " << loss_normalized << std::endl;

    expStarSixAlg_.setLossValues(curr_time_step_samp_, loss_normalized); // assign loss to expstarsix class obj

    // update the EXPStarSIX using the reward_______
    expStarSixAlg_.update_experts(curr_time_step_samp_); // update weights for drawing action on next time step

    curr_time_step_samp_++; // increment by 1 for each t in T
}

void MultiTargetTrackingCoordination::marginalGainFunctionBSG(double& marginal_gain)
{
    // first calculate f_t(a_i | A_(i-1)) = f_t(a_i U A_(i-1))
    double total_ft;
    objectiveFunctionBSG(total_ft, robot_idx_self_);

    double val_ft_aim1; // this is f_t of all robots until robot i-1, where i is current robot
    int prev_drone_id = robot_idx_self_ - 1;
    objectiveFunctionBSG(val_ft_aim1, prev_drone_id);

    previous_robots_obj_func_val_ = val_ft_aim1;

    marginal_gain = total_ft - val_ft_aim1;
}

void MultiTargetTrackingCoordination::objectiveFunctionBSG(double& obj_func_val, int& up_to_drone_id)
{
    // (NOTE: with marginal gain in sequential order)
    multi_target_tracking::PursuerEvaderDataArrayConstPtr pe_data_msg = ros::topic::waitForMessage<multi_target_tracking::PursuerEvaderDataArray>("/pursuer_evader_data");
    pursuer_evader_data_ = *pe_data_msg; // get a single PursuerEvaderDataArray message

    std::vector<multi_target_tracking::PursuerEvaderData> pe_data_vec = pursuer_evader_data_.pursuer_evader_info_array;
    if (pe_data_vec.empty()) {
        std::cout << "pursuer_evader_data_ vector is empty." << std::endl;
        return;
    }
    else {
        std::cout << "Timestamp of last saved PEdata msg: " << pursuer_evader_data_.header.stamp.sec << "." << pursuer_evader_data_.header.stamp.nsec << " seconds" << std::endl;
    }

    std::map<int, std::vector<double>> evader_observer_ranges; // key int is evader ID, std::vector<double> is vector of ranges of pursuers which are observing evader in ID

    for (int i = 0; i < up_to_drone_id; i++) {
        auto curr_pursuer_drone_data = pe_data_vec.at(i);
        std::vector<double> distvec; // empty vec initialized
        std::vector<int> observer_ids_in_fov = curr_pursuer_drone_data.evaders_in_FOV;
        for (int& j : observer_ids_in_fov) {
            evader_observer_ranges[j] = distvec;
        }
    }

    for (int i = 0; i < up_to_drone_id; i++) {
        auto curr_pursuer_drone_data = pe_data_vec.at(i);
        std::vector<int> observer_ids_in_fov = curr_pursuer_drone_data.evaders_in_FOV;

        for (int& evader_id : observer_ids_in_fov) {
            if (evader_observer_ranges.find(evader_id) != evader_observer_ranges.end()) {
                // Check if the pursuer observes the evader
                if (std::find(curr_pursuer_drone_data.evaders_in_FOV.begin(), curr_pursuer_drone_data.evaders_in_FOV.end(), evader_id) != curr_pursuer_drone_data.evaders_in_FOV.end()) {
                    // Find the range between the evader and pursuer and push it back to the vector
                    // int index = std::find(curr_pursuer_drone_data.evaders_in_FOV.begin(), curr_pursuer_drone_data.evaders_in_FOV.end(), evader_id) - curr_pursuer_drone_data.evaders_in_FOV.begin();
                    int index = std::find(curr_pursuer_drone_data.evaders_range_bearing_ids.begin(), curr_pursuer_drone_data.evaders_range_bearing_ids.end(), evader_id) - curr_pursuer_drone_data.evaders_range_bearing_ids.begin();
                    double range = curr_pursuer_drone_data.evaders_range_bearing_values[2 * index];
                    evader_observer_ranges[evader_id].push_back(range);
                }
            }
        }
    }

    // Populate evader_observer_ranges with default distance for unobserved evaders
    for (int i = (num_robots_ + 1); i <= (num_robots_ + num_evaders_ + 1); i++) {
        if (evader_observer_ranges.find(i) == evader_observer_ranges.end()) {
            // Evader not observed, assign default distance
            std::vector<double> distvec = { 4 * max_sensing_range_ };
            evader_observer_ranges[i] = distvec;
        }
    }

    // Calculate the objective function value
    obj_func_val = 0.0; // Initialize the sum

    // Iterate through the map and perform the required calculations
    for (const auto& entry : evader_observer_ranges) {
        int evader_id = entry.first;
        const std::vector<double>& distances = entry.second;

        // Sum the reciprocal of distances for the current key's vector
        double sum_reciprocal_distances = 0.0;
        for (double distance : distances) {
            sum_reciprocal_distances += 1.0 / distance;
        }

        // Multiply by -1, take the reciprocal, and add to the overall objective function value
        obj_func_val += (1.0 / (-1.0 * sum_reciprocal_distances));
    }
}

void MultiTargetTrackingCoordination::marginalGainFunctionRBG(double& marginal_gain)
{
    multi_target_tracking::PursuerEvaderDataArrayConstPtr pe_data_msg = ros::topic::waitForMessage<multi_target_tracking::PursuerEvaderDataArray>("/pursuer_evader_data");
    pursuer_evader_data_ = *pe_data_msg; // get a single PursuerEvaderDataArray message

    std::vector<multi_target_tracking::PursuerEvaderData> pe_data_vec = pursuer_evader_data_.pursuer_evader_info_array;
    std::vector<int> in_neighbors_ids = pe_data_vec.at(robot_idx_self_ - 1).in_neighbor_ids;

    std::vector<int> self_and_in_neighbor_ids = in_neighbors_ids;
    self_and_in_neighbor_ids.push_back(robot_idx_self_);

    double total_ft;
    objectiveFunctionRBG(total_ft, self_and_in_neighbor_ids);

    double val_ft_aim1; // this is f_t of all in neighbor robots until robot i-1, where i is current robot
    objectiveFunctionRBG(val_ft_aim1, in_neighbors_ids);
    previous_robots_obj_func_val_ = val_ft_aim1;

    marginal_gain = total_ft - val_ft_aim1;
}

void MultiTargetTrackingCoordination::objectiveFunctionRBG(double& obj_func_val, std::vector<int>& drone_self_in_neighbors_ids)
{
    multi_target_tracking::PursuerEvaderDataArrayConstPtr pe_data_msg = ros::topic::waitForMessage<multi_target_tracking::PursuerEvaderDataArray>("/pursuer_evader_data");
    pursuer_evader_data_ = *pe_data_msg; // get a single PursuerEvaderDataArray message

    std::vector<multi_target_tracking::PursuerEvaderData> pe_data_vec = pursuer_evader_data_.pursuer_evader_info_array; // contains data of all ids
    if (pe_data_vec.empty()) {
        std::cout << "pursuer_evader_data_ vector is empty." << std::endl;
        return;
    }
    else {
        //    std::cout << "Timestamp of last saved PEdata msg: " << pursuer_evader_data_.header.stamp.sec << "." << pursuer_evader_data_.header.stamp.nsec << " seconds" << std::endl;
    }

    std::vector<multi_target_tracking::PursuerEvaderData> pe_data_selected_ids;
    for (auto& idx : drone_self_in_neighbors_ids) {
        pe_data_selected_ids.push_back(pe_data_vec.at(idx - 1));
    }

    std::map<int, std::vector<double>> evader_observer_ranges; // key int is evader ID, std::vector<double> is vector of ranges of pursuers which are observing evader in ID

    for (auto& curr_pursuer_drone_data : pe_data_selected_ids) {
        std::vector<double> distvec; // empty vec initialized
        std::vector<int> observer_ids_in_fov = curr_pursuer_drone_data.evaders_in_FOV;

        for (int& j : observer_ids_in_fov) {
            evader_observer_ranges[j] = distvec;
        }
    }

    for (int i = 0; i < pe_data_selected_ids.size(); i++) {
        auto curr_pursuer_drone_data = pe_data_selected_ids.at(i);
        std::vector<int> observer_ids_in_fov = curr_pursuer_drone_data.evaders_in_FOV;

        for (int& evader_id : observer_ids_in_fov) {
            if (evader_observer_ranges.find(evader_id) != evader_observer_ranges.end()) {
                // Check if the pursuer observes the evader
                if (std::find(curr_pursuer_drone_data.evaders_in_FOV.begin(), curr_pursuer_drone_data.evaders_in_FOV.end(), evader_id) != curr_pursuer_drone_data.evaders_in_FOV.end()) {
                    // Find the range between the evader and pursuer and push it back to the vector
                    int index = std::find(curr_pursuer_drone_data.evaders_range_bearing_ids.begin(), curr_pursuer_drone_data.evaders_range_bearing_ids.end(), evader_id) - curr_pursuer_drone_data.evaders_range_bearing_ids.begin();
                    double range = curr_pursuer_drone_data.evaders_range_bearing_values[2 * index];
                    evader_observer_ranges[evader_id].push_back(range);
                }
            }
        }
    }

    // Populate evader_observer_ranges with default distance for unobserved evaders
    for (int i = (num_robots_ + 1); i < (num_robots_ + num_evaders_ + 1); i++) {
        if (evader_observer_ranges.find(i) == evader_observer_ranges.end()) {
            // Evader not observed, assign default distance
            std::vector<double> distvec = { 4. * 500. };
            evader_observer_ranges[i] = distvec;
        }
    }

    // Calculate the objective function value
    obj_func_val = 0.0; // Initialize the sum

    // Iterate through the map and perform the required calculations
    for (const auto& entry : evader_observer_ranges) {
        int evader_id = entry.first;
        const std::vector<double>& distances = entry.second;

        // Sum the reciprocal of distances for the current key's vector
        double sum_reciprocal_distances = 0.0;
        for (double distance : distances) {
            // sum_reciprocal_distances += 1.0 / distance;
            if (distance <= 0.05) {
                distance = 0.005;
            }

            sum_reciprocal_distances += 1.0 / double(distance); //NOTE!!! subtraction value is to keep a safe distance to evaders
        }

        // Multiply by -1, take the reciprocal, and add to the overall objective function value
        obj_func_val += (1.0 / (-1.0 * sum_reciprocal_distances));
    }
}

msr::airlib::Pose MultiTargetTrackingCoordination::giveGoalPoseFormat(msr::airlib::MultirotorState& curr_pose, float disp_x, float disp_y, float disp_z, float new_yaw)
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

// Convert Euler angles to quaternion
Eigen::Quaternionf MultiTargetTrackingCoordination::quaternionFromEuler(float roll, float pitch, float yaw)
{
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
    return q;
}

// Convert quaternion to Euler angles
Eigen::Vector3f MultiTargetTrackingCoordination::eulerFromQuaternion(const Eigen::Quaternionf& q)
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

void MultiTargetTrackingCoordination::drawDiscreteRandomSample(int& curr_time_step_samp, Eigen::MatrixXd& action_prob_dist, int& selected_action_index)
{
    // Extract a row as an Eigen vector
    Eigen::VectorXd weights_row = action_prob_dist.row(curr_time_step_samp);

    // Convert Eigen vector to std::vector
    std::vector<double> weights_vector(weights_row.data(), weights_row.data() + weights_row.size());

    // Use std::discrete_distribution with your weights
    std::random_device rd;
    std::mt19937 rng(rd());
    std::discrete_distribution<int> dist(weights_vector.begin(), weights_vector.end());
    selected_action_index = dist(rng);
}

void MultiTargetTrackingCoordination::simulatedCommTimeDelayBSG()
{
    double time_delay_per_msg = total_data_msg_size_ / communication_data_rate_;
    double delay_factor = robot_idx_self_ * (robot_idx_self_ - 1) / 2;
    // double delay_factor = double(robot_idx_self_); // for test
    int total_time_delay = delay_factor * time_delay_per_msg; // milliseconds
    communication_round_ += 1;
    if (total_time_delay != 0) {
        // std::cout << "Communication delay of " << total_time_delay << " s" << std::endl;
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start_;
        std::cout << duration.count() << " COMMUNICATION ROUND COUNT: " << communication_round_ << std::endl;
        ros::Duration(total_time_delay).sleep();
    }
}

void MultiTargetTrackingCoordination::simulatedCommTimeDelayRBG()
{
    // NOTE!!! This is for Resource-aware Bandit Greedy
    double time_delay_per_msg = total_data_msg_size_ / communication_data_rate_;
    // std::cout << "Communication delay of " << time_delay_per_msg << " s" << std::endl;

    communication_round_ += 1;
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start_;

    std::cout << duration.count() << " COMMUNICATION ROUND COUNT: " << communication_round_ << std::endl;

    ros::Duration(time_delay_per_msg).sleep();
}

double MultiTargetTrackingCoordination::normalizeYaw(double yaw)
{
    yaw = fmod(yaw, 2 * M_PI);
    if (yaw > M_PI) {
        yaw -= 2 * M_PI;
    }
    return yaw;
}

float MultiTargetTrackingCoordination::yawAngleDiff(float targetA, float sourceA)
{
    float a = targetA - sourceA;
    if (a > M_PI) {
        a -= 2 * M_PI;
    }
    else if (a < -M_PI) {
        a += 2 * M_PI;
    }
    // float a = a2 - a1;
    // return ((a > M_PI) ? a - 2 * M_PI : a);
    return a;
}

std::queue<msr::airlib::Pose> MultiTargetTrackingCoordination::interpolatePose(const Eigen::Matrix<double, 3, 1>& selected_action_displacement,
                                                                               int num_interp_points, msr::airlib::MultirotorState curr_pose, bool use_currpose_yaw)
{
    Eigen::Vector3f curr_pose_vec = curr_pose.getPosition();
    // Get current x, y, z
    float curr_x = curr_pose_vec(0);
    float curr_y = curr_pose_vec(1);
    float curr_z = curr_pose_vec(2);

    std::queue<msr::airlib::Pose> interpolated_poses;
    float yaw_theta, yaw_step, curr_yaw, curr_yaw_norm, yaw_theta_norm; // note yaw_theta here is the goal absolute yaw
    float num_interp_pts = static_cast<float>(num_interp_points);
    float num_interp_points_yaw = 1.; // NOTE!!! this sets how fast yawing happens
    bool yaw_updated = false;
    double yaw_diff;
    // set number of checkpoints during which yaw is altered, and then stops yawing
    double step_time_single_chkpt = interp_total_time_ / num_interpolation_pts_;

    float delta_x, delta_y, delta_forward_body_frame;

    double yaw_time_needed;
    if (!(use_currpose_yaw)) {
        // get current yaw
        auto orientation = curr_pose.kinematics_estimated.pose.orientation;
        Eigen::Vector3f euler = eulerFromQuaternion(orientation.cast<float>());
        curr_yaw = euler[2];

        yaw_theta = selected_action_displacement(2, 0); // goal yaw angle
        // yaw_step = yaw_theta / num_interp_points_yaw; //NOTE!! using different step size for yaw because we want it to yaw at high rate

        // find yaw steps and num_interp_points_yaw based on the angular rate
        yaw_diff = yawAngleDiff(yaw_theta, curr_yaw); // target, source

        yaw_time_needed = abs(yaw_diff / drone_yaw_angular_speed_);
        num_interp_points_yaw = ceil(yaw_time_needed / step_time_single_chkpt);
        yaw_step = drone_yaw_angular_speed_ * step_time_single_chkpt; // make sure that on last num_interp_points_yaw number the robot has reached goal yaw
        if (abs(yaw_diff) < yaw_step) {
            yaw_step = yaw_diff;
            num_interp_points_yaw = 1;
        }

        // Get delta x, y, z from action
        delta_x = selected_action_displacement(0, 0);
        delta_y = selected_action_displacement(1, 0);
    }
    else if (use_currpose_yaw) { // this is if it is moving in a straight line
        auto orientation = curr_pose.kinematics_estimated.pose.orientation;
        Eigen::Vector3f euler = eulerFromQuaternion(orientation.cast<float>());
        yaw_theta = euler[2]; // absolute yaw at which the drone needs to be; this is also the current yaw
        yaw_step = 0.; //NOTE!! using different step size for yaw because we want it to yaw at high rate

        delta_forward_body_frame = selected_action_displacement(0, 0);
        delta_x = delta_forward_body_frame * cos(yaw_theta);
        delta_y = delta_forward_body_frame * sin(yaw_theta);
    }

    // Calculate step size for each coordinate
    float x_step = delta_x / num_interp_pts;
    float y_step = delta_y / num_interp_pts;

    // interpolated step distance
    float interp_step_distance = interp_checkpoint_total_dist_ / num_interpolation_pts_; // need to use updated angle to find x (forward) and y (sideways right) component

    float finished_yawing_yaw = -1.;
    float num_yaw_points_added = 0;

    Eigen::Vector3f posevec_prev_timestep;
    msr::airlib::MultirotorState last_pose;

    float next_x_displacement, next_y_displacement;
    // Generate interpolated positions
    for (int i = 1; i <= num_interp_points; i++) {

        // finding X and Y translation components***********
        float next_yaw = 0.;
        float x_step_rotating, y_step_rotating;
        if (use_currpose_yaw) {
            next_x_displacement = i * x_step;
            next_y_displacement = i * y_step;
            // next_yaw = yaw_theta;
        }
        else if (!(use_currpose_yaw) && (num_yaw_points_added > num_interp_points_yaw)) {
            next_x_displacement = i * x_step;
            next_y_displacement = i * y_step;
        }
        //**************************************************

        if ((!yaw_updated) && (num_yaw_points_added <= num_interp_points_yaw)) {

            if (use_currpose_yaw) // IF FOLLOWING STRAIGHT LINE FORWARD MOTION*******
            {
                next_yaw = yaw_theta;
                if (num_yaw_points_added == num_interp_points_yaw) {
                    yaw_updated = true;
                    finished_yawing_yaw = next_yaw;
                }
            }

            if (!(use_currpose_yaw)) // IF YAWING WHILE TRANSLATING ************
            {
                if (yaw_diff < 0) // CCW direction
                {

                    if (num_yaw_points_added == (num_interp_points_yaw - 1)) {
                        next_yaw = yaw_theta;
                    }
                    else {
                        next_yaw = i * yaw_step;
                    }
                }
                else if (yaw_diff > 0) {

                    if (num_yaw_points_added == (num_interp_points_yaw - 1)) {
                        next_yaw = yaw_theta;
                    }
                    else {
                        next_yaw = i * yaw_step; // this is absolute yaw
                    }
                }

                // next_yaw = yaw_step + i * yaw_step;  // current absolute yaw plus an increment

                // find new x and y step based on angle while rotating
                x_step_rotating = interp_step_distance * cos(next_yaw);
                y_step_rotating = interp_step_distance * sin(next_yaw);
                next_x_displacement = x_step_rotating;
                next_y_displacement = y_step_rotating;

                if (num_yaw_points_added == (num_interp_points_yaw-1)) {
                    next_yaw = yaw_theta;
                    yaw_updated = true;
                    finished_yawing_yaw = next_yaw;
                }
            }

            num_yaw_points_added += 1;
        }

        else {
            next_yaw = finished_yawing_yaw; // absolute
        }

        if (i == 1) {
            last_pose = curr_pose;
        }

        // Convert to Pose and add to queue NOTE:: USE LAST POSE here not curr pose!!!
        msr::airlib::Pose next_pose = giveGoalPoseFormat(last_pose, next_x_displacement, next_y_displacement, altitude_float_, next_yaw);

        // update last pose here
        posevec_prev_timestep << last_pose.getPosition()(0) + next_x_displacement, last_pose.getPosition()(1) + next_y_displacement, curr_z;
        last_pose.kinematics_estimated.pose.position = posevec_prev_timestep; // set position here
        Eigen::Quaternionf qrot = quaternionFromEuler(0., 0., next_yaw);
        last_pose.kinematics_estimated.pose.orientation = qrot;

        interpolated_poses.push(next_pose);
    }

    return interpolated_poses;
}

int main(int argc, char** argv)
{

    // Initialize ROS node
    ros::init(argc, argv, "multi_target_coord_node");
    ros::NodeHandle nh_mtt;

    MultiTargetTrackingCoordination node(nh_mtt);

    // ros::spin();
    ros::waitForShutdown(); // use with async multithreaded spinner

    return 0;
}

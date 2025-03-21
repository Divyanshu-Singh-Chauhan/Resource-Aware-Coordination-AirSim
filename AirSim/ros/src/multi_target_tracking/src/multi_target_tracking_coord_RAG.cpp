/*
NOTE: This script contains the algorithms RBG and BSG only; for RAG-H view multi_target_tracking_coord_RAG.cpp
 */
#include "multi_target_tracking/multi_target_tracking_coord_RAG.h"

MultiTargetTrackingCoordRAG::MultiTargetTrackingCoordRAG(ros::NodeHandle& nh)
    : node_handle_(nh)
{
    // Get ROS params
    ros::param::get("~drone_name", drone_name_self_);
    ros::param::get("~number_of_pursuers", num_robots_);
    ros::param::get("~robot_index_ID", robot_idx_self_);
    ros::param::get("~number_of_evaders", num_evaders_);
    ros::param::get("~camera_name", camera_name_); // need to set default here if not mentioned
    ros::param::get("~communication_range", communication_range_); // need to set default here if not mentioned

    std::cout << "ALGORITHM: RAGH" << std::endl;

    async_spinner_.reset(new ros::AsyncSpinner(4)); 
    async_spinner_->start();

    // ROS publishers
    std::string depth_img_pub_topic = drone_name_self_ + "/depth_image";
    depth_image_pub_ = node_handle_.advertise<sensor_msgs::Image>(depth_img_pub_topic, 10);

    std::string mg_msg_topic = drone_name_self_ + "/mg_rag_data";
    mg_rag_msg_pub_ = node_handle_.advertise<multi_target_tracking::MarginalGainRAG>(mg_msg_topic, 5);

    // ROS subscribers
    // pursuer_evader_data_sub_ = node_handle_.subscribe("/pursuer_evader_data", 5, &MultiTargetTrackingCoordRAG::pursuerEvaderDataCallback, this);
    // NOTE!!! Subscribe to /clock topic for putting a callback which checks whether n_time_step_ amount of time has passed
    clock_sub_ = node_handle_.subscribe("/clock", 2, &MultiTargetTrackingCoordRAG::clockCallback, this); // for action selection frequency
    marginal_gain_pub_clock_sub_ = node_handle_.subscribe("/clock", 2, &MultiTargetTrackingCoordRAG::marginalGainPublisherclockCallback, this); // for action selection frequency
    check_pursuers_selection_sub_ = node_handle_.subscribe("/clock", 2, &MultiTargetTrackingCoordRAG::updateAllRobotsSelectionStatusCallback, this);


    total_data_msg_size_ = 1024.; // depth frame+pose will be 1024 KB
    communication_data_rate_ = 50. * total_data_msg_size_; // KB/s
    communication_round_ = 0;

    // set actions x,y,theta. Note x,y are independent of theta
    max_sensing_range_ = 45.;
    // max_sensing_range_ = 80.;
    flight_level_ = 0.5;
    altitude_flight_level_ = -40. - 0.5 * (num_robots_ * flight_level_) + double(robot_idx_self_) * flight_level_;

    selected_action_ind_ = -1; // initialized to be -1 indicating no action has been selected yet
    best_marginal_gain_self_ = -1.;
    finished_action_selection_ = false;
    waiting_for_robots_to_finish_selection_ = false;
    takeoff_wait_flag_ = true;

    fov_x_ = M_PI / 2.; // in rads; 90 deg
    fov_y_ = 1.274; // in rads; 73 deg

    actions_.resize(3, 7);
    // actions_ << 2.12, 2.77, 3.0, 2.77, 2.12,
    //     -2.12, -1.15, 0, 1.15, 2.12,
    //     -0.7854, -0.393, 0, 0.393, 0.7854;
    actions_ << 1.15, 2.12, 2.77, 3.0, 2.77, 2.12, 1.15,
            -2.772, -2.12, -1.15, 0, 1.15, 2.12, 2.772, 
            -1.18, -0.7854, -0.393, 0, 0.393, 0.7854, 1.18;

    objective_function_value_ = -4.0 * max_sensing_range_; // initialized to be 4x max sensing range
    // previous_robots_obj_func_val_ = -4.0 * max_sensing_range_ * double(num_robots_);
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

    client_drivetrain_ = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    client_.confirmConnection(); // may need to put this in callback
    client_.enableApiControl(true, drone_name_self_);
    client_.armDisarm(true);

    // takeoff pursuer drone to 40m altitude here
    // client_.moveToZAsync(-40.0, 8.0, client_timeout_sec_, client_default_yaw_mode_, client_lookahead_, client_adaptive_lookahead_, drone_name_self_)->waitOnLastTask();
    
    // auto start = std::chrono::high_resolution_clock::now();
    // ros::Duration(4.0).sleep(); // sleep 4 seconds
    // auto end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> duration = end - start;
    // std::cout << "Takeoff sleep time: " << duration.count() << " seconds." << std::endl;
    // takeoff_wait_flag_ = false;

    for (int i = 0; i < num_robots_; i++)
    {
        all_robots_selected_map_[i+1] = false; // initialized all to be false, no pursuer has initially selected an action
    }
    
    start_ = std::chrono::high_resolution_clock::now();
}

// destructor
MultiTargetTrackingCoordRAG::~MultiTargetTrackingCoordRAG() {
  async_spinner_->stop();
}

void MultiTargetTrackingCoordRAG::marginalGainPublisherclockCallback(const rosgraph_msgs::Clock& msg) {
    // access last set best marginal gain from clock callback and continuously publish it
    multi_target_tracking::MarginalGainRAG mg_msg;
    mg_msg.header.stamp = ros::Time::now();
    mg_msg.best_marginal_gain = best_marginal_gain_self_;
    mg_msg.flag_completed_action_selection = finished_action_selection_;
    // std::cout << mg_msg.flag_completed_action_selection << " : "<< finished_action_selection_ << std::endl;
    mg_rag_msg_pub_.publish(mg_msg);
    // std::cout << std::boolalpha;
    // std::cout << "Published mg_rag msg with best marginal gain: " <<  mg_msg.best_marginal_gain << " and action selection flag: "<< std::to_string(mg_msg.flag_completed_action_selection) << std::endl;
}


void MultiTargetTrackingCoordRAG::updateAllRobotsSelectionStatusCallback(const rosgraph_msgs::Clock& msg) {

    ros::Duration timeout(5.0);  // Timeout of 5 seconds
    for (int i = 0; i < num_robots_; i++)
    {
        std::string mg_msg_topic = "Drone" + std::to_string(i+1) + "/mg_rag_data";
        multi_target_tracking::MarginalGainRAGConstPtr mg_data_msg = ros::topic::waitForMessage<multi_target_tracking::MarginalGainRAG>(mg_msg_topic, timeout);

        if (mg_data_msg) {
            multi_target_tracking::MarginalGainRAG mg_msg_topic_data = *mg_data_msg;   
            all_robots_selected_map_[i+1] = mg_msg_topic_data.flag_completed_action_selection;
        } else {
            // Handle the case where no message was received within the timeout
            // std::cout << "could not get mg msg from robot " << (i+1) << std::endl;
            all_robots_selected_map_[i+1] =false;
        }
    }
}


// Get wall clock time; algorithm continues from here
void MultiTargetTrackingCoordRAG::clockCallback(const rosgraph_msgs::Clock& msg)
{   
    // wait to proceed until drone has taken off
    if (takeoff_wait_flag_ == true)
    {
        client_.moveToZAsync(altitude_flight_level_, 5.0, client_timeout_sec_, client_default_yaw_mode_, client_lookahead_, client_adaptive_lookahead_, drone_name_self_)->waitOnLastTask();
        // auto start = std::chrono::high_resolution_clock::now();
        // ros::Duration(2.5).sleep(); // sleep 4 seconds
        // auto end = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> duration = end - start;
        // std::cout << "Takeoff sleep time: " << duration.count() << " seconds." << std::endl;

        // double bmg;
        // int best_init_control_action;
        // std::vector<int> innvec;
        // bestInitActionObjFunctionRAG(bmg,best_init_control_action, innvec);

        // Eigen::Matrix<double, 3, 1> selected_action_displacement = actions_.block(0, 2, 3, 1);
        // selected_action_ind_ = 2;
        // float yaw_rate = selected_action_displacement(2) / yaw_rate_timestep_;
        // client_.moveByRollPitchYawrateZAsync(0, 0, yaw_rate, -40.0, yaw_rate_timestep_, drone_name_self_)->waitOnLastTask(); // works!

        // client_.moveByVelocityZBodyFrameAsync(0.6*selected_action_displacement(0), 0.6*selected_action_displacement(1), -40, 30.0, client_drivetrain_, 
        //                                         client_default_yaw_mode_, drone_name_self_); //->waitOnLastTask();

        takeoff_wait_flag_ = false;
        return;
    }

    // to begin with, choose an initial action and go with it
    
    // get a single PursuerEvaderDataArray message
    multi_target_tracking::PursuerEvaderDataArrayConstPtr pe_data_msg = ros::topic::waitForMessage<multi_target_tracking::PursuerEvaderDataArray>("/pursuer_evader_data");
    pursuer_evader_data_ = *pe_data_msg;   

    // for self robot, do as many rounds of RAG-H needed to find the actions each will execute. Program communication delays where needed.
    std::map<int, double> selected_action_inneigh_ids_mg;
    while ((finished_action_selection_ == false) && (waiting_for_robots_to_finish_selection_ == false)) // need to have flag here that indicates all robots have executed actions
    {   
        // for each drone in the in-neighbors set, subscribe to their gain topic, while continuously publishing your own best gain (out neighbors will subscribe to this anyway)
        // subscribe to all in neighbors' mg rag msgs and collect ids of those in-neighbors that have selected ids. pass this into the marginal gain function
        getSelectedActionsInNeighborIDs(selected_action_inneigh_ids_mg);
        std::vector<int> inneighs_selected_action_ids;
        for(auto const& imap: selected_action_inneigh_ids_mg) {
            inneighs_selected_action_ids.push_back(imap.first);
        }

        int best_control_action_ind;
        double best_marginal_gain_self;
        marginalGainFunctionRAG(best_marginal_gain_self, best_control_action_ind, inneighs_selected_action_ids); // wrt only to in-neighbors that have selected actions
        best_marginal_gain_self_ = best_marginal_gain_self; // will publish this in the mg gain publisher callback thread

        // auto start = std::chrono::high_resolution_clock::now();
        // ros::Duration(4.0).sleep(); // sleep 4 seconds
        // auto end = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> duration = end - start;
        // std::cout << "Takeoff sleep time: " << duration.count() << " seconds." << std::endl;


        std::map<int, double> non_selected_action_inneigh_ids_mg;
        auto start1 = std::chrono::high_resolution_clock::now();
        getNonSelectedActionsInNeighborIDs(non_selected_action_inneigh_ids_mg); // key id : best mg value
        auto end1 = std::chrono::high_resolution_clock::now();
        double duration1 = (end1 - start1).count();
        simulatedCommTimeDelayRAG(duration1);

        // compare your best mg with those of in neighbors that have not selected an action yet
        bool is_my_mg_best;
        isMyMarginalGainBest(is_my_mg_best, non_selected_action_inneigh_ids_mg);
        std::map<int, double> new_selected_action_inneigh_ids_mg;

        if (is_my_mg_best == true)
        {
            selected_action_ind_ = best_control_action_ind; 
            finished_action_selection_ = true;
        }
        else
        {           
            auto start2 = std::chrono::high_resolution_clock::now();
            getSelectedActionsInNeighborIDs(new_selected_action_inneigh_ids_mg); // combine this (union) with selected_action_inneigh_ids_mg
            auto end2 = std::chrono::high_resolution_clock::now();
            double duration2 = (end2 - start2).count();
            simulatedCommTimeDelayRAG(duration2);
        }
        takeUnionOfMaps(selected_action_inneigh_ids_mg, new_selected_action_inneigh_ids_mg);
    }
    
    // Once every pursuer has selected its action, execute this self robot's action. NOTE: We assume all robots execute actions at the same time,
    // until then they execute zero-order hold on their previously selected control action.
    Eigen::Matrix<double, 3, 1> selected_action_displacement = actions_.block(0, selected_action_ind_, 3, 1);
    last_selected_displacement_ = selected_action_displacement;

    // !!!CHECK if all pursuers have already selected actions
    bool all_robots_ready_to_proceed = areAllValuesTrue();
    if (all_robots_ready_to_proceed == true)
    {   
        // execute selected action using airsim client and action table________
        float yaw_rate = selected_action_displacement(2) / yaw_rate_timestep_;
        if (yaw_rate < 0.)
        {
            yaw_rate = -1.*yaw_rate_;
            yaw_duration_ = abs(selected_action_displacement(2) /yaw_rate_);
        }
        else if (yaw_rate > 0.)
        {
            yaw_rate = yaw_rate_;
            yaw_duration_ = abs(selected_action_displacement(2) /yaw_rate_);
        }
        else
        {
            yaw_rate=0.;
            yaw_duration_ = 0.;
        }

        client_.moveByRollPitchYawrateZAsync(0, 0, yaw_rate, altitude_flight_level_, yaw_duration_, drone_name_self_)->waitOnLastTask(); // works!

        client_.moveByVelocityZBodyFrameAsync(drone_lin_vel_factor_ * selected_action_displacement(0), drone_lin_vel_factor_ * selected_action_displacement(1), 
        altitude_flight_level_, 1.0, client_drivetrain_, client_default_yaw_mode_, drone_name_self_)->waitOnLastTask(); //->waitOnLastTask();
        
        finished_action_selection_ = false; // reset this flag for next timestep
        waiting_for_robots_to_finish_selection_ = false;
    }
    else
    {   
        waiting_for_robots_to_finish_selection_ = true;
        // execute the last select action, zero-order hold
        // float yaw_rate = last_selected_displacement_(2) / yaw_rate_timestep_;
        // if (yaw_rate < 0.)
        // {
        //     yaw_rate = -1.*yaw_rate_;
        //     yaw_duration_ = abs(selected_action_displacement(2) /yaw_rate_);
        // }
        // else if (yaw_rate > 0.)
        // {
        //     yaw_rate = yaw_rate_;
        //     yaw_duration_ = abs(selected_action_displacement(2) /yaw_rate_);
        // }
        // else
        // {
        //     yaw_rate=0.;
        //     yaw_duration_ = 0.;
        // }

        // client_.moveByRollPitchYawrateZAsync(0, 0, yaw_rate, altitude_flight_level_, yaw_duration_, drone_name_self_)->waitOnLastTask(); // works!

        client_.moveByVelocityZBodyFrameAsync(drone_lin_vel_factor_ * 3.0, 0., altitude_flight_level_, 50.0, client_drivetrain_, 
                                                client_default_yaw_mode_, drone_name_self_); //->waitOnLastTask();
    }
    
    curr_time_step_samp_++; // increment by 1 for each t in T
}


void MultiTargetTrackingCoordRAG::takeUnionOfMaps(std::map<int, double>& selected_action_inneigh_ids_mg, std::map<int, double>& new_selected_action_inneigh_ids_mg, std::map<int, double>& combined_map) {
    // Insert elements from first map
    for(auto const& element : selected_action_inneigh_ids_mg) {
    combined_map[element.first] = element.second;  
    }

    // Insert elements from second map  
    for(auto const& element : new_selected_action_inneigh_ids_mg) {
    combined_map[element.first] = element.second;
    }
}


//override function
void MultiTargetTrackingCoordRAG::takeUnionOfMaps(std::map<int, double>& selected_action_inneigh_ids_mg, std::map<int, double>& new_selected_action_inneigh_ids_mg) {
    // Insert elements from second map  
    for(auto const& element : new_selected_action_inneigh_ids_mg) {
    selected_action_inneigh_ids_mg[element.first] = element.second;
    }
}


void MultiTargetTrackingCoordRAG::isMyMarginalGainBest(bool& is_my_mg_best, std::map<int, double>& non_selected_action_inneigh_ids_mg) {
    is_my_mg_best = true;  // Assume initially that your mg is the best

    for (const auto& pair : non_selected_action_inneigh_ids_mg) {
        // Compare your mg value with each value in the map
        if (best_marginal_gain_self_ < pair.second) {
            is_my_mg_best = false;  // Your mg is not the best
            break;  // No need to continue checking
        }
    }
}


void MultiTargetTrackingCoordRAG::getSelectedActionsInNeighborIDs(std::map<int, double>& selected_action_inneigh_ids_mg) {
    std::vector<int> all_inneigh_ids = pursuer_evader_data_.pursuer_evader_info_array.at(robot_idx_self_-1).in_neighbor_ids;

    // poll other pursuer drone topics here 
    ros::Duration timeout(5.0);  // Timeout of 5 seconds
    for (int& pursuer_id : all_inneigh_ids)
    {
        std::string mg_msg_topic = "Drone" + std::to_string(pursuer_id) + "/mg_rag_data";
        multi_target_tracking::MarginalGainRAGConstPtr mg_data_msg = ros::topic::waitForMessage<multi_target_tracking::MarginalGainRAG>(mg_msg_topic, timeout);
        // Check if the pointer is not null before dereferencing
        if (mg_data_msg) {
            multi_target_tracking::MarginalGainRAG mg_msg_topic_data = *mg_data_msg;   
            if (mg_msg_topic_data.flag_completed_action_selection == true) {
                selected_action_inneigh_ids_mg[pursuer_id] = mg_msg_topic_data.best_marginal_gain;
            }
        } else {
            // Handle the case where no message was received within the timeout
            std::cout << "could not get mg msg from robot " << pursuer_id << std::endl;
            // selected_action_inneigh_ids_mg[pursuer_id] = -1.; // setting a default value if no message received from other drone within 5 seconds
            continue;
        }
    }
}


void MultiTargetTrackingCoordRAG::getNonSelectedActionsInNeighborIDs(std::map<int, double>& non_selected_action_inneigh_ids_mg) {
    std::vector<int> all_inneigh_ids = pursuer_evader_data_.pursuer_evader_info_array.at(robot_idx_self_-1).in_neighbor_ids;

    // poll other pursuer drone topics here 
    ros::Duration timeout(5.0);  // Timeout of 5 seconds
    for (int& pursuer_id : all_inneigh_ids)
    {
        std::string mg_msg_topic = "Drone" + std::to_string(pursuer_id) + "/mg_rag_data";
        multi_target_tracking::MarginalGainRAGConstPtr mg_data_msg = ros::topic::waitForMessage<multi_target_tracking::MarginalGainRAG>(mg_msg_topic, timeout);
        // Check if the pointer is not null before dereferencing
        if (mg_data_msg) {
            multi_target_tracking::MarginalGainRAG mg_msg_topic_data = *mg_data_msg;   
            if (mg_msg_topic_data.flag_completed_action_selection == false) {
                non_selected_action_inneigh_ids_mg[pursuer_id] = mg_msg_topic_data.best_marginal_gain;
            }
        } else {
            // Handle the case where no message was received within the timeout
            std::cout << "could not get mg msg from robot " << pursuer_id << std::endl;
            non_selected_action_inneigh_ids_mg[pursuer_id] = -1.; // setting a default value if no message received from other drone within 5 seconds
        }
    }
}


void MultiTargetTrackingCoordRAG::marginalGainFunctionRAG(double& best_marginal_gain, int& optimal_action_id, std::vector<int>& inneighs_selected_action) {
    /* evalutates the best marginal gain of all control actions in actions_ 
    */
    
    std::map<int, double> marginal_gains_map; // Map to store control action ID and corresponding marginal gain

    // do below for every action in action set
    for (int i = 0; i < actions_.cols(); i++)
    {
        double total_ft;
        bool include_self1 = true;
        objectiveFunctionRAG(total_ft, inneighs_selected_action, include_self1, i);

        double val_ft_aim1 = 0.; // this is f_t of all robots until robot i-1, where i is current robot
        bool include_self2 = false;
        objectiveFunctionRAG(val_ft_aim1, inneighs_selected_action, include_self2, i);

        // marginal_gain = total_ft - val_ft_aim1;
        marginal_gains_map[i] = total_ft - val_ft_aim1;
        // std::cout << "marginal gain for control action ID " << i << ": " << marginal_gains_map[i] << std::endl;
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
}


void MultiTargetTrackingCoordRAG::bestInitActionObjFunctionRAG(double& best_marginal_gain, int& optimal_action_id, std::vector<int>& inneighs_selected_action) {
    std::map<int, double> objfunc_vals_map; // Map to store control action ID and corresponding marginal gain
    
    // do below for every action in action set
    for (int i = 0; i < actions_.cols(); i++)
    {
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
 

void MultiTargetTrackingCoordRAG::objectiveFunctionRAG(double& obj_func_val, std::vector<int>& in_neigh_ids_select_actions, bool& include_self_robot, int& self_robot_action_id) {

    if ((in_neigh_ids_select_actions.size() == 0) && (include_self_robot==false))
    {
        return;
    }
    

    if (include_self_robot == true)
    {
       in_neigh_ids_select_actions.push_back(robot_idx_self_); // will include self robot index in computing f_(t-1) 
    }

    std::vector<multi_target_tracking::PursuerEvaderData> pe_data_vec = pursuer_evader_data_.pursuer_evader_info_array; // contains data of all ids
    if (pe_data_vec.empty()) {
        // std::cout << "pursuer_evader_data_ vector is empty." << std::endl;
        return;
    }
    else
    {
       std::cout << "Timestamp of last saved PEdata msg: " << pursuer_evader_data_.header.stamp.sec << "." << pursuer_evader_data_.header.stamp.nsec << " seconds" << std::endl;
    }

    std::vector<multi_target_tracking::PursuerEvaderData> pe_data_selected_ids;
    for (auto& idx : in_neigh_ids_select_actions)
    {
        pe_data_selected_ids.push_back(pe_data_vec.at(idx-1));
    }
    
    // if include_self_robot is true then use self_robot_action_id to select the control action displacement from actions_ Eigen 3x5 matrix (cols are ids) 
    // and virtually move the self robots pose. 
    if (include_self_robot == true) //start if cond___________________________
    {
        // multi_target_tracking::PursuerEvaderData curr_robot_pe_data = pe_data_selected_ids.at(pe_data_selected_ids.size() - 1); // gets last robots pe data which is that of robot self id
        multi_target_tracking::PursuerEvaderData robot_pe_data;
        for(int i = 0; i < pe_data_selected_ids.size(); i++) {
            if(pe_data_selected_ids[i].drone_id == robot_idx_self_) {
                robot_pe_data = pe_data_selected_ids[i];
                break;
            }
        }

        // set new pose
        std::vector<double> curr_pose = robot_pe_data.self_drone_pose;
        std::vector<double> new_pose = {curr_pose[0] + actions_(0,self_robot_action_id), curr_pose[1] + actions_(1,self_robot_action_id), curr_pose[2], curr_pose[3] + actions_(2,self_robot_action_id)}; //x,y,z,theta
        
        std::vector<int> evaders_ids = robot_pe_data.evaders_range_bearing_ids; // ids of all evaders
        std::vector<double> evader_xyz_data = robot_pe_data.evaders_x_y_z_values;
        std::vector<double> evader_rb_data = robot_pe_data.evaders_range_bearing_values;
        for (int& evader_id : evaders_ids)
        {   
            std::vector<double> curr_evader_pose = {evader_xyz_data.at(3*(evader_id - num_robots_- 1)), evader_xyz_data.at(3*(evader_id - num_robots_-1) + 1), evader_xyz_data.at(3*(evader_id - num_robots_ -1)+ 2)}; 
            bool inFov = isEvaderInFov(new_pose, curr_evader_pose, max_sensing_range_);
            if (inFov == true)
            {
                auto [range, bearing] = computeRangeBearing(new_pose, curr_evader_pose);
                // correct the range and bearing of this evader
                evader_rb_data.at(2*(evader_id - num_robots_ -1)) = range;
                evader_rb_data.at(2*(evader_id - num_robots_ -1) + 1) = bearing;
            }
        }
        robot_pe_data.evaders_range_bearing_values = evader_rb_data; // corrected the rb values based on new visibility due to the hypothetical control action execution

        // set the corrected evader rb values
        for(int i = 0; i < pe_data_selected_ids.size(); i++) {
            if(pe_data_selected_ids[i].drone_id == robot_idx_self_) {
                pe_data_selected_ids[i] = robot_pe_data;
                break;
            }
        }
    } // end if condition________________________________

    // initializing evader_observer_ranges dictionary. Key is evader id, values are ranges to all robots that have this evader in FOV_________
    std::map<int, std::vector<double>> evader_observer_ranges; // key int is evader ID, std::vector<double> is vector of ranges of pursuers which are observing evader in ID
    for (auto& curr_pursuer_drone_data : pe_data_selected_ids) {
        std::vector<double> distvec; // empty vec initialized
        std::vector<int> observer_ids_in_fov = curr_pursuer_drone_data.evaders_in_FOV;

        for(int& j : observer_ids_in_fov) {
            evader_observer_ranges[j] = distvec;  
        }
    } // loop end_______


    for (int i = 0; i < pe_data_selected_ids.size(); i++) {
        auto curr_pursuer_drone_data = pe_data_selected_ids.at(i);
        std::vector<int> observer_ids_in_fov = curr_pursuer_drone_data.evaders_in_FOV;

        for (int& evader_id : observer_ids_in_fov) {
            if (evader_observer_ranges.find(evader_id) != evader_observer_ranges.end()) {
                // Check if the pursuer observes the evader
                if (std::find(curr_pursuer_drone_data.evaders_in_FOV.begin(), curr_pursuer_drone_data.evaders_in_FOV.end(), evader_id) != curr_pursuer_drone_data.evaders_in_FOV.end()) {
                    // Find the range between the evader and pursuer and push it back to the vector
                    int index = std::find(curr_pursuer_drone_data.evaders_range_bearing_ids.begin(), curr_pursuer_drone_data.evaders_range_bearing_ids.end(), evader_id) - curr_pursuer_drone_data.evaders_range_bearing_ids.begin();
                    double range = curr_pursuer_drone_data.evaders_range_bearing_values[2*index];
                    evader_observer_ranges[evader_id].push_back(range);
                }
            }
        }
    }

    // Populate evader_observer_ranges with default distance for unobserved evaders
    for (int i = (num_robots_+1); i <= (num_robots_ + num_evaders_+1); i++) {
        if (evader_observer_ranges.find(i) == evader_observer_ranges.end()) {
            // Evader not observed, assign default distance
            std::vector<double> distvec = {4 * max_sensing_range_};
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

bool MultiTargetTrackingCoordRAG::isEvaderInFov(const std::vector<double>& pursuer_pose, //4x1, x,y,z,theta
                      const std::vector<double>& evader_pose, // 3x1 x,y,z
                      double max_dist) {

    // Get x, y coords
    double dx = evader_pose[0] - pursuer_pose[0];
    double dy = evader_pose[1] - pursuer_pose[1];
    double pursuer_yaw = pursuer_pose.at(3);

    // Rotate vector
    double rotated_dx = dx * cos(pursuer_yaw) - dy * sin(pursuer_yaw);
    double rotated_dy = dx * sin(pursuer_yaw) + dy * cos(pursuer_yaw);

    // Calculate distance
    double dist = rotated_dx; // Only check x component

    // Check distance
    if (dist > max_dist) {
        return false; 
    }

    // Check angle
    if (dist > 0) {
        double angle = atan2(rotated_dy, rotated_dx);
        if (abs(angle) > fov_x_/2.) {
        return false;
        }
    }

    // Check vertical angle 
    double vertical_angle = atan2(evader_pose[2] - pursuer_pose[2], dist);
    if (abs(vertical_angle) > fov_y_/2.) {
        return false;
    }

    return true;
    }


bool MultiTargetTrackingCoordRAG::areAllValuesTrue() {
    for (const auto& pair : all_robots_selected_map_) {
        if (!pair.second) {
            return false;  // Found a false value, return false
        }
    }
    return true;  // All values are true
}


std::pair<double, double> MultiTargetTrackingCoordRAG::computeRangeBearing(const std::vector<double>& pose1, 
                                               const std::vector<double>& pose2) {

  double dx = pose2[0] - pose1[0];
  double dy = pose2[1] - pose1[1];

  double range = std::sqrt(dx*dx + dy*dy);

  double bearing = std::atan2(dy, dx);

  return {range, bearing};

}


void MultiTargetTrackingCoordRAG::simulatedCommTimeDelayRAG() {
    // NOTE!!! This is for Resource-aware Bandit Greedy
    double time_delay_per_msg = total_data_msg_size_ / communication_data_rate_;

    communication_round_ +=1;
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start_;
    std::cout << duration.count() << " COMMUNICATION ROUND COUNT: " << communication_round_ << std::endl;
    
    ros::Duration(time_delay_per_msg).sleep();
}

void MultiTargetTrackingCoordRAG::simulatedCommTimeDelayRAG(double& processed_time) {
    // NOTE!!! This is for Resource-aware Bandit Greedy
    double time_delay_per_msg = total_data_msg_size_ / communication_data_rate_;
    double comm_time = time_delay_per_msg - processed_time;

    communication_round_ +=1;
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start_;

    std::cout << duration.count() << " COMMUNICATION ROUND COUNT: " << communication_round_ << std::endl;

    if (comm_time > 0.)
    {   
        ros::Duration(comm_time).sleep();
    }
}


int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "multi_target_coord_RAG_node");
    ros::NodeHandle nh_mtt;

    MultiTargetTrackingCoordRAG node(nh_mtt);

    // ros::spin();
    ros::waitForShutdown(); // use with asyncspinner

    return 0;
}
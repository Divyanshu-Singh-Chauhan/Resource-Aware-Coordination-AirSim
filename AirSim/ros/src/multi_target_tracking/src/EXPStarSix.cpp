// EXPStarSIX.cpp

#include "multi_target_tracking/EXPStarSix.h"

//constructor must initialize various eigen matrices
// EXPStarSIX::EXPStarSIX(int n_actions_, int num_experts, int n_time_step) {

EXPStarSIX::EXPStarSIX(int num_robot, int robot_idx, Eigen::MatrixXd actions, int n_time_steps,
                       double samp)
{
    // asign private vars
    num_robots_ = num_robot; // total number of pursuer robots
    robot_idx_ = robot_idx; // current pursuer robot's index
    actions_ = actions; // 3x5 matrix of control actions to take: goal x, y
    n_actions_ = actions.cols(); // should be 5
    n_time_step_ = n_time_steps; // total number of time steps to do planning, 2000
    samp_ = samp;

    actions_indices_.resize(1, actions.cols()); // 1x5 matrix of indices of each action (remember to use 0 indexing)
    for (int i = 0; i < actions.cols(); i++) {
        actions_indices_(0, i) = i;
    }

    selected_action_index_ = Eigen::MatrixXi::Ones(n_time_step_, 1); //2000x1 double indicating which action index was chosen; n_time_step rows x 1 col
    // learning_constant_ = 25; // tuning needed
    // learning_constant_ = 50; // still need more aggressive learning
    learning_constant_ = 100; //works well with 2.0 yawrate, 1.25x3 speed factor
    // learning_constant_ = 125;

    J_ = ceil(log(n_time_steps));  // 
    e_ = learning_constant_ * sqrt(log(J_) / (2 * n_time_step_)); // 1x1 double 
    beta_ = double(1. / (double(n_time_step_) - 1.));  // 1x1 double 
    num_experts_ = int(J_);

    eta_ = Eigen::MatrixXd(num_experts_, 1); // 8x1 double
    eta_.setZero(); // 8x1 

    for (int i = 0; i < J_; i++) {
        eta_(i, 0) = learning_constant_ * sqrt(log(n_actions_ * n_time_step_) / (pow(2, i) / n_actions_));
    }

    gamma_ = eta_ / 2; // 8x1 double
    expert_weights_ = (1 / J_) * Eigen::MatrixXd::Ones(n_time_steps, num_experts_); //n_time_step_ x num_experts_; 2000 x 8 dimension

    for (int i = 0; i < n_actions_; i++) {
        action_weights_.push_back((1 / double(n_actions_)) * Eigen::MatrixXd::Ones(n_time_step_, J_));
    }

    action_prob_dist_.resize(n_time_step_, n_actions_); //n_time_step_x n_actions_ dimension, 2000x5
    action_prob_dist_.setZero();

    loss_.resize(n_time_step_, n_actions_); // 2000 x 5
    loss_.setZero();
    loss_estm_.resize(n_time_step_, n_actions_); // 2000 x 5
    loss_estm_.setZero();

    reward_.resize(n_time_step_, n_actions_); // 2000 x 5
    reward_.setZero();
    reward_estm_.resize(n_time_step_, n_actions_); // 2000 x 5
    reward_estm_.setZero();
}

EXPStarSIX::EXPStarSIX()
{
    //placeholder constructor
    int i = 1;
}

void EXPStarSIX::update_experts(int t)
{

    for (int j = 0; j < J_; j++) {
        // double action_pd_t = action_prob_dist_(t, selected_action_index_(t, 0));
        // double gamma_t = gamma_(j, 0);

        // loss_estm_(t, selected_action_index_(t, 0)) = loss_(t, selected_action_index_(t, 0)) /
        //                                               (action_pd_t + gamma_t); //return 1x1 double

        // reward_estm_(t, selected_action_index_(t, 0)) = reward_(t, selected_action_index_(t, 0)) /
        //                                                 (action_pd_t + gamma_t); //return 1x1 double

        loss_estm_(t, selected_action_index_(t, 0)) = loss_(t, selected_action_index_(t, 0)) /
                                                      (action_prob_dist_(t, selected_action_index_(t, 0)) + gamma_(j, 0)); //return 1x1 double
        // std::cout << "loss_estm_: " << loss_estm_(t, selected_action_index_(t, 0)) << std::endl;

        reward_estm_(t, selected_action_index_(t, 0)) = reward_(t, selected_action_index_(t, 0)) /
                                                        (action_prob_dist_(t, selected_action_index_(t, 0)) + gamma_(j, 0)); //return 1x1 double
        // std::cout << "reward_estm_: " << reward_estm_(t, selected_action_index_(t, 0)) << std::endl;


        Eigen::MatrixXd v(n_actions_, 1); // n_actions_ = 5; 5x1 double
        v.setZero();
        for (int i = 0; i < n_actions_; i++) {
            double exp_prod = -eta_(j, 0) * loss_estm_(t, i);
            v(i, 0) = (action_weights_.at(i))(t, j) * exp(exp_prod);
        }
        //print_eigen_mat(v, "v: ");

        double W_t = v.sum();
        // std::cout << "W_t: " << W_t << std::endl;

        Eigen::MatrixXd constantMatrix1 = (beta_ * W_t / n_actions_) * Eigen::MatrixXd::Ones(v.rows(), v.cols()); // 5x1 double
        // std::cout << "beta_: " << beta_ << std::endl;
        //print_eigen_mat(constantMatrix1, "constantMatrix1: ");

        Eigen::MatrixXd rhs1 = constantMatrix1 + ((1. - beta_) * v); // n_actions_ x 1; 5x1 double
       //print_eigen_mat(rhs1, "rhs1: "); 

        for (int i = 0; i < n_actions_; i++) {
            (action_weights_.at(i))(t + 1, j) = rhs1(i, 0);  //CHECKPOINT: line 101 in matlab bsg planner
        }

        Eigen::MatrixXd exp_term1 = -e_ * loss_estm_.block(t, 0, 1, n_actions_); //1x5
        //print_eigen_mat(exp_term1, "exp_term1");

        Eigen::MatrixXd exp_term2(n_actions_, 1);  // 5x1
        exp_term2.setZero();
        for (int i = 0; i < n_actions_; i++) {
            exp_term2(i, 0) = (action_weights_.at(i))(t, j);
        }

        double l1Norm = exp_term2.lpNorm<1>();
        double combined_rhs_mult = ((1 / l1Norm) * exp_term1 * exp_term2)(0, 0);

        expert_weights_(t + 1, j) = expert_weights_(t, j) * exp(combined_rhs_mult); 
    }

    // Normalize weights by 1-Norm
    for (int j = 0; j < J_; j++) {
        Eigen::MatrixXd exp_term2(n_actions_, 1); // 5x1
        exp_term2.setZero();
        for (int i = 0; i < n_actions_; i++) {
            exp_term2(i, 0) = action_weights_.at(i)(t + 1, j);
        }
        for (int i = 0; i < n_actions_; i++) {
            action_weights_.at(i)(t + 1, j) = (1. / exp_term2.lpNorm<1>()) * action_weights_.at(i)(t + 1, j);
        }

        Eigen::MatrixXd exp_term2_postnorm(n_actions_, 1);
        exp_term2_postnorm.setZero();
        for (int i = 0; i < n_actions_; i++) {
            exp_term2(i, 0) = action_weights_.at(i)(t + 1, j);
        }


        int countNaN = (exp_term2.array().isNaN()).count();
        try {
            if (countNaN > 0) {
                throw std::runtime_error("NaN values found in action_prob_dist row.");
            }
        }
        catch (const std::runtime_error& e) {
            // Handle the exception
            std::cerr << "Exception update_experts() function: " << e.what() << std::endl;
        }
    }

    expert_weights_.block(t + 1, 0, 1, int(J_)) = (1 / expert_weights_.block(t + 1, 0, 1, int(J_)).lpNorm<1>()) * expert_weights_.block(t + 1, 0, 1, int(J_));
}

void EXPStarSIX::update_action_prob_dist(int t) // t is timestep
{
    Eigen::MatrixXd q_t = expert_weights_.block(t, 0, 1, J_); // 1 x J_ dimension NOTE: q_t is all negative
    // Eigen::MatrixXd q_t = -1. * expert_weights_.block(t, 0, 1, J_);
    //print_eigen_mat(q_t, "q_t: ");

    Eigen::MatrixXd p_t_mat(int(J_), n_actions_); // J_ x n_actions_ dimension
    p_t_mat.setZero(); // 8x5
    for (int i = 0; i < n_actions_; i++) {
        p_t_mat.block(0, i, int(J_), 1) = (action_weights_.at(i).block(t, 0, 1, J_)).transpose();
    }
    //print_eigen_mat(p_t_mat, "p_t_mat: ");

    Eigen::MatrixXd action_pd_updated = q_t * p_t_mat; // should be 1 x n_actions_
    //print_eigen_mat(action_pd_updated, "action_pd_updated: ");
    action_prob_dist_.block(t, 0, 1, n_actions_) = action_pd_updated; // may need to index into the RHS using .block()

    // checking for errors in computation_____
    int countNaN = (action_prob_dist_.row(t).array().isNaN()).count();
    // std::cout << "countNaN: " << countNaN << std::endl;
    try {
        if (countNaN > 0) {
            throw std::runtime_error("NaN values found in action_prob_dist row.");
        }
    }
    catch (const std::runtime_error& e) {
        // Handle the exception
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    double sumRow = action_prob_dist_.row(t).sum();
    // std::cout << "sumRow: " << sumRow << std::endl;
    try {
        if (std::abs(sumRow - 1.0) > 1e-4) {
            // std::cout << "sumRow: " << sumRow << std::endl;
            throw std::runtime_error("Incorrect probability distribution");
        }
    }
    catch (const std::runtime_error& e) {
        // Handle the exception
        std::cerr << "Exception: " << e.what() << std::endl;
    }

}

void EXPStarSIX::setLossValues(int& t, double& loss_normalized)
{
    int action_ind = selected_action_index_(t, 0);
    loss_(t, action_ind) = loss_normalized;
}

void EXPStarSIX::setSelectedActionIndex(int& t_row, int& ind)
{
    selected_action_index_(t_row, 0) = ind;
}

void EXPStarSIX::getActionProbDist(Eigen::MatrixXd& action_prob_dist_mat)
{
    action_prob_dist_mat = action_prob_dist_;
}

// helper functions
void EXPStarSIX::print_eigen_mat(Eigen::MatrixXd& matToPrint, std::string string_to_print)
{
    // Eigen::MatrixXd
    std::cout << string_to_print << " rows: " << matToPrint.rows() << " cols: " << matToPrint.cols() << std::endl;
    std::cout << matToPrint << std::endl;
    std::cout << " " << std::endl;
}
// EXPStarSIX.h

#ifndef EXP_STAR_SIX_H
#define EXP_STAR_SIX_H

#include <vector>
#include <Eigen/Dense>
#include <iostream>

class EXPStarSIX
{
private:
    int num_robots_;
    int robot_idx_;
    int n_actions_;
    int num_experts_;
    int n_time_step_;
    double samp_;
    double learning_constant_;
    double J_;
    double e_;
    double beta_;

    Eigen::MatrixXd actions_;
    Eigen::MatrixXd actions_indices_;
    Eigen::MatrixXi selected_action_index_;

    Eigen::MatrixXd eta_;
    Eigen::MatrixXd gamma_;

    Eigen::MatrixXd expert_weights_;
    std::vector<Eigen::MatrixXd> action_weights_;
    Eigen::MatrixXd action_prob_dist_;
    Eigen::MatrixXd loss_;
    Eigen::MatrixXd loss_estm_;
    Eigen::MatrixXd reward_;
    Eigen::MatrixXd reward_estm_;

    // Other parameters such as loss, reward, etc

public:
    // EXPStarSIX(int num_actions, int num_experts, int n_time_step);
    // num_robot, robot_idx, actions, n_time_step, samp
    EXPStarSIX(int num_robot, int robot_idx, Eigen::MatrixXd actions, int n_time_step, double samp); // constructor
    EXPStarSIX(); // alternative constructor
    void update_experts(int t);

    void update_action_prob_dist(int t);

    // setter and getter functions to receive the reward of control action
    void getActionProbDist(Eigen::MatrixXd& action_prob_dist_mat);
    void setLossValues(int& t, double& reward_normalized);
    void setSelectedActionIndex(int& t_row, int& ind);

    // helper function
    void print_eigen_mat(Eigen::MatrixXd& matToPrint, std::string string_to_print);

};

#endif //EXP_STAR_SIX_H 
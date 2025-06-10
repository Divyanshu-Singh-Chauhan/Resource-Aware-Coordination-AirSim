/* 
UNIT TESTS FOR AIRSIM DRONE CONTROL API CALLS
 */
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>

using namespace std;

// Convert Euler angles to quaternion
Eigen::Quaternionf quaternionFromEuler(float roll, float pitch, float yaw) {
  Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

  Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
  return q;
}

// Convert quaternion to Euler angles
Eigen::Vector3f eulerFromQuaternion(const Eigen::Quaternionf& q) {
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

int main()
{
    using namespace msr::airlib;

    msr::airlib::MultirotorRpcLibClient client;
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;
    typedef common_utils::FileSystem FileSystem;

    float client_timeout_sec_;
    float client_lookahead_;
    float client_adaptive_lookahead_ ;
    float client_margin_;
    Eigen::MatrixXd actions_; actions_.resize(3, 7);
    actions_ << 1.15, 2.12, 2.77, 3.0, 2.77, 2.12, 1.15,
            -2.772, -2.12, -1.15, 0, 1.15, 2.12, 2.772, 
            -1.18, -0.7854, -0.393, 0, 0.393, 0.7854, 1.18;
    double yaw_rate_ = 2.36;
    double yaw_duration_ = 0.;
    try {
        std::string drone_name = "Drone1";
        
        client.confirmConnection();
        client.enableApiControl(true, drone_name);    
        client.armDisarm(true);

        client_timeout_sec_ = (3.4028235E38F);
        msr::airlib::YawMode client_default_yaw_mode_ = msr::airlib::YawMode();
        client_lookahead_ = (-1.0F);
        client_adaptive_lookahead_ = (1.0F);
        client_margin_ = (5.0F);
        DrivetrainType client_drivetrain_ = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;

        auto pose = client.getMultirotorState(drone_name);
        auto position = pose.getPosition();
        auto orientation = pose.kinematics_estimated.pose.orientation;
        double curr_yaw = 2.0 * atan2(orientation.z(), orientation.w());

        float z = position.z(); // current position (NED coordinate system).
        constexpr float speed = 3.0f;
        constexpr float size = 10.0f;
        constexpr float duration = size / speed;
        DrivetrainType drivetrain = DrivetrainType::ForwardOnly;
        YawMode yaw_mode(true, 0); //is_rate, yaw_or_rate

        client.hoverAsync(drone_name)->waitOnLastTask();
        std::vector<int> indices = {0,6,4,5,1,3,2};

        // take off here
        msr::airlib::Pose goal_pose;
        goal_pose.position.x() = position.x(); 
        goal_pose.position.y() = position.y();   
        goal_pose.position.z() = -40;  

        // Calculate the quaternion components based on the yaw angle
        double goal_yaw = 0.;
        double halfYaw = 0.5 * goal_yaw;
        double sinHalfYaw = sin(goal_yaw);
        double cosHalfYaw = cos(goal_yaw);

        goal_pose.orientation.w() = cosHalfYaw;  // Set W component of the quaternion
        goal_pose.orientation.x() = 0.0;         // Set X component of the quaternion
        goal_pose.orientation.y() = 0.0;         // Set Y component of the quaternion
        goal_pose.orientation.z() = sinHalfYaw;
        bool ignore_collision = true;
        client.simSetVehiclePose(goal_pose, ignore_collision, drone_name); 



        // our flight pattern
        for (int i = 0; i < indices.size(); i++)
        {   
            auto pose = client.getMultirotorState(drone_name);
            auto position = pose.getPosition();
            auto orientation = pose.kinematics_estimated.pose.orientation;
            auto q=orientation;

            Eigen::Vector3f euler = eulerFromQuaternion(q.cast<float>());
            double curr_yaw = euler[2];
            cout << "curr_yaw: " << curr_yaw << endl;

            int ind;
            std::cout << "Enter control action index" << std::endl;
            std::cin >> ind;
            // ind = indices.at(i);
            if (ind == -1)
            {
                break;
            }
            
            Eigen::Matrix<double, 3, 1> selected_action_displacement = actions_.block(0, ind, 3, 1);
            
            // setting goal pose here____________
            msr::airlib::Pose goal_pose;
            goal_pose.position.x() = position.x() + selected_action_displacement(0); 
            goal_pose.position.y() = position.y() + selected_action_displacement(1);   
            goal_pose.position.z() = -40.;  

            // Calculate the quaternion components based on the yaw angle
            float goal_yaw = selected_action_displacement(2);
            Eigen::Quaternionf qrot = quaternionFromEuler(0., 0., goal_yaw);
            goal_pose.orientation =qrot;

            bool ignore_collision = true;
            client.simSetVehiclePose(goal_pose, ignore_collision, drone_name); 
            // client.hoverAsync(drone_name)->waitOnLastTask();
        }
        
        // std::cout << "Press Enter to disarm" << std::endl;
        // std::cin.get();
        client.armDisarm(false, drone_name);
    }


    catch (rpc::rpc_error& e) {
        const auto msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl
                  << msg << std::endl;
    }

    return 0;
}

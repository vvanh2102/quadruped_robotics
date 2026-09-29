#ifndef VANH_ROS_SIMULATOR__SIMULATOR_NODE_HPP_
#define VANH_ROS_SIMULATOR__SIMULATOR_NODE_HPP_

#include <string>
#include <vector>
#include <chrono>
#include <stdexcept>
#include <mutex>
#include <thread>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vanh_msgs/msg/robot_information.hpp>
#include <vanh_msgs/msg/manual_control.hpp>

#include "vanh_ros_simulator/solver_data.h"

namespace vanh_ros_simulator
{

class SimulatorNode : public rclcpp::Node
{
public:
    SimulatorNode();
    ~SimulatorNode();
    // HELPER
    template <class... Args> void log(const char *msg, Args... args);
    
    void setup();

private:
    void initParam();
    bool checkTime(const double &ref, const double &limit);
    /*
     * ACTION, SERVICE, PUB, SUB
     */

    // System information subscription
    // Subscribe target point (in robot coord), to publish jointstate
    void onRobotInfo(const vanh_msgs::msg::RobotInformation::SharedPtr msg);
    rclcpp::Subscription<vanh_msgs::msg::RobotInformation>::SharedPtr sub_robot_info_;
    std::vector<std::string> joint_names_;

    // Joint state publisher
    void publishJointState();
    std::thread joint_thread_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
    sensor_msgs::msg::JointState current_jointstate_;

    // Shared between the subscription callback and the publisher thread
    std::mutex state_mutex_;
    std::vector<float> joint_states_;
    double last_info_time_ = 0.0;
};

}  // namespace vanh_ros_simulator

#endif  // VANH_ROS_SIMULATOR__SIMULATOR_NODE_HPP_

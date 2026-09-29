#include "vanh_ros_simulator/simulator_node.h"

using namespace std::chrono_literals;

namespace vanh_ros_simulator
{
    SimulatorNode::SimulatorNode(): Node("simulator_node"){}
    SimulatorNode::~SimulatorNode()
    {
        if (joint_thread_.joinable()) {
            joint_thread_.join();
        }
    }
    template <class... Args> void SimulatorNode::log(const char *msg, Args... args)
    {
        RCLCPP_INFO(this->get_logger(), msg, args...);
    }

    void SimulatorNode::setup()
    {
        initParam();

        // Publisher
        pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states", rclcpp::SystemDefaultsQoS());

        // Subscriber
        sub_robot_info_ = this->create_subscription<vanh_msgs::msg::RobotInformation>(
            "robot_info", 
            rclcpp::SystemDefaultsQoS(), 
            std::bind(&SimulatorNode::onRobotInfo, this, std::placeholders::_1));

        // Thread
        joint_thread_ = std::thread(&SimulatorNode::publishJointState, this);
        
    }

    void SimulatorNode::initParam()
    {
        this->declare_parameter("robot_joints", rclcpp::PARAMETER_STRING_ARRAY);
        joint_names_ = this->get_parameter("robot_joints").as_string_array();
        joint_states_ = std::vector<float>(12, 0);
        
        log("PARAM SET");
        for (const auto & name : joint_names_)
        {
            log("- robot_joint: %s", name.c_str());
        }
    }

    bool SimulatorNode::checkTime(const double &ref, const double &limit)
    {
        return get_clock()->now().seconds() - ref >= limit;
    }

    void SimulatorNode::onRobotInfo(const vanh_msgs::msg::RobotInformation::SharedPtr msg)
    {
        joint_states_ = msg->joint.positions;
    }

    void SimulatorNode::publishJointState()
    {
        while (rclcpp::ok())
        {
            this->current_jointstate_.header.stamp = this->get_clock()->now();
            this->current_jointstate_.name = joint_names_;

            this->current_jointstate_.position.resize(12);
            this->current_jointstate_.position[ROBOT_JOINT_ORDERS::FRONT_LEFT_JOINT_1] =
                joint_states_[vanh_msgs::msg::ManualControl::FRONT_LEFT_JOINT_1];
            this->current_jointstate_.position[ROBOT_JOINT_ORDERS::FRONT_LEFT_JOINT_2] =
                joint_states_[vanh_msgs::msg::ManualControl::FRONT_LEFT_JOINT_2];
            this->current_jointstate_.position[ROBOT_JOINT_ORDERS::FRONT_LEFT_JOINT_3] =
                joint_states_[vanh_msgs::msg::ManualControl::FRONT_LEFT_JOINT_3];
            this->current_jointstate_.position[ROBOT_JOINT_ORDERS::FRONT_RIGHT_JOINT_1] =
                joint_states_[vanh_msgs::msg::ManualControl::FRONT_RIGHT_JOINT_1];
            this->current_jointstate_.position[ROBOT_JOINT_ORDERS::FRONT_RIGHT_JOINT_2] =
                joint_states_[vanh_msgs::msg::ManualControl::FRONT_RIGHT_JOINT_2];
            this->current_jointstate_.position[ROBOT_JOINT_ORDERS::FRONT_RIGHT_JOINT_3] =
                joint_states_[vanh_msgs::msg::ManualControl::FRONT_RIGHT_JOINT_3];
            this->current_jointstate_.position[ROBOT_JOINT_ORDERS::BACK_LEFT_JOINT_1] =
                joint_states_[vanh_msgs::msg::ManualControl::BACK_LEFT_JOINT_1];
            this->current_jointstate_.position[ROBOT_JOINT_ORDERS::BACK_LEFT_JOINT_2] =
                joint_states_[vanh_msgs::msg::ManualControl::BACK_LEFT_JOINT_2];
            this->current_jointstate_.position[ROBOT_JOINT_ORDERS::BACK_LEFT_JOINT_3] =
                joint_states_[vanh_msgs::msg::ManualControl::BACK_LEFT_JOINT_3];
            this->current_jointstate_.position[ROBOT_JOINT_ORDERS::BACK_RIGHT_JOINT_1] =
                joint_states_[vanh_msgs::msg::ManualControl::BACK_RIGHT_JOINT_1];
            this->current_jointstate_.position[ROBOT_JOINT_ORDERS::BACK_RIGHT_JOINT_2] =
                joint_states_[vanh_msgs::msg::ManualControl::BACK_RIGHT_JOINT_2];
            this->current_jointstate_.position[ROBOT_JOINT_ORDERS::BACK_RIGHT_JOINT_3] =
                joint_states_[vanh_msgs::msg::ManualControl::BACK_RIGHT_JOINT_3];

            pub_joint_state_->publish(this->current_jointstate_);
            std::this_thread::sleep_for(200ms);
        }
    }
}  // namespace vanh_ros_simulator

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vanh_ros_simulator::SimulatorNode>();
    node->setup();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

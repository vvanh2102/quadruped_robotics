#ifndef __SOLVER_DATA__HPP__
#define __SOLVER_DATA__HPP__
#include <atomic>
#include <mutex>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>

namespace vanh_ros_simulator
{   
    enum ROBOT_JOINT_ORDERS 
    {
        FRONT_LEFT_JOINT_1,  FRONT_LEFT_JOINT_2,  FRONT_LEFT_JOINT_3,
        FRONT_RIGHT_JOINT_1, FRONT_RIGHT_JOINT_2, FRONT_RIGHT_JOINT_3,
        BACK_LEFT_JOINT_1,   BACK_LEFT_JOINT_2,   BACK_LEFT_JOINT_3,
        BACK_RIGHT_JOINT_1,  BACK_RIGHT_JOINT_2,  BACK_RIGHT_JOINT_3
    };
} // namespace ros_moveit

#endif
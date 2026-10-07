alias ros_vanh="export ROS_DOMAIN_ID=110"

alias humble="source /opt/ros/humble/setup.bash"
alias abuild="colcon build --symlink-install"
alias pbuild="abuild --packages-select"

alias control = "ros2 launch vanh_ros_control ros_control.launch.py"
alias gait = "ros2 launch vanh_gait ros_gait.launch.py"
alias sim = "ros2 launch vanh_ros_simulator simulator.launch.py"
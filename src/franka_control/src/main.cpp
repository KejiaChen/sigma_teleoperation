#include "rclcpp/rclcpp.hpp"
#include "franka_control/franka_node.h"

#include <fstream>
#include <iostream>

int main(int argc, char **argv) {


    // // for logging robot states into txt file
    // static std::ofstream franka_log_file("/docker_volume/ros2_ws/src/RobotState_logger/franka_log.txt");
    // std::cout.rdbuf(franka_log_file.rdbuf());
    // std::cout.setf(std::ios::unitbuf); 

    

    rclcpp::init(argc, argv);
    std::string robot_IP = "192.168.5.11";

    // create Publisher, Subscriber and run Franka Control thread
    std::shared_ptr<FrankaNode> robot = std::make_shared<FrankaNode>("franka_right", robot_IP);

    // Start subscriber callback
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(robot);
    exec.spin();

    std::cout << "Shutting down ROS 2 process" << std::endl;
    rclcpp::shutdown();

    return 0;
}
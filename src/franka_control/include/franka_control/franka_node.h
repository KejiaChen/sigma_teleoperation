#pragma once

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/gripper.h>
#include <franka/rate_limiting.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include "custom_msgs/msg/lambda_command.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>


#include "franka_control/recorder.h"
#include "franka_control/motion_generator.h"


using std::placeholders::_1;
using namespace std::chrono_literals;


class FrankaNode : public rclcpp::Node {
   public:
    FrankaNode(std::string node_name, std::string robot_IP);
    ~FrankaNode();

   private:
    franka::Robot robot_;
    franka::Gripper gripper_;

    // Controller variables
    double v_x_ = 0.0;
    double v_y_ = 0.0;
    double v_z_ = 0.0;
    double w_x_ = 0.0;
    double w_y_ = 0.0;
    double w_z_ = 0.0;
    double v_gripper_ = 0.0;
    bool close_gripper_ = false;

    double f_x_ = 0.0;
    double f_y_ = 0.0;
    double f_z_ = 0.0;

    // I/O variables
    geometry_msgs::msg::Twist input_twist_;
    geometry_msgs::msg::Wrench output_wrench_;

    // Threadings
    std::thread control_thread_;
    std::thread gripper_thread_;
    std::thread gripper_state_thread_;
    std::mutex mutex_grip_;
    std::mutex mutex_gripper_api_;
    std::mutex mutex_vel_;
    std::mutex mutex_pose_;
    double time_ = 0.0;
    double lambda_last_update_time_ = 0.0;

    bool lambda_running = true;
    std::atomic<bool> gripper_threads_running_{true};
    std::atomic<double> cached_gripper_width_{0.0};
    std::atomic<bool> cached_is_grasped_{false};

    // ROS communication
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<custom_msgs::msg::LambdaCommand>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr delta_pose_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr frame_flange2base_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ee_velocity_cmd_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ee_velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ee_pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_velocity_cmd_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_cmd_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_width_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_grasped_publisher_;
    geometry_msgs::msg::Wrench wrench_msg_;
    sensor_msgs::msg::JointState joint_states_msgs_;
    sensor_msgs::msg::JointState joint_vel_cmd_msg_;
    geometry_msgs::msg::PoseStamped ee_pose_msg_;
    geometry_msgs::msg::PoseStamped ee_pose_cmd_msg_;
    

    struct {
        struct { double x = 0, y = 0, z = 0; } position;
        struct { double x = 0, y = 0, z = 0, w = 1; } orientation;
    } ee_pose_desired_;


    // 7-dim vectors for joint pos, vel, and effort
    std::array<double, 7> sim_joint_pos_ = {{0, -M_PI_4, 0, -3*M_PI_4, 0, M_PI_2, M_PI_4}};
    std::array<double, 7> sim_joint_vel_ = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> sim_joint_eff_ = {{0, 0, 0, 0, 0, 0, 0}};
   
    Eigen::Matrix<double, 7, 1> q_dot_, q_dot_old_;

    // array of joint names
    const std::array<std::string, 7> joint_names_ = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};

    rclcpp::SubscriptionOptions sub_options_;
    rclcpp::PublisherOptions pub_options_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    // Send/Receive Callbacks
    void send_callback();
    // void receive_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    custom_msgs::msg::LambdaCommand lambda_command_;
    void lambdaCommandCallback(const custom_msgs::msg::LambdaCommand::SharedPtr msg);
    void deltaPoseCommandCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void initializePoseTarget(const franka::RobotState& robot_states);
    void integrateVelocityToPoseTarget(const Eigen::Matrix<double, 6, 1>& velocity_command,
                                       double dt_sec);
    Eigen::Matrix<double, 6, 1> computePoseTargetCommand(
        const franka::RobotState& robot_states);
    Eigen::MatrixXd dampedLeastSquaresIK(Eigen::MatrixXd a, double dampingFactor);

    
    // Control/Shutdown Callback
    std::function<franka::JointVelocities(const franka::RobotState &, franka::Duration)> joint_velocity_callback;
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)> impedance_control_callback;
    std::function<void()> control_shutdown;

    bool use_pose_target_backend_ = false;
    bool pose_target_initialized_ = false;
    bool has_pending_delta_pose_ = false;
    double velocity_command_timeout_sec_ = 0.01;
    double pose_position_gain_ = 1.2;
    double pose_orientation_gain_ = 1.5;
    double pose_position_tolerance_ = 0.002;
    double pose_orientation_tolerance_ = 0.02;
    double pose_backend_max_linear_speed_ = 0.15;
    double pose_backend_max_angular_speed_ = 0.35;
    double delta_pose_position_scale_ = 1.0;
    double delta_pose_orientation_scale_ = 1.0;
    Eigen::Vector3d target_position_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond target_orientation_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d pending_delta_position_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond pending_delta_orientation_ = Eigen::Quaterniond::Identity();

};
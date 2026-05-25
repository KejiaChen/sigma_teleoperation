#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "custom_msgs/msg/lambda_command.hpp"
#include "franka_control/motion_generator.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;

namespace {

double clampValue(double value, double min_value, double max_value) {
  return std::min(max_value, std::max(min_value, value));
}

Eigen::Vector3d clampNorm(const Eigen::Vector3d& value, double max_norm) {
  if (max_norm <= 0.0) {
    return Eigen::Vector3d::Zero();
  }

  const double norm = value.norm();
  if (norm <= max_norm || norm < 1e-9) {
    return value;
  }

  return value * (max_norm / norm);
}

}  // namespace

class FrankaPoseNode : public rclcpp::Node {
 public:
  enum class CommandMode {
    kVelocity,
    kPose
  };

  FrankaPoseNode(const std::string& node_name, const std::string& robot_ip)
      : Node(node_name), robot_(robot_ip), gripper_(robot_ip) {
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    sub_options_.callback_group = callback_group_;
    pub_options_.callback_group = callback_group_;

    const rclcpp::QoS qos_profile(10);

    command_mode_ =
        parseCommandMode(declare_parameter<std::string>("command_mode", "pose"));
    translational_scale_factor_ =
        declare_parameter<double>("translational_scale_factor", 0.8);
    rotational_scale_factor_ =
        declare_parameter<double>("rotational_scale_factor", 0.25);
    velocity_command_timeout_sec_ =
        declare_parameter<double>("velocity_command_timeout_sec", 0.01);
    pose_position_gain_ = declare_parameter<double>("pose_position_gain", 1.2);
    pose_orientation_gain_ =
        declare_parameter<double>("pose_orientation_gain", 1.5);
    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.15);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 0.35);
    pose_position_tolerance_ =
        declare_parameter<double>("pose_position_tolerance", 0.002);
    pose_orientation_tolerance_ =
        declare_parameter<double>("pose_orientation_tolerance", 0.02);

    subscription_ = create_subscription<custom_msgs::msg::LambdaCommand>(
        "/TwistRight", qos_profile,
        std::bind(&FrankaPoseNode::lambdaCommandCallback, this, _1), sub_options_);

    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/PoseRight", qos_profile,
        std::bind(&FrankaPoseNode::poseCommandCallback, this, _1), sub_options_);

    joint_states_publisher_ =
        create_publisher<sensor_msgs::msg::JointState>("/frankaRight/joint_states", 1);
    ee_pose_publisher_ =
        create_publisher<std_msgs::msg::Float64MultiArray>("/frankaRight/ee_pose", 1);
    wrench_publisher_ =
        create_publisher<geometry_msgs::msg::Wrench>("/WrenchRight", 1);
    frame_flange2base_publisher_ =
        create_publisher<std_msgs::msg::Float64MultiArray>("/O_T_FTSensorRight", 1);
    joint_velocity_cmd_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
        "/frankaRight/joint_velocity_cmd", 1);
    ee_velocity_cmd_publisher_ =
        create_publisher<geometry_msgs::msg::Twist>("/frankaRight/ee_velocity_cmd", 1);
    ee_velocity_publisher_ =
        create_publisher<geometry_msgs::msg::Twist>("/frankaRight/ee_velocity", 1);
    ee_pose_cmd_publisher_ =
        create_publisher<geometry_msgs::msg::PoseStamped>("/frankaRight/ee_pose_cmd", 1);

    control_shutdown_ = [&]() {
      robot_.stop();
      gripper_.stop();

      geometry_msgs::msg::Wrench output_wrench;
      output_wrench.force.x = 0.0;
      output_wrench.force.y = 0.0;
      output_wrench.force.z = 0.0;
      wrench_publisher_->publish(output_wrench);

      std::cout << "Stopped Franka Pose Control" << std::endl;
    };

    rclcpp::on_shutdown(control_shutdown_);

    RCLCPP_INFO(
        get_logger(), "Starting franka_pose_node in %s mode",
        command_mode_ == CommandMode::kPose ? "pose" : "velocity");

    startGripperThread();
    startControlThread();
  }

  ~FrankaPoseNode() override {
    if (control_thread_.joinable()) {
      std::cout << "Joining Control thread" << std::endl;
      control_thread_.join();
    }

    if (gripper_thread_.joinable()) {
      std::cout << "Joining Gripper thread" << std::endl;
      gripper_thread_.join();
    }

    std::cout << "Shutting down pose robot node" << std::endl;
  }

 private:
  void startGripperThread() {
    gripper_thread_ = std::thread([this]() {
      try {
        std::cout << "Initializing gripper..." << std::endl;
        gripper_.stop();
        gripper_.homing();

        const franka::GripperState gripper_state = gripper_.readOnce();
        const double gripper_max_width = gripper_state.max_width;
        const double grasping_width = 0.0;
        const double grasping_force = 100.0;

        bool close_gripper_flag = false;
        bool grasped_flag = false;

        gripper_.grasp(grasping_width, 0.5, grasping_force, 0.1, 0.1);

        while (true) {
          if (mutex_grip_.try_lock()) {
            close_gripper_flag = close_gripper_;
            mutex_grip_.unlock();
          }

          if (close_gripper_flag) {
            if (!grasped_flag) {
              grasped_flag =
                  gripper_.grasp(grasping_width, 0.5, grasping_force, 0.1, 0.1);
              std::cout << "Gripper success: " << grasped_flag << std::endl;
            }
          } else {
            gripper_.stop();
            gripper_.move(gripper_max_width, 0.1);
            grasped_flag = false;
          }

          std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
      } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
      }
    });
  }

  void startControlThread() {
    control_thread_ = std::thread([this]() {
      try {
        const std::array<double, 7> q_goal = {{0.515222, -0.27677, -0.107084,
                                               -2.64876, 0.204069, 2.43711,
                                               -0.456169}};

        MotionGenerator motion_generator(0.1, q_goal);
        std::cout << "MOVING ROBOT TO HOME POSITION." << std::endl;
        robot_.control(motion_generator);

        franka::Model model = robot_.loadModel();
        q_dot_.setZero();
        q_dot_old_.setZero();

        joint_velocity_callback_ =
            [&](const franka::RobotState& robot_states,
                franka::Duration dt) -> franka::JointVelocities {
          time_ += dt.toSec();

          const std::array<double, 42> jacobian_array =
              model.zeroJacobian(franka::Frame::kEndEffector, robot_states);
          const Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(
              jacobian_array.data());

          const Eigen::Matrix<double, 6, 1> cartesian_command =
              computeCartesianCommand(robot_states);

          Eigen::Matrix<double, 6, 1> safe_cartesian_command = cartesian_command;
          applyWorkspaceSafety(safe_cartesian_command, robot_states, dt.toSec());

          const Eigen::MatrixXd inv_jacobian = dampedLeastSquaresIK(jacobian, 0.01);
          q_dot_ = inv_jacobian * safe_cartesian_command;

          const double approximate_joint_acc_threshold = 10.0;
          const double low_pass_filter = 0.6;
          for (int idx = 0; idx < 7; ++idx) {
            q_dot_[idx] =
                low_pass_filter * q_dot_old_[idx] + (1.0 - low_pass_filter) * q_dot_[idx];
            q_dot_[idx] = std::min(
                approximate_joint_acc_threshold * dt.toSec() + q_dot_old_[idx],
                std::max(-approximate_joint_acc_threshold * dt.toSec() + q_dot_old_[idx],
                         q_dot_[idx]));
            q_dot_old_[idx] = q_dot_[idx];
          }

          publishState(robot_states, jacobian);
          publishDesiredPose(robot_states);
          publishWrench(robot_states);
          publishFtSensorFrame(robot_states);
          publishJointVelocityCommand();

          return franka::JointVelocities{{q_dot_[0], q_dot_[1], q_dot_[2], q_dot_[3],
                                          q_dot_[4], q_dot_[5], q_dot_[6]}};
        };

        std::cout << "WARNING: Starting teleoperation! Please make sure to have "
                     "the user stop button at hand !"
                  << std::endl;

        while (true) {
          try {
            robot_.control(joint_velocity_callback_);
          } catch (const franka::ControlException& e) {
            std::cout << "Running error recovery..." << std::endl;
            std::cout << e.what() << std::endl;
            robot_.automaticErrorRecovery();
          }
        }
      } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        control_shutdown_();
        std::cout << "Shutting down ROS 2 process" << std::endl;
        rclcpp::shutdown();
      }
    });
  }

  CommandMode parseCommandMode(const std::string& mode) {
    if (mode == "velocity") {
      return CommandMode::kVelocity;
    }
    if (mode == "pose") {
      return CommandMode::kPose;
    }

    RCLCPP_WARN(
        get_logger(), "Unknown command_mode '%s', defaulting to pose", mode.c_str());
    return CommandMode::kPose;
  }

  Eigen::Matrix<double, 6, 1> computeCartesianCommand(
      const franka::RobotState& robot_states) {
    if (command_mode_ == CommandMode::kPose) {
      return computePoseServoCommand(robot_states);
    }

    return computeVelocityCommand();
  }

  Eigen::Matrix<double, 6, 1> computeVelocityCommand() {
    Eigen::Matrix<double, 6, 1> command;
    command.setZero();

    if (mutex_vel_.try_lock()) {
      command << v_x_, v_y_, v_z_, w_x_, w_y_, w_z_;
      mutex_vel_.unlock();
    }

    if (now().seconds() - lambda_last_update_time_ > velocity_command_timeout_sec_) {
      command.setZero();
      return command;
    }

    command[0] *= translational_scale_factor_;
    command[1] *= translational_scale_factor_;
    command[2] *= translational_scale_factor_;
    command[3] *= rotational_scale_factor_;
    command[4] *= rotational_scale_factor_;
    command[5] *= rotational_scale_factor_;

    return command;
  }

  Eigen::Matrix<double, 6, 1> computePoseServoCommand(
      const franka::RobotState& robot_states) {
    Eigen::Matrix<double, 6, 1> command;
    command.setZero();

    Eigen::Vector3d target_position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond target_orientation = Eigen::Quaterniond::Identity();
    bool has_pose_target = false;

    if (mutex_pose_.try_lock()) {
      target_position = target_position_;
      target_orientation = target_orientation_;
      has_pose_target = pose_target_received_;
      mutex_pose_.unlock();
    }

    if (!has_pose_target) {
      return command;
    }

    const Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> current_pose(
        robot_states.O_T_EE.data());
    const Eigen::Vector3d current_position = current_pose.block<3, 1>(0, 3);
    const Eigen::Matrix3d current_rotation = current_pose.block<3, 3>(0, 0);
    Eigen::Quaterniond current_orientation(current_rotation);
    current_orientation.normalize();

    const Eigen::Vector3d position_error = target_position - current_position;

    Eigen::Quaterniond q_error = target_orientation * current_orientation.conjugate();
    if (q_error.w() < 0.0) {
      q_error.coeffs() *= -1.0;
    }
    q_error.normalize();

    Eigen::Vector3d orientation_error = Eigen::Vector3d::Zero();
    const Eigen::AngleAxisd angle_axis(q_error);
    if (std::abs(angle_axis.angle()) > pose_orientation_tolerance_) {
      orientation_error = angle_axis.axis() * angle_axis.angle();
    }

    Eigen::Vector3d linear_cmd = pose_position_gain_ * position_error;
    Eigen::Vector3d angular_cmd = pose_orientation_gain_ * orientation_error;

    if (position_error.norm() < pose_position_tolerance_) {
      linear_cmd.setZero();
    }

    linear_cmd = clampNorm(linear_cmd, max_linear_speed_);
    angular_cmd = clampNorm(angular_cmd, max_angular_speed_);

    command << linear_cmd, angular_cmd;
    return command;
  }

  void applyWorkspaceSafety(Eigen::Matrix<double, 6, 1>& command,
                            const franka::RobotState& robot_states,
                            double dt_sec) const {
    const double x_min_external = 0.20;
    const double x_max_external = 0.80;
    const double y_min_external = -0.45 - 0.281;
    const double y_max_external = 0.85 - 0.281;
    const double z_min_external = 0.11;
    const double z_max_external = 0.55;

    if (robot_states.O_T_EE[12] + command[0] * dt_sec < x_min_external && command[0] < 0.0) {
      command[0] = 0.0;
    }
    if (robot_states.O_T_EE[12] + command[0] * dt_sec > x_max_external && command[0] > 0.0) {
      command[0] = 0.0;
    }
    if (robot_states.O_T_EE[13] + command[1] * dt_sec < y_min_external && command[1] < 0.0) {
      command[1] = 0.0;
    }
    if (robot_states.O_T_EE[13] + command[1] * dt_sec > y_max_external && command[1] > 0.0) {
      command[1] = 0.0;
    }
    if (robot_states.O_T_EE[14] + command[2] * dt_sec < z_min_external && command[2] < 0.0) {
      command[2] = 0.0;
    }
    if (robot_states.O_T_EE[14] + command[2] * dt_sec > z_max_external && command[2] > 0.0) {
      command[2] = 0.0;
    }
  }

  void publishState(const franka::RobotState& robot_states,
                    const Eigen::Matrix<double, 6, 7>& jacobian) {
    geometry_msgs::msg::Twist ee_velocity_cmd_msg;
    const std::array<double, 7> dq_d_array = robot_states.dq_d;
    const Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq_d_eigen(dq_d_array.data());
    const Eigen::Matrix<double, 6, 1> ee_velocity_cmd = jacobian * dq_d_eigen;
    ee_velocity_cmd_msg.linear.x = ee_velocity_cmd[0];
    ee_velocity_cmd_msg.linear.y = ee_velocity_cmd[1];
    ee_velocity_cmd_msg.linear.z = ee_velocity_cmd[2];
    ee_velocity_cmd_msg.angular.x = ee_velocity_cmd[3];
    ee_velocity_cmd_msg.angular.y = ee_velocity_cmd[4];
    ee_velocity_cmd_msg.angular.z = ee_velocity_cmd[5];
    ee_velocity_cmd_publisher_->publish(ee_velocity_cmd_msg);

    geometry_msgs::msg::Twist ee_vel_msg;
    const std::array<double, 7> dq_array = robot_states.dq;
    const Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq_eigen(dq_array.data());
    const Eigen::Matrix<double, 6, 1> ee_velocity = jacobian * dq_eigen;
    ee_vel_msg.linear.x = ee_velocity[0];
    ee_vel_msg.linear.y = ee_velocity[1];
    ee_vel_msg.linear.z = ee_velocity[2];
    ee_vel_msg.angular.x = ee_velocity[3];
    ee_vel_msg.angular.y = ee_velocity[4];
    ee_vel_msg.angular.z = ee_velocity[5];
    ee_velocity_publisher_->publish(ee_vel_msg);

    joint_states_msg_.header.stamp = now();
    joint_states_msg_.name.assign(joint_names_.begin(), joint_names_.end());
    joint_states_msg_.position.assign(robot_states.q.begin(), robot_states.q.end());
    joint_states_msg_.velocity.assign(robot_states.dq.begin(), robot_states.dq.end());
    joint_states_msg_.effort.assign(robot_states.tau_J.begin(), robot_states.tau_J.end());
    joint_states_publisher_->publish(joint_states_msg_);

    std_msgs::msg::Float64MultiArray current_pose_msg;
    current_pose_msg.data.assign(robot_states.O_T_EE.begin(), robot_states.O_T_EE.end());
    ee_pose_publisher_->publish(current_pose_msg);
  }

  void publishDesiredPose(const franka::RobotState& robot_states) {
    std::array<double, 16> desired_pose = robot_states.O_T_EE_d;

    if (command_mode_ == CommandMode::kPose) {
      Eigen::Vector3d target_position = Eigen::Vector3d::Zero();
      Eigen::Quaterniond target_orientation = Eigen::Quaterniond::Identity();
      bool has_pose_target = false;

      if (mutex_pose_.try_lock()) {
        target_position = target_position_;
        target_orientation = target_orientation_;
        has_pose_target = pose_target_received_;
        mutex_pose_.unlock();
      }

      if (has_pose_target) {
        Eigen::Matrix4d desired_pose_eigen = Eigen::Matrix4d::Identity();
        desired_pose_eigen.block<3, 3>(0, 0) = target_orientation.toRotationMatrix();
        desired_pose_eigen.block<3, 1>(0, 3) = target_position;
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(desired_pose.data()) =
            desired_pose_eigen;
      } else {
        desired_pose = robot_states.O_T_EE;
      }
    }
    geometry_msgs::msg::PoseStamped desired_pose_msg;
    desired_pose_msg.header.stamp = now();
    desired_pose_msg.header.frame_id = "panda_link0";

    const Eigen::Vector3d command_position(desired_pose[12], desired_pose[13], desired_pose[14]);
    const Eigen::Map<const Eigen::Matrix4d> desired_pose_matrix(desired_pose.data());
    Eigen::Quaterniond command_orientation(desired_pose_matrix.block<3, 3>(0, 0));
    command_orientation.normalize();

    desired_pose_msg.pose.position.x = command_position.x();
    desired_pose_msg.pose.position.y = command_position.y();
    desired_pose_msg.pose.position.z = command_position.z();
    desired_pose_msg.pose.orientation.x = command_orientation.x();
    desired_pose_msg.pose.orientation.y = command_orientation.y();
    desired_pose_msg.pose.orientation.z = command_orientation.z();
    desired_pose_msg.pose.orientation.w = command_orientation.w();
    ee_pose_cmd_publisher_->publish(desired_pose_msg);
  }

  void publishWrench(const franka::RobotState& robot_states) {
    wrench_msg_.force.x = robot_states.O_F_ext_hat_K[0];
    wrench_msg_.force.y = robot_states.O_F_ext_hat_K[1];
    wrench_msg_.force.z = robot_states.O_F_ext_hat_K[2];
    wrench_msg_.torque.x = robot_states.O_F_ext_hat_K[3];
    wrench_msg_.torque.y = robot_states.O_F_ext_hat_K[4];
    wrench_msg_.torque.z = robot_states.O_F_ext_hat_K[5];
    wrench_publisher_->publish(wrench_msg_);
  }

  void publishFtSensorFrame(const franka::RobotState& robot_states) {
    const std::array<double, 16> O_T_EE = robot_states.O_T_EE;
    const std::array<double, 16> F_T_EE = robot_states.F_T_EE;

    const Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> O_T_EE_eigen(
        O_T_EE.data());
    const Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> F_T_EE_eigen(
        F_T_EE.data());

    const Eigen::Matrix<double, 4, 4> O_T_F = O_T_EE_eigen * F_T_EE_eigen.inverse();

    Eigen::Matrix<double, 4, 4> O_F_FTSensor;
    O_F_FTSensor << 0, 1, 0, 0,
                    -1, 0, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;

    const Eigen::Matrix<double, 4, 4> O_T_FTSensor = O_T_F * O_F_FTSensor;

    std::array<double, 9> R_T_FTSensor_array;
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R_T_FTSensor_array.data()) =
        O_T_FTSensor.block<3, 3>(0, 0);

    std_msgs::msg::Float64MultiArray rotation_msg;
    rotation_msg.data.assign(R_T_FTSensor_array.begin(), R_T_FTSensor_array.end());
    frame_flange2base_publisher_->publish(rotation_msg);
  }

  void publishJointVelocityCommand() {
    joint_vel_cmd_msg_.header.stamp = now();
    joint_vel_cmd_msg_.name.assign(joint_names_.begin(), joint_names_.end());
    joint_vel_cmd_msg_.velocity.clear();
    for (int i = 0; i < 7; ++i) {
      joint_vel_cmd_msg_.velocity.push_back(q_dot_[i]);
    }
    joint_velocity_cmd_publisher_->publish(joint_vel_cmd_msg_);
  }

  Eigen::MatrixXd dampedLeastSquaresIK(const Eigen::MatrixXd& jacobian,
                                       double damping_factor) const {
    if (jacobian.rows() > jacobian.cols()) {
      return jacobian.transpose() *
             (jacobian * jacobian.transpose() +
              damping_factor * damping_factor *
                  Eigen::MatrixXd::Identity(jacobian.rows(), jacobian.rows()))
                 .inverse();
    }

    if (jacobian.rows() < jacobian.cols()) {
      return (jacobian.transpose() * jacobian +
              damping_factor * damping_factor *
                  Eigen::MatrixXd::Identity(jacobian.cols(), jacobian.cols()))
                 .inverse() *
             jacobian.transpose();
    }

    return jacobian.inverse();
  }

  void lambdaCommandCallback(const custom_msgs::msg::LambdaCommand::SharedPtr msg) {
    lambda_command_ = *msg;
    if (mutex_vel_.try_lock()) {
      v_x_ = clampValue(lambda_command_.linear.x, -2.0, 2.0);
      v_y_ = clampValue(lambda_command_.linear.y, -2.0, 2.0);
      v_z_ = clampValue(lambda_command_.linear.z, -2.0, 2.0);
      w_x_ = clampValue(lambda_command_.angular.x, -2.0, 2.0);
      w_y_ = clampValue(lambda_command_.angular.y, -2.0, 2.0);
      w_z_ = clampValue(lambda_command_.angular.z, -2.0, 2.0);
      if (mutex_grip_.try_lock()) {
        v_gripper_ = lambda_command_.v_gripper;
        close_gripper_ = lambda_command_.enable_backlash_compensation;
        mutex_grip_.unlock();
      }
      lambda_last_update_time_ = now().seconds();
      mutex_vel_.unlock();
    }
  }

  void poseCommandCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    Eigen::Quaterniond target_orientation(msg->pose.orientation.w, msg->pose.orientation.x,
                                          msg->pose.orientation.y, msg->pose.orientation.z);
    if (target_orientation.norm() < 1e-6) {
      bool has_pose_target = false;
      if (mutex_pose_.try_lock()) {
        target_orientation = target_orientation_;
        has_pose_target = pose_target_received_;
        mutex_pose_.unlock();
      }
      if (!has_pose_target) {
        target_orientation = Eigen::Quaterniond::Identity();
      }
    } else {
      target_orientation.normalize();
    }

    Eigen::Vector3d target_position(msg->pose.position.x, msg->pose.position.y,
                                    msg->pose.position.z);
    target_position.x() = clampValue(target_position.x(), 0.20, 0.80);
    target_position.y() = clampValue(target_position.y(), -0.45 - 0.281, 0.85 - 0.281);
    target_position.z() = clampValue(target_position.z(), 0.11, 0.55);

    if (mutex_pose_.try_lock()) {
      pose_command_ = *msg;
      target_position_ = target_position;
      target_orientation_ = target_orientation;
      pose_target_received_ = true;
      mutex_pose_.unlock();
    }
  }

  franka::Robot robot_;
  franka::Gripper gripper_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::SubscriptionOptions sub_options_;
  rclcpp::PublisherOptions pub_options_;

  rclcpp::Subscription<custom_msgs::msg::LambdaCommand>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr wrench_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr frame_flange2base_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ee_velocity_cmd_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ee_velocity_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ee_pose_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_velocity_cmd_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_cmd_publisher_;

  std::thread control_thread_;
  std::thread gripper_thread_;
  std::mutex mutex_grip_;
  std::mutex mutex_vel_;
  std::mutex mutex_pose_;

  std::function<franka::JointVelocities(const franka::RobotState&, franka::Duration)>
      joint_velocity_callback_;
  std::function<void()> control_shutdown_;

  sensor_msgs::msg::JointState joint_states_msg_;
  sensor_msgs::msg::JointState joint_vel_cmd_msg_;
  geometry_msgs::msg::Wrench wrench_msg_;
  custom_msgs::msg::LambdaCommand lambda_command_;
  geometry_msgs::msg::PoseStamped pose_command_;

  Eigen::Matrix<double, 7, 1> q_dot_;
  Eigen::Matrix<double, 7, 1> q_dot_old_;
  Eigen::Vector3d target_position_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond target_orientation_ = Eigen::Quaterniond::Identity();

  std::array<std::string, 7> joint_names_ = {{"panda_joint1", "panda_joint2",
                                              "panda_joint3", "panda_joint4",
                                              "panda_joint5", "panda_joint6",
                                              "panda_joint7"}};

  CommandMode command_mode_;

  double v_x_ = 0.0;
  double v_y_ = 0.0;
  double v_z_ = 0.0;
  double w_x_ = 0.0;
  double w_y_ = 0.0;
  double w_z_ = 0.0;
  double v_gripper_ = 0.0;
  bool close_gripper_ = false;

  double time_ = 0.0;
  double lambda_last_update_time_ = 0.0;
  double translational_scale_factor_ = 0.8;
  double rotational_scale_factor_ = 0.25;
  double velocity_command_timeout_sec_ = 0.01;
  double pose_position_gain_ = 1.2;
  double pose_orientation_gain_ = 1.5;
  double max_linear_speed_ = 0.15;
  double max_angular_speed_ = 0.35;
  double pose_position_tolerance_ = 0.002;
  double pose_orientation_tolerance_ = 0.02;
  bool pose_target_received_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  std::string robot_ip = "192.168.5.11";
  for (int i = 1; i < argc - 1; ++i) {
    if (std::string(argv[i]) == "--robot-ip") {
      robot_ip = argv[i + 1];
      break;
    }
  }

  std::shared_ptr<FrankaPoseNode> robot =
      std::make_shared<FrankaPoseNode>("franka_pose_right", robot_ip);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(robot);
  exec.spin();

  std::cout << "Shutting down ROS 2 process" << std::endl;
  rclcpp::shutdown();
  return 0;
}

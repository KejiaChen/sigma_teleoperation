#include "franka_control/franka_node_inference.h"

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

Eigen::Vector3d extractFrankaPosition(const std::array<double, 16>& pose) {
    return Eigen::Vector3d(pose[12], pose[13], pose[14]);
}

Eigen::Quaterniond extractFrankaOrientation(const std::array<double, 16>& pose) {
    const Eigen::Map<const Eigen::Matrix4d> pose_matrix(pose.data());
    Eigen::Quaterniond orientation(pose_matrix.block<3, 3>(0, 0));
    orientation.normalize();
    return orientation;
}

std::array<double, 16> makeFrankaPoseArray(const Eigen::Vector3d& position,
                                           const Eigen::Quaterniond& orientation) {
    Eigen::Matrix4d pose_matrix = Eigen::Matrix4d::Identity();
    pose_matrix.block<3, 3>(0, 0) = orientation.toRotationMatrix();
    pose_matrix.block<3, 1>(0, 3) = position;

    std::array<double, 16> pose{};
    Eigen::Map<Eigen::Matrix4d>(pose.data()) = pose_matrix;
    return pose;
}

constexpr double kWorkspaceXMin = 0.20;
constexpr double kWorkspaceXMax = 0.80;
constexpr double kWorkspaceYMin = -0.45 - 0.281;
constexpr double kWorkspaceYMax = 0.85 - 0.281;
constexpr double kWorkspaceZMin = -0.135;
constexpr double kWorkspaceZMax = 0.55;

}  // namespace


FrankaNodeInference::FrankaNodeInference(std::string node_name, std::string robot_IP) : Node(node_name), robot_(robot_IP), gripper_(robot_IP) {
    
    // Multithread publisher and subscriber
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    sub_options_.callback_group = callback_group_;
    pub_options_.callback_group = callback_group_;

    // QoS profile
    rclcpp::QoS qos_profile(10);

    use_pose_target_backend_ = this->declare_parameter<bool>("use_pose_target_backend", false);
    velocity_command_timeout_sec_ = this->declare_parameter<double>("velocity_command_timeout_sec", 0.01);
    pose_position_gain_ = this->declare_parameter<double>("pose_position_gain", 1.2);
    pose_orientation_gain_ = this->declare_parameter<double>("pose_orientation_gain", 1.5);
    pose_position_tolerance_ = this->declare_parameter<double>("pose_position_tolerance", 0.002);
    pose_orientation_tolerance_ = this->declare_parameter<double>("pose_orientation_tolerance", 0.02);
    pose_backend_max_linear_speed_ =
        this->declare_parameter<double>("pose_backend_max_linear_speed", 0.15);
    pose_backend_max_angular_speed_ =
        this->declare_parameter<double>("pose_backend_max_angular_speed", 0.35);
    // Create subscriber
    absolute_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/AbsolutePoseRight", qos_profile, std::bind(&FrankaNodeInference::absolutePoseCommandCallback, this, _1),
        sub_options_);
    gripper_command_subscription_ = create_subscription<std_msgs::msg::Float64>(
        "/GripperCmdRight", qos_profile,
        std::bind(&FrankaNodeInference::gripperCommandCallback, this, _1), sub_options_);
    
    joint_states_publisher_ =
      this->create_publisher<sensor_msgs::msg::JointState>("/frankaRight/joint_states", 1);

    ee_pose_publisher_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>("/frankaRight/ee_pose", 1);
    
    publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/WrenchRight", 1); // TODO: read left/right from launch file
    
    frame_flange2base_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/O_T_FTSensorRight", 1);

    joint_velocity_cmd_publisher_ = 
        this->create_publisher<sensor_msgs::msg::JointState>("/frankaRight/joint_velocity_cmd", 1);

    ee_velocity_cmd_publisher_ = 
        this->create_publisher<geometry_msgs::msg::Twist>("/frankaRight/ee_velocity_cmd", 1);
    
    ee_velocity_publisher_ = 
        this->create_publisher<geometry_msgs::msg::Twist>("/frankaRight/ee_velocity", 1);

    ee_pose_cmd_publisher_ = 
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/frankaRight/ee_pose_cmd", 1);

    gripper_width_publisher_ =
        this->create_publisher<std_msgs::msg::Float64>("/frankaRight/gripper_width", 1);

    is_grasped_publisher_ =
        this->create_publisher<std_msgs::msg::Bool>("/frankaRight/is_grasped", 1);

    RCLCPP_INFO(this->get_logger(), "Pose target backend: %s",
                use_pose_target_backend_ ? "enabled" : "disabled");

    
    // create shutdown handler
    control_shutdown = [&]() {
        gripper_threads_running_ = false;
        robot_.stop();
        {
            std::lock_guard<std::mutex> gripper_api_lock(mutex_gripper_api_);
            gripper_.stop();
        }

        output_wrench_.force.x = 0.0;
        output_wrench_.force.y = 0.0;
        output_wrench_.force.z = 0.0;
        publisher_->publish(output_wrench_);

        std::cout << "Stopped Franka Control" << std::endl;
    };

    rclcpp::on_shutdown(control_shutdown);

    ///////////////////////////// CONTROL THREAD ////////////////////////////////
    gripper_thread_ = std::thread([this]() {
        try {
            std::cout << "Initializing gripper..." << std::endl;

            const double grasping_width = 0.0;
            const double grasping_force = 100.0;  // TODO: SET REASONABLE VALUE
            double gripper_max_width = 0.0;
            bool close_gripper_flag = false;
            bool grasped_flag = false;

            {
                std::lock_guard<std::mutex> grip_lock(mutex_grip_);
                close_gripper_flag = close_gripper_;
            }

            {
                std::lock_guard<std::mutex> gripper_api_lock(mutex_gripper_api_);
                gripper_.stop();
                gripper_.homing();

                const franka::GripperState gripper_state = gripper_.readOnce();
                gripper_max_width = gripper_state.max_width;
                cached_gripper_width_.store(gripper_state.width);
                cached_is_grasped_.store(gripper_state.is_grasped);

                if (close_gripper_flag) {
                    grasped_flag = gripper_.grasp(grasping_width, 0.5, grasping_force, 0.1, 0.1);
                    std::cout << "Gripper success: " << grasped_flag << std::endl;
                } else {
                    gripper_.move(gripper_max_width, 0.1);
                }
            }

            while (gripper_threads_running_.load()) {
                {
                    std::lock_guard<std::mutex> grip_lock(mutex_grip_);
                    close_gripper_flag = close_gripper_;
                }

                if (close_gripper_flag && !grasped_flag) {
                    std::lock_guard<std::mutex> gripper_api_lock(mutex_gripper_api_);
                    grasped_flag = gripper_.grasp(grasping_width, 0.5, grasping_force, 0.1, 0.1);
                    std::cout << "Gripper success: " << grasped_flag << std::endl;
                } else if (!close_gripper_flag && grasped_flag) {
                    std::lock_guard<std::mutex> gripper_api_lock(mutex_gripper_api_);
                    gripper_.stop();
                    gripper_.move(gripper_max_width, 0.1);
                    grasped_flag = false;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        } catch (const franka::Exception& e) {
            std::cout << e.what() << std::endl;
        }
    });

    gripper_state_thread_ = std::thread([this]() {
        try {
            std_msgs::msg::Float64 gripper_width_msg;
            std_msgs::msg::Bool is_grasped_msg;

            while (gripper_threads_running_.load()) {
                franka::GripperState gripper_state;
                {
                    std::lock_guard<std::mutex> gripper_api_lock(mutex_gripper_api_);
                    gripper_state = gripper_.readOnce();
                }

                cached_gripper_width_.store(gripper_state.width);
                cached_is_grasped_.store(gripper_state.is_grasped);
                gripper_width_msg.data = gripper_state.width;
                is_grasped_msg.data = gripper_state.is_grasped;
                gripper_width_publisher_->publish(gripper_width_msg);
                is_grasped_publisher_->publish(is_grasped_msg);

                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        } catch (const franka::Exception& e) {
            std::cout << e.what() << std::endl;
        }
    });

    control_thread_ = std::thread([this]() { 
        try {
            // Drive into home position
            // std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3*M_PI_4, 0, M_PI_2, M_PI_4}};
            // std::array<double, 7> q_goal = {{-0.117556,0.905409,0.546081,-1.82483,0.301528,1.4908,-1.73522}};  //bottle            
            // std::array<double, 7> q_goal = {{-0.524871,-0.0753273,0.634727,-2.11237,0.0445151,2.06996,-0.74712}};  //initial
            // std::array<double, 7> q_goal = {{-0.0515103,0.008143,0.164156,-2.54973,1.09974,2.1721,-1.56182}};  //cable
            std::array<double, 7> q_goal = {{0.43226084,  0.36218423, -0.3371766 , -2.19546306,  0.22759046, 2.59549384,  0.79302517}};
            
            //=========xuanming init pose========================================================
            // std::array<double, 7> q_goal = {{-1.23556,0.95542,1.40872,-2.45478,0.0725108,2.71996,2.31233}}; // xuanming init pose 20250909
            // std::array<double, 7> q_goal = {{0.191995,-0.0277851,0.0784247,-2.48233,1.34177,2.18028,1.61919}}; // 2025.12.15
            //===================================================================================


            //=========Tailai init pose==========================================================
            // std::array<double, 7> q_goal = {{-0.57476,0.0360376,0.424265,-2.6671,1.44048,2.16108,1.18508}};  //Tailai init pose
            //===================================================================================

            //=========Xiangyu init pose==========================================================
            // std::array<double, 7> q_goal = {{0.515222,-0.27677,-0.107084,-2.64876,0.204069,2.43711,-0.456169}};  //Xiangyu init pose
            //===================================================================================



            MotionGenerator motion_generator(0.1, q_goal);
            std::cout << "MOVING ROBOT TO HOME POSITION." << std::endl;
            robot_.control(motion_generator);

            franka::Model model = robot_.loadModel();

            Eigen::VectorXd v_command(6);
            Eigen::VectorXd v_buffer(6);
            v_command.setZero();
            v_buffer.setZero();
            q_dot_.setZero();
            q_dot_old_.setZero();

            franka::JointVelocities velocities = {{ 0., 0., 0., 0., 0., 0., 0. }};

            // Creating recorder
            // double t_rec = 20.0;
            // double SampletimeInit = 0.001;
            // int NODataRec = 10;
            // string name = "DATA";

            // Recorder rec(t_rec, SampletimeInit, NODataRec, name);


            joint_velocity_callback = [&](const franka::RobotState& robot_states, franka::Duration dt) -> franka::JointVelocities {
                time_ += dt.toSec();

                // std::cout << robot_states << std::endl;
                //calculate desired EE Pose outside the controller
                std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_states);
                // convert to Eigen
                Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

                // // ----------  xuanming: test control part  ---------------------------
                // // --------------------------------------------------------------------

                // std::cout << "time_: " << time_ << std::endl;

                // // calculate filtered cartician velocity from dq_d
                // Eigen::VectorXd dq_desire(7);
                // for (size_t i = 0; i < 7; ++i) {
                //     dq_desire(i) = robot_states.dq_d[i];
                // }
                // Eigen::VectorXd v_desire(6);
                // v_desire = jacobian * dq_desire;
                
                // //calculate integration for desired EE pose
                // double dt_sec = dt.toSec();
                // ee_pose_desired_.position.x = robot_states.O_T_EE[12] + v_desire(0) * dt_sec;
                // ee_pose_desired_.position.y = robot_states.O_T_EE[13] + v_desire(1) * dt_sec;
                // ee_pose_desired_.position.z = robot_states.O_T_EE[14] + v_desire(2) * dt_sec;
                // //update desired orientation (keep constant for now)
                // ee_pose_desired_.orientation.x = robot_states.O_T_EE_d[4 * 3 + 0];
                // ee_pose_desired_.orientation.y = robot_states.O_T_EE_d[4 * 3 + 1];
                // ee_pose_desired_.orientation.z = robot_states.O_T_EE_d[4 * 3 + 2];
                // ee_pose_desired_.orientation.w = robot_states.O_T_EE_d[4 * 3 + 3];  
                // //print desired EE pose
                // std::cout << "Desired EE position: " << ee_pose_desired_.position.x << " " << ee_pose_desired_.position.y << " " << ee_pose_desired_.position.z << std::endl;
                
                
                // std::cout << "O_T_EE_d: ";
                // for (size_t i = 0; i < robot_states.O_T_EE_d.size(); ++i) {
                //     std::cout << robot_states.O_T_EE_d[i];
                //     if (i + 1 < robot_states.O_T_EE_d.size()) std::cout << " ";
                // }
                // std::cout << std::endl;

                // std::cout << "dq_d:  " ;
                // for (size_t i = 0; i < robot_states.dq_d.size(); ++i) {
                //     std::cout << robot_states.dq_d[i];
                //     if (i + 1 < robot_states.dq_d.size()) std::cout << " ";
                // }
                // std::cout << std::endl;

                // std::cout << "---------------------------------------------------------" << std::endl;


                // //------------  xuanming: test control part -- END --------------------------------------------
                // //---------------------------------------------------------------------------------------------
                
                
                
                
                // fake input test begin
                // double cycle = std::floor(pow(-1.0, (time_ - std::fmod(time_, time_max)) / time_max));
                // double v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time_));
                // double v_x = std::cos(angle) * v;
                // double v_z = -std::sin(angle) * v;
                
                // // std::cout << "v_x: " << v_x << " v_z: " << v_z << " v: " << v << std::endl;
                // // init velocity with eigen vector with v_x and v_z
                // Eigen::VectorXd v_fake(6);
                // v_fake << v_x, 0.0, v_z, 0.0, 0.0, 0.0;
                // // std::cout << "v_fake: " << v_fake[0] << " " << v_fake[1] << " " << v_fake[2] << " " << v_fake[3] << " " << v_fake[4] << " " << v_fake[5] << std::endl;
                // std::array<double, 42> jacobian_array = model.bodyJacobian(franka::Frame::kEndEffector, robot_states);

                // // convert to Eigen
                // Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

                // Eigen::MatrixXd inv_Jacobian = dampedLeastSquaresIK(jacobian, 0.01);

                // // print inv_Jacobian
                // // std::cout << "inv_Jacobian: " << inv_Jacobian << std::endl;
                // // calculate joint velocities with inv_jacobian * v_fake
                // Eigen::VectorXd q_dot_fake(7);
                // q_dot_fake = inv_Jacobian * v_fake;
                // //print q_dot_fake
                // std::cout << "q_dot_fake: " << q_dot_fake[0] << " " << q_dot_fake[1] << " " << q_dot_fake[2] << " " << q_dot_fake[3] << " " << q_dot_fake[4] << " " << q_dot_fake[5] << " " << q_dot_fake[6] << std::endl;
                // double q_dot_scale = 0.6;
                // // scale q_dot_fake
                // q_dot_fake = q_dot_scale * q_dot_fake;
                
                // fake input test end
    
                // array<double, 16> pose = robot_states.O_T_EE;
                // std::cout << "Pose: " << pose[12] << " " << pose[13] << " " << pose[14] << std::endl;
                /* RECEIVING PART */
                double translational_scale_factor_x = 3.5;
                double translational_scale_factor_y = 3.5;
                double translational_scale_factor_z = 3.5;

                // Config: Lock the rotation
                // double rotational_scale_factor = 0.4;
                double rotational_scale_factor = 0.4;
                // double rotational_scale_factor = 0;

                if (mutex_vel_.try_lock()) {
                        v_command << v_x_, v_y_, v_z_, w_x_, w_y_, w_z_;
                    mutex_vel_.unlock();
                }

                v_buffer = v_command;
                    if (this->now().seconds() - lambda_last_update_time_ > velocity_command_timeout_sec_) {
                    lambda_running = false;
                }
                else {
                    lambda_running = true;
                }

                if (!lambda_running) {
                    v_command[0] = 0.0;
                    v_command[1] = 0.0;
                    v_command[2] = 0.0;
                    v_command[3] = 0.0;
                    v_command[4] = 0.0;
                    v_command[5] = 0.0;
                }
                else{
                v_command[0] = translational_scale_factor_x * v_command[0];
                v_command[1] = translational_scale_factor_y * v_command[1];
                v_command[2] = translational_scale_factor_z * v_command[2];
                v_command[3] = rotational_scale_factor * v_command[3];
                v_command[4] = rotational_scale_factor * v_command[4];
                v_command[5] = rotational_scale_factor * v_command[5];

                // v_command[0] = 0;
                // v_command[1] = 0;
                // v_command[2] = 0;
                // v_command[3] = 0;
                // v_command[4] = 0;
                // v_command[5] = 0;
                }

                if (use_pose_target_backend_) {
                    initializePoseTarget(robot_states);
                    if (lambda_running) {
                        integrateVelocityToPoseTarget(v_command, dt.toSec());
                    }
                    v_command = computePoseTargetCommand(robot_states);
                }
                /* FILTER */
                // v_command = 0.1*v_command + 0.9*v_buffer;
                

                //=============================================================================
                // -----------Safe Wall settings-----------------------------------------------
                // virtual wall
                const double z_min_external = kWorkspaceZMin;
                const double z_max_external = kWorkspaceZMax;
                
                // if (robot_states.O_T_EE[12] + v_command[0] * dt.toSec() < x_min_external && v_command[0] < 0) {
                //     v_command[0] = 0;
                // }
                // if (robot_states.O_T_EE[12] + v_command[0] * dt.toSec() > x_max_external && v_command[0] > 0) {
                //     v_command[0] = 0;
                // }
                // if (robot_states.O_T_EE[13] + v_command[1] * dt.toSec() < y_min_external && v_command[1] < 0) {
                //     v_command[1] = 0;
                // }
                // if (robot_states.O_T_EE[13] + v_command[1] * dt.toSec() > y_max_external && v_command[1] > 0) {
                //     v_command[1] = 0;
                // }
                if (robot_states.O_T_EE[14] + v_command[2] * dt.toSec() < z_min_external && v_command[2] < 0) {
                    v_command[2] = 0;
                }
                if (robot_states.O_T_EE[14] + v_command[2] * dt.toSec() > z_max_external && v_command[2] > 0) {
                    v_command[2] = 0;
                }
                //=============================================================================

                
                //==========================================================================================================
                // -----------calculation of joint velocity cmd from end effector velocity and apply low pass filter-------


                // std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_states);
                // convert to Eigen
                // Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
                Eigen::MatrixXd inv_Jacobian = dampedLeastSquaresIK(jacobian, 0.01);

                // print inv_Jacobian
                q_dot_ = inv_Jacobian * v_command;
                double approximate_joint_acc_threshold = 10.;
                double low_pass_filter = 0.6;
                /* Saturate commandes velocities */
                for (auto idx = 0; idx < 7; idx++) {
                    // Filter joint velocities to avoid discontinuities
                    q_dot_[idx] = low_pass_filter * q_dot_old_[idx] + (1 - low_pass_filter) * q_dot_[idx];
                    q_dot_[idx] = std ::min(approximate_joint_acc_threshold * dt.toSec() + q_dot_old_[idx], std ::max(-approximate_joint_acc_threshold * dt.toSec() + q_dot_old_[idx], q_dot_[idx])); 
                    q_dot_old_[idx] = q_dot_[idx];
                }
                //==========================================================================================================


                //=============================================================================
                // -----------publisher: ee velocity cmd after filter---------------------------

                // Publish v_command to /franka/ee_velocity_cmd_Right as geometry_msgs::msg::Twist
                
                geometry_msgs::msg::Twist ee_velocity_cmd_msg;
                
                std::array<double, 7> dq_d_array = robot_states.dq_d;
                Eigen::Map<const Eigen::VectorXd> dq_d_eigen(dq_d_array.data(), dq_d_array.size());
                Eigen::VectorXd ee_velocity_cmd = jacobian * dq_d_eigen;

                ee_velocity_cmd_msg.linear.x = ee_velocity_cmd[0];
                ee_velocity_cmd_msg.linear.y = ee_velocity_cmd[1];
                ee_velocity_cmd_msg.linear.z = ee_velocity_cmd[2];
                ee_velocity_cmd_msg.angular.x = ee_velocity_cmd[3];
                ee_velocity_cmd_msg.angular.y = ee_velocity_cmd[4];
                ee_velocity_cmd_msg.angular.z = ee_velocity_cmd[5];


                ee_velocity_cmd_publisher_->publish(ee_velocity_cmd_msg);

                //=============================================================================


                //=============================================================================
                // -----------publisher: ee velocity from robot state-----------------------------------------

                // Publish v_command to /franka/ee_velocity_Right as geometry_msgs::msg::Twist
                geometry_msgs::msg::Twist ee_vel_msg;
                //get joint velocity from robot_states
                std::array<double, 7> dq_array = robot_states.dq;
                Eigen::Map<const Eigen::VectorXd> dq_eigen(dq_array.data(), dq_array.size());
                Eigen::VectorXd ee_velocity = jacobian * dq_eigen;
                ee_vel_msg.linear.x = ee_velocity[0];
                ee_vel_msg.linear.y = ee_velocity[1];
                ee_vel_msg.linear.z = ee_velocity[2];

                ee_vel_msg.angular.x = ee_velocity[3];
                ee_vel_msg.angular.y = ee_velocity[4];
                ee_vel_msg.angular.z = ee_velocity[5];  
                ee_velocity_publisher_->publish(ee_vel_msg);

                //=============================================================================


                //=============================================================================
                // -----------publisher: joint states-----------------------------------------
                // read the joint states
                joint_states_msgs_.header.stamp = this->now();
                joint_states_msgs_.name.assign(joint_names_.begin(), joint_names_.end());
                joint_states_msgs_.position.assign(robot_states.q.begin(), robot_states.q.end());
                joint_states_msgs_.velocity.assign(robot_states.dq.begin(), robot_states.dq.end());
                joint_states_msgs_.effort.assign(robot_states.tau_J.begin(), robot_states.tau_J.end());
                joint_states_publisher_->publish(joint_states_msgs_);
                //=============================================================================




                //=============================================================================
                // -----------publisher: force feedback for teleoperation--------------------------

                // wrench_msg_.force.x = 0.;
                // wrench_msg_.force.y = 0.;
                // wrench_msg_.force.z = 0.;
                // wrench_msg_.torque.x = 0;
                // wrench_msg_.torque.y = 0;
                // wrench_msg_.torque.z = 0;

                wrench_msg_.force.x = robot_states.O_F_ext_hat_K[0];
                wrench_msg_.force.y = robot_states.O_F_ext_hat_K[1];
                wrench_msg_.force.z = robot_states.O_F_ext_hat_K[2];
                wrench_msg_.torque.x = robot_states.O_F_ext_hat_K[3];
                wrench_msg_.torque.y = robot_states.O_F_ext_hat_K[4];
                wrench_msg_.torque.z = robot_states.O_F_ext_hat_K[5];
                publisher_->publish(wrench_msg_);
                //=============================================================================

                
                
                //=============================================================================
                // -----------publisher: end-effector pose--------------------------------
                // calculate 0_T_F
                // read 0_T_EE and F_T_EE from robot_states
                std::array<double, 16> O_T_EE = robot_states.O_T_EE;
                std::array<double, 16> F_T_EE = robot_states.F_T_EE;

                // Publish the whole 4x4 transformation matrix as Float64MultiArray
                std_msgs::msg::Float64MultiArray O_T_EE_msg;
                O_T_EE_msg.data.assign(O_T_EE.begin(), O_T_EE.end());

                ee_pose_publisher_->publish(O_T_EE_msg);
                //=============================================================================

                //=============================================================================
                // -----------publisher: end-effector pose cmd--------------------------------
                std::array<double, 16> O_T_EE_d = robot_states.O_T_EE_d;
                if (use_pose_target_backend_ && mutex_pose_.try_lock()) {
                    if (pose_target_initialized_) {
                        O_T_EE_d = makeFrankaPoseArray(target_position_, target_orientation_);
                    }
                    mutex_pose_.unlock();
                }
                ee_pose_cmd_msg_.header.stamp = this->now();
                ee_pose_cmd_msg_.header.frame_id = "panda_link0";

                const Eigen::Vector3d command_position = extractFrankaPosition(O_T_EE_d);
                const Eigen::Quaterniond command_orientation = extractFrankaOrientation(O_T_EE_d);

                ee_pose_cmd_msg_.pose.position.x = command_position.x();
                ee_pose_cmd_msg_.pose.position.y = command_position.y();
                ee_pose_cmd_msg_.pose.position.z = command_position.z();
                ee_pose_cmd_msg_.pose.orientation.x = command_orientation.x();
                ee_pose_cmd_msg_.pose.orientation.y = command_orientation.y();
                ee_pose_cmd_msg_.pose.orientation.z = command_orientation.z();
                ee_pose_cmd_msg_.pose.orientation.w = command_orientation.w();
                ee_pose_cmd_publisher_->publish(ee_pose_cmd_msg_);

                //=============================================================================



                //=============================================================================
                // -----------fucking code which I don't understand---------------------------
                // convert to Eigen
                Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> O_T_EE_eigen(O_T_EE.data());
                Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> F_T_EE_eigen(F_T_EE.data());

                // calculate 0_T_F
                Eigen::Matrix<double, 4, 4> O_T_F = O_T_EE_eigen * F_T_EE_eigen.inverse();

                // create 0_F_FTSensor [0, 1, 0, 0; -1, 0, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1]
                Eigen::Matrix<double, 4, 4> O_F_FTSensor;
                O_F_FTSensor << 0, 1, 0, 0,
                              -1, 0, 0, 0,
                              0, 0, 1, 0,
                              0, 0, 0, 1;

                // calculate 0_T_FTSensor
                Eigen::Matrix<double, 4, 4> O_T_FTSensor = O_T_F * O_F_FTSensor;
                // Eigen::Matrix<double, 4, 4> O_T_FTSensor = O_T_F;

                // convert the rotational matrix part of O_T_FTSensor to array
                std::array<double, 9> R_T_FTSensor_array;
                Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R_T_FTSensor_array.data()) = O_T_FTSensor.block<3, 3>(0, 0);

                // publish 0_T_FTSensor
                std_msgs::msg::Float64MultiArray R_T_FTSensor_array_msg;
                for (auto i = 0; i < 9; i++) {
                    R_T_FTSensor_array_msg.data.push_back(R_T_FTSensor_array[i]);
                }
                frame_flange2base_publisher_->publish(R_T_FTSensor_array_msg);
                // Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(O_T_F_array.data()) = O_T_F;

                // // publish 0_T_F
                // std_msgs::msg::Float64MultiArray O_T_F_array_msg;
                // for (auto i = 0; i < 16; i++) {
                //     O_T_F_array_msg.data.push_back(O_T_F_array[i]);
                // }
                // frame_flange2base_publisher_->publish(O_T_F_array_msg);


                // print force x y z
                // std::cout << "force x y z: " << robot_states.O_F_ext_hat_K[0] << " " << robot_states.O_F_ext_hat_K[1] << " " << robot_states.O_F_ext_hat_K[2] << std::endl;
                

                // rec.addToRec(time_);
                // rec.addToRec(v_command);

                // rec.next();

                // print q_dot_
                // std::cout << "q_dot_: " << q_dot_[0] << " " << q_dot_[1] << " " << q_dot_[2] << " " << q_dot_[3] << " " << q_dot_[4] << " " << q_dot_[5] << " " << q_dot_[6] << std::endl;
                //=============================================================================



                //=============================================================================
                //--------------publisher: joint velocity command(already low pass filtered)------------------------------

                // sensor_msgs::msg::JointState joint_vel_cmd_msg_;
                joint_vel_cmd_msg_.header.stamp = this->now();
                joint_vel_cmd_msg_.name.assign(joint_names_.begin(), joint_names_.end());

                joint_vel_cmd_msg_.velocity.clear();
                for (int i = 0; i < 7; i++) {
                    joint_vel_cmd_msg_.velocity.push_back(q_dot_[i]);
                }
                joint_velocity_cmd_publisher_->publish(joint_vel_cmd_msg_);
                //=============================================================================





                velocities = {{ q_dot_[0], q_dot_[1], q_dot_[2], q_dot_[3], q_dot_[4], q_dot_[5], q_dot_[6] }};
                // velocities = {{0, 0, 0, 0, 0, 0, 0}};
                return velocities;
            };

            // Set gains for the joint impedance control.
            // Stiffness
            const std::array<double, 7> k_gains = {{600.0, 600.0, 600.0, 600.0, 50.0, 50.0, 50.0}};
            // const std::array<double, 7> k_gains = {{800.0, 800.0, 800.0, 800.0, 75.0, 75.0, 75.0}};
            // Damping
            const std::array<double, 7> d_gains = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};
            
            // Define callback for the joint torque control loop.
            std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
                impedance_control_callback =
                    [&model, k_gains, d_gains](
                        const franka::RobotState& state, franka::Duration /*period*/) -> franka::Torques {
            // Read current coriolis terms from model.
            std::array<double, 7> coriolis = model.coriolis(state);

            // Compute torque command from joint impedance control law.
            // Note: The answer to our Cartesian pose inverse kinematics is always in state.q_d with one
            // time step delay.
            std::array<double, 7> tau_d_calculated;
            for (size_t i = 0; i < 7; i++) {
                tau_d_calculated[i] =
                    k_gains[i] * (state.q_d[i] - state.q[i]) - d_gains[i] * state.dq[i] + coriolis[i];
            }

            // The following line is only necessary for printing the rate limited torque. As we activated
            // rate limiting for the control loop (activated by default), the torque would anyway be
            // adjusted!
            std::array<double, 7> tau_d_rate_limited =
                franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);

            // Send torque command.
            return tau_d_rate_limited;
            };

            std::cout << "WARNING: Starting teleoperation! "
                      << "Please make sure to have the user stop button at hand !" << std::endl;
            // Start robot control
            while(true) {
                try{
                    // Start real-time control loop.
                    // robot_.control(impedance_control_callback, joint_velocity_callback);
                    robot_.control(joint_velocity_callback);
                } catch (const franka::ControlException& e) {
                    std::cout << "Running error recovery..." << std::endl;
                    robot_.automaticErrorRecovery();
                }
            }

        } catch (const franka::Exception& e) {
            std::cout << e.what() << std::endl;

            control_shutdown();

            std::cout << "Shutting down ROS 2 process" << std::endl;
            rclcpp::shutdown();
        }
    });
}

FrankaNodeInference::~FrankaNodeInference() {
    gripper_threads_running_ = false;

    if (gripper_state_thread_.joinable()) {
        std::cout << "Joining Gripper state thread" << std::endl;
        gripper_state_thread_.join();
    }

    if (control_thread_.joinable()) {
        std::cout << "Joining Control thread" << std::endl;
        control_thread_.join();
    }

    if (gripper_thread_.joinable()) {
        std::cout << "Joining Gripper thread" << std::endl;
        gripper_thread_.join();
    }

    std::cout << "Shutting down robot node" << std::endl;
}


void FrankaNodeInference::initializePoseTarget(const franka::RobotState& robot_states) {
    if (mutex_pose_.try_lock()) {
        if (!pose_target_initialized_) {
            target_position_ = extractFrankaPosition(robot_states.O_T_EE);
            target_orientation_ = extractFrankaOrientation(robot_states.O_T_EE);

            if (has_pending_pose_) {
                target_position_ = pending_absolute_position_;
                target_orientation_ = pending_absolute_orientation_;
                target_orientation_.normalize();
                has_pending_pose_ = false;
            }

            target_position_.x() = clampValue(target_position_.x(), kWorkspaceXMin, kWorkspaceXMax);
            target_position_.y() = clampValue(target_position_.y(), kWorkspaceYMin, kWorkspaceYMax);
            target_position_.z() = clampValue(target_position_.z(), kWorkspaceZMin, kWorkspaceZMax);
            pose_target_initialized_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "Initialized pose target current=(%.4f, %.4f, %.4f) target=(%.4f, %.4f, %.4f)",
                        robot_states.O_T_EE[12], robot_states.O_T_EE[13], robot_states.O_T_EE[14],
                        target_position_.x(), target_position_.y(), target_position_.z());
        }
        mutex_pose_.unlock();
    }
}

void FrankaNodeInference::integrateVelocityToPoseTarget(
    const Eigen::Matrix<double, 6, 1>& velocity_command, double dt_sec) {
    if (dt_sec <= 0.0) {
        return;
    }

    if (mutex_pose_.try_lock()) {
        if (pose_target_initialized_) {
            target_position_ += velocity_command.head<3>() * dt_sec;

            const Eigen::Vector3d delta_rotation = velocity_command.tail<3>() * dt_sec;
            const double delta_angle = delta_rotation.norm();
            if (delta_angle > 1e-9) {
                const Eigen::Quaterniond delta_quaternion(
                    Eigen::AngleAxisd(delta_angle, delta_rotation / delta_angle));
                target_orientation_ = delta_quaternion * target_orientation_;
                target_orientation_.normalize();
            }

            target_position_.x() = clampValue(target_position_.x(), kWorkspaceXMin, kWorkspaceXMax);
            target_position_.y() = clampValue(target_position_.y(), kWorkspaceYMin, kWorkspaceYMax);
            target_position_.z() = clampValue(target_position_.z(), kWorkspaceZMin, kWorkspaceZMax);
        }
        mutex_pose_.unlock();
    }
}

Eigen::Matrix<double, 6, 1> FrankaNodeInference::computePoseTargetCommand(
    const franka::RobotState& robot_states) {
    Eigen::Matrix<double, 6, 1> command;
    command.setZero();

    Eigen::Vector3d target_position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond target_orientation = Eigen::Quaterniond::Identity();
    bool has_pose_target = false;

    if (mutex_pose_.try_lock()) {
        target_position = target_position_;
        target_orientation = target_orientation_;
        has_pose_target = pose_target_initialized_;
        mutex_pose_.unlock();
    }

    if (!has_pose_target) {
        return command;
    }

    const Eigen::Vector3d current_position = extractFrankaPosition(robot_states.O_T_EE);
    const Eigen::Quaterniond current_orientation = extractFrankaOrientation(robot_states.O_T_EE);

    const Eigen::Vector3d position_error = target_position - current_position;

    Eigen::Quaterniond q_error = target_orientation * current_orientation.conjugate();
    if (q_error.w() < 0.0) {
        q_error.coeffs() *= -1.0;
    }
    q_error.normalize();

    Eigen::Vector3d orientation_error = Eigen::Vector3d::Zero();
    const Eigen::AngleAxisd angle_axis(q_error);
    if (std::abs(angle_axis.angle()) > pose_orientation_tolerance_ &&
        angle_axis.axis().norm() > 1e-9) {
        orientation_error = angle_axis.axis() * angle_axis.angle();
    }

    Eigen::Vector3d linear_cmd = pose_position_gain_ * position_error;
    Eigen::Vector3d angular_cmd = pose_orientation_gain_ * orientation_error;

    if (position_error.norm() < pose_position_tolerance_) {
        linear_cmd.setZero();
    }

    linear_cmd = clampNorm(linear_cmd, pose_backend_max_linear_speed_);
    angular_cmd = clampNorm(angular_cmd, pose_backend_max_angular_speed_);

    command << linear_cmd, angular_cmd;
    return command;
}

void FrankaNodeInference::absolutePoseCommandCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!use_pose_target_backend_) {
        return;
    }

    Eigen::Vector3d absolute_position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    Eigen::Quaterniond absolute_orientation(msg->pose.orientation.w, msg->pose.orientation.x,
                                         msg->pose.orientation.y, msg->pose.orientation.z);

    if (mutex_pose_.try_lock()) {
        if (pose_target_initialized_) {
            target_position_ = absolute_position;
            target_position_.x() = clampValue(target_position_.x(), kWorkspaceXMin, kWorkspaceXMax);
            target_position_.y() = clampValue(target_position_.y(), kWorkspaceYMin, kWorkspaceYMax);
            target_position_.z() = clampValue(target_position_.z(), kWorkspaceZMin, kWorkspaceZMax);
            if (absolute_orientation.norm() < 1e-6) {
                absolute_orientation = Eigen::Quaterniond::Identity();
            } else {
                absolute_orientation.normalize();
            }
            target_orientation_ = absolute_orientation;
        } else {
            if (absolute_orientation.norm() < 1e-6) {
                absolute_orientation = Eigen::Quaterniond::Identity();
            } else {
                absolute_orientation.normalize();
            }
            pending_absolute_position_ = absolute_position;
            pending_absolute_orientation_ = absolute_orientation;
            has_pending_pose_ = true;
        }
        mutex_pose_.unlock();
    }
}

Eigen::MatrixXd FrankaNodeInference::dampedLeastSquaresIK(Eigen::MatrixXd a, double dampingFactor) // dampedLeastSquaresIK(J,0.01)
{
    Eigen::MatrixXd InverseJacobian(a.cols(), a.rows());
    if (a.rows() > a.cols())
    {
        InverseJacobian =
            a.transpose() * (a * a.transpose() + dampingFactor * dampingFactor * Eigen::MatrixXd::Identity(a.rows(), a.rows()))
                                .inverse();
    }
    else if (a.rows() < a.cols())
    {
        InverseJacobian = (a.transpose() * a + dampingFactor * dampingFactor * Eigen::MatrixXd::Identity(a.cols(), a.cols()))
                              .inverse() *
                          a.transpose();
    }
    return InverseJacobian;
}

void FrankaNodeInference::lambdaCommandCallback(const custom_msgs::msg::LambdaCommand::SharedPtr msg) {
    lambda_command_ = *msg;

    {
        std::lock_guard<std::mutex> vel_lock(mutex_vel_);
        v_x_ = clampValue(-lambda_command_.linear.y, -2.0, 2.0);
        v_y_ = clampValue(lambda_command_.linear.x, -2.0, 2.0);
        v_z_ = clampValue(lambda_command_.linear.z, -2.0, 2.0);
        w_x_ = clampValue(lambda_command_.angular.x, -2.0, 2.0);
        w_y_ = clampValue(lambda_command_.angular.y, -2.0, 2.0);
        w_z_ = clampValue(lambda_command_.angular.z, -2.0, 2.0);
        lambda_last_update_time_ = this->now().seconds();
    }

    {
        std::lock_guard<std::mutex> grip_lock(mutex_grip_);
        v_gripper_ = lambda_command_.v_gripper;
        close_gripper_ = lambda_command_.enable_backlash_compensation;
    }
}

void FrankaNodeInference::gripperCommandCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    std::lock_guard<std::mutex> grip_lock(mutex_grip_);
    v_gripper_ = msg->data;
    close_gripper_ = msg->data > 0.0;
}

#include "franka_control/franka_node.h"


FrankaNode::FrankaNode(std::string node_name, std::string robot_IP) : Node(node_name), robot_(robot_IP), gripper_(robot_IP) {
    
    // Multithread publisher and subscriber
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    sub_options_.callback_group = callback_group_;
    pub_options_.callback_group = callback_group_;

    // QoS profile
    rclcpp::QoS qos_profile(10);

    // Create subscriber
    subscription_ = create_subscription<custom_msgs::msg::LambdaCommand>("/TwistRight", qos_profile, std::bind(&FrankaNode::lambdaCommandCallback, this, _1), sub_options_);
    
    joint_states_publisher_ =
      this->create_publisher<sensor_msgs::msg::JointState>("/frankaRight/joint_states", 1);

    ee_pose_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/frankaRight/ee_pose", 1);
    
      
    publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/WrenchRight", 1); // TTODO: read left/right from launch file
    
    frame_flange2base_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/O_T_FTSensorRight", 1);

    joint_velocity_cmd_publisher_ = 
        this->create_publisher<sensor_msgs::msg::JointState>("/frankaRight/joint_velocity_cmd", 1);

    ee_velocity_cmd_publisher_ = 
        this->create_publisher<geometry_msgs::msg::Twist>("/frankaRight/ee_velocity_cmd", 1);
    

    ee_pose_cmd_publisher_ = 
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/frankaRight/ee_pose_cmd", 1);

    

    
    // create shutdown handler
    control_shutdown = [&]() {
        robot_.stop();
        gripper_.stop();

        output_wrench_.force.x = 0.0;
        output_wrench_.force.y = 0.0;
        output_wrench_.force.z = 0.0;
        publisher_->publish(output_wrench_);

        std::cout << "Stopped Franka Control" << std::endl;
    };

    rclcpp::on_shutdown(control_shutdown);

    ///////////////////////////// CONTROL THREAD ////////////////////////////////
    gripper_thread_ = std::thread([this]() {

        try{
            std::cout << "Initializing gripper..." << std::endl;
            gripper_.stop();
            gripper_.homing();

            franka::GripperState gripper_state = gripper_.readOnce();
            double gripper_max_width = gripper_state.max_width;
            double grasping_width = 0.0;
            // double grasping_width = 0.06;
            double grasping_force = 100.0; // TODO: SET REASONABLE VALUE
            // double grasping_force = 20.0; // TODO: SET REASONABLE VALUE

            bool close_gripper_flag = false;
            bool grasped_flag = false;



            //for bottle
            gripper_.grasp(grasping_width, 0.5, grasping_force, 0.1, 0.1);

            while (true){
            while (lambda_running) {
                // std::cout << "lambda_running: " << lambda_running << std::endl;
                if (mutex_grip_.try_lock()) {
                    close_gripper_flag =  close_gripper_;
                    mutex_grip_.unlock();
                }

                if (close_gripper_flag) {
                    // std::cout << "Grasping object...\n" << std::endl;
                    if (!grasped_flag) {
                        grasped_flag = gripper_.grasp(grasping_width, 0.5, grasping_force, 0.1, 0.1);
                        std::cout << "Gripper sucess: " << grasped_flag << std::endl;
                    }  
                } else {
                    // std::cout << "Releasing ...\n" << std::endl;
                    gripper_.stop();
                    gripper_.move(gripper_max_width, 0.1);
                    grasped_flag = false;
                }

                this_thread::sleep_for(std::chrono::milliseconds(200));
            }
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
            
            //=========xuanming init pose========================================================
            // std::array<double, 7> q_goal = {{-1.23556,0.95542,1.40872,-2.45478,0.0725108,2.71996,2.31233}}; // xuanming init pose 20250909
            // std::array<double, 7> q_goal = {{-0.400869,0.161234,0.761493,-2.41041,1.04765,2.49407,1.81783}};  
            std::array<double, 7> q_goal = {{-0.063376,-0.0975732,0.0659204,-2.10033,0.126964,1.98867,-0.788463}};
            //===================================================================================


            //=========Tailai init pose==========================================================
            // std::array<double, 7> q_goal = {{-0.57476,0.0360376,0.424265,-2.6671,1.44048,2.16108,1.18508}};  //Tailai init pose
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

                //std::cout << robot_states << std::endl;
                //calculate desired EE Pose outside the controller
                std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_states);
                // convert to Eigen
                Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

                // ----------  xuanming: test control part  ---------------------------
                // --------------------------------------------------------------------

                std::cout << "time_: " << time_ << std::endl;

                // calculate filtered cartician velocity from dq_d
                Eigen::VectorXd dq_desire(7);
                for (size_t i = 0; i < 7; ++i) {
                    dq_desire(i) = robot_states.dq_d[i];
                }
                Eigen::VectorXd v_desire(6);
                v_desire = jacobian * dq_desire;
                
                //calculate integration for desired EE pose
                double dt_sec = dt.toSec();
                ee_pose_desired_.position.x = robot_states.O_T_EE[12] + v_desire(0) * dt_sec;
                ee_pose_desired_.position.y = robot_states.O_T_EE[13] + v_desire(1) * dt_sec;
                ee_pose_desired_.position.z = robot_states.O_T_EE[14] + v_desire(2) * dt_sec;
                //update desired orientation (keep constant for now)
                ee_pose_desired_.orientation.x = robot_states.O_T_EE_d[4 * 3 + 0];
                ee_pose_desired_.orientation.y = robot_states.O_T_EE_d[4 * 3 + 1];
                ee_pose_desired_.orientation.z = robot_states.O_T_EE_d[4 * 3 + 2];
                ee_pose_desired_.orientation.w = robot_states.O_T_EE_d[4 * 3 + 3];  
                //print desired EE pose
                std::cout << "Desired EE position: " << ee_pose_desired_.position.x << " " << ee_pose_desired_.position.y << " " << ee_pose_desired_.position.z << std::endl;
                
                
                std::cout << "O_T_EE_d: ";
                for (size_t i = 0; i < robot_states.O_T_EE_d.size(); ++i) {
                    std::cout << robot_states.O_T_EE_d[i];
                    if (i + 1 < robot_states.O_T_EE_d.size()) std::cout << " ";
                }
                std::cout << std::endl;

                std::cout << "dq_d:  " ;
                for (size_t i = 0; i < robot_states.dq_d.size(); ++i) {
                    std::cout << robot_states.dq_d[i];
                    if (i + 1 < robot_states.dq_d.size()) std::cout << " ";
                }
                std::cout << std::endl;

                std::cout << "---------------------------------------------------------" << std::endl;


                //------------  xuanming: test control part -- END --------------------------------------------
                //---------------------------------------------------------------------------------------------
                
                
                
                
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
                double translational_scale_factor = 0.8;

                // Config: Lock the rotation
                // double rotational_scale_factor = 0.4;
                double rotational_scale_factor = 0.25;
                // double rotational_scale_factor = 0;

                if (mutex_vel_.try_lock()) {
                        v_command << v_x_, v_y_, v_z_, w_x_, w_y_, w_z_;
                    mutex_vel_.unlock();
                }

                // calculate the difference of norm of v_command and v_buffer
                double diff_norm = (v_command - v_buffer).norm();
                v_buffer = v_command;
                    if (this->now().seconds() - lambda_last_update_time_ > 0.01) {
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
                v_command[0] = translational_scale_factor * v_command[0];
                v_command[1] = translational_scale_factor * v_command[1];
                v_command[2] = translational_scale_factor * v_command[2];
                // v_command[3] = rotational_scale_factor * v_command[3];
                v_command[4] = rotational_scale_factor * v_command[4];
                v_command[5] = rotational_scale_factor * v_command[5];

                // v_command[0] = 0;
                // v_command[1] = 0;
                // v_command[2] = 0;
                v_command[3] = 0;
                // v_command[4] = 0;
                // v_command[5] = 0;
                }
                /* FILTER */
                // v_command = 0.1*v_command + 0.9*v_buffer;
                

                //=============================================================================
                // -----------Safe Wall settings-----------------------------------------------
                // virtual wall
                double x_min_external = 0.20;
                double x_max_external = 0.80;
                double y_min_external = -0.45 - 0.281;
                double y_max_external = 0.85 - 0.281;
                double z_min_external = 0.11;
                double z_max_external = 0.55;
                
                if (robot_states.O_T_EE[12] + v_command[0] * dt.toSec() < x_min_external && v_command[0] < 0) {
                    v_command[0] = 0;
                }
                if (robot_states.O_T_EE[12] + v_command[0] * dt.toSec() > x_max_external && v_command[0] > 0) {
                    v_command[0] = 0;
                }
                if (robot_states.O_T_EE[13] + v_command[1] * dt.toSec() < y_min_external && v_command[1] < 0) {
                    v_command[1] = 0;
                }
                if (robot_states.O_T_EE[13] + v_command[1] * dt.toSec() > y_max_external && v_command[1] > 0) {
                    v_command[1] = 0;
                }
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
                // -----------publisher: ee velocity without low pass filter-----------------------------------------

                // Publish v_command to /franka/ee_velocity_cmd_Right as geometry_msgs::msg::Twist
                geometry_msgs::msg::Twist v_command_msg;
                v_command_msg.linear.x = v_command[0];
                v_command_msg.linear.y = v_command[1];
                v_command_msg.linear.z = v_command[2];
                v_command_msg.angular.x = v_command[3];
                v_command_msg.angular.y = v_command[4];
                v_command_msg.angular.z = v_command[5];
                ee_velocity_cmd_publisher_->publish(v_command_msg);

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

                // rotation matrix to quaternion
                Eigen::Map<const Eigen::Matrix4d> O_T_EE_cart(O_T_EE.data());
                
                ee_pose_msg_.header.stamp = this->now();
                ee_pose_msg_.header.frame_id = "panda_link0"; 

                //translation
                ee_pose_msg_.pose.position.x = O_T_EE_cart(0, 3);
                ee_pose_msg_.pose.position.y = O_T_EE_cart(1, 3);
                ee_pose_msg_.pose.position.z = O_T_EE_cart(2, 3);

                //rotation
                Eigen::Quaterniond q(O_T_EE_cart.block<3, 3>(0, 0));
                q.normalize();
                ee_pose_msg_.pose.orientation.x = q.x();
                ee_pose_msg_.pose.orientation.y = q.y();
                ee_pose_msg_.pose.orientation.z = q.z();
                ee_pose_msg_.pose.orientation.w = q.w();

                ee_pose_publisher_->publish(ee_pose_msg_);
                //=============================================================================

                //=============================================================================
                // -----------publisher: end-effector pose cmd--------------------------------
                std::array<double, 16> O_T_EE_d = robot_states.O_T_EE_d;
                

                // rotation matrix to quaternion
                Eigen::Map<const Eigen::Matrix4d> O_T_EE_d_cart(O_T_EE_d.data());
                ee_pose_cmd_msg_.header.stamp = this->now();
                ee_pose_cmd_msg_.header.frame_id = "panda_link0";

                //translation
                ee_pose_cmd_msg_.pose.position.x = O_T_EE_d_cart(0, 3);
                ee_pose_cmd_msg_.pose.position.y = O_T_EE_d_cart(1, 3);
                ee_pose_cmd_msg_.pose.position.z = O_T_EE_d_cart(2, 3);

                //rotation
                Eigen::Quaterniond q_cmd(O_T_EE_d_cart.block<3, 3>(0, 0));
                q_cmd.normalize();
                ee_pose_cmd_msg_.pose.orientation.x = q_cmd.x();
                ee_pose_cmd_msg_.pose.orientation.y = q_cmd.y();
                ee_pose_cmd_msg_.pose.orientation.z = q_cmd.z();
                ee_pose_cmd_msg_.pose.orientation.w = q_cmd.w();

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

FrankaNode::~FrankaNode() {
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

Eigen::MatrixXd FrankaNode::dampedLeastSquaresIK(Eigen::MatrixXd a, double dampingFactor) // dampedLeastSquaresIK(J,0.01)
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

void FrankaNode::lambdaCommandCallback(const custom_msgs::msg::LambdaCommand::SharedPtr msg) { 
        lambda_command_ = *msg;
    if (mutex_vel_.try_lock()) {
        // print original lambda_command_
        // std::cout << "lambda_command_: " << lambda_command_.linear.x << " " << lambda_command_.linear.y << " " << lambda_command_.linear.z << " " << lambda_command_.angular.x << " " << lambda_command_.angular.y << " " << lambda_command_.angular.z << std::endl;
        v_x_ = std ::min(2.0, std ::max(-2.0, lambda_command_.linear.x));  
        v_y_ = std ::min(2.0, std ::max(-2.0, lambda_command_.linear.y));  
        v_z_ = std ::min(2.0, std ::max(-2.0, lambda_command_.linear.z));
        w_x_ = std ::min(2.0, std ::max(-2.0, lambda_command_.angular.x));
        w_y_ = std ::min(2.0, std ::max(-2.0, lambda_command_.angular.y));
        w_z_ = std ::min(2.0, std ::max(-2.0, lambda_command_.angular.z));
        if (mutex_grip_.try_lock()) {
            v_gripper_ = lambda_command_.v_gripper;
            close_gripper_ = lambda_command_.enable_backlash_compensation;
            mutex_grip_.unlock();
        }
        lambda_last_update_time_ = this->now().seconds();

        mutex_vel_.unlock();
    }
    }
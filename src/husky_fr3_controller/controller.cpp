#include "husky_fr3_controller/controller.h"

namespace HuskyFR3Controller
{
    Controller::Controller(const rclcpp::Node::SharedPtr& node, double dt, JointDict jd)
    : ControllerInterface(node, dt, std::move(jd))
    {
        std::string urdf_path = ament_index_cpp::get_package_share_directory("dyros_robot_controller")
                                + "/robot/husky_fr3.urdf";
        robot_ = std::make_shared<RobotData>(urdf_path, true);

        key_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
                    "husky_fr3_controller/mode_input", 10,
                    std::bind(&Controller::keyCallback, this, std::placeholders::_1));
        target_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                            "husky_fr3_controller/target_pose", 10,
                            std::bind(&Controller::subtargetPoseCallback, this, std::placeholders::_1));
        base_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
                            "husky_fr3_controller/cmd_vel", 10,
                            std::bind(&Controller::subtargetBaseVelCallback, this, std::placeholders::_1));
        ee_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
                        "husky_fr3_controller/ee_pose", 10);
        
        base_vel_.setZero();
        base_vel_desired_.setZero();

        q_virtual_.setZero();
        q_virtual_init_.setZero();
        q_virtual_desired_.setZero();
        qdot_virtual_.setZero();
        qdot_virtual_init_.setZero();
        qdot_virtual_desired_.setZero();
        
        q_mani_.setZero();
        q_mani_init_.setZero();
        q_mani_desired_.setZero();
        qdot_mani_.setZero();
        qdot_mani_init_.setZero();
        qdot_mani_desired_.setZero();
        
        q_mobile_.setZero();
        q_mobile_init_.setZero();
        q_mobile_desired_.setZero();
        qdot_mobile_.setZero();
        qdot_mobile_init_.setZero();
        qdot_mobile_desired_.setZero();
        
        x_.setIdentity();
        x_init_.setIdentity();
        x_desired_.setIdentity();
        xdot_.setZero();
        xdot_init_.setZero();
        xdot_desired_.setZero();
        
        torque_mani_desired_.setZero();
        qdot_mobile_desired_.setZero();

        wholebody_qp_ = std::make_unique<QP::MOMAWholebody>();
    }

    Controller::~Controller()
    {

    }

    void Controller::starting()
    {
        is_mode_changed_ = false;
        mode_ = "home";
        control_start_time_ = current_time_;

        base_vel_desired_= base_vel_;

        q_virtual_init_ = q_virtual_;
        qdot_virtual_init_ = qdot_virtual_;
        q_virtual_desired_ = q_virtual_init_;
        qdot_virtual_desired_.setZero();

        q_mani_init_ = q_mani_;
        qdot_mani_init_ = qdot_mani_;
        q_mani_desired_ = q_mani_init_;
        qdot_mani_desired_.setZero();
        
        q_mobile_init_ = q_mobile_;
        qdot_mobile_init_ = qdot_mobile_;
        q_mobile_desired_ = q_mobile_init_;
        qdot_mobile_desired_.setZero();

        x_init_ = x_;
        x_desired_ = x_init_;
        x_goal_ = x_init_;

        xdot_init_ = xdot_;
        xdot_desired_.setZero();

        ee_pose_pub_timer_ = node_->create_wall_timer(
                           std::chrono::milliseconds(100),
                           std::bind(&Controller::pubEEPoseCallback, this)
                           );
    }

    void Controller::updateState(const VecMap& pos_dict, 
                                 const VecMap& vel_dict,
                                 const VecMap& tau_ext_dict, 
                                 const VecMap& sensors_dict, 
                                 double current_time)
    {
        current_time_ = current_time;

        // get virtual joint
        q_virtual_.head(2) = sensors_dict.at("position_sensor").head(2);
        Quaterniond quat(sensors_dict.at("orientation_sensor")(0),
                         sensors_dict.at("orientation_sensor")(1),
                         sensors_dict.at("orientation_sensor")(2),
                         sensors_dict.at("orientation_sensor")(3));
        Vector3d euler_rpy = DyrosMath::rot2Euler(quat.toRotationMatrix());
        q_virtual_(2) = euler_rpy(2);

        qdot_virtual_.head(2) = sensors_dict.at("linear_velocity_sensor").head(2);
        qdot_virtual_(2) = sensors_dict.at("angular_velocity_sensor")(2);
        
        // get mobile base velocity
        Matrix2d rot_base2world;
        rot_base2world << cos(q_virtual_(2)), sin(q_virtual_(2)),
                         -sin(q_virtual_(2)), cos(q_virtual_(2));
        base_vel_.head(2) = rot_base2world * qdot_virtual_.head(2);
        base_vel_(2) = qdot_virtual_(2);

        // get manipulator joint
        for(size_t i=0; i<7; i++)
        {
            const std::string& name = "fr3_joint" + std::to_string(i+1);
            q_mani_(i) = pos_dict.at(name)(0);
            qdot_mani_(i) = vel_dict.at(name)(0);
        }

        // get mobile wheel joint
        q_mobile_(0) = pos_dict.at("front_left_wheel")(0);
        q_mobile_(1) = pos_dict.at("front_right_wheel")(0);
        qdot_mobile_(0) = vel_dict.at("front_left_wheel")(0);
        qdot_mobile_(1) = vel_dict.at("front_right_wheel")(0);

        JointVec q;
        JointVec qdot;
        q.segment<VIRTUAL_DOF>(joint_idx.virtual_start) = q_virtual_;
        q.segment<MANI_DOF>(joint_idx.mani_start) = q_mani_;
        q.segment<MOBI_DOF>(joint_idx.mobi_start) = q_mobile_;
        qdot.segment<VIRTUAL_DOF>(joint_idx.virtual_start) = qdot_virtual_;
        qdot.segment<MANI_DOF>(joint_idx.mani_start) = qdot_mani_;
        qdot.segment<MOBI_DOF>(joint_idx.mobi_start) = qdot_mobile_;

        if(!robot_->updateState(q, qdot))
        {
            RCLCPP_ERROR(node_->get_logger(), "[HuskyFR3RobotData] Failed to update robot state.");
        }

        // get ee
        x_ = robot_->getPose();
        xdot_ = robot_->getVelocity();
    }
    
    void Controller::compute()
    {
        if(is_mode_changed_)
        {
            is_mode_changed_ = false;
            control_start_time_ = current_time_;

            base_vel_desired_= base_vel_;

            q_virtual_init_ = q_virtual_;
            qdot_virtual_init_ = qdot_virtual_;
            q_virtual_desired_ = q_virtual_init_;
            qdot_virtual_desired_.setZero();

            q_mani_init_ = q_mani_;
            qdot_mani_init_ = qdot_mani_;
            q_mani_desired_ = q_mani_init_;
            qdot_mani_desired_.setZero();
            
            q_mobile_init_ = q_mobile_;
            qdot_mobile_init_ = qdot_mobile_;
            q_mobile_desired_ = q_mobile_init_;
            qdot_mobile_desired_.setZero();

            x_init_ = x_;
            x_desired_ = x_init_;
            x_goal_ = x_init_;

            xdot_init_ = xdot_;
            xdot_desired_.setZero();
        }
        
        if(mode_ == "home")
        {
            ManiVec q_mani_target;
            q_mani_target << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
            
            
            q_mani_desired_ = DyrosMath::cubicVector<MANI_DOF>(current_time_,
                                                               control_start_time_,
                                                               control_start_time_ + 2.0,
                                                               q_mani_init_,
                                                               q_mani_target,
                                                               qdot_mani_init_,
                                                               ManiVec::Zero());
            qdot_mani_desired_ = DyrosMath::cubicDotVector<MANI_DOF>(current_time_,
                                                                     control_start_time_,
                                                                     control_start_time_ + 2.0,
                                                                     q_mani_init_,
                                                                     q_mani_target,
                                                                     qdot_mani_init_,
                                                                     ManiVec::Zero());
                                                              
            torque_mani_desired_ = ManiPDControl(q_mani_desired_, qdot_mani_desired_);
            qdot_mobile_desired_.setZero();
        }
        else if(mode_ == "wholebody_grav_comp")
        {
            ActuatorVec torque_desired = robot_->getGravityActuated();
            torque_mani_desired_ = torque_desired.segment<MANI_DOF>(actuator_idx.mani_start); 
            qdot_mobile_desired_ = MobileAdmControl(torque_desired.segment<MOBI_DOF>(actuator_idx.mobi_start));
        }
        else if(mode_ == "wholebody_impedence")
        {
            if(is_goal_pose_changed_)
            {
                control_start_time_ = current_time_;
                x_init_ = x_;
                xdot_init_ = xdot_;
                is_goal_pose_changed_ = false;
            }
            x_desired_.translation() = DyrosMath::cubicVector<3>(current_time_,
                                                                 control_start_time_,
                                                                 control_start_time_ + 5.0,
                                                                 x_init_.translation(),
                                                                 x_goal_.translation(),
                                                                 xdot_init_.head(3),
                                                                 Vector3d::Zero());
            xdot_desired_.head(3) = DyrosMath::cubicDotVector<3>(current_time_,
                                                                 control_start_time_,
                                                                 control_start_time_ + 5.0,
                                                                 x_init_.translation(),
                                                                 x_goal_.translation(),
                                                                 xdot_init_.head(3),
                                                                 Vector3d::Zero());
            x_desired_.linear() = DyrosMath::rotationCubic(current_time_,
                                                           control_start_time_,
                                                           control_start_time_ + 5.0,
                                                           x_init_.rotation(),
                                                           x_goal_.rotation());
            xdot_desired_.tail(3) = DyrosMath::rotationCubicDot(current_time_,
                                                                control_start_time_,
                                                                control_start_time_ + 5.0,
                                                                xdot_init_.tail(3),
                                                                Vector3d::Zero(),
                                                                x_init_.rotation(),
                                                                x_goal_.rotation());
            TaskVec x_error, xdot_error;
            x_error.head(3) = x_desired_.translation() - x_.translation();
            x_error.tail(3) = DyrosMath::getPhi(x_desired_.rotation(), x_.rotation());
            xdot_error = xdot_desired_ - xdot_;

            TaskVec Kp, Kv;
            Kp << 400,400,400,400,400,400;
            Kv << 40,40,40,40,40,40;

            TaskVec xddot_desired = Kv.asDiagonal() * xdot_error + Kp.asDiagonal() * x_error;

            // ============================ Version 1 ======================================
            // Using HQP formulation (not exact HQP)
        
            auto J_tilda = robot_->getJacobianActuated();
            auto J_tilda_dot = robot_->getJacobianActuatedTimeVariation();
            auto eta = robot_->getJointVelocityActuated();
            auto M_tilda = robot_->getMassMatrixActuated();
            auto g_tilda = robot_->getGravityActuated();

            ActuatorVec eta_dot = DyrosMath::PinvCOD(J_tilda.transpose() * J_tilda) * J_tilda.transpose() * (xddot_desired - J_tilda_dot * eta);
            ActuatorVec torque_desired = M_tilda * eta_dot + g_tilda;

            torque_mani_desired_ = torque_desired.segment<MANI_DOF>(actuator_idx.mani_start);
            qdot_mobile_desired_ = MobileAdmControl(torque_desired.segment<MOBI_DOF>(actuator_idx.mobi_start));

            // ============================ Version 2 ======================================
            // Using Operation Space Control
            
            // auto J_tilda = robot_->getJacobianActuated();
            // auto M_tilda_inv = robot_->getMassMatrixActuatedInv();
            // auto g_tilda = robot_->getGravityActuated();
            
            // TaskMat Gamma_tilda = DyrosMath::PinvCOD(J_tilda * M_tilda_inv * J_tilda.transpose());
            // // Vector6d mu_tilda = Gamma_tilda * J_tilda * M_tilda_inv * g_tilda;
            // // J_tilda_pinv = Gamma_tilda * J_tilda * M_tilda_inv;

            // TaskVec F = Gamma_tilda * xddot_desired; // + mu_tilda;

            // ActuatorVec torque_desired = J_tilda.transpose() * F + g_tilda; // + (identity - J_tilda.transpose() * J_tilda_pinv) * torque_null;

            // torque_mani_desired_ = torque_desired.segment<MANI_DOF>(actuator_idx.mani_start);
            // qdot_mobile_desired_ = MobileAdmControl(torque_desired.segment<MOBI_DOF>(actuator_idx.mobi_start));

            // ============================ Version 3 ======================================

            // wholebody_qp_->setDesiredEEAcc(xddot_desired);
            // wholebody_qp_->setCurrentState(robot_);

            // ActuatorVec torque_desired;
            // ActuatorVec qddot_desired;
            // QP::TimeDuration tmp;
            // if(!wholebody_qp_->getOptJoint(qddot_desired, torque_desired, tmp))
            // {
            //     RCLCPP_INFO(node_->get_logger(), "\033[31m QP did not solve properly! \033[0m");
            //     torque_mani_desired_ = (robot_->getGravityActuated()).head(7);
            //     qdot_mobile_desired_.setZero();
            // }
            // else
            // {
            //     // std::cout << "\n\nSet cost: "        << tmp.set_cost*1000 << std::endl;
            //     // std::cout << "Set bound: "       << tmp.set_bound*1000 << std::endl;
            //     // std::cout << "Set ineq: "        << tmp.set_ineq*1000 << std::endl;
            //     // std::cout << "Set eq: "          << tmp.set_eq*1000 << std::endl;
            //     // std::cout << "Set constraint: "  << tmp.set_constraint*1000 << std::endl;
            //     // std::cout << "Set solver: "      << tmp.set_solver*1000 << std::endl;
            //     // std::cout << "Solve qp: "        << tmp.solve_qp*1000 << std::endl;
            //     torque_mani_desired_ = torque_desired.segment<MANI_DOF>(actuator_idx.mani_start);
            //     qdot_mobile_desired_ = MobileAdmControl(torque_desired.segment<MOBI_DOF>(actuator_idx.mobi_start));
            // }


        }
        else if(mode_ == "base_vel_tracking")
        {
            torque_mani_desired_ = ManiPDControl(q_mani_desired_, qdot_mani_desired_);
            qdot_mobile_desired_ = MobileIK(base_vel_desired_);
        }
        else
        {
            torque_mani_desired_ = (robot_->getGravityActuated()).segment<MANI_DOF>(actuator_idx.mani_start);
            qdot_mobile_desired_.setZero();
        }
    }

    CtrlInputMap Controller::getCtrlInput() const
    {
        CtrlInputMap ctrl_dict;
        ctrl_dict["left_wheel"] = qdot_mobile_desired_(0);
        ctrl_dict["right_wheel"] = qdot_mobile_desired_(1);
        for(size_t i=0; i<7; i++)
        {
            const std::string name = "fr3_joint" + std::to_string(i+1);
            ctrl_dict[name] = torque_mani_desired_(i);
        }

        return ctrl_dict;
    }

    void Controller::setMode(const std::string& mode)
    {
      is_mode_changed_ = true;
      mode_ = mode;
      RCLCPP_INFO(node_->get_logger(), "\033[34m Mode changed: %s\033[0m", mode.c_str());
    }

    void Controller::keyCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
      RCLCPP_INFO(node_->get_logger(), "Key input received: %d", msg->data);
      if(msg->data == 1)      setMode("home");
      else if(msg->data == 2) setMode("wholebody_grav_comp");
      else if(msg->data == 3) setMode("wholebody_impedence");
      else if(msg->data == 4) setMode("base_vel_tracking");
      else                    setMode("none");
  
    }

    void Controller::subtargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      RCLCPP_INFO(node_->get_logger(),
              " Target pose received: position=(%.3f, %.3f, %.3f), "
              "orientation=(%.3f, %.3f, %.3f, %.3f)",
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
              msg->pose.orientation.x, msg->pose.orientation.y,
              msg->pose.orientation.z, msg->pose.orientation.w);
      is_goal_pose_changed_ = true;

      // Convert to 4x4 homogeneous transform
      Eigen::Quaterniond quat(msg->pose.orientation.w,
                              msg->pose.orientation.x,
                              msg->pose.orientation.y,
                              msg->pose.orientation.z);

      x_goal_.linear() = quat.toRotationMatrix();
      x_goal_.translation() << msg->pose.position.x,
                               msg->pose.position.y,
                               msg->pose.position.z;
    }

    void Controller::subtargetBaseVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(),
              " Target Base vel received: linear = %.3f, %.3f, angular = %.3f",
              msg->linear.x, msg->linear.y, msg->angular.z);

        base_vel_desired_(0) = msg->linear.x;
        base_vel_desired_(1) = msg->linear.y;
        base_vel_desired_(2) = msg->angular.z;
    }

    void Controller::pubEEPoseCallback()
    {
      auto ee_pose_msg = geometry_msgs::msg::PoseStamped();
      ee_pose_msg.header.frame_id = "world";
      ee_pose_msg.header.stamp = node_->now();

      ee_pose_msg.pose.position.x = x_.translation()(0);
      ee_pose_msg.pose.position.y = x_.translation()(1);
      ee_pose_msg.pose.position.z = x_.translation()(2);

      Eigen::Quaterniond q(x_.rotation());
      ee_pose_msg.pose.orientation.x = q.x();
      ee_pose_msg.pose.orientation.y = q.y();
      ee_pose_msg.pose.orientation.z = q.z();
      ee_pose_msg.pose.orientation.w = q.w();
      
      ee_pose_pub_->publish(ee_pose_msg);
    }

    ManiVec Controller::ManiPDControl(const ManiVec& q_mani_desired, const ManiVec& qdot_mani_desired)
    {
        ManiVec Kp, Kv;
        Kp.setConstant(400);
        Kv.setConstant(40);
        ManiVec f = Kp.asDiagonal() * (q_mani_desired - q_mani_) + Kv.asDiagonal() * (qdot_mani_desired - qdot_mani_);
        return (robot_->getMassMatrixActuated()).block<MANI_DOF,MANI_DOF>(actuator_idx.mani_start,actuator_idx.mani_start) * f + (robot_->getGravityActuated()).segment<MANI_DOF>(actuator_idx.mani_start);
    }

    MobiVec Controller::MobileAdmControl(const MobiVec& torque_mobile_desired)
    {
        double J = 0.02;   // wheel inertia [kg·m²]
        double B = 0.1;    // wheel damping [N·m·s/rad]

        MobiVec alpha = (torque_mobile_desired - B * qdot_mobile_) / J;
        MobiVec omega = qdot_mobile_ + alpha * dt_;

        return omega;
    }

    MobiVec Controller::MobileIK(const Vector3d& desired_base_vel)
    {
        const double L = 0.2854*2*1.875; // Distance between wheels
        const double R = 0.1651;         // Radius of wheels

        Vector2d base_vel_lim, base_acc_lim; // 
        base_vel_lim << 1.0, 3.0; // [linear (m/s), angular (rad/s)]
        base_acc_lim << 2.0, 6.0; // [linear (m/s^2), angular (rad/s^2)]

        Vector2d desired_v = desired_base_vel.head(2); // [vel_x, vel_y]
        double desired_v_norm = desired_v.norm();
        Vector2d desired_v_dir;
        if(fabs(desired_v_norm) < 1E-4)
        {
            desired_v_dir.setZero();
        }
        else
        {
            desired_v_dir = desired_v / desired_v_norm;
        }

        desired_v_norm = std::min(std::max(desired_v_norm, -base_vel_lim(0)), base_vel_lim(0));
        double desired_w = std::min(std::max(desired_base_vel(2), -base_vel_lim(1)), base_vel_lim(1));

        Vector3d desired_vel;
        desired_vel.head(2) = desired_v_dir * desired_v_norm;
        desired_vel(2) = desired_w;
        
        MobiVec desired_wheel_vel;
        desired_wheel_vel(0) = (desired_vel(0) - L/2 * desired_vel(2)) / R; // left
        desired_wheel_vel(1) = (desired_vel(0) + L/2 * desired_vel(2)) / R; // right

        return desired_wheel_vel;
    }

    /* register with the global registry */
    REGISTER_MJ_CONTROLLER(Controller, "HuskyFR3Controller")
}
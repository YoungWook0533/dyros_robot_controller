#include "fr3_controller/controller.h"
#include <Eigen/Dense>

namespace FR3Controller
{
    Controller::Controller(const rclcpp::Node::SharedPtr& node, double dt, JointDict jd)
    : ControllerInterface(node, dt, std::move(jd))
    {
      std::string urdf_path = ament_index_cpp::get_package_share_directory("dyros_robot_controller")
                              + "/robot/fr3.urdf";
      robot_ = std::make_unique<RobotData>(urdf_path, true);
      rd_joint_names_ = robot_->getJointNames();

      key_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
                 "fr3_controller/mode_input", 10,
                 std::bind(&Controller::keyCallback, this, std::placeholders::_1));
      target_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                         "fr3_controller/target_pose", 10,
                         std::bind(&Controller::subtargetPoseCallback, this, std::placeholders::_1));
      ee_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
                     "fr3_controller/ee_pose", 10);
    }

    Controller::~Controller()
    {
      if (slow_thread_.joinable()) slow_thread_.join();
    }
    
    void Controller::starting()
    {
      is_mode_changed_ = false;
      mode_ = "home";
      control_start_time_ = current_time_;

      q_init_ = q_;
      qdot_init_ = qdot_;
      x_init_ = x_;
      xdot_init_ = xdot_;

      q_desired_ = q_init_;
      qdot_desired_.setZero();
      tau_desired_.setZero();
      x_desired_ = x_init_;
      xdot_desired_.setZero();

      ee_pose_pub_timer_ = node_->create_wall_timer(
                           std::chrono::milliseconds(100),
                           std::bind(&Controller::pubEEPoseCallback, this)
                           );

      slow_thread_ = std::thread(&Controller::computeSlowLoop, this);
    }

    void Controller::updateState(const VecMap& pos_dict, 
                                 const VecMap& vel_dict,
                                 const VecMap& tau_ext_dict, 
                                 const VecMap& sensors_dict, 
                                 double current_time)
    {
      current_time_ = current_time;

      for(size_t i=0; i<rd_joint_names_.size(); i++)
      {
        const auto &name = rd_joint_names_[i];
        q_(i) = pos_dict.at(name)(0);
        qdot_(i) = vel_dict.at(name)(0);
        tau_ext_(i) = tau_ext_dict.at(name)(0);
      }
      if(!robot_->updateState(q_, qdot_))
      {
        RCLCPP_ERROR(node_->get_logger(), "[FR3RobotData] Failed to update robot state.");
      }
      x_.matrix() = robot_->getPose();
      xdot_ = robot_->getVelocity();
    }

    void Controller::compute()
    {
      if(is_mode_changed_)
      {
        is_mode_changed_ = false;
        control_start_time_ = current_time_;
        is_goal_pose_changed_ = false;

        q_init_ = q_;
        qdot_init_ = qdot_;
        x_init_ = x_;
        xdot_init_ = xdot_;
        q_desired_ = q_init_;
        qdot_desired_.setZero();
        x_desired_ = x_init_;
        xdot_desired_.setZero();
      }

      if(mode_ == "init")
      {
        Vector7d target_q;
        target_q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, M_PI / 4.;
        q_desired_ = DyrosMath::cubicVector<7>(current_time_, 
                                               control_start_time_,
                                               control_start_time_ + 2.0, 
                                               q_init_, 
                                               target_q, 
                                               Vector7d::Zero(), 
                                               Vector7d::Zero());
        qdot_desired_ = DyrosMath::cubicDotVector<7>(current_time_, 
                                                     control_start_time_,
                                                     control_start_time_ + 2.0, 
                                                     q_init_, 
                                                     target_q, 
                                                     Vector7d::Zero(), 
                                                     Vector7d::Zero());
        tau_desired_ = PDControl(q_desired_, qdot_desired_, Vector7d::Constant(400),Vector7d::Constant(40));
      }
      else if(mode_ == "home")
      {
        Vector7d target_q;
        target_q << 0.0, 0.0, 0.0, -M_PI/2., 0.0, M_PI/2., M_PI / 4.;
        q_desired_ = DyrosMath::cubicVector<7>(current_time_, 
                                               control_start_time_,
                                               control_start_time_ + 2.0, 
                                               q_init_, 
                                               target_q, 
                                               Vector7d::Zero(), 
                                               Vector7d::Zero());
        qdot_desired_ = DyrosMath::cubicDotVector<7>(current_time_, 
                                                     control_start_time_,
                                                     control_start_time_ + 2.0, 
                                                     q_init_, 
                                                     target_q, 
                                                     Vector7d::Zero(), 
                                                     Vector7d::Zero());
        tau_desired_ = PDControl(q_desired_, qdot_desired_, Vector7d::Constant(400),Vector7d::Constant(40));
      }
      else if(mode_ == "clik")
      {
        q_desired_ = CLIK(x_goal_, 2.0);
        tau_desired_ = PDControl(q_desired_, qdot_desired_, Vector7d::Constant(400),Vector7d::Constant(40));
      }
      else
      {
        tau_desired_.setZero();
      }
    }

    CtrlInputMap Controller::getCtrlInput() const
    {
      CtrlInputMap ctrl_dict;
      for(size_t i=0; i<rd_joint_names_.size(); i++)
      {
        const auto &name = rd_joint_names_[i];
        ctrl_dict[name] = tau_desired_[i];
      }
      return ctrl_dict;
    }

    void Controller::setMode(const std::string& mode)
    {
      is_mode_changed_ = true;
      mode_ = mode;
      RCLCPP_INFO(node_->get_logger(), "[FR3Controller] Mode changed: %s", mode.c_str());
    }


    void Controller::keyCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
      RCLCPP_INFO(node_->get_logger(), "[FR3Controller] Key input received: %d", msg->data);
      if(msg->data == 1)      setMode("init");
      else if(msg->data == 2) setMode("home");
      else if(msg->data == 3) setMode("clik");
      else                    setMode("none");
  
    }
    void Controller::subtargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      RCLCPP_INFO(node_->get_logger(),
              "[FR3Controller] Target pose received: position=(%.3f, %.3f, %.3f), "
              "orientation=(%.3f, %.3f, %.3f, %.3f)",
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
              msg->pose.orientation.x, msg->pose.orientation.y,
              msg->pose.orientation.z, msg->pose.orientation.w);
      is_goal_pose_changed_ = true;

      // Convert to 4x4 homogeneous transform
      Eigen::Quaterniond quat(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z);
      Eigen::Matrix3d R = quat.toRotationMatrix();

      x_goal_.linear() = R;
      x_goal_.translation() << msg->pose.position.x,
                               msg->pose.position.y,
                               msg->pose.position.z;
    }
    void Controller::pubEEPoseCallback()
    {
      Eigen::Matrix4d T = x_.matrix();

      auto ee_pose_msg = geometry_msgs::msg::PoseStamped();
      ee_pose_msg.header.frame_id = "fr3_link0";
      ee_pose_msg.header.stamp = node_->now();

      ee_pose_msg.pose.position.x = T(0, 3);
      ee_pose_msg.pose.position.y = T(1, 3);
      ee_pose_msg.pose.position.z = T(2, 3);

      Eigen::Matrix3d R = T.block<3,3>(0,0);
      Eigen::Quaterniond q(R);
      ee_pose_msg.pose.orientation.x = q.x();
      ee_pose_msg.pose.orientation.y = q.y();
      ee_pose_msg.pose.orientation.z = q.z();
      ee_pose_msg.pose.orientation.w = q.w();
      
      ee_pose_pub_->publish(ee_pose_msg);
    }
    
    void Controller::computeSlowLoop()
    {
      rclcpp::Rate r(1000);
      while (rclcpp::ok()) {
        r.sleep();
      }
    }
    
    Vector7d Controller::PDControl(const Vector7d& q_desired, 
                                   const Vector7d& qdot_desired, 
                                   const Vector7d& kp,
                                   const Vector7d& kv)
    {
      Vector7d q_error = q_desired - q_;
      Vector7d qdot_error = qdot_desired - qdot_;
      return robot_->getMassMatrix() * (kp.asDiagonal() * q_error + kv.asDiagonal() * qdot_error) + robot_->getCoriolis();
    }

    Vector7d Controller::CLIK(const Affine3d& target_x, const double& duration)
    {
      if(is_goal_pose_changed_)
      {
        is_goal_pose_changed_ = false;
        control_start_time_ = current_time_;
        x_init_ = x_;
        xdot_init_ = xdot_;
      }

      x_desired_.translation() = DyrosMath::cubicVector<3>(current_time_,
                                                           control_start_time_,
                                                           control_start_time_ + duration,
                                                           x_init_.translation(),
                                                           target_x.translation(),
                                                           xdot_init_.head(3),
                                                           Vector3d::Zero());
      x_desired_.linear() = DyrosMath::rotationCubic(current_time_,
                                                     control_start_time_,
                                                     control_start_time_ + duration,
                                                     x_init_.rotation(),
                                                     target_x.rotation());
      xdot_desired_.head(3) = DyrosMath::cubicDotVector<3>(current_time_,
                                                           control_start_time_,
                                                           control_start_time_ + duration,
                                                           x_init_.translation(),
                                                           target_x.translation(),
                                                           xdot_init_.head(3),
                                                           Vector3d::Zero());
      xdot_desired_.tail(3) = DyrosMath::rotationCubicDot(current_time_,
                                                          control_start_time_,
                                                          control_start_time_ + duration,
                                                          Vector3d::Zero(),
                                                          Vector3d::Zero(),
                                                          x_init_.rotation(),
                                                          target_x.rotation());
      Vector6d x_error, xdot_error;
      x_error.head(3) = x_desired_.translation() - x_.translation();
      x_error.tail(3) = DyrosMath::getPhi(x_desired_.rotation(), x_.rotation());
      xdot_error = xdot_desired_ - xdot_;
      
      MatrixXd J = robot_->getJacobian();
      // MatrixXd J_pinv = J.transpose() * (J*J.transpose()).inverse();
      MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();

      Vector6d kp, kv;
      kp << 50, 50, 50, 100, 100, 100;
      kv << 5, 5, 5, 10, 10, 10;

      qdot_desired_ = J_pinv * (kp.asDiagonal() * x_error + kv.asDiagonal() * xdot_error);
      return q_ + qdot_desired_ * dt_;
    }

    
    /* register with the global registry */
    REGISTER_MJ_CONTROLLER(Controller, "FR3Controller")
}


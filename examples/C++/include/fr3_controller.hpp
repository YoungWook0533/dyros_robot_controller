#pragma once
#include <Eigen/Dense>
#include <unordered_map>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "dyros_robot_controller/manipulator/robot_data.h"
#include "dyros_robot_controller/manipulator/robot_controller.h"

class FR3Controller 
{
public:
  FR3Controller(const double dt);
  ~FR3Controller();

  void updateModel(const double current_time,
                   const std::unordered_map<std::string, double>& qpos_dict,
                   const std::unordered_map<std::string, double>& qvel_dict);

  // returns actuator name -> torque
  std::unordered_map<std::string, double> compute();

  void setMode(const std::string& control_mode);

private:
  const double dt_;
  int dof_{0};

  // Joint space
  Eigen::VectorXd q_;            // Current joint positions
  Eigen::VectorXd qdot_;         // Current joint velocities
  Eigen::VectorXd q_desired_;    // Desired joint positions
  Eigen::VectorXd qdot_desired_; // Desired joint velocities
  Eigen::VectorXd q_init_;       // Initial joint positions at mode start
  Eigen::VectorXd qdot_init_;    // Initial joint velocities at mode start
  Eigen::VectorXd tau_desired_;  // Desired joint torques (control input) 

  // Task space
  Eigen::Affine3d x_;            // Current end-effector pose (homogeneous transform)
  Eigen::Vector6d xdot_;         // Current end-effector velocity (linear + angular) 
  Eigen::Affine3d x_desired_;    // Desired end-effector pose
  Eigen::Vector6d xdot_desired_; // Desired end-effector velocity       
  Eigen::Affine3d x_init_;       // Initial end-effector pose at mode start
  Eigen::Vector6d xdot_init_;    // Initial end-effector velocity at mode start      
  std::string ee_link_name_{"fr3_link8"}; // End-effector link name from FR3 URDF

  // Controller state
  std::string control_mode_{"Home"};
  bool   is_mode_changed_{true};
  double sim_time_{0.0};
  double control_start_time_{0.0};

  // Robot model and controller (Dyros Robot Controller)
  std::shared_ptr<drc::Manipulator::RobotData>       robot_data_;
  std::shared_ptr<drc::Manipulator::RobotController> robot_controller_;

  // Keyboard Interface
  void startKeyListener_();
  void stopKeyListener_();
  void keyLoop_();

  void setRawMode_();
  void restoreTerm_();

  std::atomic<bool> stop_key_{false};
  std::thread key_thread_;
  bool tty_ok_{false};
  struct termios orig_term_{};
};
  
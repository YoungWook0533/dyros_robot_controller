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
#include <mutex>
#include <chrono>

#include "dyros_robot_controller/mobile/robot_data.h"
#include "dyros_robot_controller/mobile/robot_controller.h"

class XLSController 
{
public:
  XLSController(const double dt);
  ~XLSController();

  void updateModel(const double current_time,
                   const std::unordered_map<std::string, double>& qpos_dict,
                   const std::unordered_map<std::string, double>& qvel_dict);

  std::unordered_map<std::string, double> compute();
  void setMode(const std::string& control_mode);

private:
  const double dt_;
  int wheel_num_{0};

  Eigen::VectorXd wheel_pos_;
  Eigen::VectorXd wheel_vel_;
  Eigen::VectorXd wheel_vel_desired_;
  Eigen::Vector3d base_vel_desired_{Eigen::Vector3d::Zero()};

  std::string control_mode_{"Stop"};
  bool   is_mode_changed_{true};
  double sim_time_{0.0};
  double control_start_time_{0.0};

  std::shared_ptr<drc::Mobile::RobotData>       robot_data_;
  std::shared_ptr<drc::Mobile::RobotController> robot_controller_;

  // ===== Keyboard Interface =====
  void startKeyListener_();
  void stopKeyListener_();
  void keyLoop_();
  void setRawMode_();
  void restoreTerm_();

  // Map held keys -> [vx, vy, w]
  Eigen::Vector3d keysToxdot() const;

  // Internal key state
  struct Keys 
  {
    bool up{false}, down{false}, left{false}, right{false};
    bool b{false}, v{false};
    std::chrono::steady_clock::time_point last_event{std::chrono::steady_clock::now()};
  };

  mutable std::mutex keys_mtx_;
  Keys keys_;

  std::atomic<bool> stop_key_{false};
  std::thread key_thread_;
  bool tty_ok_{false};
  struct termios orig_term_{};
};

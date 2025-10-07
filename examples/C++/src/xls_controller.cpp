#include "xls_controller.hpp"

#ifndef ROBOTS_DIRECTORY
#  error "ROBOTS_DIRECTORY is not defined. Define in CMake: -DROBOTS_DIRECTORY=\"${CMAKE_SOURCE_DIR}/../robots\""
#endif

XLSController::XLSController(const double dt)
: dt_(dt)
{
  // Define mobile base kinematics (Mecanum drive) and constraints
  // All geometry is expressed in the base frame:
  //  - base2wheel_positions: 2D position of each wheel center wrt base origin
  //  - base2wheelㄴ_angles: wheel steering angles (fixed here)
  //  - roller_angles: Mecanum roller angles for each wheel
  //  - max_*: saturation limits (used by controller for velocity/acceleration limits)
  drc::Mobile::KinematicParam param;
  param.type = drc::Mobile::DriveType::Mecanum;
  param.wheel_radius = 0.120;
  param.base2wheel_positions = {Eigen::Vector2d( 0.2225,  0.2045),  // Front Left
                                Eigen::Vector2d( 0.2225, -0.2045),  // Front Right
                                Eigen::Vector2d(-0.2225,  0.2045),  // Rear Left
                                Eigen::Vector2d(-0.2225, -0.2045)}; // Rear Right
  param.base2wheel_angles = {0, 0, 0, 0};
  param.roller_angles = {-M_PI/4,  // Front Left
                          M_PI/4,  // Front Right
                          M_PI/4,  // Rear Left
                         -M_PI/4}; // Rear Right
  param.max_lin_speed = 2.0;
  param.max_ang_speed = 3.0;
  param.max_lin_acc = 3.0;
  param.max_ang_acc = 6.0;

  // Initialize robot model and controller (Dyros Robot Controller)
  robot_data_ = std::make_shared<drc::Mobile::RobotData>(param);
  robot_controller_ = std::make_shared<drc::Mobile::RobotController>(dt_, robot_data_);

  // Number of wheels (derived from param)
  wheel_num_ = robot_data_->getWheelNum();

  // Wheel states and command buffers
  wheel_pos_.setZero(wheel_num_);
  wheel_vel_.setZero(wheel_num_);
  wheel_vel_desired_.setZero(wheel_num_);

  // Desired base twist in base frame: [vx, vy, w]
  base_vel_desired_.setZero(3);

  // Information for xls robot
  std::cout << "info: \n" << robot_data_->getVerbose() << std::endl; 

  // Start a keyboard listener (runs in a background thread)
  startKeyListener_();
}

XLSController::~XLSController() 
{
  stopKeyListener_();
}

void XLSController::updateModel(const double current_time,
                                const std::unordered_map<std::string, double>& qpos_dict,
                                const std::unordered_map<std::string, double>& qvel_dict)
{
  // Update simulation time
  sim_time_ = current_time;

  // Read joint positions/velocities from simulation dictionaries
  // NOTE: The order must match the order assumed by KinematicParam above.
  wheel_pos_(0) = qpos_dict.at("front_left_wheel");  // Front Left
  wheel_pos_(1) = qpos_dict.at("front_right_wheel"); // Front Right
  wheel_pos_(2) = qpos_dict.at("rear_left_wheel");   // Rear Left
  wheel_pos_(3) = qpos_dict.at("rear_right_wheel");  // Rear Right
  wheel_vel_(0) = qvel_dict.at("front_left_wheel");
  wheel_vel_(1) = qvel_dict.at("front_right_wheel");
  wheel_vel_(2) = qvel_dict.at("rear_left_wheel");
  wheel_vel_(3) = qvel_dict.at("rear_right_wheel");

  // Push latest wheel states into the model
  robot_data_->updateState(wheel_pos_, wheel_vel_);
}

std::unordered_map<std::string, double> XLSController::compute() 
{
  // One-time initialization per mode entry
  if (is_mode_changed_) 
  {
    is_mode_changed_ = false;
    control_start_time_ = sim_time_;
  }
  // --- Control mode: Stop (zero command) ---
  if (control_mode_ == "Stop") {
    wheel_vel_desired_.setZero();
  }
  // --- Control mode: Base Velocity Tracking ---
  // Convert current key set -> desired base twist [vx, vy, w]
  // Then compute wheel velocities via the controller's mapping
  else if (control_mode_ == "Base Velocity Tracking") 
  {
    base_vel_desired_ = keysToxdot();
    wheel_vel_desired_ = robot_controller_->VelocityCommand(base_vel_desired_);
  }
  
  // Format controller outputs for MuJoCo (actuator name -> value)
  std::unordered_map<std::string, double> ctrl_dict;
  ctrl_dict.reserve(wheel_num_);
  ctrl_dict["front_left_wheel"]  = wheel_vel_desired_(0);
  ctrl_dict["front_right_wheel"] = wheel_vel_desired_(1);
  ctrl_dict["rear_left_wheel"]   = wheel_vel_desired_(2);
  ctrl_dict["rear_right_wheel"]  = wheel_vel_desired_(3);
  return ctrl_dict;
}

void XLSController::setMode(const std::string& control_mode) 
{
  // Change mode and flag reset
  is_mode_changed_ = true;
  control_mode_ = control_mode;
  std::cout << "Control Mode Changed: " << control_mode_ << std::endl;
}

void XLSController::startKeyListener_() 
{
  tty_ok_ = ::isatty(STDIN_FILENO);
  if (!tty_ok_) {
    std::cout << "[XLSController] stdin is not a TTY; keyboard control disabled.\n";
    return;
  }
  setRawMode_();
  stop_key_ = false;
  key_thread_ = std::thread(&XLSController::keyLoop_, this);

  std::cout << "[XLSController] Keyboard: [1]=Stop, [2]=Base Velocity Tracking, arrows=xy, b/v=yaw, [q]=quit listener\n";
}

void XLSController::stopKeyListener_() 
{
  if (!tty_ok_) return;
  stop_key_ = true;
  if (key_thread_.joinable()) key_thread_.join();
  restoreTerm_();
}

void XLSController::keyLoop_() 
{
  using clock = std::chrono::steady_clock;
  const auto clear_after = std::chrono::milliseconds(120);

  while (!stop_key_) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 10 * 1000;

    int ret = ::select(STDIN_FILENO + 1, &rfds, nullptr, nullptr, &tv);
    const auto now = clock::now();

    if (ret > 0 && FD_ISSET(STDIN_FILENO, &rfds)) {
      char buf[32];
      ssize_t nread = ::read(STDIN_FILENO, buf, sizeof(buf));
      if (nread > 0) {
        for (ssize_t i = 0; i < nread; ++i) {
          char c = buf[i];
          if (c == 'q' || c == 'Q') { stop_key_ = true; break; }
          else if (c == '1') { setMode("Stop"); }
          else if (c == '2') { setMode("Base Velocity Tracking"); }
          else if (c == '\x1b') {
            if (i + 2 < nread && buf[i+1] == '[') {
              char code = buf[i+2];
              std::lock_guard<std::mutex> lk(keys_mtx_);
              if      (code == 'A') keys_.up    = true;
              else if (code == 'B') keys_.down  = true;
              else if (code == 'C') keys_.right = true;
              else if (code == 'D') keys_.left  = true;
              keys_.last_event = now;
              i += 2;
            }
          } else {
            std::lock_guard<std::mutex> lk(keys_mtx_);
            if (c == 'b' || c == 'B') keys_.b = true;
            if (c == 'v' || c == 'V') keys_.v = true;
            keys_.last_event = now;
          }
        }
      }
    } else {
      std::lock_guard<std::mutex> lk(keys_mtx_);
      if (now - keys_.last_event > clear_after) {
        keys_.up = keys_.down = keys_.left = keys_.right = false;
        keys_.b = keys_.v = false;
      }
    }
  }
}


void XLSController::setRawMode_() 
{
  if (!tty_ok_) return;
  struct termios raw;
  if (tcgetattr(STDIN_FILENO, &orig_term_) == -1) 
  {
    perror("tcgetattr");
    tty_ok_ = false;
    return;
  }
  raw = orig_term_;
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VMIN]  = 0;
  raw.c_cc[VTIME] = 1;
  if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == -1) 
  {
    perror("tcsetattr");
    tty_ok_ = false;
  }
}

void XLSController::restoreTerm_() 
{
  if (!tty_ok_) return;
  if (tcsetattr(STDIN_FILENO, TCSANOW, &orig_term_) == -1) 
  {
    perror("tcsetattr restore");
  }
}

Eigen::Vector3d XLSController::keysToxdot() const
{
  double vx = 0.0, vy = 0.0, w = 0.0;
  {
    std::lock_guard<std::mutex> lk(keys_mtx_);
    if (keys_.up)    vx += 1.0;   // forward
    if (keys_.down)  vx -= 1.0;   // backward
    if (keys_.left)  vy += 1.0;   // left-to-right
    if (keys_.right) vy -= 1.0;   // right-to-left
    if (keys_.b)     w  += 1.0;   // yaw +
    if (keys_.v)     w  -= 1.0;   // yaw -
  }
  return Eigen::Vector3d(vx, vy, w);
}
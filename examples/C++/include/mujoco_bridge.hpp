#pragma once
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <unordered_map>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <array>

class FR3Controller;

class MujocoBridge {
public:
  explicit MujocoBridge(const std::string& robot_name);
  ~MujocoBridge();

  void run();  // physics/control loop @ ~1kHz

private:
  // helpers
  static std::string get_name(const mjModel* m, int adr);

  // render thread main
  void renderThread();

  // GLFW callbacks (run in render thread)
  static void key_cb(GLFWwindow* w, int key, int scancode, int action, int mods);
  static void mouse_button_cb(GLFWwindow* w, int button, int action, int mods);
  static void cursor_pos_cb(GLFWwindow* w, double xpos, double ypos);
  static void scroll_cb(GLFWwindow* w, double xoff, double yoff);

  // picking helper (render thread)
  int pickBodyAtCursor_(double xpos, double ypos);

private:
  std::string robot_name_;
  mjModel* m_{nullptr};
  mjData*  d_{nullptr};

  // snapshot for rendering
  mjData*  d_render_{nullptr};

  // dictionaries
  std::unordered_map<std::string, int> joint_idx_;
  std::unordered_map<std::string, int> act_idx_;
  std::unordered_map<std::string, int> act_for_joint_;  // joint -> actuator

  // viewer (render thread owns GL context)
  GLFWwindow* window_{nullptr};
  mjvCamera   cam_;
  mjvOption   opt_;
  mjvScene    scn_;
  mjrContext  con_;

  // mouse state (render thread)
  bool   button_left_{false};
  bool   button_middle_{false};
  bool   button_right_{false};
  double lastx_{0.0};
  double lasty_{0.0};

  // “left/right tab” toggles (overlay)
  bool show_left_overlay_{true};
  bool show_right_overlay_{true};

  // selection & external wrench (shared between threads)
  std::atomic<int>    selbody_{-1};   // selected body id, -1 if none
  std::atomic<bool>   apply_force_{false};
  std::atomic<bool>   apply_torque_{false};
  std::mutex          wrench_mtx_;
  std::array<double,3> force_w_{ {0,0,0} };   // world-frame force
  std::array<double,3> torque_w_{ {0,0,0} };  // world-frame torque
  double force_gain_{50.0};   // N per normalized mouse delta
  double torque_gain_{5.0};   // N·m per normalized mouse delta

  // threads & sync
  std::thread render_thread_;
  std::mutex  render_mtx_;             // protects d_render_
  std::atomic<bool> running_{false};
  std::atomic<bool> reset_requested_{false};

  // controller
  std::shared_ptr<FR3Controller> controller_;
};

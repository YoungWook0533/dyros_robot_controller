#pragma once
#include "mujoco_ros_sim/ControllerInterface.hpp"
#include "mujoco_ros_sim/ControllerRegistry.hpp"

#include "pcv_fr3_controller/robot_data.h"

#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <thread>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcutils/logging_macros.h>

#include "math_type_define.h"
#include "suhan_benchmark.h"

using namespace Eigen;

/*
PCV FR3 Joint/Sensor Information
 id | name                 | type   | nq | nv | idx_q | idx_v
----+----------------------+--------+----+----+-------+------
  1 | front_left_steer     | _Hinge |  1 |  1 |     7 |    6
  2 | front_left_rotate    | _Hinge |  1 |  1 |     8 |    7
  3 | rear_left_steer      | _Hinge |  1 |  1 |     9 |    8
  4 | rear_left_rotate     | _Hinge |  1 |  1 |    10 |    9
  5 | rear_right_steer     | _Hinge |  1 |  1 |    11 |   10
  6 | rear_right_rotate    | _Hinge |  1 |  1 |    12 |   11
  7 | front_right_steer    | _Hinge |  1 |  1 |    13 |   12
  8 | front_right_rotate   | _Hinge |  1 |  1 |    14 |   13
  9 | fr3_joint1           | _Hinge |  1 |  1 |    15 |   14
 10 | fr3_joint2           | _Hinge |  1 |  1 |    16 |   15
 11 | fr3_joint3           | _Hinge |  1 |  1 |    17 |   16
 12 | fr3_joint4           | _Hinge |  1 |  1 |    18 |   17
 13 | fr3_joint5           | _Hinge |  1 |  1 |    19 |   18
 14 | fr3_joint6           | _Hinge |  1 |  1 |    20 |   19
 15 | fr3_joint7           | _Hinge |  1 |  1 |    21 |   20

 id | name                 | trn     | target_joint
----+----------------------+---------+-------------
  0 | front_left_rotate    | _Joint  | front_left_rotate
  1 | front_right_rotate   | _Joint  | front_right_rotate
  2 | rear_left_rotate     | _Joint  | rear_left_rotate
  3 | rear_right_rotate    | _Joint  | rear_right_rotate
  4 | front_left_steer     | _Joint  | front_left_steer
  5 | front_right_steer    | _Joint  | front_right_steer
  6 | rear_left_steer      | _Joint  | rear_left_steer
  7 | rear_right_steer     | _Joint  | rear_right_steer
  8 | fr3_joint1           | _Joint  | fr3_joint1
  9 | fr3_joint2           | _Joint  | fr3_joint2
 10 | fr3_joint3           | _Joint  | fr3_joint3
 11 | fr3_joint4           | _Joint  | fr3_joint4
 12 | fr3_joint5           | _Joint  | fr3_joint5
 13 | fr3_joint6           | _Joint  | fr3_joint6
 14 | fr3_joint7           | _Joint  | fr3_joint7

 id | name                        | type             | dim | adr | target (obj)
----+-----------------------------+------------------+-----+-----+----------------
  0 | position_sensor             | Framepos         |   3 |   0 | Site:dyros_pcv_site
  1 | orientation_sensor          | Framequat        |   4 |   3 | Site:dyros_pcv_site
  2 | linear_velocity_sensor      | Framelinvel      |   3 |   7 | Site:dyros_pcv_site
  3 | angular_velocity_sensor     | Frameangvel      |   3 |  10 | Site:dyros_pcv_site

*/

namespace PCVFR3Controller
{
  class Controller : public ControllerInterface
  {
    public:
        // ====================================================================================
        // ================================== Core Functions ================================== 
        // ====================================================================================
        Controller(const rclcpp::Node::SharedPtr& node, double dt, JointDict jd);
        ~Controller() override;
        void starting() override;
        void updateState(const VecMap&,const VecMap&, const VecMap&, const VecMap&, double) override;
        void compute() override;
        CtrlInputMap getCtrlInput() const override;

        // ====================================================================================
        // ===================== Helper / CB / Background Thread Functions ==================== 
        // ====================================================================================
        void setMode(const std::string& mode);
        void keyCallback(const std_msgs::msg::Int32::SharedPtr);
        void subtargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr);
        void subtargetBaseVelCallback(const geometry_msgs::msg::Twist::SharedPtr);
        void pubEEPoseCallback();
        void computeSlowLoop();

        // ====================================================================================
        // ================================= Control Functions ================================
        // ====================================================================================
        ManiVec ManiPDControl(const ManiVec& q_mani_desired, const ManiVec& qdot_mani_desired);
        MobiVec MobileAdmControl(const MobiVec& torque_mobile_desired);
        MobiVec MobileIK(const Vector3d& desired_base_vel);

    private :
        std::shared_ptr<RobotData> robot_;

        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr            key_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr       base_vel_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    ee_pose_pub_;
        
        rclcpp::TimerBase::SharedPtr ee_pose_pub_timer_;
        
        bool is_mode_changed_{false};
        bool is_goal_pose_changed_{false};
        std::string mode_{"home"};
        double control_start_time_;
        double current_time_;

        // mobile base
        Vector3d base_vel_; // [lin_x, lin_y, ang] wrt base frame
        Vector3d base_vel_desired_;
        
        //// joint space state
        VirtualVec q_virtual_;
        VirtualVec q_virtual_init_;
        VirtualVec q_virtual_desired_;
        VirtualVec qdot_virtual_;
        VirtualVec qdot_virtual_init_;
        VirtualVec qdot_virtual_desired_;

        ManiVec q_mani_;
        ManiVec q_mani_init_;
        ManiVec q_mani_desired_;
        ManiVec qdot_mani_;
        ManiVec qdot_mani_init_;
        ManiVec qdot_mani_desired_;

        MobiVec q_mobile_;
        MobiVec q_mobile_init_;
        MobiVec q_mobile_desired_;
        MobiVec qdot_mobile_;
        MobiVec qdot_mobile_init_;

        //// operation space state
        Affine3d x_;
        Affine3d x_goal_;
        Affine3d x_init_;
        Affine3d x_desired_;
        TaskVec xdot_;
        TaskVec xdot_init_;
        TaskVec xdot_desired_;

        //// control input
        ManiVec torque_mani_desired_;
        MobiVec qdot_mobile_desired_;

        // std::unique_ptr<QP::MOMAWholebody> wholebody_qp_;
    };
} // namespace PCVFR3Controller